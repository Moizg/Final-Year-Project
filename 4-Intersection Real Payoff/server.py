"""
╔══════════════════════════════════════════════════════════════════════════╗
║  Adaptive Traffic Signal Scheduling — WebSocket Simulation Server      ║
║  Game Theory (Nash Equilibrium) Controller with Live Data Streaming    ║
║                                                                        ║
║  Architecture:                                                         ║
║    • asyncio event loop on the main thread (WebSocket server)          ║
║    • TraCI simulation runs on a background thread via run_in_executor  ║
║    • Thread-safe shared state via threading.Lock                       ║
║    • Bi-directional comms: server pushes data, client pushes weights   ║
╚══════════════════════════════════════════════════════════════════════════╝
"""

import asyncio
import json
import threading
import time
import os
import sys
import math
import signal
from collections import deque

import websockets

# ─── SUMO / TraCI Setup ─────────────────────────────────────────────────
if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("ERROR: Please declare environment variable 'SUMO_HOME'")

import traci

# ─── Simulation Configuration ───────────────────────────────────────────
SUMO_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "run.sumocfg")
SUMO_CMD = ["sumo", "-c", SUMO_CONFIG, "--no-warnings"]  # Headless mode

MIN_GREEN_TIME = 10   # Minimum seconds before allowing a phase switch
MAX_GREEN_TIME = 60   # Force-switch after this many seconds
YELLOW_DURATION = 4   # Yellow phase duration (seconds)

WS_HOST = "0.0.0.0"
WS_PORT = 8765


# ═══════════════════════════════════════════════════════════════════════════
# Thread-Safe Shared State
# ═══════════════════════════════════════════════════════════════════════════
class SimulationState:
    """
    Shared state object protected by a threading lock.
    The TraCI thread writes simulation data here; the WebSocket thread reads it.
    The WebSocket thread writes weight updates; the TraCI thread reads them.
    """

    def __init__(self):
        self.lock = threading.Lock()

        # ── Tunable Game-Theory Weights (client can adjust live) ──
        # α: Weight for cumulative waiting time cost
        # β: Weight for delay/congestion penalty (quadratic queue term)
        # γ: Weight for residual queue cost (unserved vehicles)
        self.alpha = 1.0
        self.beta = 0.5
        self.gamma = 0.8

        # ── Simulation output data (written by TraCI thread) ──
        self.vehicles = []          # [{id, x, y, speed, lane, angle, type}]
        self.queues = {             # Queue lengths per axis
            "ns": 0, "ew": 0,
            "ns_wait": 0.0, "ew_wait": 0.0
        }
        self.traffic_light = {      # Current signal state
            "phase": 0,
            "phase_name": "NS_GREEN",
            "ns_state": "green",
            "ew_state": "red",
            "time_in_phase": 0
        }
        self.nash_matrix = {        # 2×2 Game Theory payoff matrix
            "ns_go": 0.0,           # Cost(NS Go, EW Yield)
            "ew_go": 0.0,           # Cost(NS Yield, EW Go)
            "ns_yield": 0.0,        # Cost if NS yields
            "ew_yield": 0.0,        # Cost if EW yields
            "equilibrium": "NS",    # Which axis the Nash Eq. favors
            "components": {
                "ns": {"wait": 0, "delay": 0, "residual": 0},
                "ew": {"wait": 0, "delay": 0, "residual": 0}
            }
        }
        self.metrics = {
            "step": 0,
            "total_vehicles": 0,
            "avg_speed": 0.0,
            "avg_waiting_time": 0.0,
            "throughput": 0,         # Vehicles that completed their trip
            "sim_time": 0
        }

        # ── Control flags ──
        self.running = False        # Is simulation active?
        self.should_stop = False    # Graceful shutdown signal
        self.sim_finished = False   # Simulation completed naturally

        # ── Historical metrics for averaging ──
        self._wait_history = deque(maxlen=100)
        self._speed_history = deque(maxlen=100)
        self._throughput_count = 0

    def get_weights(self):
        """Thread-safe read of current game theory weights."""
        with self.lock:
            return self.alpha, self.beta, self.gamma

    def set_weights(self, alpha=None, beta=None, gamma=None):
        """Thread-safe update of game theory weights from the frontend."""
        with self.lock:
            if alpha is not None:
                self.alpha = float(alpha)
            if beta is not None:
                self.beta = float(beta)
            if gamma is not None:
                self.gamma = float(gamma)

    def get_snapshot(self):
        """Thread-safe read of the full simulation state for WebSocket broadcast."""
        with self.lock:
            return {
                "vehicles": self.vehicles.copy(),
                "queues": dict(self.queues),
                "traffic_light": dict(self.traffic_light),
                "nash_matrix": json.loads(json.dumps(self.nash_matrix)),
                "metrics": dict(self.metrics),
                "weights": {
                    "alpha": self.alpha,
                    "beta": self.beta,
                    "gamma": self.gamma
                }
            }


# Global state instance
state = SimulationState()


# ═══════════════════════════════════════════════════════════════════════════
# Game Theory Mathematics
# ═══════════════════════════════════════════════════════════════════════════

def get_lane_metrics(detectors):
    """
    Aggregates queue length (Q) and waiting time (W) across all detector
    lanes for a given axis (NS or EW).

    Q comes from E2 laneAreaDetectors (camera-based vehicle count),
    W comes from the underlying lane's cumulative waiting time.

    Returns:
        (total_queue_length, total_waiting_time)
    """
    total_q = 0
    total_w = 0.0

    for det in detectors:
        try:
            # Queue length from the E2 (area) detector — vehicles in the 80m box
            q = traci.lanearea.getLastStepVehicleNumber(det)
            # Waiting time from the lane itself (more accurate than detector)
            lane_id = traci.lanearea.getLaneID(det)
            w = traci.lane.getWaitingTime(lane_id)
            total_q += q
            total_w += w
        except traci.TraCIException:
            continue

    return total_q, total_w


def calculate_payoff(q_total, w_total, alpha, beta, gamma, g_estimated=30):
    """
    Advanced Payoff Function for the Game-Theoretic Signal Controller.

    Models the "pressure" (cost of NOT serving) for a traffic axis:
        Pressure = α·W + β·D + γ·Q_res

    Where:
        W (Waiting Cost)   = α × total_waiting_time
            → Penalizes cumulative delay experienced by all queued vehicles.

        D (Delay Cost)     = β × Q²
            → Quadratic penalty inspired by Webster's delay formula.
            → Small queues are cheap; large queues are disproportionately expensive.
            → Approximation of D = Q² / (2·s·g) with constants folded into β.

        Q_res (Residual)   = γ × max(0, Q − capacity)
            → Estimates how many vehicles will remain unserved if we give
              the standard green time (g_estimated seconds).
            → Capacity = saturation_flow × g = 2.0 veh/s × 30s = 60 vehicles
            → Penalizes situations where demand exceeds service capacity.

    Returns:
        (pressure, wait_component, delay_component, residual_component)
    """
    # 1. Waiting Cost — direct penalty for accumulated wait
    cost_wait = alpha * w_total

    # 2. Delay Cost — quadratic queue penalty (Webster's approximation)
    cost_delay = beta * (q_total ** 2)

    # 3. Residual Queue Cost — unserved vehicles after green phase
    # Saturation flow ≈ 0.5 veh/s/lane × 4 lanes = 2.0 veh/s
    capacity = 2.0 * g_estimated
    q_residual = max(0, q_total - capacity)
    cost_residual = gamma * q_residual

    pressure = cost_wait + cost_delay + cost_residual
    return pressure, cost_wait, cost_delay, cost_residual


def compute_nash_matrix(q_ns, w_ns, q_ew, w_ew, alpha, beta, gamma):
    """
    Constructs the 2×2 Normal-Form Game Matrix for the intersection.

    Players:  NS-axis (Player 1) vs EW-axis (Player 2)
    Strategies: {Go, Yield} for each player

    The payoff represents the COST to the system. Lower cost = better.

    Matrix layout (costs):
                    EW Go           EW Yield
    NS Go       [conflict]      [ns_go, ew_yield]
    NS Yield    [ns_yield, ew_go]   [both_yield]

    Nash Equilibrium: The axis with higher pressure should Go (lower total cost).
    In a traffic signal context, only one axis can have green at a time,
    so the equilibrium is always one of the off-diagonal cells.
    """
    # Pressure = cost of keeping this axis RED (i.e., benefit of giving it GREEN)
    p_ns, w_ns_c, d_ns_c, r_ns_c = calculate_payoff(q_ns, w_ns, alpha, beta, gamma)
    p_ew, w_ew_c, d_ew_c, r_ew_c = calculate_payoff(q_ew, w_ew, alpha, beta, gamma)

    # Off-diagonal: feasible states (one goes, one yields)
    # When NS Goes: NS cost is LOW (served), EW cost is HIGH (still waiting)
    # When EW Goes: EW cost is LOW (served), NS cost is HIGH (still waiting)
    ns_go_cost = p_ew      # Cost to system when NS goes = EW's suffering
    ew_go_cost = p_ns      # Cost to system when EW goes = NS's suffering
    # Yield costs (the axis that yields continues accumulating pressure)
    ns_yield_cost = p_ns   # NS yields → NS keeps suffering
    ew_yield_cost = p_ew   # EW yields → EW keeps suffering

    # Nash Equilibrium: the axis with higher pressure should receive green
    equilibrium = "NS" if p_ns >= p_ew else "EW"

    return {
        "ns_go": round(ns_go_cost, 2),
        "ew_go": round(ew_go_cost, 2),
        "ns_yield": round(ns_yield_cost, 2),
        "ew_yield": round(ew_yield_cost, 2),
        "ns_pressure": round(p_ns, 2),
        "ew_pressure": round(p_ew, 2),
        "equilibrium": equilibrium,
        "components": {
            "ns": {
                "wait": round(w_ns_c, 2),
                "delay": round(d_ns_c, 2),
                "residual": round(r_ns_c, 2)
            },
            "ew": {
                "wait": round(w_ew_c, 2),
                "delay": round(d_ew_c, 2),
                "residual": round(r_ew_c, 2)
            }
        }
    }


# ═══════════════════════════════════════════════════════════════════════════
# TraCI Simulation Loop (runs on background thread)
# ═══════════════════════════════════════════════════════════════════════════

# E2 Lane-Area Detector IDs (configured in detectors.add.xml)
DETS_NS = [f"cam_{d}_{i}" for d in ("N", "S") for i in range(4)]
DETS_EW = [f"cam_{d}_{i}" for d in ("E", "W") for i in range(4)]

# Phase mapping for the "Center" traffic light
PHASE_NAMES = {
    0: "NS_GREEN",   # North-South has green
    1: "NS_YELLOW",  # Transitioning NS → EW
    2: "EW_GREEN",   # East-West has green
    3: "EW_YELLOW"   # Transitioning EW → NS
}


def get_phase_states(phase):
    """Maps SUMO phase index to human-readable signal states."""
    if phase == 0:
        return "green", "red"
    elif phase == 1:
        return "yellow", "red"
    elif phase == 2:
        return "red", "green"
    elif phase == 3:
        return "red", "yellow"
    return "unknown", "unknown"


def run_traci_loop():
    """
    Main simulation loop. Runs on a background thread.

    1. Starts SUMO in headless mode
    2. Each step: reads vehicle data, computes game theory, updates signals
    3. Writes all data to the shared SimulationState object
    4. Respects the should_stop flag for graceful shutdown
    """
    global state

    try:
        traci.start(SUMO_CMD)
        print(f"[SUMO] Simulation started — config: {SUMO_CONFIG}")
    except Exception as e:
        print(f"[SUMO] Failed to start: {e}")
        with state.lock:
            state.running = False
            state.sim_finished = True
        return

    with state.lock:
        state.running = True
        state.sim_finished = False

    step = 0
    last_switch_time = 0
    departed_count = 0

    # Initialize traffic light to NS Green (Phase 0)
    try:
        traci.trafficlight.setPhase("Center", 0)
    except traci.TraCIException:
        pass

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            # ── Check for graceful shutdown request ──
            if state.should_stop:
                print("[SUMO] Graceful shutdown requested.")
                break

            traci.simulationStep()

            # ── Read current game theory weights (may be updated by client) ──
            alpha, beta, gamma = state.get_weights()

            # ── Gather Vehicle Data ──
            vehicle_ids = traci.vehicle.getIDList()
            vehicles = []
            total_speed = 0.0
            total_wait = 0.0

            for vid in vehicle_ids:
                try:
                    x, y = traci.vehicle.getPosition(vid)
                    speed = traci.vehicle.getSpeed(vid)
                    lane = traci.vehicle.getLaneID(vid)
                    angle = traci.vehicle.getAngle(vid)
                    vtype = traci.vehicle.getTypeID(vid)
                    wait = traci.vehicle.getWaitingTime(vid)

                    vehicles.append({
                        "id": vid,
                        "x": round(x, 1),
                        "y": round(y, 1),
                        "speed": round(speed, 2),
                        "lane": lane,
                        "angle": round(angle, 1),
                        "type": vtype,
                        "waiting": round(wait, 1)
                    })

                    total_speed += speed
                    total_wait += wait
                except traci.TraCIException:
                    continue

            # ── Gather Queue Data from E2 Detectors ──
            q_ns, w_ns = get_lane_metrics(DETS_NS)
            q_ew, w_ew = get_lane_metrics(DETS_EW)

            # ── Traffic Light State ──
            curr_phase = traci.trafficlight.getPhase("Center")
            time_since_switch = step - last_switch_time
            ns_state, ew_state = get_phase_states(curr_phase)

            # ── Compute Nash Equilibrium Matrix ──
            nash = compute_nash_matrix(q_ns, w_ns, q_ew, w_ew, alpha, beta, gamma)

            # ═══════════════════════════════════════════════════
            # Game Theory Signal Switching Decision
            # ═══════════════════════════════════════════════════
            if (curr_phase in (0, 2)) and time_since_switch > MIN_GREEN_TIME:
                p_ns = nash["ns_pressure"]
                p_ew = nash["ew_pressure"]

                if curr_phase == 0:  # Currently NS Green
                    # Switch to EW if EW pressure exceeds NS by 10% (hysteresis)
                    # Hysteresis prevents rapid oscillation near equal pressures
                    if p_ew > (p_ns * 1.1):
                        traci.trafficlight.setPhase("Center", 1)  # → NS Yellow
                        last_switch_time = step

                elif curr_phase == 2:  # Currently EW Green
                    if p_ns > (p_ew * 1.1):
                        traci.trafficlight.setPhase("Center", 3)  # → EW Yellow
                        last_switch_time = step

                # Force switch at maximum green time to ensure fairness
                if time_since_switch > MAX_GREEN_TIME:
                    if curr_phase == 0:
                        traci.trafficlight.setPhase("Center", 1)
                    else:
                        traci.trafficlight.setPhase("Center", 3)
                    last_switch_time = step

            # ── Handle Yellow → Green transitions ──
            if curr_phase == 1 and time_since_switch >= YELLOW_DURATION:
                traci.trafficlight.setPhase("Center", 2)  # NS Yellow → EW Green
                last_switch_time = step
            elif curr_phase == 3 and time_since_switch >= YELLOW_DURATION:
                traci.trafficlight.setPhase("Center", 0)  # EW Yellow → NS Green
                last_switch_time = step

            # ── Throughput: count departed vehicles ──
            departed_count += traci.simulation.getDepartedNumber()
            arrived = traci.simulation.getArrivedNumber()

            # ── Compute rolling averages ──
            n_veh = len(vehicles)
            avg_speed = (total_speed / n_veh) if n_veh > 0 else 0.0
            avg_wait = (total_wait / n_veh) if n_veh > 0 else 0.0

            state._speed_history.append(avg_speed)
            state._wait_history.append(avg_wait)
            state._throughput_count += arrived

            smoothed_speed = sum(state._speed_history) / len(state._speed_history) if state._speed_history else 0
            smoothed_wait = sum(state._wait_history) / len(state._wait_history) if state._wait_history else 0

            # Re-read phase after potential switch
            curr_phase = traci.trafficlight.getPhase("Center")
            ns_state, ew_state = get_phase_states(curr_phase)

            # ── Write everything to shared state ──
            with state.lock:
                state.vehicles = vehicles
                state.queues = {
                    "ns": q_ns,
                    "ew": q_ew,
                    "ns_wait": round(w_ns, 1),
                    "ew_wait": round(w_ew, 1)
                }
                state.traffic_light = {
                    "phase": curr_phase,
                    "phase_name": PHASE_NAMES.get(curr_phase, "UNKNOWN"),
                    "ns_state": ns_state,
                    "ew_state": ew_state,
                    "time_in_phase": time_since_switch
                }
                state.nash_matrix = nash
                state.metrics = {
                    "step": step,
                    "total_vehicles": n_veh,
                    "avg_speed": round(smoothed_speed, 2),
                    "avg_waiting_time": round(smoothed_wait, 2),
                    "throughput": state._throughput_count,
                    "sim_time": step
                }

            step += 1

            # Small sleep to prevent CPU saturation and allow WS thread to breathe
            time.sleep(0.01)

    except Exception as e:
        print(f"[SUMO] Simulation error: {e}")

    finally:
        try:
            traci.close()
            print("[SUMO] TraCI connection closed cleanly.")
        except Exception:
            pass

        with state.lock:
            state.running = False
            state.sim_finished = True
            state._throughput_count = 0
            state._wait_history.clear()
            state._speed_history.clear()


# ═══════════════════════════════════════════════════════════════════════════
# WebSocket Server
# ═══════════════════════════════════════════════════════════════════════════

# Track connected clients
connected_clients = set()


async def handler(websocket):
    """
    Handles a single WebSocket client connection.

    - Sends simulation snapshots at ~10 FPS while the simulation runs.
    - Listens for incoming messages (weight updates, control commands).
    """
    connected_clients.add(websocket)
    client_addr = websocket.remote_address
    print(f"[WS] Client connected: {client_addr}  (total: {len(connected_clients)})")

    try:
        # Task for receiving client messages
        async def receive_messages():
            async for message in websocket:
                try:
                    data = json.loads(message)
                    msg_type = data.get("type", "")

                    if msg_type == "update_weights":
                        # Client adjusted α/β/γ sliders
                        state.set_weights(
                            alpha=data.get("alpha"),
                            beta=data.get("beta"),
                            gamma=data.get("gamma")
                        )
                        print(f"[WS] Weights updated: α={state.alpha}, β={state.beta}, γ={state.gamma}")

                    elif msg_type == "start_simulation":
                        if not state.running:
                            state.should_stop = False
                            loop = asyncio.get_event_loop()
                            loop.run_in_executor(None, run_traci_loop)
                            print("[WS] Simulation start requested.")

                    elif msg_type == "stop_simulation":
                        state.should_stop = True
                        print("[WS] Simulation stop requested.")

                except json.JSONDecodeError:
                    pass

        # Task for sending simulation data
        async def send_updates():
            while True:
                if state.running:
                    snapshot = state.get_snapshot()
                    snapshot["type"] = "simulation_update"
                    try:
                        await websocket.send(json.dumps(snapshot))
                    except websockets.ConnectionClosed:
                        break
                    await asyncio.sleep(0.1)  # ~10 FPS update rate
                elif state.sim_finished:
                    try:
                        await websocket.send(json.dumps({
                            "type": "simulation_ended",
                            "metrics": state.get_snapshot()["metrics"]
                        }))
                    except websockets.ConnectionClosed:
                        break
                    state.sim_finished = False
                    await asyncio.sleep(0.5)
                else:
                    # Idle — send heartbeat with current weights
                    try:
                        await websocket.send(json.dumps({
                            "type": "idle",
                            "weights": {
                                "alpha": state.alpha,
                                "beta": state.beta,
                                "gamma": state.gamma
                            }
                        }))
                    except websockets.ConnectionClosed:
                        break
                    await asyncio.sleep(1.0)

        # Run both tasks concurrently
        await asyncio.gather(
            receive_messages(),
            send_updates()
        )

    except websockets.ConnectionClosed:
        pass
    finally:
        connected_clients.discard(websocket)
        print(f"[WS] Client disconnected: {client_addr}  (total: {len(connected_clients)})")


async def main():
    """Entry point: starts the WebSocket server."""
    print("=" * 58)
    print("  Traffic Signal WebSocket Server")
    print(f"  Listening on ws://{WS_HOST}:{WS_PORT}")
    print("  Press Ctrl+C to stop")
    print("=" * 58)

    # Graceful shutdown on Ctrl+C
    stop = asyncio.Future()

    def signal_handler():
        state.should_stop = True
        if not stop.done():
            stop.set_result(None)

    loop = asyncio.get_event_loop()
    try:
        loop.add_signal_handler(signal.SIGINT, signal_handler)
        loop.add_signal_handler(signal.SIGTERM, signal_handler)
    except NotImplementedError:
        # Windows doesn't support add_signal_handler
        pass

    async with websockets.serve(handler, WS_HOST, WS_PORT):
        print("[WS] Server ready. Waiting for connections...")
        try:
            await asyncio.Future()  # Run forever
        except asyncio.CancelledError:
            pass

    # Cleanup
    state.should_stop = True
    print("[WS] Server shut down.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        state.should_stop = True
        print("\n[WS] Server stopped by user.")
