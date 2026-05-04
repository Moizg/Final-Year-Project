import traci
import time
import sys
import os

# --- CONFIGURATION ---
SUMO_CMD = ["sumo-gui", "-c", "run.sumocfg"]  # Change to "sumo" for no-GUI
MIN_GREEN_TIME = 10
MAX_GREEN_TIME = 60

# Payoff Weights (Tunable Parameters)
ALPHA = 1.0  # Weight for Waiting Time (High priority)
BETA  = 0.5  # Weight for Delay/Queue Squared (Penalty for massive queues)
GAMMA = 0.8  # Weight for Residual (Unserved cars)

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

def get_lane_metrics(detectors):
   
    total_q = 0
    total_w = 0
    
    for det in detectors:
        # 1. Get Queue Length (Q) from the E2 Detector (Camera)
        # This counts vehicles strictly inside the camera's 80m box
        q = traci.lanearea.getLastStepVehicleNumber(det)
        
        # 2. Get Waiting Time (W) from the LANE
        # The camera object doesn't track waiting time live, but the Lane does.
        # First, find out which lane this camera is on (e.g., "N2C_0")
        lane_id = traci.lanearea.getLaneID(det)
        
        # Now ask the Lane: "How many seconds have cars waited here?"
        w = traci.lane.getWaitingTime(lane_id)
        
        total_q += q
        total_w += w
        
    return total_q, total_w

def calculate_payoff(q_total, w_total, g_estimated=30):
    """
    Implements the Advanced Payoff Function:
    Cost = -(α·W + β·D + γ·Q_residual)
    
    We return positive 'Pressure' (Cost of NOT serving this lane).
    Higher Pressure = Needs Green Light more urgently.
    """
    # 1. Waiting Cost (W_i)
    # The user defined W = Q * (T - g). 
    # SUMO gives us exact 'w_total' (seconds waited so far), which is more accurate.
    cost_wait = ALPHA * w_total

    # 2. Delay Cost (D_i) ~ Webster's Delay
    # D = Q^2 / (2 * s * g). 
    # Simplified: We treat Q^2 as the penalty for overcrowding.
    cost_delay = BETA * (q_total ** 2)

    # 3. Residual Cost (Q_residual)
    # Estimate: How many cars will be left if we give 30s Green?
    # Saturation flow (s) approx 0.5 veh/sec per lane.
    # 4 lanes * 0.5 = 2.0 veh/sec capacity.
    capacity = 2.0 * g_estimated 
    q_residual = max(0, q_total - capacity)
    cost_residual = GAMMA * q_residual

    # Total Pressure (The cost we are suffering by keeping this Red)
    pressure = cost_wait + cost_delay + cost_residual
    return pressure

def run_simulation():
    step = 0
    traci.start(SUMO_CMD)
    
    # Detector Groups
    dets_ns = [
        "cam_N_0", "cam_N_1", "cam_N_2", "cam_N_3",
        "cam_S_0", "cam_S_1", "cam_S_2", "cam_S_3"
    ]
    dets_ew = [
        "cam_E_0", "cam_E_1", "cam_E_2", "cam_E_3",
        "cam_W_0", "cam_W_1", "cam_W_2", "cam_W_3"
    ]

    last_switch_time = 0
    current_phase_idx = 0 
    # Phase 0 = NS Green, Phase 1 = NS Yellow
    # Phase 2 = EW Green, Phase 3 = EW Yellow
    traci.trafficlight.setPhase("Center", 0)

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        # Put this right after you calculate q_ns, w_ns, q_ew, w_ew
        if step == 300: # 5 minutes into the simulation
            print("\n--- SNAPSHOT FOR NASH MATRIX (T=300) ---")
            print(f"NS Axis -> Queue (D): {q_ns}, Total Wait (W): {w_ns}")
            print(f"EW Axis -> Queue (D): {q_ew}, Total Wait (W): {w_ew}")
            print("----------------------------------------\n")
        # Only make decisions if we are in a Green Phase (0 or 2)
        # and Minimum Green Time has passed
        curr_phase = traci.trafficlight.getPhase("Center")
        time_since_switch = step - last_switch_time

        if (curr_phase == 0 or curr_phase == 2) and time_since_switch > MIN_GREEN_TIME:
            
            # 1. Gather Data
            q_ns, w_ns = get_lane_metrics(dets_ns)
            q_ew, w_ew = get_lane_metrics(dets_ew)
            
            
            
            # 2. Calculate Payoffs (Pressures)
            score_ns = calculate_payoff(q_ns, w_ns)
            score_ew = calculate_payoff(q_ew, w_ew)

            # 3. Game Theory Decision (Nash Equilibrium Logic)
            # If we are NS Green (Phase 0) but EW score is significantly higher -> Switch
            if curr_phase == 0:
                # Add a hysteresis factor (10%) to prevent rapid flickering
                if score_ew > (score_ns * 1.1):
                    print(f"T={step}: Switching to EW. (NS Score: {score_ns:.1f} vs EW Score: {score_ew:.1f})")
                    traci.trafficlight.setPhase("Center", 1) # Yellow
                    last_switch_time = step

            # If we are EW Green (Phase 2) but NS score is higher -> Switch
            elif curr_phase == 2:
                if score_ns > (score_ew * 1.1):
                    print(f"T={step}: Switching to NS. (NS Score: {score_ns:.1f} vs EW Score: {score_ew:.1f})")
                    traci.trafficlight.setPhase("Center", 3) # Yellow
                    last_switch_time = step
            
            # Force switch if Maximum Green Time exceeded
            if time_since_switch > MAX_GREEN_TIME:
                if curr_phase == 0:
                    traci.trafficlight.setPhase("Center", 1)
                else:
                    traci.trafficlight.setPhase("Center", 3)
                last_switch_time = step

        # Handle Yellow Transitions (Auto-switch to Red/Green after 4s)
        if curr_phase == 1 and time_since_switch >= 4:
            traci.trafficlight.setPhase("Center", 2) # NS Yellow -> EW Green
            last_switch_time = step
        elif curr_phase == 3 and time_since_switch >= 4:
            traci.trafficlight.setPhase("Center", 0) # EW Yellow -> NS Green
            last_switch_time = step

        step += 1

    traci.close()

if __name__ == "__main__":
    run_simulation()