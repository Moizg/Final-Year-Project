import traci
import csv
import itertools
import os

# ==========================================
# 1. DEFINE THE GRID SEARCH PARAMETERS
# ==========================================
weight_values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
standard_combos = list(itertools.product(weight_values, weight_values, weight_values))

edge_cases = [
    (1.0, 0.0, 0.0), 
    (0.0, 1.0, 0.0), 
    (0.0, 0.0, 1.0)
]
all_weight_combos = standard_combos + edge_cases # 30 total combinations

environments = [
    {"file": "low_balanced.sumocfg", "volume": "Low", "split": "Balanced"},
    {"file": "med_balanced.sumocfg", "volume": "Medium", "split": "Balanced"},
    {"file": "high_balanced.sumocfg", "volume": "High", "split": "Balanced"},
    {"file": "over_balanced.sumocfg", "volume": "Oversaturated", "split": "Balanced"},
    {"file": "low_heavy.sumocfg", "volume": "Low", "split": "Heavy_80_20"},
    {"file": "med_heavy.sumocfg", "volume": "Medium", "split": "Heavy_80_20"},
    {"file": "high_heavy.sumocfg", "volume": "High", "split": "Heavy_80_20"},
    {"file": "over_heavy.sumocfg", "volume": "Oversaturated", "split": "Heavy_80_20"}
]

# ==========================================
# 2. SUMO NETWORK CONSTANTS (VERIFIED)
# ==========================================
TLS_ID = "Center" 

# Mapped directly from your tlLogic in intersection.net.xml
PHASE_NS_GREEN = 0
PHASE_NS_YELLOW = 1
PHASE_EW_GREEN = 4  # Jump to phase 4 for main EW Green
PHASE_EW_YELLOW = 5 # Phase 5 is the EW Yellow

# Map ALL 4 lanes for each incoming edge based on your XML
LANES_NS = [
    "N2C_0", "N2C_1", "N2C_2", "N2C_3",
    "S2C_0", "S2C_1", "S2C_2", "S2C_3"
]

LANES_EW = [
    "E2C_0", "E2C_1", "E2C_2", "E2C_3",
    "W2C_0", "W2C_1", "W2C_2", "W2C_3"
]

DETECTORS_NS = [
    "cam_N_0", "cam_N_1", "cam_N_2", "cam_N_3",
    "cam_S_0", "cam_S_1", "cam_S_2", "cam_S_3"
] 

DETECTORS_EW = [
    "cam_E_0", "cam_E_1", "cam_E_2", "cam_E_3",
    "cam_W_0", "cam_W_1", "cam_W_2", "cam_W_3"
]

MIN_GREEN_TIME = 10

# ==========================================
# 3. SIMULATION RUNNER FUNCTION
# ==========================================
def run_simulation(env, alpha, beta, gamma):
    """Runs a single headless SUMO simulation and returns the metrics."""
    
    sumo_cmd = ["sumo", "-c", env["file"], "--no-warnings", "--step-length", "1"]
    traci.start(sumo_cmd)
    
    step = 0
    time_since_last_switch = 0
    
    # Metrics to track
    total_throughput = 0
    total_teleports = 0
    max_queue_recorded = 0
    cumulative_wait_time = 0
    
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        
        current_phase = traci.trafficlight.getPhase(TLS_ID)
        
        # --- TRAFFIC EXTRACTION LOGIC ---
        # Sum the queues and wait times for North-South
        q_ns = sum(traci.lanearea.getJamLengthVehicle(d) for d in DETECTORS_NS)
        w_ns = sum(traci.lane.getWaitingTime(l) for l in LANES_NS)
        
        # Sum the queues and wait times for East-West
        q_ew = sum(traci.lanearea.getJamLengthVehicle(d) for d in DETECTORS_EW)
        w_ew = sum(traci.lane.getWaitingTime(l) for l in LANES_EW)
        
        # --- GAME THEORY PAYOFF CALCULATION ---
        # Cost function prioritizes serving the axis with the highest accumulated delay/queue
        cost_ns = (alpha * w_ns) + (beta * (q_ns**2)) + (gamma * 0) # Residual set to 0 as slipstream is dropped
        cost_ew = (alpha * w_ew) + (beta * (q_ew**2)) + (gamma * 0)
        
        # --- PHASE SWITCHING LOGIC ---
        # Only evaluate switching if we are currently in a green phase and min green time has passed
        if current_phase in [PHASE_NS_GREEN, PHASE_EW_GREEN]:
            if time_since_last_switch >= MIN_GREEN_TIME:
                
                # If East-West has higher cost/pressure, but light is currently North-South Green
                if cost_ew > cost_ns and current_phase == PHASE_NS_GREEN:
                    traci.trafficlight.setPhase(TLS_ID, PHASE_NS_YELLOW)
                    time_since_last_switch = 0
                    
                # If North-South has higher cost/pressure, but light is currently East-West Green
                elif cost_ns > cost_ew and current_phase == PHASE_EW_GREEN:
                    traci.trafficlight.setPhase(TLS_ID, PHASE_EW_YELLOW)
                    time_since_last_switch = 0
                    
        # Handle Yellow Light Transitions automatically
        elif current_phase == PHASE_NS_YELLOW:
            # Check if yellow phase has finished its default duration (usually 3-4 seconds in SUMO)
            if time_since_last_switch >= 3: 
                traci.trafficlight.setPhase(TLS_ID, PHASE_EW_GREEN)
                time_since_last_switch = 0
                
        elif current_phase == PHASE_EW_YELLOW:
            if time_since_last_switch >= 3:
                traci.trafficlight.setPhase(TLS_ID, PHASE_NS_GREEN)
                time_since_last_switch = 0
                
        # --- TRACK METRICS FOR THE DATASET ---
        total_throughput += traci.simulation.getArrivedNumber()
        total_teleports += traci.simulation.getStartingTeleportNumber()
        
        current_max_q = max(q_ns, q_ew)
        if current_max_q > max_queue_recorded:
            max_queue_recorded = current_max_q
            
        cumulative_wait_time += (w_ns + w_ew)
        
        step += 1
        time_since_last_switch += 1

    traci.close()
    
    avg_wait = cumulative_wait_time / step if step > 0 else 0
    
    return {
        "throughput": total_throughput,
        "teleports": total_teleports,
        "max_queue": max_queue_recorded,
        "avg_wait": round(avg_wait, 2)
    }

# ==========================================
# 4. MAIN EXECUTION (GRID SEARCH)
# ==========================================
if __name__ == "__main__":
    output_file = "fyp_weight_dataset.csv"
    
    print(f"Starting Grid Search. Total Simulations: {len(environments) * len(all_weight_combos)}")
    
    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Volume", "Split", "Alpha", "Beta", "Gamma", "Throughput", "Teleports", "Max_Queue", "Avg_Wait_Time"])
        
        sim_count = 1
        
        for env in environments:
            if not os.path.exists(env["file"]):
                print(f"Skipping {env['file']} - Config file not found!")
                continue
                
            for (a, b, g) in all_weight_combos:
                print(f"Running Sim {sim_count}: Env={env['volume']}_{env['split']} | Weights=({a},{b},{g})")
                
                metrics = run_simulation(env, a, b, g)
                
                writer.writerow([
                    env["volume"], 
                    env["split"], 
                    a, b, g, 
                    metrics["throughput"], 
                    metrics["teleports"], 
                    metrics["max_queue"], 
                    metrics["avg_wait"]
                ])
                # Flush the file to ensure data is written to disk immediately
                file.flush() 
                
                sim_count += 1
                
    print(f"\n✅ Grid search complete! Dataset saved to {output_file}")