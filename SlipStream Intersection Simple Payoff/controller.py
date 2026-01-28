import traci
import time
import sys
import os

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

def get_queue_lengths():
    """
    Reads the 'Cameras' (E2 Detectors) to see how many cars are waiting.
    """
    queues = {}
    cameras = [
        "cam_N_0", "cam_N_1", "cam_N_2",
        "cam_S_0", "cam_S_1", "cam_S_2",
        "cam_E_0", "cam_E_1", "cam_E_2",
        "cam_W_0", "cam_W_1", "cam_W_2"
    ]
    
    for cam in cameras:
        # Returns the number of vehicles on the detector
        queues[cam] = traci.lanearea.getLastStepVehicleNumber(cam)
        
    return queues

def get_payoff(queues):
    """
    GAME THEORY LOGIC: Calculates the 'Pressure' (Payoff) for each direction.
    
    Player 1: North-South (NS)
    Player 2: East-West (EW)
    
    Payoff = Total Queue Length in that direction.
    """
    # Sum up all cars waiting in North and South lanes
    payoff_ns = (queues["cam_N_0"] + queues["cam_N_1"] + queues["cam_N_2"] +
                 queues["cam_S_0"] + queues["cam_S_1"] + queues["cam_S_2"])
                 
    # Sum up all cars waiting in East and West lanes
    payoff_ew = (queues["cam_E_0"] + queues["cam_E_1"] + queues["cam_E_2"] +
                 queues["cam_W_0"] + queues["cam_W_1"] + queues["cam_W_2"])
                 
    return payoff_ns, payoff_ew

def run_simulation():
    step = 0
    traci.start(["sumo-gui", "-c", "run.sumocfg"])
    
    # Define our Phase IDs (These typically match the XML)
    # Phase 0: NS Green (East-West Red)
    # Phase 1: NS Yellow
    # Phase 2: EW Green (North-South Red)
    # Phase 3: EW Yellow
    
    current_phase = 0
    traci.trafficlight.setPhase("Center", current_phase)
    
    # Minimum Green Time (seconds) to prevent flickering
    MIN_GREEN_TIME = 10
    last_switch_time = 0

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        
        # --- GAME THEORY DECISION LOOP ---
        # We don't want to switch every single second. 
        # Only check after the minimum green time has passed.
        if step - last_switch_time > MIN_GREEN_TIME:
            
            queues = get_queue_lengths()
            payoff_ns, payoff_ew = get_payoff(queues)
            
            # Get current phase
            current_phase = traci.trafficlight.getPhase("Center")
            
            # LOGIC: If we are Green for NS, but EW has higher payoff, we switch.
            
            # Case 1: Currently NS Green (Phase 0)
            if current_phase == 0:
                if payoff_ew > payoff_ns:
                    print(f"Time {step}: Switching to EW! (NS Score: {payoff_ns} vs EW Score: {payoff_ew})")
                    traci.trafficlight.setPhase("Center", 1) # Set Yellow
                    last_switch_time = step # Reset timer
            
            # Case 2: Currently EW Green (Phase 2)
            elif current_phase == 2:
                if payoff_ns > payoff_ew:
                    print(f"Time {step}: Switching to NS! (NS Score: {payoff_ns} vs EW Score: {payoff_ew})")
                    traci.trafficlight.setPhase("Center", 3) # Set Yellow
                    last_switch_time = step # Reset timer
                    
            # Handle Yellow Phases (Automatically transition to Red after 4s)
            # (In a real advanced setup, you'd manage yellow timing precisely, 
            # but SUMO often handles duration if we let it run).
            # For this simple script, we need to manually move from Yellow -> Green
            elif current_phase == 1 and (step - last_switch_time > 4):
                 traci.trafficlight.setPhase("Center", 2) # Go EW Green
                 last_switch_time = step
            elif current_phase == 3 and (step - last_switch_time > 4):
                 traci.trafficlight.setPhase("Center", 0) # Go NS Green
                 last_switch_time = step

        step += 1

    traci.close()

if __name__ == "__main__":
    run_simulation()