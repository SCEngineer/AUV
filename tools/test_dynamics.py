#!/usr/bin/env python3
"""
Standalone VehicleDynamics test runner for SIMPLR-AUV
This script demonstrates the dynamics in isolation.
"""

import math
import time

# --- Import the system-friendly vehicle_dynamics class ---
from vehicle_dynamics import VehicleDynamics

# --- Enhanced vehicle state object ---
class DummyVehicleState:
    def __init__(self):
        self.true_state = {
            'true_speed': 0.0,
            'true_pitch': 0.0,  # radians
            'true_roll': 0.0,
            'true_heading': 0.0,  # degrees
            'true_depth': 0.0,
            'true_lat': 34.25,    # arbitrary start lat
            'true_lon': -117.18,  # arbitrary start lon
            'true_pitch_rate': 0.0,
            'true_yaw_rate': 0.0,
            'true_roll_rate': 0.0,
            'true_vertical_velocity': 0.0
        }
        # Actuator commands that will drive the dynamics
        self.actuator_commands = {
            'thruster_cmd': 60,  # percent (0-100)
            'UL': 0,   # Upper Left fin position (-100 to 100)
            'UR': 0,   # Upper Right fin position
            'LR': 0,   # Lower Right fin position  
            'LL': 0    # Lower Left fin position
        }
        self.buoyancy_state = 'NEUTRAL'

def run_test_scenario(name, duration, thruster, ul, ur, lr, ll):
    """Run a test scenario with specific control inputs"""
    print(f"\n=== {name} ===")
    
    # Create fresh vehicle state and dynamics
    vs = DummyVehicleState()
    dynamics = VehicleDynamics(vs, dt=0.1)
    
    # Set actuator commands
    vs.actuator_commands['thruster_cmd'] = thruster
    vs.actuator_commands['UL'] = ul
    vs.actuator_commands['UR'] = ur  
    vs.actuator_commands['LR'] = lr
    vs.actuator_commands['LL'] = ll
    
    steps = int(duration / dynamics.dt)
    
    print(f"Thruster: {thruster}%, Fins: UL={ul}, UR={ur}, LR={lr}, LL={ll}")
    print("Time | Speed | Depth | Heading | Pitch | V_vertical")
    print("-" * 60)
    
    for t in range(steps):
        try:
            dynamics.update()
        except Exception as e:
            import traceback
            print("Exception occurred during dynamics step:")
            traceback.print_exc()
            break

        # Print key states every few steps
        if t % 5 == 0 or t == steps - 1:  # Every 0.5 seconds
            s = vs.true_state
            print(f"{t*dynamics.dt:4.1f} | {s['true_speed']:5.2f} | {s['true_depth']:5.3f} | "
                  f"{s['true_heading']:7.1f} | {math.degrees(s['true_pitch']):5.1f} | "
                  f"{s['true_vertical_velocity']:8.3f}")

# --- Run different test scenarios ---
if __name__ == "__main__":
    print("Starting vehicle dynamics test scenarios...\n")
    
    # Test 1: Forward motion only
    run_test_scenario("Forward Motion Test", 3.0, thruster=80, 
                     ul=0, ur=0, lr=0, ll=0)
    
    # Test 2: Dive maneuver (nose down)
    run_test_scenario("Dive Maneuver", 5.0, thruster=70,
                     ul=-20, ur=-20, lr=20, ll=20)
    
    # Test 3: Surface maneuver (nose up) 
    run_test_scenario("Surface Maneuver", 4.0, thruster=60,
                     ul=15, ur=15, lr=-15, ll=-15)
    
    # Test 4: Right turn
    run_test_scenario("Right Turn", 4.0, thruster=50,
                     ul=0, ur=20, lr=20, ll=0)
    
    # Test 5: Complex maneuver - right turn while diving
    run_test_scenario("Right Turn + Dive", 6.0, thruster=75,
                     ul=-10, ur=10, lr=30, ll=-10)
    
    print("\nAll test scenarios completed!")
    print("\nFin control explanation:")
    print("- Pitch: Top fins vs bottom fins (UL+UR vs LR+LL)")
    print("- Yaw: Left fins vs right fins (UL+LL vs UR+LR)")
    print("- Positive fin = deflection in positive direction")
    print("- Thruster: 0-100% forward thrust")