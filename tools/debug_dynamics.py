#!/usr/bin/env python3
"""
Debug version to see what's happening with vertical dynamics
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
        # Dive command - strong nose-down pitch
        self.actuator_commands = {
            'thruster_cmd': 70,  # percent
            'UL': -50,   # Strong nose down
            'UR': -50,
            'LR': 50,
            'LL': 50
        }
        self.buoyancy_state = 'NEUTRAL'

# Create vehicle state and dynamics
vs = DummyVehicleState()
dynamics = VehicleDynamics(vs, dt=0.1)

print("=== DIVE DEBUG TEST ===")
print("Strong dive command: UL=-50, UR=-50, LR=50, LL=50, Thruster=70%")
print()

for t in range(30):  # 3 seconds
    # Get values before update
    old_depth = vs.true_state['true_depth']
    old_vz = vs.true_state['true_vertical_velocity']
    old_pitch = vs.true_state['true_pitch']
    old_speed = vs.true_state['true_speed']
    
    # Update dynamics
    dynamics.update()
    
    # Get values after update
    new_depth = vs.true_state['true_depth']
    new_vz = vs.true_state['true_vertical_velocity'] 
    new_pitch = vs.true_state['true_pitch']
    new_speed = vs.true_state['true_speed']
    
    # Calculate some key values manually for debugging
    vertical_thrust_component = new_speed * math.sin(new_pitch)
    buoyancy_force = dynamics.buoyancy - dynamics.mass * dynamics.gravity
    total_fz = -vertical_thrust_component + buoyancy_force
    
    # Print detailed info every few steps
    if t % 5 == 0 or t < 5:
        print(f"t={t*0.1:.1f}s:")
        print(f"  Speed: {new_speed:.2f} m/s")
        print(f"  Pitch: {math.degrees(new_pitch):.1f}° (rad: {new_pitch:.3f})")
        print(f"  Vertical thrust component: {vertical_thrust_component:.3f} N")
        print(f"  Buoyancy force: {buoyancy_force:.3f} N") 
        print(f"  Total vertical force: {total_fz:.3f} N")
        print(f"  Vertical velocity: {new_vz:.3f} m/s")
        print(f"  Depth change: {new_depth - old_depth:.4f} m")
        print(f"  Depth: {new_depth:.3f} m")
        print()

print("Debug test completed!")
print(f"Final depth: {vs.true_state['true_depth']:.3f} m")
print(f"Final vertical velocity: {vs.true_state['true_vertical_velocity']:.3f} m/s")