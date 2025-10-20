#!/usr/bin/env python3
"""
test_dive.py - Dive control test for SIMPLR-AUV using EnhancedHAL + Dynamics
FIXED: Import paths, method calls, and integration issues
"""

import time

from vehicle_state import VehicleState
from enhanced_hal import EnhancedHAL
from vehicle_control import VehicleControl  # FIXED: correct module name
from navigation import Navigation  # CRITICAL: Add navigation import

# Simulation parameters
DT = 0.5  # seconds
SIM_DURATION = 20.0  # seconds
TARGET_DEPTH = 5.0  # meters
TARGET_BUOYANCY = 'POSITIVE'  # POSITIVE = tank EMPTY, NEUTRAL = tank FULL

def main():
    print("=== Testing Dive Control System (Enhanced HAL + Dynamics) ===")

    # Initialize vehicle state
    vs = VehicleState()
    
    # Initialize HAL in simulation mode
    hal = EnhancedHAL(config={'mode': 'simulation'}, vehicle_state=vs)
    
    # CRITICAL: Initialize navigation system
    navigation = Navigation(vs, hal)
    
    # Initialize control module - FIXED: no parameters needed
    ctrl = VehicleControl()

    print("✅ Simulation components (dynamics + sensors) initialized in HAL")
    print("✅ Initial simulation position set")
    print(f"Initial state: Depth={vs.nav_state['depth']:.1f} m, Speed={vs.nav_state['speed']:.1f} m/s")
    print(f"Target depth: {TARGET_DEPTH:.1f} m, Buoyancy={TARGET_BUOYANCY}")

    print("\nTime   | Depth | Error | Pitch | PitchCmd |  UL   |  UR   |  LR   |  LL   | Thruster")
    print("-" * 84)

    t = 0.0
    while t <= SIM_DURATION:
        # FIXED: Use proper control interface - update() method
        command = {
            'depth': TARGET_DEPTH,
            'speed': 2.0,  # Target forward speed
            'heading': vs.nav_state['heading']  # Maintain current heading
        }
        
        # Get current state for controller
        current_state = {
            'depth': vs.nav_state['depth'],
            'pitch': vs.nav_state['pitch'], 
            'roll': vs.nav_state['roll'],
            'heading': vs.nav_state['heading'],
            'speed': vs.nav_state['speed']
        }
        
        # Use update method instead of compute_depth_control
        control_output = ctrl.update(command, current_state, DT)
        
        # Extract control outputs
        fin_angles = control_output['fin_angles']
        thruster_cmd = control_output['thruster']
        
        # Set actuator commands in vehicle state
        vs.actuator_commands['UL'] = fin_angles[0]  # Port Upper
        vs.actuator_commands['UR'] = fin_angles[2]  # Stbd Upper  
        vs.actuator_commands['LR'] = fin_angles[3]  # Stbd Lower
        vs.actuator_commands['LL'] = fin_angles[1]  # Port Lower
        vs.actuator_commands['thruster_cmd'] = thruster_cmd

        # Update target_state for dynamics
        vs.target_state.update({
            'thruster': thruster_cmd,
            'UL': fin_angles[0],
            'UR': fin_angles[2], 
            'LR': fin_angles[3],
            'LL': fin_angles[1]
        })

        # Map buoyancy command to ballast pump
        if TARGET_BUOYANCY == 'POSITIVE':
            vs.actuator_commands['ballast_cmd'] = 'EMPTY'
        elif TARGET_BUOYANCY == 'NEUTRAL':
            vs.actuator_commands['ballast_cmd'] = 'FILL'
        else:
            vs.actuator_commands['ballast_cmd'] = 'OFF'

        # Update HAL (runs dynamics, sensors, and ballast)
        hal.update(DT)
        
        # CRITICAL: Update navigation to process sensor_data -> nav_state
        navigation.update(DT)

        # After update, if tank is FULL or EMPTY, turn pump OFF
        if vs.ballast_state['tank_status'] in ['FULL', 'EMPTY']:
            vs.actuator_commands['ballast_cmd'] = 'OFF'

        # Read current depth and pitch from vehicle state
        depth = vs.nav_state['depth']
        speed = vs.nav_state['speed']
        error = TARGET_DEPTH - depth
        pitch = vs.nav_state['pitch']
        ul = vs.actuator_commands['UL']
        ur = vs.actuator_commands['UR']
        lr = vs.actuator_commands['LR']
        ll = vs.actuator_commands['LL']
        thruster = vs.actuator_commands['thruster_cmd']

        # FIXED: Get pitch command from controller for display
        pitch_cmd = ctrl.target_depth - depth  # Simple approximation for display

        print(f"{t:5.1f} | {depth:5.1f} | {error:5.1f} | {pitch:5.1f} | {pitch_cmd:7.2f} | "
              f"{ul:5.2f} | {ur:5.2f} | {lr:5.2f} | {ll:5.2f} | {thruster:7.2f}")

        t += DT
        time.sleep(0.01)  # small delay to simulate real-time

if __name__ == "__main__":
    main()