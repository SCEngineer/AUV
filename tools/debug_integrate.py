#!/usr/bin/env python3
"""
debug_crash.py - Minimal diagnostic to find where the silent crash occurs
"""

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.resolve()
sys.path.insert(0, str(PROJECT_ROOT))

print("Starting diagnostic...")

try:
    print("Step 1: Importing vehicle_state...")
    from vehicle_state import VehicleState
    print("✓ vehicle_state imported")
    
    print("Step 2: Creating VehicleState...")
    vehicle_state = VehicleState()
    print("✓ VehicleState created")
    
    print("Step 3: Importing executive...")
    from executive import Executive
    print("✓ executive imported")
    
    print("Step 4: Creating Executive...")
    executive = Executive(vehicle_state)
    print("✓ Executive created")
    
    print("Step 5: Importing plugin manager...")
    from guidance.plugin_manager import GuidancePluginManager
    print("✓ plugin_manager imported")
    
    print("Step 6: Creating GuidancePluginManager...")
    guidance = GuidancePluginManager(vehicle_state)
    print("✓ GuidancePluginManager created")
    
    print("Step 7: Testing task assignment...")
    vehicle_state.current_task = 'SWIM_TO_WAYPOINT'
    vehicle_state.update_target_state(lat=34.257, lon=-117.199, speed=2.0)
    print(f"✓ Task assigned: {vehicle_state.current_task}")
    print(f"✓ target_state: {vehicle_state.target_state}")
    
    print("Step 8: Testing guidance update...")
    guidance.update(0.1)
    print("✓ Guidance update completed")
    
    if guidance.current_plugin:
        print(f"✓ Plugin started: {guidance.current_plugin.name}")
    else:
        print("✗ No plugin started")
        
    print("\nDiagnostic completed successfully!")
    
except Exception as e:
    print(f"\n✗ CRASH at current step: {e}")
    import traceback
    traceback.print_exc()
except SystemExit as e:
    print(f"\n✗ System exit called: {e}")
except KeyboardInterrupt:
    print("\n✗ Interrupted by user")
except:
    print(f"\n✗ Unknown error: {sys.exc_info()[0]}")
    import traceback
    traceback.print_exc()