#!/usr/bin/env python3
"""
simple_control_test.py - Test the actual VehicleControl class
"""

import sys
sys.path.append('.')

from vehicle_control import VehicleControl

class MockVehicleState:
    def __init__(self):
        self.nav_state = {
            'heading': 0.0,
            'speed': 0.0,
            'depth': 0.0
        }
        self.target_state = {
            'target_heading': 0.0,
            'target_speed': 0.0,
            'target_depth': 0.0
        }
        self.actuator_commands = {}
        self.control_errors = {}

    def update(self, **kwargs):
        pass

# Test 1: Basic initialization
print("Test 1: VehicleControl initialization")
vs = MockVehicleState()
vc = VehicleControl(vs)
print("✓ VehicleControl created successfully")

# Test 2: Speed error test
print("\nTest 2: Speed error generation")
vs.nav_state['speed'] = 0.0
vs.target_state['target_speed'] = 2.0
vs.nav_state['heading'] = 270.0
vs.target_state['target_heading'] = 270.0

vc.update(0.1)

print(f"Actuator commands: {vs.actuator_commands}")
print(f"Control errors: {vs.control_errors}")

thruster_cmd = vs.actuator_commands.get('thruster_cmd', 0)
speed_error = vs.control_errors.get('speed_error', 0)

if thruster_cmd > 0:
    print(f"✓ Thruster command generated: {thruster_cmd:.1f}%")
else:
    print(f"✗ No thruster command generated (got {thruster_cmd})")

if abs(speed_error - 2.0) < 0.1:
    print(f"✓ Speed error calculated correctly: {speed_error:.1f} m/s")
else:
    print(f"✗ Speed error incorrect (got {speed_error}, expected 2.0)")

# Test 3: Heading error test  
print("\nTest 3: Heading error generation")
vs.nav_state['heading'] = 0.0
vs.target_state['target_heading'] = 90.0

vc.update(0.1)

heading_error = vs.control_errors.get('heading_error', 0)
ul_fin = vs.actuator_commands.get('UL', 0)

if abs(heading_error - 90.0) < 1.0:
    print(f"✓ Heading error calculated correctly: {heading_error:.1f}°")
else:
    print(f"✗ Heading error incorrect (got {heading_error}, expected 90.0)")

if abs(ul_fin) > 0.1:
    print(f"✓ Fin commands generated: UL={ul_fin:.1f}°")
else:
    print(f"✗ No fin commands generated")

# Test 4: Error handling test
print("\nTest 4: Error handling")
vs_bad = MockVehicleState()
vs_bad.nav_state = None  # This should cause an error

vc_bad = VehicleControl(vs_bad)
try:
    vc_bad.update(0.1)
    print("✓ Error handling works - no crash")
except Exception as e:
    print(f"✗ Error handling failed: {e}")

print("\nTest completed!")