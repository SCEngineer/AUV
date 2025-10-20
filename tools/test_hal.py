#!/usr/bin/env python3
"""
test_enhanced_hal.py - Unit test for enhanced_hal.py
"""

import sys
sys.path.append('.')

from enhanced_hal import EnhancedHAL
from vehicle_state import VehicleState

# Mock sensor_models to avoid dependency
class MockSensorModels:
    def __init__(self, vehicle_state):
        self.vehicle_state = vehicle_state
    
    def update(self, dt=None):
        return {
            'compass': self.vehicle_state.true_state.get('true_heading', 0),
            'depth': self.vehicle_state.true_state.get('true_depth', 0),
            'water_speed': self.vehicle_state.true_state.get('true_speed', 0)
        }

# Test 1: HAL initialization
print("Test 1: HAL Initialization")

config = {
    'mode': 'simulation',
    'sensor_update_rate': 10.0
}

vehicle_state = VehicleState()
hal = EnhancedHAL(config, vehicle_state)

print(f"HAL mode: {hal.mode}")
print(f"Vehicle dynamics exists: {hal.vehicle_dynamics is not None}")
print(f"Sensor models exists: {hal.sensor_models is not None}")

if hal.vehicle_dynamics is not None:
    print("✓ Vehicle dynamics initialized successfully")
else:
    print("✗ Vehicle dynamics initialization failed")

if hal.sensor_models is not None:
    print("✓ Sensor models initialized successfully")
else:
    print("✗ Sensor models initialization failed")

# Test 2: Initial state check
print("\nTest 2: Initial Vehicle State")
print(f"Initial position set: {vehicle_state.is_initial_position_set()}")
print(f"True state: {vehicle_state.true_state}")

initial_speed = vehicle_state.true_state.get('true_speed', 0)
initial_heading = vehicle_state.true_state.get('true_heading', 0)

print(f"Initial speed: {initial_speed}")
print(f"Initial heading: {initial_heading}")

# Test 3: Set some actuator commands to test dynamics
print("\nTest 3: Setting Actuator Commands")

# Set thruster and fin commands like the working test
vehicle_state.actuator_commands.update({
    'thruster_cmd': 80.0,
    'UL': 0.0,
    'UR': 0.0,
    'LR': 0.0, 
    'LL': 0.0
})

print(f"Actuator commands set: {vehicle_state.actuator_commands}")

# Test 4: HAL update cycle
print("\nTest 4: HAL Update Cycle")

# Store initial values
initial_true_speed = vehicle_state.true_state.get('true_speed', 0)
initial_true_heading = vehicle_state.true_state.get('true_heading', 0)

print(f"Before update - Speed: {initial_true_speed}, Heading: {initial_true_heading}")

try:
    # Run several update cycles
    for i in range(10):
        hal.update(0.1)
        
        if i == 0:  # Check after first update
            speed_after_1 = vehicle_state.true_state.get('true_speed', 0)
            heading_after_1 = vehicle_state.true_state.get('true_heading', 0)
            print(f"After 1 update - Speed: {speed_after_1}, Heading: {heading_after_1}")
        
        if i == 4:  # Check after 5 updates
            speed_after_5 = vehicle_state.true_state.get('true_speed', 0)
            heading_after_5 = vehicle_state.true_state.get('true_heading', 0)
            print(f"After 5 updates - Speed: {speed_after_5}, Heading: {heading_after_5}")
    
    # Final check
    final_speed = vehicle_state.true_state.get('true_speed', 0)
    final_heading = vehicle_state.true_state.get('true_heading', 0)
    print(f"After 10 updates - Speed: {final_speed}, Heading: {final_heading}")
    
    # Test if dynamics actually ran
    if final_speed > initial_true_speed:
        print("✓ Vehicle dynamics are updating true_state correctly")
        print(f"  Speed changed from {initial_true_speed} to {final_speed}")
    else:
        print("✗ Vehicle dynamics are NOT updating true_state")
        print(f"  Speed remained at {final_speed}")
    
    print("✓ HAL update completed without errors")
    
except Exception as e:
    print(f"✗ HAL update failed with error: {e}")
    import traceback
    traceback.print_exc()

# Test 5: Sensor data update
print("\nTest 5: Sensor Data Update")

sensor_data = vehicle_state.sensor_data
print(f"Sensor data type: {type(sensor_data)}")
print(f"Sensor data: {sensor_data}")

if sensor_data and isinstance(sensor_data, dict):
    print("✓ Sensor data is being updated")
else:
    print("✗ Sensor data update failed")

# Test 6: Ballast simulation
print("\nTest 6: Ballast Simulation")

# Test ballast fill
vehicle_state.actuator_commands['ballast_cmd'] = 'FILL'
initial_volume = vehicle_state.ballast_state.get('tank_volume', 0)

hal.update(0.1)
after_fill_volume = vehicle_state.ballast_state.get('tank_volume', 0)

print(f"Ballast volume - Initial: {initial_volume}, After FILL: {after_fill_volume}")

if after_fill_volume > initial_volume:
    print("✓ Ballast FILL command working")
else:
    print("✗ Ballast FILL command not working")

# Test ballast empty
vehicle_state.actuator_commands['ballast_cmd'] = 'EMPTY'
hal.update(0.1)
after_empty_volume = vehicle_state.ballast_state.get('tank_volume', 0)

print(f"After EMPTY: {after_empty_volume}")

if after_empty_volume < after_fill_volume:
    print("✓ Ballast EMPTY command working")
else:
    print("✗ Ballast EMPTY command not working")

# Test 7: HAL status
print("\nTest 7: HAL Status")

status = hal.get_simulation_status()
print(f"HAL status: {status}")

# Summary
print("\n" + "="*50)
print("HAL TEST SUMMARY")
print("="*50)

if hal.vehicle_dynamics and final_speed > initial_true_speed:
    print("✓ HAL is properly integrating vehicle dynamics")
    print("✓ True state is being updated correctly")
    print("✓ The HAL-dynamics integration is working")
else:
    print("✗ CRITICAL: HAL-dynamics integration is broken")
    print("✗ Vehicle dynamics are not being called or not updating true_state")
    
    # Additional debugging
    print("\nDEBUG INFO:")
    print(f"  hal.vehicle_dynamics object: {hal.vehicle_dynamics}")
    print(f"  hal.mode: {hal.mode}")
    print(f"  Vehicle dynamics type: {type(hal.vehicle_dynamics)}")
    
    if hal.vehicle_dynamics:
        print("  Vehicle dynamics exists but true_state not changing")
        print("  -> Dynamics update() method not being called properly")
    else:
        print("  Vehicle dynamics object is None")
        print("  -> Initialization failed")

print("\nTest completed!")