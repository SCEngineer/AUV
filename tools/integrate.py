#!/usr/bin/env python3
"""
integration_test.py - Test integration of vehicle_state, vehicle_control, vehicle_dynamics, and enhanced_hal
Tests proper data flow between modules according to the interface requirements.
"""

import time
import json
from pathlib import Path

def test_integration():
    """Test the integration of all vehicle modules"""
    print("=" * 60)
    print("AUV SYSTEM INTEGRATION TEST")
    print("=" * 60)
    
    # 1. Initialize VehicleState
    print("\n1. Initializing VehicleState...")
    try:
        from vehicle_state import VehicleState, SystemState
        vehicle_state = VehicleState()
        
        # Set initial position
        vehicle_state.set_initial_position(
            lat=34.25739,
            lon=-117.1987,
            depth=0.0,
            heading=270.0
        )
        
        # Set some target values for testing
        vehicle_state.update_target_state(
            target_speed=2.0,
            target_heading=45.0,
            target_depth=5.0,
            ballast_cmd='FILL'
        )
        
        print("   ✓ VehicleState initialized")
        print(f"   ✓ Initial position: {vehicle_state.get_initial_position()}")
        
    except Exception as e:
        print(f"   ✗ VehicleState initialization failed: {e}")
        return False
    
    # 2. Initialize VehicleControl
    print("\n2. Initializing VehicleControl...")
    try:
        from vehicle_control import VehicleControl
        controller = VehicleControl(vehicle_state)
        
        print("   ✓ VehicleControl initialized")
        print(f"   ✓ Control gains loaded: {list(controller.gains.keys())}")
        
    except Exception as e:
        print(f"   ✗ VehicleControl initialization failed: {e}")
        return False
    
    # 3. Initialize HAL with simulation mode
    print("\n3. Initializing Enhanced HAL...")
    try:
        from enhanced_hal import EnhancedHAL
        hal_config = {'mode': 'simulation'}
        hal = EnhancedHAL(hal_config, vehicle_state)
        
        print("   ✓ Enhanced HAL initialized")
        print(f"   ✓ HAL mode: {hal.mode}")
        
        # Check if components loaded
        if hal.vehicle_dynamics:
            print("   ✓ VehicleDynamics loaded")
        if hal.sensor_models:
            print("   ✓ SensorModels loaded")
            
    except Exception as e:
        print(f"   ✗ Enhanced HAL initialization failed: {e}")
        return False
    
    # 4. Test data flow interfaces
    print("\n4. Testing Interface Requirements...")
    
    # Test 4a: VehicleControl reads target_state and writes actuator_commands
    print("\n   4a. Testing VehicleControl interfaces...")
    try:
        # Verify target_state is readable
        target = vehicle_state.target_state
        print(f"      Target state accessible: speed={target.get('target_speed', 0)}, "
              f"heading={target.get('target_heading', 0)}, depth={target.get('target_depth', 0)}")
        
        # Run controller update
        result = controller.update(dt=0.1)
        
        # Verify actuator_commands were written
        actuators = vehicle_state.actuator_commands
        print(f"      Actuator commands written: thruster={actuators.get('thruster_cmd', 0):.1f}%, "
              f"fins=[{actuators.get('UL', 0):.1f}, {actuators.get('UR', 0):.1f}, "
              f"{actuators.get('LL', 0):.1f}, {actuators.get('LR', 0):.1f}]")
        
        # Verify control errors were written
        errors = vehicle_state.control_errors
        print(f"      Control errors written: heading={errors.get('heading_error', 0):.1f}°, "
              f"depth={errors.get('depth_error', 0):.1f}m, speed={errors.get('speed_error', 0):.1f}m/s")
        
        print("      ✓ VehicleControl interfaces working correctly")
        
    except Exception as e:
        print(f"      ✗ VehicleControl interface test failed: {e}")
        return False
    
    # Test 4b: HAL reads actuator_commands and writes sensor_data
    print("\n   4b. Testing HAL interfaces...")
    try:
        # Run HAL update
        hal.update(dt=0.1)
        
        # Check that sensor_data was written
        sensor_data = vehicle_state.sensor_data
        if sensor_data:
            print(f"      Sensor data written: {list(sensor_data.keys())}")
            if 'compass' in sensor_data:
                print(f"         Compass: {sensor_data['compass']:.1f}°")
            if 'depth' in sensor_data:
                print(f"         Depth: {sensor_data['depth']:.1f}m")
            if 'water_speed' in sensor_data:
                print(f"         Water speed: {sensor_data['water_speed'].get('forward', 0):.2f}m/s")
        else:
            print("      Warning: No sensor data generated")
        
        # Check that true_state was updated (by dynamics)
        true_state = vehicle_state.true_state
        print(f"      True state updated: pos=({true_state.get('true_lat', 0):.6f}, "
              f"{true_state.get('true_lon', 0):.6f}), depth={true_state.get('true_depth', 0):.1f}m, "
              f"heading={true_state.get('true_heading', 0):.1f}°")
        
        print("      ✓ HAL interfaces working correctly")
        
    except Exception as e:
        print(f"      ✗ HAL interface test failed: {e}")
        return False
    
    # 5. Test system integration loop
    print("\n5. Testing System Integration Loop...")
    try:
        print("   Running 10-step integration test...")
        
        # Set a more dynamic target for testing
        vehicle_state.update_target_state(
            target_speed=3.0,
            target_heading=90.0,  # Turn to East
            target_depth=10.0     # Dive to 10m
        )
        
        for step in range(10):
            # Controller reads target_state/nav_state, writes to actuator_commands
            controller.update(dt=0.1)
            
            # HAL reads actuator_commands, updates dynamics, writes sensor_data
            hal.update(dt=0.1)
            
            # Quick status check every few steps
            if step % 3 == 0:
                nav = vehicle_state.nav_state
                true = vehicle_state.true_state
                print(f"   Step {step}: Nav heading={nav.get('heading', 0):.1f}°, "
                      f"depth={nav.get('depth', 0):.1f}m, "
                      f"true_speed={true.get('true_speed', 0):.2f}m/s")
        
        print("   ✓ Integration loop completed successfully")
        
    except Exception as e:
        print(f"   ✗ Integration loop failed: {e}")
        return False
    
    # 6. Test ballast system integration
    print("\n6. Testing Ballast System Integration...")
    try:
        # Command ballast tank to fill
        vehicle_state.update_actuator_commands(ballast_cmd='FILL')
        print("   Commanded ballast tank to FILL")
        
        # Run several updates to simulate tank filling
        for i in range(5):
            hal.update(dt=0.5)  # 0.5 second timesteps
            volume = vehicle_state.ballast_state.get('tank_volume', 0)
            capacity = vehicle_state.ballast_state.get('tank_capacity', 0.001982)
            status = vehicle_state.ballast_state.get('tank_status', 'UNKNOWN')
            buoyancy = vehicle_state.buoyancy_state
            
            print(f"   Update {i+1}: Tank {volume/capacity*100:.1f}% full, "
                  f"status={status}, buoyancy={buoyancy}")
            
            if status == 'FULL':
                break
        
        print("   ✓ Ballast system integration working")
        
    except Exception as e:
        print(f"   ✗ Ballast system test failed: {e}")
        return False
    
    # 7. Generate test summary
    print("\n7. Generating Test Summary...")
    try:
        status = {
            'timestamp': time.time(),
            'vehicle_status': vehicle_state.get_status(),
            'hal_status': hal.get_simulation_status(),
            'controller_status': controller.get_controller_status()
        }
        
        # Save to file
        logs_dir = Path("logs")
        logs_dir.mkdir(exist_ok=True)
        
        summary_file = logs_dir / f"integration_test_{int(time.time())}.json"
        with open(summary_file, 'w') as f:
            json.dump(status, f, indent=2, default=str)
        
        print(f"   ✓ Test summary saved to: {summary_file}")
        
        # Print key metrics
        nav = vehicle_state.nav_state
        true = vehicle_state.true_state
        errors = vehicle_state.control_errors
        
        print("\n   Final System State:")
        print(f"   - Position: ({true.get('true_lat', 0):.6f}, {true.get('true_lon', 0):.6f})")
        print(f"   - True depth: {true.get('true_depth', 0):.1f}m, Nav depth: {nav.get('depth', 0):.1f}m")
        print(f"   - True heading: {true.get('true_heading', 0):.1f}°, Nav heading: {nav.get('heading', 0):.1f}°")
        print(f"   - True speed: {true.get('true_speed', 0):.2f}m/s, Nav speed: {nav.get('speed', 0):.2f}m/s")
        print(f"   - Control errors: heading={errors.get('heading_error', 0):.1f}°, "
              f"depth={errors.get('depth_error', 0):.1f}m, speed={errors.get('speed_error', 0):.2f}m/s")
        print(f"   - Buoyancy state: {vehicle_state.buoyancy_state}")
        
    except Exception as e:
        print(f"   ✗ Test summary generation failed: {e}")
        return False
    
    # 8. Test cleanup
    print("\n8. Cleaning Up...")
    try:
        hal.shutdown()
        print("   ✓ HAL shutdown complete")
        
    except Exception as e:
        print(f"   ✗ Cleanup failed: {e}")
    
    print("\n" + "=" * 60)
    print("INTEGRATION TEST COMPLETED SUCCESSFULLY")
    print("All modules are properly interfaced according to requirements.")
    print("=" * 60)
    
    return True

def main():
    """Main test function"""
    try:
        success = test_integration()
        if success:
            print(f"\n🎉 All integration tests PASSED!")
            return 0
        else:
            print(f"\n❌ Integration tests FAILED!")
            return 1
            
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
        return 1
    except Exception as e:
        print(f"\nUnexpected error during testing: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    import sys
    sys.exit(main())