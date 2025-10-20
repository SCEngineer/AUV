#!/usr/bin/env python3
"""
test_control.py - Updated to match current vehicle_state and system architecture
Tests VehicleControl, VehicleDynamics, and DivePlugin integration with accurate buoyancy states
"""

import sys
import os
import math
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'guidance'))

from vehicle_state import VehicleState, SystemState
from vehicle_control import VehicleControl
from vehicle_dynamics import VehicleDynamics
from plugin_registry import PluginRegistry


def test_control_system():
    """Test VehicleControl outputs with updated buoyancy system"""
    print("\n=== Testing VehicleControl ===")

    vs = VehicleState()
    controller = VehicleControl(vs)

    # Set initial position - this initializes both true_state and nav_state
    vs.set_initial_position(34.257, -117.198, 0.0, 290.0)

    # Test heading error (controller now reads from true_state, not nav_state)
    vs.update_true_state(true_heading=290.0)
    vs.update_target_state(target_heading=300.0)
    print(f"\nTesting 10° heading error (290° -> 300°):")
    result = controller.update(0.1)

    print(f"  Heading error: {result['heading_error']:.1f}°")
    print(f"  Fin commands: UL={result['fins']['UL']:.1f}° UR={result['fins']['UR']:.1f}° "
          f"LL={result['fins']['LL']:.1f}° LR={result['fins']['LR']:.1f}°")
    print(f"  Thruster: {result['thruster_cmd']:.1f}%")

    # Test heading wrap-around
    vs.update_true_state(true_heading=350.0)
    vs.update_target_state(target_heading=10.0)
    result2 = controller.update(0.1)
    print(f"\nTesting wrap-around (350° -> 10°):")
    print(f"  Heading error: {result2['heading_error']:.1f}°")

    # Test depth error with NEUTRAL buoyancy (submerged trim)
    vs.ballast_state['tank_volume'] = vs.ballast_state['tank_capacity']  # Full tank
    vs.ballast_state['tank_status'] = 'FULL'
    vs.buoyancy_state = 'NEUTRAL'
    vs.update_true_state(true_heading=290.0, true_depth=0.0, true_speed=0.0)
    vs.update_target_state(target_heading=290.0, target_speed=1.0, target_depth=3.0, ballast_cmd='OFF')
    
    print(f"\nTesting 3m depth error in NEUTRAL buoyancy (0m -> 3m):")
    result3 = controller.update(0.1)

    print(f"  Depth error: {result3['depth_error']:.1f}m")
    print(f"  Current depth: {vs.true_state.get('true_depth', 0.0):.1f}m")
    print(f"  Current speed: {vs.true_state.get('true_speed', 0.0):.1f}m/s")
    print(f"  Fin commands: UL={result3['fins']['UL']:.1f}° UR={result3['fins']['UR']:.1f}° "
          f"LL={result3['fins']['LL']:.1f}° LR={result3['fins']['LR']:.1f}°")
    print(f"  Thruster: {result3['thruster_cmd']:.1f}%")
    print(f"  Buoyancy state: {vs.buoyancy_state}")
    
    # Verify ballast command pass-through
    assert vs.target_state.get('ballast_cmd') == 'OFF'
    assert vs.actuator_commands.get('ballast_cmd') == 'OFF'

    return controller


def test_dynamics_isolated():
    """Test VehicleDynamics with updated physics model"""
    print("\n=== Testing VehicleDynamics ===")

    vs = VehicleState()
    dynamics = VehicleDynamics(vs)

    vs.set_initial_position(34.257, -117.198, 0.0, 290.0)

    # Test with POSITIVE buoyancy (surfaced trim)
    print("Test: POSITIVE buoyancy (surfaced trim) - should have slight upward force")
    vs.ballast_state['tank_volume'] = 0.0  # Empty tank
    vs.ballast_state['tank_status'] = 'EMPTY'
    vs.buoyancy_state = 'POSITIVE'
    vs.update_actuator_commands(thruster_cmd=0.0, UL=0.0, UR=0.0, LL=0.0, LR=0.0, ballast_cmd='OFF')

    initial_depth = vs.true_state['true_depth']
    for _ in range(50):  # Run longer to see buoyancy effect
        dynamics.update(0.1)
    
    final_depth = vs.true_state['true_depth']
    final_vz = vs.true_state.get('true_vertical_velocity', 0.0)
    print(f"  Depth change: {initial_depth:.3f}m -> {final_depth:.3f}m")
    print(f"  Vertical velocity: {final_vz:.3f}m/s")
    print(f"  Buoyancy state: {vs.buoyancy_state}")
    
    # Test with NEUTRAL buoyancy (submerged trim)
    print("\nTest: NEUTRAL buoyancy (submerged trim) - should have slight downward force")
    vs.ballast_state['tank_volume'] = vs.ballast_state['tank_capacity']  # Full tank
    vs.ballast_state['tank_status'] = 'FULL'  
    vs.buoyancy_state = 'NEUTRAL'
    vs.update_true_state(true_depth=0.0, true_vertical_velocity=0.0)
    
    initial_depth = vs.true_state['true_depth']
    for _ in range(50):
        dynamics.update(0.1)
    
    final_depth = vs.true_state['true_depth']
    final_vz = vs.true_state.get('true_vertical_velocity', 0.0)
    print(f"  Depth change: {initial_depth:.3f}m -> {final_depth:.3f}m")
    print(f"  Vertical velocity: {final_vz:.3f}m/s")
    print(f"  Buoyancy state: {vs.buoyancy_state}")

    return dynamics


def test_dive_plugin():
    """Test DivePlugin with corrected parameter extraction"""
    print("\n=== Testing DivePlugin ===")

    vs = VehicleState()
    registry = PluginRegistry.get_instance()
    
    # Set up proper submerged trim state
    print("Setting up submerged trim state...")
    vs.ballast_state['tank_volume'] = vs.ballast_state['tank_capacity']
    vs.ballast_state['tank_status'] = 'FULL'
    vs.buoyancy_state = 'NEUTRAL'
    vs.set_initial_position(34.257, -117.198, 0.0, 290.0)
    vs.update_true_state(true_heading=290.0, true_depth=0.0, true_speed=0.0)
    print(f"  Vehicle state: depth=0.0m, buoyancy={vs.buoyancy_state}")
    
    # Test DIVE plugin with corrected parameter name
    dive_plugin = registry.create_plugin('DIVE', vs)

    # CRITICAL: Use 'target_depth' not 'depth' to match what dive.py expects
    task_params = {'target_depth': 3.0, 'speed': 1.0, 'timeout': 60.0}
    print(f"  Initializing DIVE with task_params: {task_params}")
    success = dive_plugin.initialize(task_params)
    assert success
    
    print(f"  DIVE plugin target_depth: {dive_plugin.target_depth}")
    print(f"  Target state after init: depth={vs.target_state.get('target_depth')}m, speed={vs.target_state.get('target_speed')}m/s")

    # Execute dive plugin
    vs.update_true_state(true_depth=0.0, true_speed=0.0)
    dive_plugin.execute(0.1)
    
    # Test completion when close to target
    vs.update_true_state(true_depth=2.8)
    completed = dive_plugin.check_completion()
    print(f"  Completion test: target=3.0m, current=2.8m, completed={completed}")

    return dive_plugin


def test_integrated_dive():
    """Test integrated dive with realistic physics and control"""
    print("\n=== Testing Integrated Dive ===")

    vs = VehicleState()
    controller = VehicleControl(vs)
    dynamics = VehicleDynamics(vs)
    registry = PluginRegistry.get_instance()
    dive_plugin = registry.create_plugin('DIVE', vs)

    # Set up submerged trim (ballast full, neutral buoyancy)
    print("Setting up submerged trim for dynamic diving...")
    vs.ballast_state['tank_volume'] = vs.ballast_state['tank_capacity']
    vs.ballast_state['tank_status'] = 'FULL'
    vs.buoyancy_state = 'NEUTRAL'
    
    # Initialize dive task with correct parameter
    task_params = {'target_depth': 3.0, 'speed': 1.0, 'timeout': 120.0}
    dive_plugin.initialize(task_params)

    # Set initial state
    vs.set_initial_position(34.257, -117.198, 0.0, 290.0)
    
    print(f"Target: Dive from 0m to 3m with buoyancy={vs.buoyancy_state}")
    print(f"Expected behavior: Build speed first, then use pitch/fins to dive")

    # Run simulation for 120 seconds (dynamic diver needs time to build speed)
    max_steps = 1200
    for step in range(max_steps):
        # Plugin updates target states
        dive_plugin.execute(0.1)

        # Controller computes actuator commands
        controller.update(0.1)

        # Physics updates true state  
        dynamics.update(0.1)

        # Debug output every 5 seconds
        if step % 50 == 0:
            depth = vs.true_state.get('true_depth', 0.0)
            speed = vs.true_state.get('true_speed', 0.0)
            pitch_deg = math.degrees(vs.true_state.get('true_pitch', 0.0))
            vz = vs.true_state.get('true_vertical_velocity', 0.0)
            thrust_cmd = vs.actuator_commands.get('thruster_cmd', 0.0)
            fin_ul = vs.actuator_commands.get('UL', 0.0)
            fin_ur = vs.actuator_commands.get('UR', 0.0)
            fin_ll = vs.actuator_commands.get('LL', 0.0)
            fin_lr = vs.actuator_commands.get('LR', 0.0)
            
            print(f"  t={step*0.1:.1f}s | depth={depth:.2f}m | speed={speed:.2f}m/s | pitch={pitch_deg:.1f}°")
            print(f"    vz={vz:.3f}m/s | thrust={thrust_cmd:.1f}% | buoyancy={vs.buoyancy_state}")
            print(f"    fins: UL={fin_ul:.1f}° UR={fin_ur:.1f}° LL={fin_ll:.1f}° LR={fin_lr:.1f}°")

        # Check completion
        if dive_plugin.check_completion():
            print(f"  SUCCESS: Dive completed at step {step} ({step*0.1:.1f}s)")
            final_depth = vs.true_state.get('true_depth', 0.0)
            final_speed = vs.true_state.get('true_speed', 0.0)
            print(f"  Final state: depth={final_depth:.2f}m, speed={final_speed:.2f}m/s")
            return True

        # Progress check - dynamic diver should build speed first
        if step == 100:  # After 10 seconds
            speed_10s = vs.true_state.get('true_speed', 0.0)
            if speed_10s < 0.1:
                print(f"  WARNING: Low speed after 10s ({speed_10s:.2f}m/s) - thrust/drag may be unbalanced")
        
        if step == 300:  # After 30 seconds
            speed_30s = vs.true_state.get('true_speed', 0.0)
            depth_30s = vs.true_state.get('true_depth', 0.0)
            if speed_30s < 0.5:
                print(f"  WARNING: Still low speed after 30s ({speed_30s:.2f}m/s)")
            if depth_30s < 0.1:
                print(f"  WARNING: No depth progress after 30s ({depth_30s:.3f}m)")

    # Test failed
    print("  FAILED: Did not complete dive in simulation time")
    final_depth = vs.true_state.get('true_depth', 0.0)
    final_speed = vs.true_state.get('true_speed', 0.0)
    print(f"  Final state: depth={final_depth:.2f}m (target: 3.0m), speed={final_speed:.2f}m/s")
    
    return False


def test_buoyancy_forces():
    """Test that buoyancy forces match specifications"""
    print("\n=== Testing Buoyancy Forces ===")
    
    vs = VehicleState()
    dynamics = VehicleDynamics(vs)
    
    # Test POSITIVE buoyancy
    vs.buoyancy_state = 'POSITIVE'
    pos_force = dynamics._get_buoyancy_force()
    print(f"POSITIVE buoyancy force: {pos_force:.6f}N (expected: +4.4660145N)")
    
    # Test NEUTRAL buoyancy  
    vs.buoyancy_state = 'NEUTRAL'
    neu_force = dynamics._get_buoyancy_force()
    print(f"NEUTRAL buoyancy force: {neu_force:.6f}N (expected: -0.0266N)")
    
    # Verify values match specifications
    assert abs(pos_force - 4.4660145) < 0.0001, f"POSITIVE force mismatch: got {pos_force}, expected 4.4660145"
    assert abs(neu_force - (-0.0266)) < 0.0001, f"NEUTRAL force mismatch: got {neu_force}, expected -0.0266"
    
    print("  Buoyancy forces match specifications ✓")


if __name__ == "__main__":
    try:
        print("=== UPDATED CONTROL SYSTEM TESTS ===")
        test_buoyancy_forces()
        test_control_system()
        test_dynamics_isolated() 
        test_dive_plugin()
        success = test_integrated_dive()
        
        if success:
            print("\n=== ALL TESTS PASSED ===")
        else:
            print("\n=== INTEGRATED DIVE TEST FAILED ===")
            print("This may indicate thrust/drag imbalance or control gains need adjustment")
            
    except Exception as e:
        print(f"\nTest failed with error: {e}")
        import traceback
        traceback.print_exc()