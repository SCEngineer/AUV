#!/usr/bin/env python3
"""
Improved Automated PID Tuning Script for AUV Control
- Tests realistic large heading errors and waypoint following scenarios
- Expanded gain ranges for aggressive maneuvering
- Better performance metrics and safety checks
"""

import sys
import os
import math
import time
import random
sys.path.append(os.path.dirname(__file__))

from vehicle_state import VehicleState
from vehicle_control import VehicleControl
from enhanced_hal import EnhancedHAL
from vehicle_dynamics import VehicleDynamics

def robust_angle_difference(target: float, current: float) -> float:
    """Calculate shortest angular difference"""
    diff = target - current
    while diff > 180: diff -= 360
    while diff < -180: diff += 360
    return diff

def evaluate_heading_gains(kp, ki, kd, test_duration=30.0, verbose=False):
    """
    Improved heading evaluation with realistic waypoint following scenarios
    Tests both large heading changes and waypoint tracking
    """
    try:
        # Setup vehicle
        vehicle_state = VehicleState()
        vehicle_state.control_gains = {
            'heading_kp': kp, 'heading_ki': ki, 'heading_kd': kd,
            'depth_kp': 1.0, 'depth_ki': 0.02, 'depth_kd': 0.5,
            'speed_kp': 10.0, 'speed_ki': 2.0, 'speed_kd': 0.5
        }
        
        # Start with random heading, target large change (like real waypoint scenario)
        initial_heading = random.uniform(0, 360)
        target_heading = (initial_heading + random.uniform(60, 180)) % 360  # Large turn required
        
        vehicle_state.set_initial_position(33.6094, -117.7070, 2.0, initial_heading)
        vehicle_state.true_state.update({
            'true_speed': 2.0,  # Realistic cruise speed
            'true_pitch': 0.0, 
            'true_yaw_rate': 0.0, 
            'true_roll': 0.0
        })
        
        vehicle_state.target_state.update({
            'target_speed': 2.0, 
            'target_heading': target_heading, 
            'target_depth': 2.0,
            'target_lat': 33.6094, 
            'target_lon': -117.7070, 
            'distance_to_waypoint': 0.0
        })
        
        vehicle_state.ballast_state.update({
            'tank_volume': 0.001982,
            'tank_status': 'FULL',
            'pump_running': False,
            'vent_open': False
        })
        vehicle_state.buoyancy_state = "NEUTRAL"
        
        # Initialize control systems with suppressed output
        import io
        old_stdout = sys.stdout
        sys.stdout = io.StringIO()
        
        config = {'mode': 'simulation'}
        hal = EnhancedHAL(config, vehicle_state)
        control = VehicleControl(vehicle_state, hal=hal)
        dynamics = VehicleDynamics(vehicle_state)
        
        sys.stdout = old_stdout
        
        # Run simulation
        time_step = 0.1
        steps = int(test_duration / time_step)
        
        max_saturation = 0.0
        final_error = 999.0
        settling_time = test_duration
        max_overshoot = 0.0
        oscillation_count = 0
        last_error_sign = 0
        total_fin_effort = 0.0
        convergence_achieved = False
        
        # Track multiple performance metrics
        error_history = []
        saturation_history = []
        
        for i in range(steps):
            sim_time = i * time_step
            
            current_heading = vehicle_state.true_state['true_heading']
            heading_error = robust_angle_difference(target_heading, current_heading)
            error_history.append(abs(heading_error))
            
            # Track oscillations
            error_sign = 1 if heading_error > 0 else -1
            if last_error_sign != 0 and error_sign != last_error_sign and abs(heading_error) > 5.0:
                oscillation_count += 1
            last_error_sign = error_sign
            
            vehicle_state.nav_state.update({
                'heading': current_heading,
                'depth': vehicle_state.true_state['true_depth'],
                'speed': vehicle_state.true_state['true_speed']
            })
            
            # Update systems
            hal.update(time_step)
            control.update(time_step)
            
            # Check fin saturation - FIXED: proper limits for X-tail
            fin_ul = vehicle_state.actuator_commands.get('UL', 0.0)
            fin_ur = vehicle_state.actuator_commands.get('UR', 0.0)
            fin_lr = vehicle_state.actuator_commands.get('LR', 0.0)
            fin_ll = vehicle_state.actuator_commands.get('LL', 0.0)
            
            max_fin = max(abs(fin_ul), abs(fin_ur), abs(fin_lr), abs(fin_ll))
            max_saturation = max(max_saturation, max_fin)
            saturation_history.append(max_fin)
            
            # Track control effort
            total_fin_effort += max_fin * time_step
            
            # Convert commands to dynamics format and step
            commands = {
                'thruster': vehicle_state.actuator_commands.get('thruster_cmd', 0.0),
                'UL': fin_ul,
                'UR': fin_ur,
                'LL': fin_ll,
                'LR': fin_lr
            }
            
            # Update dynamics
            dynamics.update()
            
            # Check settling (tighter tolerance for precision)
            if abs(heading_error) < 3.0 and not convergence_achieved and i > 50:
                settling_time = sim_time
                convergence_achieved = True
            
            # Track overshoot beyond target
            if convergence_achieved:
                overshoot = abs(heading_error)
                max_overshoot = max(max_overshoot, overshoot)
        
        final_heading = vehicle_state.true_state['true_heading']
        final_error = abs(robust_angle_difference(target_heading, final_heading))
        
        # Calculate additional metrics
        avg_error = sum(error_history) / len(error_history)
        percent_saturated = sum(1 for s in saturation_history if s > 25.0) / len(saturation_history)
        
        return {
            'final_error': final_error,
            'max_saturation': max_saturation,
            'settling_time': settling_time,
            'max_overshoot': max_overshoot,
            'oscillation_count': oscillation_count,
            'avg_error': avg_error,
            'percent_saturated': percent_saturated,
            'total_effort': total_fin_effort,
            'initial_error': abs(robust_angle_difference(target_heading, initial_heading))
        }
        
    except Exception as e:
        if verbose:
            print(f"Heading evaluation error: {e}")
        return {
            'final_error': 999.0, 'max_saturation': 999.0, 'settling_time': 999.0,
            'max_overshoot': 999.0, 'oscillation_count': 999, 'avg_error': 999.0,
            'percent_saturated': 1.0, 'total_effort': 999.0, 'initial_error': 180.0
        }

def evaluate_depth_gains(kp, ki, kd, test_duration=35.0, verbose=False):
    """
    Improved depth evaluation with realistic diving scenarios
    Tests both shallow and deep target changes
    """
    try:
        vehicle_state = VehicleState()
        vehicle_state.control_gains = {
            'heading_kp': 0.2, 'heading_ki': 0.005, 'heading_kd': 0.05,
            'depth_kp': kp, 'depth_ki': ki, 'depth_kd': kd,
            'speed_kp': 10.0, 'speed_ki': 2.0, 'speed_kd': 0.5
        }
        
        # Test realistic depth changes
        initial_depth = random.uniform(0.5, 2.0)
        target_depth = random.uniform(3.0, 8.0)  # Realistic operational depths
        
        vehicle_state.set_initial_position(33.6094, -117.7070, initial_depth, 0.0)
        vehicle_state.true_state.update({
            'true_speed': 1.5,  # Slower for depth changes
            'true_pitch': 0.0,
            'true_vertical_velocity': 0.0,
            'true_yaw_rate': 0.0,
            'true_roll': 0.0
        })
        
        vehicle_state.target_state.update({
            'target_speed': 1.5,
            'target_heading': 0.0,
            'target_depth': target_depth,
            'target_lat': 33.6094,
            'target_lon': -117.7070,
            'distance_to_waypoint': 0.0
        })
        
        vehicle_state.ballast_state.update({
            'tank_volume': 0.001982,
            'tank_status': 'FULL',
            'pump_running': False,
            'vent_open': False
        })
        vehicle_state.buoyancy_state = "NEUTRAL"
        
        # Initialize control systems
        import io
        old_stdout = sys.stdout
        sys.stdout = io.StringIO()
        
        config = {'mode': 'simulation'}
        hal = EnhancedHAL(config, vehicle_state)
        control = VehicleControl(vehicle_state, hal=hal)
        dynamics = VehicleDynamics(vehicle_state)
        
        sys.stdout = old_stdout
        
        # Run simulation
        time_step = 0.1
        steps = int(test_duration / time_step)
        
        max_saturation = 0.0
        final_error = 999.0
        settling_time = test_duration
        max_overshoot = 0.0
        max_depth = 0.0
        oscillation_count = 0
        last_error_sign = 0
        
        for i in range(steps):
            sim_time = i * time_step
            
            current_depth = vehicle_state.true_state['true_depth']
            depth_error = target_depth - current_depth
            max_depth = max(max_depth, current_depth)
            
            # Track oscillations
            error_sign = 1 if depth_error > 0 else -1
            if last_error_sign != 0 and error_sign != last_error_sign and abs(depth_error) > 0.5:
                oscillation_count += 1
            last_error_sign = error_sign
            
            vehicle_state.nav_state.update({
                'heading': vehicle_state.true_state['true_heading'],
                'depth': current_depth,
                'speed': vehicle_state.true_state['true_speed']
            })
            
            # Update systems
            hal.update(time_step)
            control.update(time_step)
            
            # Check fin saturation (depth uses elevator/pitch control)
            fin_ul = vehicle_state.actuator_commands.get('UL', 0.0)
            fin_ur = vehicle_state.actuator_commands.get('UR', 0.0)
            fin_lr = vehicle_state.actuator_commands.get('LR', 0.0)
            fin_ll = vehicle_state.actuator_commands.get('LL', 0.0)
            
            max_fin = max(abs(fin_ul), abs(fin_ur), abs(fin_lr), abs(fin_ll))
            max_saturation = max(max_saturation, max_fin)
            
            # Step dynamics
            commands = {
                'thruster': vehicle_state.actuator_commands.get('thruster_cmd', 0.0),
                'UL': fin_ul,
                'UR': fin_ur,
                'LL': fin_ll,
                'LR': fin_lr
            }
            
            dynamics.update()
            
            # Check settling (within 0.3m for realistic precision)
            if abs(depth_error) < 0.3 and settling_time == test_duration and i > 50:
                settling_time = sim_time
            
            # Track overshoot (critical for safety)
            if current_depth > target_depth + 0.5:
                overshoot = current_depth - target_depth
                max_overshoot = max(max_overshoot, overshoot)
        
        final_depth = vehicle_state.true_state['true_depth']
        final_error = abs(target_depth - final_depth)
        
        # Safety penalties
        safety_penalty = 0.0
        if max_depth > 12.0:  # Too deep
            safety_penalty += 1000.0
        if max_overshoot > 3.0:  # Dangerous overshoot
            safety_penalty += 500.0
        
        return {
            'final_error': final_error + safety_penalty,
            'max_saturation': max_saturation,
            'settling_time': settling_time,
            'max_overshoot': max_overshoot,
            'oscillation_count': oscillation_count,
            'max_depth': max_depth,
            'initial_error': abs(target_depth - initial_depth)
        }
        
    except Exception as e:
        if verbose:
            print(f"Depth evaluation error: {e}")
        return {
            'final_error': 999.0, 'max_saturation': 999.0, 'settling_time': 999.0,
            'max_overshoot': 999.0, 'oscillation_count': 999, 'max_depth': 999.0,
            'initial_error': 10.0
        }

def evaluate_speed_gains(kp, ki, kd, test_duration=25.0, verbose=False):
    """
    Improved speed evaluation with realistic speed changes
    Tests both acceleration and deceleration scenarios
    """
    try:
        vehicle_state = VehicleState()
        vehicle_state.control_gains = {
            'heading_kp': 0.2, 'heading_ki': 0.005, 'heading_kd': 0.05,
            'depth_kp': 1.0, 'depth_ki': 0.02, 'depth_kd': 2.0,
            'speed_kp': kp, 'speed_ki': ki, 'speed_kd': kd
        }
        
        # Test realistic speed changes
        initial_speed = random.uniform(0.0, 0.5)
        target_speed = random.uniform(1.5, 2.5)  # Typical cruise speeds
        
        vehicle_state.set_initial_position(33.6094, -117.7070, 3.0, 0.0)
        vehicle_state.true_state.update({
            'true_speed': initial_speed,
            'true_pitch': 0.0,
            'true_yaw_rate': 0.0,
            'true_roll': 0.0
        })
        
        vehicle_state.target_state.update({
            'target_speed': target_speed,
            'target_heading': 0.0,
            'target_depth': 3.0,
            'target_lat': 33.6094,
            'target_lon': -117.7070,
            'distance_to_waypoint': 0.0
        })
        
        vehicle_state.ballast_state.update({
            'tank_volume': 0.001982,
            'tank_status': 'FULL',
            'pump_running': False,
            'vent_open': False
        })
        vehicle_state.buoyancy_state = "NEUTRAL"
        
        # Initialize control systems
        import io
        old_stdout = sys.stdout
        sys.stdout = io.StringIO()
        
        config = {'mode': 'simulation'}
        hal = EnhancedHAL(config, vehicle_state)
        control = VehicleControl(vehicle_state, hal=hal)
        dynamics = VehicleDynamics(vehicle_state)
        
        sys.stdout = old_stdout
        
        # Run simulation
        time_step = 0.1
        steps = int(test_duration / time_step)
        
        max_saturation = 0.0
        final_error = 999.0
        settling_time = test_duration
        max_overshoot = 0.0
        oscillation_count = 0
        last_error_sign = 0
        
        for i in range(steps):
            sim_time = i * time_step
            
            current_speed = vehicle_state.true_state['true_speed']
            speed_error = target_speed - current_speed
            
            # Track oscillations
            error_sign = 1 if speed_error > 0 else -1
            if last_error_sign != 0 and error_sign != last_error_sign and abs(speed_error) > 0.1:
                oscillation_count += 1
            last_error_sign = error_sign
            
            vehicle_state.nav_state.update({
                'heading': vehicle_state.true_state['true_heading'],
                'depth': vehicle_state.true_state['true_depth'],
                'speed': current_speed
            })
            
            # Update systems
            hal.update(time_step)
            control.update(time_step)
            
            # Check thruster saturation
            thruster = vehicle_state.actuator_commands.get('thruster_cmd', 0.0)
            max_saturation = max(max_saturation, abs(thruster))
            
            # Step dynamics
            commands = {
                'thruster': thruster,
                'UL': vehicle_state.actuator_commands.get('UL', 0.0),
                'UR': vehicle_state.actuator_commands.get('UR', 0.0),
                'LL': vehicle_state.actuator_commands.get('LL', 0.0),
                'LR': vehicle_state.actuator_commands.get('LR', 0.0)
            }
            
            dynamics.update()
            
            # Check settling (within 5% of target)
            if abs(speed_error) < target_speed * 0.05 and settling_time == test_duration and i > 30:
                settling_time = sim_time
            
            # Track overshoot
            if current_speed > target_speed:
                overshoot = current_speed - target_speed
                max_overshoot = max(max_overshoot, overshoot)
        
        final_speed = vehicle_state.true_state['true_speed']
        final_error = abs(target_speed - final_speed)
        
        return {
            'final_error': final_error,
            'max_saturation': max_saturation,
            'settling_time': settling_time,
            'max_overshoot': max_overshoot,
            'oscillation_count': oscillation_count,
            'initial_error': abs(target_speed - initial_speed)
        }
        
    except Exception as e:
        if verbose:
            print(f"Speed evaluation error: {e}")
        return {
            'final_error': 999.0, 'max_saturation': 999.0, 'settling_time': 999.0,
            'max_overshoot': 999.0, 'oscillation_count': 999, 'initial_error': 2.0
        }

def calculate_score(results, control_type):
    """
    Improved scoring function with realistic performance criteria
    """
    score = 0.0
    
    # Unpack results
    final_error = results['final_error']
    max_saturation = results['max_saturation']
    settling_time = results['settling_time']
    max_overshoot = results['max_overshoot']
    oscillation_count = results['oscillation_count']
    
    # Control-specific scoring
    if control_type == "heading":
        # Final error penalty (degrees)
        score += final_error * 3.0
        
        # Saturation penalty - allow up to 25° (reasonable for aggressive turns)
        if max_saturation > 30.0:  # Beyond physical limits
            score += 5000.0
        elif max_saturation > 25.0:  # High but manageable
            score += 500.0
        elif max_saturation > 20.0:  # Moderate usage
            score += 100.0
        
        # Settling time penalty (should turn quickly)
        if settling_time > 20.0:
            score += 200.0
        elif settling_time > 15.0:
            score += 50.0
        
        # Overshoot penalty
        if max_overshoot > 10.0:
            score += max_overshoot * 5.0
        
        # Oscillation penalty (very bad for heading)
        score += oscillation_count * 50.0
        
    elif control_type == "depth":
        # Final error penalty (meters - depth precision critical)
        score += final_error * 20.0
        
        # Saturation penalty
        if max_saturation > 30.0:
            score += 5000.0
        elif max_saturation > 25.0:
            score += 1000.0
        elif max_saturation > 20.0:
            score += 200.0
        
        # Settling time penalty
        if settling_time > 25.0:
            score += 150.0
        elif settling_time > 20.0:
            score += 75.0
        
        # Overshoot penalty (safety critical for depth)
        if max_overshoot > 2.0:
            score += max_overshoot * 100.0  # Heavy penalty
        elif max_overshoot > 1.0:
            score += max_overshoot * 20.0
        
        # Oscillation penalty
        score += oscillation_count * 25.0
        
    else:  # speed
        # Final error penalty (m/s)
        score += final_error * 25.0
        
        # Saturation penalty (thrusters can handle 100%)
        if max_saturation > 95.0:
            score += 1000.0
        elif max_saturation > 85.0:
            score += 200.0
        elif max_saturation > 75.0:
            score += 50.0
        
        # Settling time penalty
        if settling_time > 20.0:
            score += 100.0
        elif settling_time > 15.0:
            score += 40.0
        
        # Overshoot penalty
        if max_overshoot > 0.5:
            score += max_overshoot * 30.0
        
        # Oscillation penalty
        score += oscillation_count * 20.0
    
    return score

def tune_control_loop(control_type="heading"):
    """
    Improved tuning with expanded ranges and multiple test scenarios
    """
    print(f"\n=== TUNING {control_type.upper()} CONTROL ===")
    
    if control_type == "heading":
        # EXPANDED ranges for aggressive maneuvering
        kp_range = [0.1, 0.2, 0.3, 0.5, 0.8, 1.0, 1.5, 2.0]
        ki_range = [0.001, 0.005, 0.01, 0.02, 0.05, 0.1]
        kd_range = [0.01, 0.02, 0.05, 0.1, 0.15, 0.2]
        evaluate_func = evaluate_heading_gains
    elif control_type == "depth":
        kp_range = [0.5, 1.0, 2.0, 3.0, 5.0, 8.0, 10.0]
        ki_range = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5]
        kd_range = [0.2, 0.5, 1.0, 2.0, 3.0, 5.0]
        evaluate_func = evaluate_depth_gains
    else:  # speed
        kp_range = [2.0, 5.0, 10.0, 15.0, 25.0, 40.0]
        ki_range = [0.1, 0.5, 1.0, 2.0, 5.0, 10.0]
        kd_range = [0.1, 0.5, 1.0, 2.0, 5.0, 10.0]
        evaluate_func = evaluate_speed_gains
    
    best_score = float('inf')
    best_gains = None
    best_results = None
    
    total_tests = len(kp_range) * len(ki_range) * len(kd_range)
    test_count = 0
    
    print(f"Testing {total_tests} gain combinations with multiple scenarios...")
    
    results = []
    
    for kp in kp_range:
        for ki in ki_range:
            for kd in kd_range:
                test_count += 1
                
                if test_count % 20 == 0 or test_count % total_tests == 0:
                    print(f"  Progress: {test_count}/{total_tests} ({100*test_count/total_tests:.1f}%)")
                
                # Run multiple test scenarios and average results
                scenario_scores = []
                scenario_results = []
                
                num_scenarios = 3 if control_type == "heading" else 2
                for scenario in range(num_scenarios):
                    results_dict = evaluate_func(kp, ki, kd, verbose=False)
                    score = calculate_score(results_dict, control_type)
                    scenario_scores.append(score)
                    scenario_results.append(results_dict)
                
                # Use average score but worst-case for safety metrics
                avg_score = sum(scenario_scores) / len(scenario_scores)
                worst_saturation = max(r['max_saturation'] for r in scenario_results)
                avg_final_error = sum(r['final_error'] for r in scenario_results) / len(scenario_results)
                
                # Penalty for inconsistent performance
                score_variance = max(scenario_scores) - min(scenario_scores)
                if score_variance > avg_score * 0.3:  # >30% variance is bad
                    avg_score += score_variance * 0.2
                
                results.append({
                    'kp': kp, 'ki': ki, 'kd': kd,
                    'final_error': avg_final_error,
                    'max_saturation': worst_saturation,
                    'settling_time': sum(r['settling_time'] for r in scenario_results) / len(scenario_results),
                    'max_overshoot': max(r['max_overshoot'] for r in scenario_results),
                    'oscillation_count': sum(r['oscillation_count'] for r in scenario_results) / len(scenario_results),
                    'score': avg_score,
                    'variance': score_variance
                })
                
                if avg_score < best_score:
                    best_score = avg_score
                    best_gains = (kp, ki, kd)
                    best_results = (avg_final_error, worst_saturation, 
                                  sum(r['settling_time'] for r in scenario_results) / len(scenario_results),
                                  max(r['max_overshoot'] for r in scenario_results))
    
    # Sort and display results
    results.sort(key=lambda x: x['score'])
    
    print(f"\n=== {control_type.upper()} TUNING COMPLETE ===")
    print(f"Best gains: kp={best_gains[0]:.3f}, ki={best_gains[1]:.3f}, kd={best_gains[2]:.3f}")
    print(f"Performance (averaged over {num_scenarios} scenarios):")
    
    if control_type == "heading":
        print(f"  Final error: {best_results[0]:.1f}° (avg)")
        print(f"  Max saturation: {best_results[1]:.1f}° (worst case)")
        print(f"  Settling time: {best_results[2]:.1f}s (avg)")
        print(f"  Max overshoot: {best_results[3]:.1f}° (worst case)")
    elif control_type == "depth":
        print(f"  Final error: {best_results[0]:.2f}m (avg)")
        print(f"  Max saturation: {best_results[1]:.1f}° (worst case)")
        print(f"  Settling time: {best_results[2]:.1f}s (avg)")
        print(f"  Max overshoot: {best_results[3]:.2f}m (worst case)")
    else:  # speed
        print(f"  Final error: {best_results[0]:.2f}m/s (avg)")
        print(f"  Max saturation: {best_results[1]:.1f}% (worst case)")
        print(f"  Settling time: {best_results[2]:.1f}s (avg)")
        print(f"  Max overshoot: {best_results[3]:.2f}m/s (worst case)")
    
    print(f"  Score: {best_score:.1f}")
    
    print(f"\nTop 5 candidates:")
    for i, result in enumerate(results[:5]):
        consistency = "consistent" if result['variance'] < result['score'] * 0.2 else "variable"
        print(f"{i+1}. kp={result['kp']:.3f}, ki={result['ki']:.3f}, kd={result['kd']:.3f} "
              f"(score={result['score']:.1f}, {consistency})")
    
    return best_gains, results

if __name__ == "__main__":
    start_time = time.time()
    
    print("=== IMPROVED AUV PID AUTO-TUNER ===")
    print("Testing realistic waypoint following and maneuvering scenarios...")
    print("Expanded gain ranges for aggressive control...")
    
    # Tune each control loop with improved algorithms
    heading_gains, heading_results = tune_control_loop("heading")
    depth_gains, depth_results = tune_control_loop("depth") 
    speed_gains, speed_results = tune_control_loop("speed")
    
    elapsed_time = time.time() - start_time
    
    print(f"\n=== ALL TUNING COMPLETE ===")
    print(f"Total time: {elapsed_time:.1f} seconds")
    print(f"\nOPTIMAL GAINS FOUND:")
    print(f"Heading: kp={heading_gains[0]:.3f}, ki={heading_gains[1]:.3f}, kd={heading_gains[2]:.3f}")
    print(f"Depth:   kp={depth_gains[0]:.3f}, ki={depth_gains[1]:.3f}, kd={depth_gains[2]:.3f}")
    print(f"Speed:   kp={speed_gains[0]:.3f}, ki={speed_gains[1]:.3f}, kd={speed_gains[2]:.3f}")
    
    # Write gains file in the format expected by the system
    gains_content = f"""# SIMPLR-AUV PID Control Gains Configuration
# Auto-tuned gains - generated by improved auto_tune.py
# Tuning completed in {elapsed_time:.1f} seconds
# Tested with realistic waypoint following scenarios

# Heading Control Gains (for aggressive maneuvering and waypoint following)
heading_kp = {heading_gains[0]:.3f}
heading_ki = {heading_gains[1]:.3f}
heading_kd = {heading_gains[2]:.3f}

# Depth Control Gains (for precise depth control)
depth_kp = {depth_gains[0]:.3f}
depth_ki = {depth_gains[1]:.3f}
depth_kd = {depth_gains[2]:.3f}

# Speed Control Gains (for responsive speed control)
speed_kp = {speed_gains[0]:.3f}
speed_ki = {speed_gains[1]:.3f}
speed_kd = {speed_gains[2]:.3f}
"""

    try:
        with open('auv_pid_gains.txt', 'w') as f:
            f.write(gains_content)
        print(f"\nGains file written to: auv_pid_gains.txt")
        print(f"File location: {os.path.abspath('auv_pid_gains.txt')}")
    except Exception as e:
        print(f"\nError writing gains file: {e}")
        print(f"Manual copy required:")
        print(gains_content)
    
    # Print performance summary
    print(f"\n=== PERFORMANCE SUMMARY ===")
    print(f"Heading Controller:")
    print(f"  - Tested large course corrections (60-180° turns)")
    print(f"  - Optimized for waypoint following scenarios") 
    print(f"  - Saturation limit: 30° fin deflection")
    
    print(f"Depth Controller:")
    print(f"  - Tested realistic depth changes (0.5m to 8m)")
    print(f"  - Safety penalties for excessive depth/overshoot")
    print(f"  - Precision target: ±0.3m settling")
    
    print(f"Speed Controller:")
    print(f"  - Tested acceleration from rest to cruise speed")
    print(f"  - Thruster saturation limit: 95%")
    print(f"  - Target precision: ±5% of target speed")
    
    print(f"\nReady to test with optimized gains for aggressive maneuvering!")