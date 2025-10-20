#!/usr/bin/env python3
"""
trim_validation.py - Zero Bubble Trim Validation Script

Validates vehicle trim by simulating equilibrium conditions in both surfaced 
and submerged states. Checks if the vehicle sits level or has proper pitch attitude.

This script can be used as a standalone validation tool or later integrated 
as a guidance task plugin that executes before swim missions.

NEW: Added plotting capabilities to visualize trim validation results
"""

import math
import json
import glob
import os
import time
import argparse  # NEW: For command line arguments
from datetime import datetime

# NEW: Import plotting libraries (with error handling for missing packages)
try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    PLOTTING_AVAILABLE = True
    print("Plotting capabilities enabled (matplotlib found)")
except ImportError:
    PLOTTING_AVAILABLE = False
    print("WARNING: matplotlib not found - plotting disabled")
    print("To enable plotting, install matplotlib: pip install matplotlib")

# Mock vehicle state class for standalone operation
class MockVehicleState:
    def __init__(self):
        self.true_state = {
            'true_speed': 0.0,
            'true_depth': 0.0,
            'true_heading': 0.0,
            'true_pitch': 0.0,
            'true_vertical_velocity': 0.0,
            'true_pitch_rate': 0.0,
            'true_yaw_rate': 0.0,
            'true_lat': 0.0,
            'true_lon': 0.0
        }
        self.actuator_commands = {
            'thruster_cmd': 0.0,
            'UL': 0.0,
            'UR': 0.0, 
            'LR': 0.0,
            'LL': 0.0
        }
        self.buoyancy_state = "POSITIVE"  # Start surfaced
        self._state_lock = MockLock()

class MockLock:
    def __enter__(self):
        return self
    def __exit__(self, *args):
        pass

class TrimValidator:
    def __init__(self, config_file=None, enable_plotting=False):
        """Initialize trim validator with vehicle configuration"""
        
        print(f"DEBUG: Starting TrimValidator initialization...")
        print(f"DEBUG: config_file={config_file}, enable_plotting={enable_plotting}")
        
        # Store plotting preference
        self.enable_plotting = enable_plotting and PLOTTING_AVAILABLE
        
        # Initialize to None first
        self.vehicle_dynamics = None
        self.vehicle_state = None
        self.VehicleDynamics = None
        
        # Try to import and initialize the actual vehicle dynamics
        try:
            print("DEBUG: Importing vehicle_dynamics...")
            from vehicle_dynamics import VehicleDynamics
            self.VehicleDynamics = VehicleDynamics
            print("✓ Successfully imported vehicle_dynamics.py")
            
            # Create mock vehicle state
            print("DEBUG: Creating vehicle state...")
            self.vehicle_state = MockVehicleState()
            print("✓ Vehicle state created")
            
            # Initialize vehicle dynamics with configuration
            print("DEBUG: Initializing vehicle dynamics...")
            self.vehicle_dynamics = VehicleDynamics(self.vehicle_state, dt=0.1, config_file=config_file)
            print("✓ Vehicle dynamics initialized successfully")
            
        except ImportError as e:
            print("✗ ERROR: vehicle_dynamics.py not found in current directory")
            print(f"   Import error: {e}")
            print("   Make sure vehicle_dynamics.py is in the same folder as this script")
            print("   Cannot run validation without vehicle dynamics")
            return
        except Exception as e:
            print(f"✗ ERROR: Failed to initialize vehicle dynamics: {e}")
            print(f"   Error type: {type(e).__name__}")
            print("   Check that vehicle_dynamics.py is valid and has required dependencies")
            import traceback
            traceback.print_exc()
            return
        
        # If we get here, initialization was successful
        
        # Validation parameters
        self.SETTLING_TIME = 120.0  # seconds to let vehicle settle
        self.DT = 0.1  # simulation timestep
        self.SURFACED_PITCH_TOLERANCE = 2.0  # ±2° for surfaced
        self.SUBMERGED_PITCH_MIN = -5.0  # -5° to 0° for submerged
        self.SUBMERGED_PITCH_MAX = 0.0
        
        # Enhanced data collection for plotting
        self.pitch_history = []
        self.time_history = []
        self.depth_history = []  # Track depth over time
        self.vertical_velocity_history = []  # Track vertical velocity
        
        # Store results for plotting
        self.surfaced_data = None
        self.submerged_data = None
        
        print("=== TRIM VALIDATION INITIALIZED ===")
        print(f"Settling time: {self.SETTLING_TIME:.0f} seconds")
        print(f"Surfaced pitch tolerance: ±{self.SURFACED_PITCH_TOLERANCE:.1f}°")
        print(f"Submerged pitch range: {self.SUBMERGED_PITCH_MIN:.1f}° to {self.SUBMERGED_PITCH_MAX:.1f}°")
        
        # Inform about plotting status
        if self.enable_plotting:
            print("Plotting: ENABLED - graphs will be generated")
        else:
            print("Plotting: DISABLED")
    
    def reset_vehicle_state(self, initial_depth=0.0, add_disturbance=True):
        """Reset vehicle to initial conditions with optional disturbance for realistic testing"""
        
        # Base initial conditions
        self.vehicle_state.true_state.update({
            'true_speed': 0.0,
            'true_depth': initial_depth,
            'true_heading': 0.0,
            'true_pitch': 0.0,
            'true_vertical_velocity': 0.0,
            'true_pitch_rate': 0.0,
            'true_yaw_rate': 0.0,
            'true_lat': 0.0,
            'true_lon': 0.0
        })
        
        # Add realistic initial disturbances to see settling behavior
        if add_disturbance:
            # Start with some initial pitch disturbance (like vehicle being bumped or deployed at angle)
            if initial_depth == 0.0:  # Surfaced test
                # Surfaced: might start slightly bow-up or bow-down
                initial_pitch_disturbance = math.radians(3.0)  # 3 degree initial disturbance
                initial_pitch_rate = math.radians(-2.0)        # Small initial pitch rate
            else:  # Submerged test
                # Submerged: might start with different disturbance
                initial_pitch_disturbance = math.radians(-2.0)  # Start slightly bow-down
                initial_pitch_rate = math.radians(1.5)          # Small initial pitch rate
            
            self.vehicle_state.true_state['true_pitch'] = initial_pitch_disturbance
            self.vehicle_state.true_state['true_pitch_rate'] = initial_pitch_rate
            
            # Small initial vertical velocity disturbance
            initial_vz = 0.05 if initial_depth == 0.0 else -0.03
            self.vehicle_state.true_state['true_vertical_velocity'] = initial_vz
            
            print(f"   Added initial disturbances:")
            print(f"     Initial pitch: {math.degrees(initial_pitch_disturbance):+.1f}°")
            print(f"     Initial pitch rate: {math.degrees(initial_pitch_rate):+.1f}°/s")
            print(f"     Initial vertical velocity: {initial_vz:+.3f} m/s")
        
        # Zero all control inputs
        self.vehicle_state.actuator_commands.update({
            'thruster_cmd': 0.0,
            'UL': 0.0,
            'UR': 0.0,
            'LR': 0.0,
            'LL': 0.0
        })
        
        # Clear all history arrays
        self.pitch_history = []
        self.time_history = []
        self.depth_history = []
        self.vertical_velocity_history = []
    
    def run_settling_simulation(self, buoyancy_state, initial_depth=0.0):
        """
        Run settling simulation for specified buoyancy state
        
        Args:
            buoyancy_state: "POSITIVE" for surfaced, "NEUTRAL" for submerged
            initial_depth: Starting depth in meters
            
        Returns:
            dict with equilibrium pitch and stability metrics
        """
        
        print(f"\n--- Running {buoyancy_state.lower()} settling simulation ---")
        
        # Reset vehicle with initial disturbances for realistic testing
        self.reset_vehicle_state(initial_depth, add_disturbance=True)
        self.vehicle_state.buoyancy_state = buoyancy_state
        
        # Reset vehicle dynamics internal state
        self.vehicle_dynamics.prev_vertical_velocity = 0.0
        self.vehicle_dynamics.prev_pitch_rate = 0.0
        
        # Run simulation
        sim_time = 0.0
        steps = int(self.SETTLING_TIME / self.DT)
        
        print(f"Running {steps} steps ({self.SETTLING_TIME:.0f}s simulation time)...")
        
        for step in range(steps):
            # Update physics using the real vehicle dynamics
            self.vehicle_dynamics.update(self.DT)
            
            # Record all data for plotting
            current_pitch_deg = math.degrees(self.vehicle_state.true_state['true_pitch'])
            current_depth = self.vehicle_state.true_state['true_depth']
            current_vz = self.vehicle_state.true_state['true_vertical_velocity']
            
            self.pitch_history.append(current_pitch_deg)
            self.time_history.append(sim_time)
            self.depth_history.append(current_depth)
            self.vertical_velocity_history.append(current_vz)
            
            sim_time += self.DT
            
            # Progress indicator - show more frequent updates to see dynamics
            if step % 300 == 0:  # Every 30 seconds
                print(f"  t={sim_time:.0f}s: pitch={current_pitch_deg:+.2f}°, "
                      f"depth={current_depth:.2f}m, "
                      f"vz={current_vz:+.3f}m/s, "
                      f"pitch_rate={math.degrees(self.vehicle_state.true_state['true_pitch_rate']):+.2f}°/s")
        
        # Analyze results
        result = self.analyze_equilibrium()
        
        # Store complete simulation data for plotting
        result['simulation_data'] = {
            'time': self.time_history.copy(),
            'pitch': self.pitch_history.copy(),
            'depth': self.depth_history.copy(),
            'vertical_velocity': self.vertical_velocity_history.copy(),
            'buoyancy_state': buoyancy_state,
            'settling_time': self.SETTLING_TIME
        }
        
        return result
    
    def analyze_equilibrium(self):
        """Analyze the settling simulation results"""
        
        if len(self.pitch_history) < 100:
            return {'error': 'Insufficient data for analysis'}
        
        # Take the last 25% of the simulation as "equilibrium"
        eq_start_idx = int(len(self.pitch_history) * 0.75)
        eq_pitch_data = self.pitch_history[eq_start_idx:]
        eq_time_data = self.time_history[eq_start_idx:]
        
        # Calculate equilibrium statistics
        eq_pitch_mean = sum(eq_pitch_data) / len(eq_pitch_data)
        eq_pitch_std = math.sqrt(sum((p - eq_pitch_mean)**2 for p in eq_pitch_data) / len(eq_pitch_data))
        eq_pitch_max = max(eq_pitch_data)
        eq_pitch_min = min(eq_pitch_data)
        eq_pitch_range = eq_pitch_max - eq_pitch_min
        
        # Check for stability (small oscillations)
        is_stable = eq_pitch_std < 0.5 and eq_pitch_range < 2.0
        
        # Final vehicle state
        final_depth = self.vehicle_state.true_state['true_depth']
        final_vz = self.vehicle_state.true_state['true_vertical_velocity']
        
        return {
            'equilibrium_pitch_deg': eq_pitch_mean,
            'pitch_std_dev': eq_pitch_std,
            'pitch_range': eq_pitch_range,
            'is_stable': is_stable,
            'final_depth': final_depth,
            'final_vertical_velocity': final_vz,
            'equilibrium_time_range': (eq_time_data[0], eq_time_data[-1]),
            'equilibrium_start_idx': eq_start_idx,  # For plotting
            'raw_pitch_data': eq_pitch_data[-10:]  # Last 10 samples for inspection
        }
    
    def validate_surfaced_trim(self):
        """Validate trim in surfaced configuration"""
        
        result = self.run_settling_simulation("POSITIVE", initial_depth=0.0)
        
        if 'error' in result:
            return result
        
        eq_pitch = result['equilibrium_pitch_deg']
        is_stable = result['is_stable']
        
        # Check criteria
        pitch_ok = abs(eq_pitch) <= self.SURFACED_PITCH_TOLERANCE
        depth_ok = result['final_depth'] <= 0.1  # Should stay at surface
        
        # Overall assessment
        passed = pitch_ok and depth_ok and is_stable
        
        validation_result = {
            'test': 'surfaced_trim',
            'equilibrium_pitch_deg': eq_pitch,
            'pitch_tolerance': self.SURFACED_PITCH_TOLERANCE,
            'pitch_ok': pitch_ok,
            'depth_ok': depth_ok,
            'is_stable': is_stable,
            'passed': passed,
            'final_depth': result['final_depth'],
            'final_vz': result['final_vertical_velocity'],
            'message': self.get_surfaced_message(eq_pitch, pitch_ok, depth_ok, is_stable),
            'simulation_data': result['simulation_data'],  # Include simulation data
            'equilibrium_start_idx': result['equilibrium_start_idx']  # For plotting
        }
        
        # Store for plotting
        self.surfaced_data = validation_result
        
        return validation_result
    
    def validate_submerged_trim(self):
        """Validate trim in submerged configuration"""
        
        result = self.run_settling_simulation("NEUTRAL", initial_depth=2.0)
        
        if 'error' in result:
            return result
        
        eq_pitch = result['equilibrium_pitch_deg']
        is_stable = result['is_stable']
        
        # Check criteria  
        pitch_ok = self.SUBMERGED_PITCH_MIN <= eq_pitch <= self.SUBMERGED_PITCH_MAX
        depth_stable = abs(result['final_vertical_velocity']) < 0.05  # Nearly neutral
        
        # Overall assessment
        passed = pitch_ok and depth_stable and is_stable
        
        validation_result = {
            'test': 'submerged_trim',
            'equilibrium_pitch_deg': eq_pitch,
            'pitch_range': (self.SUBMERGED_PITCH_MIN, self.SUBMERGED_PITCH_MAX),
            'pitch_ok': pitch_ok,
            'depth_stable': depth_stable,
            'is_stable': is_stable,
            'passed': passed,
            'final_depth': result['final_depth'],
            'final_vz': result['final_vertical_velocity'],
            'message': self.get_submerged_message(eq_pitch, pitch_ok, depth_stable, is_stable),
            'simulation_data': result['simulation_data'],  # Include simulation data
            'equilibrium_start_idx': result['equilibrium_start_idx']  # For plotting
        }
        
        # Store for plotting
        self.submerged_data = validation_result
        
        return validation_result
    
    def get_surfaced_message(self, pitch, pitch_ok, depth_ok, stable):
        """Generate message for surfaced trim results"""
        if pitch_ok and depth_ok and stable:
            return f"EXCELLENT: Vehicle sits level ({pitch:+.1f}°) at surface"
        elif not pitch_ok:
            if pitch > 0:
                return f"TRIM ISSUE: Bow up {pitch:+.1f}° - CG too far aft or add bow weight"
            else:
                return f"TRIM ISSUE: Bow down {pitch:+.1f}° - CG too far forward or add stern weight"
        elif not depth_ok:
            return "BUOYANCY ISSUE: Vehicle not staying at surface"
        elif not stable:
            return "STABILITY ISSUE: Vehicle oscillating, check damping parameters"
        else:
            return "UNKNOWN ISSUE: Multiple problems detected"
    
    def get_submerged_message(self, pitch, pitch_ok, depth_stable, stable):
        """Generate message for submerged trim results"""
        if pitch_ok and depth_stable and stable:
            return f"GOOD: Submerged attitude {pitch:+.1f}° (slightly bow-down is normal)"
        elif not pitch_ok:
            if pitch > self.SUBMERGED_PITCH_MAX:
                return f"TRIM ISSUE: Too bow-up {pitch:+.1f}° when submerged"
            else:
                return f"TRIM ISSUE: Too bow-down {pitch:+.1f}° when submerged"
        elif not depth_stable:
            return "BUOYANCY ISSUE: Vehicle not maintaining depth (not neutrally buoyant)"
        elif not stable:
            return "STABILITY ISSUE: Vehicle oscillating when submerged"
        else:
            return "UNKNOWN ISSUE: Multiple problems detected"
    
    def create_validation_plots(self):
        """
        Create comprehensive plots of trim validation results
        
        This function creates a multi-panel plot showing:
        - Pitch vs time for both surfaced and submerged tests
        - Depth vs time to show depth stability
        - Acceptable ranges highlighted
        """
        
        if not self.enable_plotting:
            print("Plotting disabled - skipping plot generation")
            return None
        
        if not self.surfaced_data or not self.submerged_data:
            print("Insufficient data for plotting - run validation first")
            return None
        
        print("Generating validation plots...")
        
        # Create figure with subplots
        # figsize=(width, height) in inches
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Trim Validation Results', fontsize=16, fontweight='bold')
        
        # Plot 1: Surfaced Pitch vs Time (top left)
        ax1 = axes[0, 0]
        self.plot_pitch_data(ax1, self.surfaced_data, "Surfaced Trim Test")
        
        # Plot 2: Submerged Pitch vs Time (top right)
        ax2 = axes[0, 1]
        self.plot_pitch_data(ax2, self.submerged_data, "Submerged Trim Test")
        
        # Plot 3: Surfaced Depth vs Time (bottom left)
        ax3 = axes[1, 0]
        self.plot_depth_data(ax3, self.surfaced_data, "Surfaced Depth Stability")
        
        # Plot 4: Submerged Depth vs Time (bottom right)
        ax4 = axes[1, 1]
        self.plot_depth_data(ax4, self.submerged_data, "Submerged Depth Stability")
        
        # Adjust layout to prevent overlap
        plt.tight_layout()
        
        # Save plot
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        plot_filename = f'C:\\Users\\E22000316\\Desktop\\simplr_auv\\trim_validation_plots_{timestamp}.png'
        
        try:
            os.makedirs(os.path.dirname(plot_filename), exist_ok=True)
            plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
            print(f"Plots saved: {plot_filename}")
            
            # Show plot if running interactively
            plt.show()
            
        except Exception as e:
            print(f"Warning: Could not save plots: {e}")
        
        return plot_filename
    
    def plot_pitch_data(self, ax, test_data, title):
        """
        Plot pitch vs time for a single test
        
        Args:
            ax: matplotlib axis object to plot on
            test_data: test result dictionary with simulation_data
            title: title for this subplot
        """
        
        sim_data = test_data['simulation_data']
        time = sim_data['time']
        pitch = sim_data['pitch']
        eq_start_idx = test_data['equilibrium_start_idx']
        
        # Plot the pitch data
        ax.plot(time, pitch, 'b-', linewidth=1, label='Pitch Angle')
        
        # Highlight equilibrium analysis region
        eq_start_time = time[eq_start_idx]
        ax.axvspan(eq_start_time, time[-1], alpha=0.2, color='green', 
                  label='Equilibrium Analysis Window')
        
        # Add tolerance/acceptance bands
        if test_data['test'] == 'surfaced_trim':
            # Surfaced: ±tolerance around 0°
            tolerance = test_data['pitch_tolerance']
            ax.axhspan(-tolerance, tolerance, alpha=0.2, color='lightgreen', 
                      label=f'Acceptable Range (±{tolerance:.1f}°)')
            
            # Add zero line
            ax.axhline(y=0, color='black', linestyle='--', alpha=0.5, label='Level (0°)')
            
        else:  # submerged_trim
            # Submerged: acceptable range
            pitch_min, pitch_max = test_data['pitch_range']
            ax.axhspan(pitch_min, pitch_max, alpha=0.2, color='lightgreen', 
                      label=f'Acceptable Range ({pitch_min:.1f}° to {pitch_max:.1f}°)')
        
        # Mark final equilibrium value
        final_pitch = test_data['equilibrium_pitch_deg']
        ax.axhline(y=final_pitch, color='red', linestyle='-', alpha=0.7, 
                  label=f'Equilibrium: {final_pitch:+.1f}°')
        
        # Formatting
        ax.set_xlabel('Time (seconds)')
        ax.set_ylabel('Pitch Angle (degrees)')
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        
        # Add pass/fail indicator in title
        status = "PASS" if test_data['passed'] else "FAIL"
        status_color = "green" if test_data['passed'] else "red"
        ax.text(0.02, 0.98, status, transform=ax.transAxes, fontsize=12, 
               fontweight='bold', color=status_color, 
               verticalalignment='top')
    
    def plot_depth_data(self, ax, test_data, title):
        """
        Plot depth vs time for a single test
        
        Args:
            ax: matplotlib axis object to plot on
            test_data: test result dictionary with simulation_data
            title: title for this subplot
        """
        
        sim_data = test_data['simulation_data']
        time = sim_data['time']
        depth = sim_data['depth']
        eq_start_idx = test_data['equilibrium_start_idx']
        
        # Plot the depth data
        ax.plot(time, depth, 'b-', linewidth=1, label='Depth')
        
        # Highlight equilibrium analysis region
        eq_start_time = time[eq_start_idx]
        ax.axvspan(eq_start_time, time[-1], alpha=0.2, color='green', 
                  label='Equilibrium Analysis Window')
        
        # Add target depth line
        if test_data['test'] == 'surfaced_trim':
            # Should stay at surface (depth ≈ 0)
            ax.axhline(y=0, color='red', linestyle='--', alpha=0.7, 
                      label='Target: Surface (0 m)')
            ax.axhspan(-0.1, 0.1, alpha=0.2, color='lightgreen', 
                      label='Acceptable Range (±0.1 m)')
        else:  # submerged_trim
            # Should maintain depth around initial value
            target_depth = 2.0  # Initial depth for submerged test
            ax.axhline(y=target_depth, color='red', linestyle='--', alpha=0.7, 
                      label=f'Target: {target_depth:.1f} m')
            ax.axhspan(target_depth-0.2, target_depth+0.2, alpha=0.2, color='lightgreen', 
                      label='Acceptable Range (±0.2 m)')
        
        # Formatting
        ax.set_xlabel('Time (seconds)')
        ax.set_ylabel('Depth (meters)')
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        
        # Invert y-axis so depth increases downward (standard oceanographic convention)
        ax.invert_yaxis()
        
        # Show final depth
        final_depth = test_data['final_depth']
        ax.text(0.98, 0.02, f'Final: {final_depth:.2f} m', 
               transform=ax.transAxes, fontsize=10, 
               horizontalalignment='right', verticalalignment='bottom',
               bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    def run_full_validation(self):
        """Run complete trim validation sequence"""
        
        print("\n" + "="*60)
        print("ZERO BUBBLE TRIM VALIDATION")
        print("="*60)
        
        # Check if initialization was successful
        if self.vehicle_dynamics is None:
            print("ERROR: Cannot run validation - vehicle_dynamics initialization failed")
            print("Check the error messages above for details")
            return False
        
        # Show configuration being tested
        config_source = self.vehicle_dynamics.config['metadata'].get('generator', 'unknown')
        print(f"Testing configuration from: {config_source}")
        
        if 'generated_date' in self.vehicle_dynamics.config['metadata']:
            print(f"Generated: {self.vehicle_dynamics.config['metadata']['generated_date']}")
        
        print(f"Vehicle mass: {self.vehicle_dynamics.mass:.2f} kg")
        print(f"CG position: {self.vehicle_dynamics.cg_from_nose:.3f} m from nose")
        print(f"Surfaced buoyancy: {self.vehicle_dynamics.surfaced_buoyancy:+.2f} N")
        print(f"Submerged buoyancy: {self.vehicle_dynamics.submerged_buoyancy:+.2f} N")
        
        # Run tests
        results = {}
        overall_passed = True
        
        # Test 1: Surfaced trim
        print(f"\n{'='*40}")
        print("TEST 1: SURFACED TRIM VALIDATION")
        print(f"{'='*40}")
        
        try:
            surfaced_result = self.validate_surfaced_trim()
            results['surfaced'] = surfaced_result
            
            print(f"Equilibrium pitch: {surfaced_result['equilibrium_pitch_deg']:+.2f}°")
            print(f"Tolerance: ±{surfaced_result['pitch_tolerance']:.1f}°")
            print(f"Final depth: {surfaced_result['final_depth']:.3f} m")
            print(f"Result: {'PASS' if surfaced_result['passed'] else 'FAIL'}")
            print(f"Message: {surfaced_result['message']}")
            
            if not surfaced_result['passed']:
                overall_passed = False
                
        except Exception as e:
            print(f"ERROR in surfaced trim test: {e}")
            results['surfaced'] = {'error': str(e)}
            overall_passed = False
        
        # Test 2: Submerged trim
        print(f"\n{'='*40}")
        print("TEST 2: SUBMERGED TRIM VALIDATION")
        print(f"{'='*40}")
        
        try:
            submerged_result = self.validate_submerged_trim()
            results['submerged'] = submerged_result
            
            print(f"Equilibrium pitch: {submerged_result['equilibrium_pitch_deg']:+.2f}°")
            print(f"Acceptable range: {submerged_result['pitch_range'][0]:.1f}° to {submerged_result['pitch_range'][1]:.1f}°")
            print(f"Final depth: {submerged_result['final_depth']:.3f} m")
            print(f"Vertical velocity: {submerged_result['final_vz']:+.4f} m/s")
            print(f"Result: {'PASS' if submerged_result['passed'] else 'FAIL'}")
            print(f"Message: {submerged_result['message']}")
            
            if not submerged_result['passed']:
                overall_passed = False
                
        except Exception as e:
            print(f"ERROR in submerged trim test: {e}")
            results['submerged'] = {'error': str(e)}
            overall_passed = False
        
        # Generate plots if enabled
        if self.enable_plotting:
            try:
                plot_file = self.create_validation_plots()
                if plot_file:
                    results['plot_file'] = plot_file
            except Exception as e:
                print(f"Warning: Could not generate plots: {e}")
        
        # Overall results
        print(f"\n{'='*60}")
        print("OVERALL TRIM VALIDATION RESULTS")
        print(f"{'='*60}")
        
        if overall_passed:
            print("✓ TRIM VALIDATION PASSED")
            print("Vehicle is properly trimmed for both surfaced and submerged operation.")
            print("Ready for mission execution.")
        else:
            print("✗ TRIM VALIDATION FAILED")
            print("Vehicle trim needs adjustment before mission execution.")
            print("Review zero bubble optimization results and adjust ballast/foam.")
        
        # Save results
        self.save_validation_report(results, overall_passed)
        
        return overall_passed
    
    def save_validation_report(self, results, overall_passed):
        """Save validation report to file"""
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'C:\\Users\\E22000316\\Desktop\\simplr_auv\\trim_validation_{timestamp}.json'
        
        report = {
            'metadata': {
                'validation_date': datetime.now().isoformat(),
                'validator_version': '1.1',  # Updated version
                'config_source': self.vehicle_dynamics.config['metadata'].get('source', 'unknown'),
                'overall_result': 'PASS' if overall_passed else 'FAIL',
                'plotting_enabled': self.enable_plotting  # Track plotting status
            },
            'vehicle_configuration': {
                'mass_kg': self.vehicle_dynamics.mass,
                'cg_from_nose_m': self.vehicle_dynamics.cg_from_nose,
                'surfaced_buoyancy_N': self.vehicle_dynamics.surfaced_buoyancy,
                'submerged_buoyancy_N': self.vehicle_dynamics.submerged_buoyancy
            },
            'validation_parameters': {
                'settling_time_s': self.SETTLING_TIME,
                'surfaced_pitch_tolerance_deg': self.SURFACED_PITCH_TOLERANCE,
                'submerged_pitch_range_deg': [self.SUBMERGED_PITCH_MIN, self.SUBMERGED_PITCH_MAX]
            },
            'test_results': results,
            'recommendations': self.generate_recommendations(results)
        }
        
        try:
            os.makedirs(os.path.dirname(filename), exist_ok=True)
            with open(filename, 'w') as f:
                json.dump(report, f, indent=2)
            print(f"\nValidation report saved: {filename}")
        except Exception as e:
            print(f"Warning: Could not save validation report: {e}")
    
    def generate_recommendations(self, results):
        """Generate recommendations based on validation results"""
        
        recommendations = []
        
        # Surfaced trim recommendations
        if 'surfaced' in results and not results['surfaced'].get('passed', False):
            surf = results['surfaced']
            if not surf.get('pitch_ok', True):
                pitch = surf.get('equilibrium_pitch_deg', 0)
                if pitch > 2.0:
                    recommendations.append("Add weight to bow or move CG forward for surfaced operation")
                elif pitch < -2.0:
                    recommendations.append("Add weight to stern or move CG aft for surfaced operation")
            
            if not surf.get('depth_ok', True):
                recommendations.append("Check surfaced buoyancy - vehicle should float at surface")
        
        # Submerged trim recommendations  
        if 'submerged' in results and not results['submerged'].get('passed', False):
            sub = results['submerged']
            if not sub.get('pitch_ok', True):
                pitch = sub.get('equilibrium_pitch_deg', 0)
                if pitch > 0:
                    recommendations.append("Submerged pitch too bow-up - adjust ballast distribution")
                elif pitch < -5.0:
                    recommendations.append("Submerged pitch too bow-down - reduce stern ballast")
            
            if not sub.get('depth_stable', True):
                recommendations.append("Vehicle not neutrally buoyant when submerged - adjust foam/ballast balance")
        
        if not recommendations:
            recommendations.append("Trim validation passed - no adjustments needed")
        
        return recommendations

def main():
    """Main execution function with command line argument parsing"""
    
    # Set up command line argument parsing
    parser = argparse.ArgumentParser(
        description='Zero Bubble Trim Validation Script v1.1',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  python validate_trim.py                    # Run validation without plots
  python validate_trim.py --plot             # Run validation with plots
  python validate_trim.py --plot --config custom_config.json   # Use custom config

Note: Plotting requires matplotlib. Install with: pip install matplotlib
        '''
    )
    
    # Add command line arguments
    parser.add_argument('--plot', '--plots', 
                       action='store_true',
                       help='Enable plotting of validation results')
    
    parser.add_argument('--config', 
                       type=str, 
                       default=None,
                       help='Path to custom vehicle configuration file')
    
    parser.add_argument('--no-interactive', 
                       action='store_true',
                       help='Disable interactive plot display (save only)')
    
    # Parse arguments
    args = parser.parse_args()
    
    print("Zero Bubble Trim Validation Script v1.1")
    print("Validates vehicle trim in surfaced and submerged states")
    
    # Set matplotlib to non-interactive mode if requested
    if args.no_interactive and PLOTTING_AVAILABLE:
        import matplotlib
        matplotlib.use('Agg')  # Non-interactive backend
        print("Plots will be saved but not displayed interactively")
    
    try:
        # Initialize validator with plotting option
        validator = TrimValidator(
            config_file=args.config, 
            enable_plotting=args.plot
        )
        
        # Check if initialization was successful
        if validator.vehicle_dynamics is None:
            print("Initialization failed - cannot proceed with validation")
            return False
        
        # Show what we're about to do
        if args.plot:
            if PLOTTING_AVAILABLE:
                print("Running validation with plotting enabled...")
            else:
                print("WARNING: Plotting requested but matplotlib not available")
                print("Install matplotlib with: pip install matplotlib")
        else:
            print("Running validation without plotting...")
        
        # Run validation
        success = validator.run_full_validation()
        
        # Additional information about results
        if success:
            print(f"\n{'='*60}")
            print("VALIDATION COMPLETE - ALL TESTS PASSED")
            print(f"{'='*60}")
            if validator.enable_plotting:
                print("• Check the generated plots for detailed analysis")
                print("• Plots show pitch and depth stability over time")
            print("• JSON report contains detailed numerical results")
            print("• Vehicle is ready for mission execution")
        else:
            print(f"\n{'='*60}")
            print("VALIDATION COMPLETE - SOME TESTS FAILED")
            print(f"{'='*60}")
            if validator.enable_plotting:
                print("• Check the generated plots to diagnose issues")
                print("• Red regions in plots show problems")
            print("• Review recommendations in JSON report")
            print("• Adjust vehicle configuration before missions")
        
        return success
        
    except KeyboardInterrupt:
        print("\nValidation interrupted by user")
        return False
        
    except Exception as e:
        print(f"FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == '__main__':
    success = main()
    if not success:
        exit(1)
    else:
        print("\nTrim validation completed successfully.")