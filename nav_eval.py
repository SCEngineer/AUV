#!/usr/bin/env python3
"""
nav_eval.py - Navigation System Dead Reckoning Evaluation Script

Tests navigation system performance during underwater operation when GPS is unavailable.
Compares navigation estimates against true vehicle state to measure:
- Position error accumulation over time
- Cross-track drift accuracy
- Heading and depth tracking errors

Test Scenario: Straight-line transit at constant depth and speed
"""

import sys
import time
import math
import csv
from typing import Dict, List, Tuple

# Import required modules
try:
    from vehicle_state import VehicleState
    from navigation import Navigation
    from vehicle_dynamics import VehicleDynamics
    from sensor_models import SensorModels
except ImportError as e:
    print(f"ERROR: Required modules not found: {e}")
    print("Please ensure all required files are in the same directory:")
    print("  - vehicle_state.py")
    print("  - navigation.py")
    print("  - vehicle_dynamics.py")
    print("  - sensor_models.py")
    sys.exit(1)


class NavigationEvaluator:
    """Evaluates navigation system dead reckoning performance"""
    
    def __init__(self):
        self.vehicle_state = VehicleState()
        self.navigation = None
        self.vehicle_dynamics = None
        self.sensor_models = None
        
        # Data collection
        self.data_log: List[Dict] = []
        self.start_time = 0.0
        
        # Test configuration
        self.test_duration = 600.0  # 10 minutes underwater
        self.timestep = 0.1  # 100ms updates
        self.target_speed = 1.5  # m/s
        self.target_depth = 5.0  # meters
        self.target_heading = 45.0  # degrees (northeast)
        
    def initialize_systems(self):
        """Initialize all vehicle systems"""
        print("=" * 70)
        print("NAVIGATION DEAD RECKONING EVALUATION")
        print("=" * 70)
        print("\nInitializing vehicle systems...")
        
        # Initialize dynamics and sensors
        self.vehicle_dynamics = VehicleDynamics(self.vehicle_state, dt=self.timestep)
        self.sensor_models = SensorModels(self.vehicle_state)
        
        # Initialize navigation
        self.navigation = Navigation(self.vehicle_state)
        
        print("✓ Vehicle dynamics initialized")
        print("✓ Sensor models initialized")
        print("✓ Navigation system initialized")
        
    def setup_initial_conditions(self):
        """Setup initial vehicle state with GPS fix and calibration period"""
        print("\nSetting up initial conditions...")
        
        # Start at a known position (surfaced with GPS)
        initial_lat = 37.7749  # San Francisco area
        initial_lon = -122.4194
        
        # STEP 1: Initialize at surface with GPS
        with self.vehicle_state._state_lock:
            self.vehicle_state.true_state.update({
                'true_lat': initial_lat,
                'true_lon': initial_lon,
                'true_depth': 0.0,  # Surface for GPS
                'true_heading': self.target_heading,
                'true_speed': 0.0,
                'true_pitch': 0.0,
                'true_vertical_velocity': 0.0,
                'true_pitch_rate': 0.0,
                'true_yaw_rate': 0.0
            })
            self.vehicle_state.buoyancy_state = 'POSITIVE'
        
        # Get GPS fix and initialize navigation
        for i in range(20):
            sensor_data = self.sensor_models.update(self.timestep)
            self.vehicle_state.sensor_data = sensor_data
            self.navigation.update(self.timestep)
        
        if not self.navigation.initialized:
            print("✗ Navigation failed to initialize from GPS")
            return False
        
        print(f"✓ Navigation initialized at surface")
        
        # STEP 2: CALIBRATION PERIOD - Move around at surface to collect data
        print(f"\nCalibration period: Collecting GPS data for sensor calibration...")
        print(f"  Need {self.navigation.calibration_config['min_samples']} samples at >0.5 m/s")
        
        calibration_time = 0.0
        max_calibration_time = 120.0  # 2 minutes max
        last_status = 0.0
        
        # Accelerate and move around at surface
        with self.vehicle_state._state_lock:
            self.vehicle_state.actuator_commands['thruster_cmd'] = 50.0
            self.vehicle_state.actuator_commands.update({
                'UL': 0.0, 'UR': 0.0, 'LR': 0.0, 'LL': 0.0
            })
        
        while calibration_time < max_calibration_time:
            # Update vehicle at surface
            self.vehicle_dynamics.update(self.timestep)
            sensor_data = self.sensor_models.update(self.timestep)
            
            # FORCE GPS to be available during calibration
            with self.vehicle_state._state_lock:
                self.vehicle_state.true_state['true_depth'] = 0.0  # Force surface
                
                # INJECT GPS data if sensor model didn't provide it
                if not sensor_data:
                    sensor_data = {}
                    
                if not sensor_data.get('gps'):
                    # Manually create GPS data from true state
                    true_lat = self.vehicle_state.true_state.get('true_lat', 0.0)
                    true_lon = self.vehicle_state.true_state.get('true_lon', 0.0)
                    true_speed = self.vehicle_state.true_state.get('true_speed', 0.0)
                    true_heading = self.vehicle_state.true_state.get('true_heading', 0.0)
                    
                    sensor_data['gps'] = {
                        'lat': true_lat,
                        'lon': true_lon,
                        'speed': true_speed,
                        'course': true_heading,
                        'fix_quality': 'excellent'
                    }
            
            self.vehicle_state.sensor_data = sensor_data
            self.navigation.update(self.timestep)
            
            # Add some heading variation to get better calibration data
            if calibration_time > 20.0 and calibration_time < 60.0:
                # Gentle turn
                with self.vehicle_state._state_lock:
                    self.vehicle_state.actuator_commands.update({
                        'UL': 5.0, 'UR': 5.0, 'LR': -5.0, 'LL': -5.0
                    })
            elif calibration_time >= 60.0:
                # Straighten out
                with self.vehicle_state._state_lock:
                    self.vehicle_state.actuator_commands.update({
                        'UL': 0.0, 'UR': 0.0, 'LR': 0.0, 'LL': 0.0
                    })
            
            # Check calibration status
            cal_status = self.navigation.get_calibration_status()
            
            if calibration_time - last_status >= 10.0:
                samples = cal_status['data_collected']
                needed = cal_status['min_samples_needed']
                speed = self.vehicle_state.true_state.get('true_speed', 0.0)
                
                # DEBUG: Check why samples aren't being collected
                sensor_data = self.vehicle_state.sensor_data
                gps_data = sensor_data.get('gps') if sensor_data else None
                if gps_data:
                    gps_quality = gps_data.get('fix_quality', 'unknown')
                    gps_speed = gps_data.get('speed', 0.0)
                    gps_course = gps_data.get('course', None)
                    print(f"  T+{calibration_time:.0f}s: {samples}/{needed} samples, speed={speed:.2f} m/s")
                    print(f"    GPS: quality={gps_quality}, speed={gps_speed:.2f}, course={gps_course}")
                else:
                    print(f"  T+{calibration_time:.0f}s: {samples}/{needed} samples, speed={speed:.2f} m/s - NO GPS DATA")
                
                last_status = calibration_time
            
            # Check if calibration is complete
            if cal_status['calibration_valid']:
                print(f"✓ Calibration complete after {calibration_time:.1f}s!")
                print(f"  Speed: scale={cal_status['speed_scale']:.3f}, bias={cal_status['speed_bias']:.3f} m/s")
                print(f"  Heading: bias={cal_status['heading_bias']:.2f}°")
                break
            
            calibration_time += self.timestep
        
        # Check if calibration succeeded
        cal_status = self.navigation.get_calibration_status()
        if not cal_status['calibration_valid']:
            print(f"⚠ Warning: Calibration not completed (only {cal_status['data_collected']} samples)")
            print("  Proceeding anyway - navigation will use uncalibrated sensors")
        
        # STEP 3: Reset to starting position and conditions for test
        print(f"\n  Resetting to test start position...")
        with self.vehicle_state._state_lock:
            self.vehicle_state.true_state.update({
                'true_lat': initial_lat,
                'true_lon': initial_lon,
                'true_depth': self.target_depth,
                'true_heading': self.target_heading,
                'true_speed': self.target_speed,
                'true_pitch': 0.0,
                'true_vertical_velocity': 0.0,
                'true_pitch_rate': 0.0,
                'true_yaw_rate': 0.0
            })
            self.vehicle_state.buoyancy_state = 'NEUTRAL'
            self.vehicle_state.actuator_commands.update({
                'thruster_cmd': 40.0,
                'UL': 0.0, 'UR': 0.0, 'LR': 0.0, 'LL': 0.0
            })
        
        # Run for a few seconds to let navigation settle at new depth
        for i in range(50):
            self.vehicle_dynamics.update(self.timestep)
            sensor_data = self.sensor_models.update(self.timestep)
            self.vehicle_state.sensor_data = sensor_data
            self.navigation.update(self.timestep)
        
        # Show final state
        print(f"✓ Test ready:")
        print(f"  True: depth={self.vehicle_state.true_state['true_depth']:.1f}m, "
              f"heading={self.vehicle_state.true_state['true_heading']:.1f}°, "
              f"speed={self.vehicle_state.true_state['true_speed']:.2f} m/s")
        print(f"  Nav:  depth={self.navigation.depth:.1f}m, "
              f"heading={self.navigation.heading:.1f}°, "
              f"speed={self.navigation.speed:.2f} m/s")
        
        return True
    
    def dive_to_test_depth(self):
        """Skip dive - already at depth"""
        return True
    
    def run_underwater_test(self):
        """Run the main underwater dead reckoning test"""
        print(f"\nStarting underwater dead reckoning test...")
        print(f"Duration: {self.test_duration:.0f} seconds")
        print(f"Target: Straight line at {self.target_heading:.0f}° heading")
        print("-" * 70)
        
        self.start_time = time.time()
        test_time = 0.0
        
        last_report_time = 0.0
        report_interval = 60.0  # Report every minute
        
        while test_time < self.test_duration:
            # Maintain straight and level flight
            with self.vehicle_state._state_lock:
                self.vehicle_state.actuator_commands.update({
                    'thruster_cmd': 40.0,  # Maintain speed
                    'UL': 0.0, 'UR': 0.0, 'LR': 0.0, 'LL': 0.0  # Level
                })
            
            # Update vehicle dynamics
            self.vehicle_dynamics.update(self.timestep)
            
            # Update sensors (no GPS underwater)
            sensor_data = self.sensor_models.update(self.timestep)
            self.vehicle_state.sensor_data = sensor_data
            
            # Update navigation (dead reckoning only)
            self.navigation.update(self.timestep)
            
            # Collect data
            data_point = self.collect_data_point(test_time)
            self.data_log.append(data_point)
            
            # Progress report
            if test_time - last_report_time >= report_interval:
                self.print_progress_report(test_time, data_point)
                last_report_time = test_time
            
            test_time += self.timestep
            time.sleep(0.001)
        
        print("\n✓ Underwater test complete")
        print(f"  Total duration: {test_time:.1f} seconds")
        print(f"  Data points collected: {len(self.data_log)}")
    
    def collect_data_point(self, test_time: float) -> Dict:
        """Collect a single data point for analysis"""
        # True state
        true_lat = self.vehicle_state.true_state.get('true_lat', 0.0)
        true_lon = self.vehicle_state.true_state.get('true_lon', 0.0)
        true_depth = self.vehicle_state.true_state.get('true_depth', 0.0)
        true_heading = self.vehicle_state.true_state.get('true_heading', 0.0)
        true_speed = self.vehicle_state.true_state.get('true_speed', 0.0)
        true_pitch = self.vehicle_state.true_state.get('true_pitch', 0.0)
        true_yaw_rate = self.vehicle_state.true_state.get('true_yaw_rate', 0.0)
        
        # Navigation estimates
        nav_lat = self.navigation.lat
        nav_lon = self.navigation.lon
        nav_depth = self.navigation.depth
        nav_heading = self.navigation.heading
        nav_speed = self.navigation.speed
        nav_pitch = self.navigation.pitch
        nav_yaw_rate = self.navigation.yaw_rate
        nav_uncertainty = self.navigation.position_uncertainty
        
        # Navigation internal state (for debugging)
        compass_heading = self.navigation.compass_heading
        gyro_heading = self.navigation.gyro_heading
        
        # Sensor readings
        sensor_data = self.vehicle_state.sensor_data
        sensor_compass = sensor_data.get('compass', 0.0) if sensor_data else 0.0
        sensor_speed = sensor_data.get('water_speed', {}).get('forward', 0.0) if sensor_data else 0.0
        
        # Calculate errors
        position_error = self.calculate_distance(true_lat, true_lon, nav_lat, nav_lon)
        
        # Calculate cross-track error (perpendicular distance from intended track)
        cross_track_error = self.calculate_cross_track_error(
            true_lat, true_lon, nav_lat, nav_lon, self.target_heading
        )
        
        heading_error = self.angle_difference(true_heading, nav_heading)
        depth_error = true_depth - nav_depth
        speed_error = true_speed - nav_speed
        
        return {
            'time': test_time,
            'true_lat': true_lat,
            'true_lon': true_lon,
            'true_depth': true_depth,
            'true_heading': true_heading,
            'true_speed': true_speed,
            'true_pitch': math.degrees(true_pitch),
            'true_yaw_rate': math.degrees(true_yaw_rate),
            'nav_lat': nav_lat,
            'nav_lon': nav_lon,
            'nav_depth': nav_depth,
            'nav_heading': nav_heading,
            'nav_speed': nav_speed,
            'nav_pitch': nav_pitch,
            'nav_yaw_rate': math.degrees(nav_yaw_rate),
            'nav_uncertainty': nav_uncertainty,
            'nav_compass_heading': compass_heading,
            'nav_gyro_heading': gyro_heading,
            'sensor_compass': sensor_compass,
            'sensor_speed': sensor_speed,
            'position_error_m': position_error,
            'cross_track_error_m': cross_track_error,
            'heading_error_deg': heading_error,
            'depth_error_m': depth_error,
            'speed_error_mps': speed_error
        }
    
    def calculate_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance between two lat/lon points in meters"""
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        avg_lat = math.radians((lat1 + lat2) / 2)
        north_m = dlat * 111111.0
        east_m = dlon * 111111.0 * math.cos(avg_lat)
        
        return math.sqrt(north_m**2 + east_m**2)
    
    def calculate_cross_track_error(self, true_lat: float, true_lon: float, 
                                    nav_lat: float, nav_lon: float, 
                                    intended_heading: float) -> float:
        """Calculate cross-track error (perpendicular distance from true position)
        
        This is the component of position error perpendicular to the vehicle's 
        intended heading direction.
        """
        # Calculate position error vector (nav - true)
        dlat = nav_lat - true_lat
        dlon = nav_lon - true_lon
        
        avg_lat = math.radians((true_lat + nav_lat) / 2)
        north_error_m = dlat * 111111.0
        east_error_m = dlon * 111111.0 * math.cos(avg_lat)
        
        # Convert intended heading to radians (nautical: 0° = North, clockwise)
        heading_rad = math.radians(intended_heading)
        
        # Cross-track is perpendicular to heading direction
        # For a heading direction (forward along track):
        #   forward_component = north_error * cos(heading) + east_error * sin(heading)
        #   cross_track = east_error * cos(heading) - north_error * sin(heading)
        
        # This gives the perpendicular component (positive = right of track)
        cross_track = east_error_m * math.cos(heading_rad) - north_error_m * math.sin(heading_rad)
        
        return cross_track
    
    def angle_difference(self, angle1: float, angle2: float) -> float:
        """Calculate shortest angular difference"""
        diff = angle2 - angle1
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff
    
    def print_progress_report(self, test_time: float, data: Dict):
        """Print progress during test"""
        print(f"T+{test_time:6.1f}s | Pos Error: {data['position_error_m']:6.2f}m | "
              f"X-Track: {data['cross_track_error_m']:+6.2f}m | "
              f"Hdg Err: {data['heading_error_deg']:+5.1f}° | "
              f"Uncertainty: {data['nav_uncertainty']:6.1f}m")
    
    def analyze_results(self):
        """Analyze and print test results"""
        print("\n" + "=" * 70)
        print("ANALYSIS RESULTS")
        print("=" * 70)
        
        if not self.data_log:
            print("No data collected!")
            return
        
        # Calculate statistics
        position_errors = [d['position_error_m'] for d in self.data_log]
        cross_track_errors = [d['cross_track_error_m'] for d in self.data_log]
        heading_errors = [d['heading_error_deg'] for d in self.data_log]
        depth_errors = [d['depth_error_m'] for d in self.data_log]
        uncertainties = [d['nav_uncertainty'] for d in self.data_log]
        
        print("\nPosition Error Statistics:")
        print(f"  Initial:  {position_errors[0]:6.2f} m")
        print(f"  Final:    {position_errors[-1]:6.2f} m")
        print(f"  Mean:     {sum(position_errors)/len(position_errors):6.2f} m")
        print(f"  Max:      {max(position_errors):6.2f} m")
        print(f"  Min:      {min(position_errors):6.2f} m")
        
        print("\nCross-Track Error Statistics:")
        print(f"  Initial:  {cross_track_errors[0]:+6.2f} m")
        print(f"  Final:    {cross_track_errors[-1]:+6.2f} m")
        print(f"  Mean:     {sum(cross_track_errors)/len(cross_track_errors):+6.2f} m")
        print(f"  Max:      {max(cross_track_errors):+6.2f} m")
        print(f"  Min:      {min(cross_track_errors):+6.2f} m")
        print(f"  RMS:      {math.sqrt(sum(e**2 for e in cross_track_errors)/len(cross_track_errors)):6.2f} m")
        
        print("\nHeading Error Statistics:")
        print(f"  Initial:  {heading_errors[0]:+6.2f}°")
        print(f"  Final:    {heading_errors[-1]:+6.2f}°")
        print(f"  Mean:     {sum(heading_errors)/len(heading_errors):+6.2f}°")
        print(f"  Max:      {max(heading_errors):+6.2f}°")
        print(f"  Min:      {min(heading_errors):+6.2f}°")
        
        print("\nDepth Error Statistics:")
        print(f"  Initial:  {depth_errors[0]:+6.2f} m")
        print(f"  Final:    {depth_errors[-1]:+6.2f} m")
        print(f"  Mean:     {sum(depth_errors)/len(depth_errors):+6.2f} m")
        print(f"  Max:      {max(depth_errors):+6.2f} m")
        print(f"  Min:      {min(depth_errors):+6.2f} m")
        
        print("\nPosition Uncertainty Tracking:")
        print(f"  Initial:  {uncertainties[0]:6.2f} m")
        print(f"  Final:    {uncertainties[-1]:6.2f} m")
        print(f"  Growth:   {uncertainties[-1] - uncertainties[0]:+6.2f} m")
        
        # Error accumulation rate
        duration_minutes = self.data_log[-1]['time'] / 60.0
        error_rate = position_errors[-1] / duration_minutes
        print(f"\nError Accumulation Rate:")
        print(f"  {error_rate:.2f} m/minute")
        print(f"  {error_rate * 60:.2f} m/hour")
        
        # Distance traveled
        distance_traveled = self.data_log[-1]['true_speed'] * self.data_log[-1]['time']
        error_percentage = (position_errors[-1] / distance_traveled) * 100 if distance_traveled > 0 else 0
        print(f"\nDistance Traveled: {distance_traveled:.1f} m")
        print(f"Position Error as % of Distance: {error_percentage:.2f}%")
        
        print("\n" + "=" * 70)
    
    def save_to_csv(self, filename: str = "nav_evaluation.csv"):
        """Save data log to CSV file"""
        if not self.data_log:
            print("No data to save!")
            return
        
        print(f"\nSaving data to {filename}...")
        
        try:
            with open(filename, 'w', newline='') as csvfile:
                fieldnames = self.data_log[0].keys()
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                writer.writeheader()
                for row in self.data_log:
                    writer.writerow(row)
            
            print(f"✓ Data saved successfully ({len(self.data_log)} rows)")
            print(f"  File: {filename}")
        except Exception as e:
            print(f"✗ Error saving CSV: {e}")
    
    def run_full_evaluation(self):
        """Run complete evaluation sequence"""
        try:
            # Initialize
            self.initialize_systems()
            
            # Setup initial conditions
            if not self.setup_initial_conditions():
                print("Failed to setup initial conditions")
                return False
            
            # Dive to test depth
            if not self.dive_to_test_depth():
                print("Failed to reach test conditions")
                return False
            
            # Run underwater test
            self.run_underwater_test()
            
            # Analyze results
            self.analyze_results()
            
            # Save data
            self.save_to_csv()
            
            return True
            
        except KeyboardInterrupt:
            print("\n\nTest interrupted by user")
            if self.data_log:
                print("Analyzing partial results...")
                self.analyze_results()
                self.save_to_csv("nav_evaluation_partial.csv")
            return False
        except Exception as e:
            print(f"\n\nERROR during evaluation: {e}")
            import traceback
            traceback.print_exc()
            return False


def main():
    """Main entry point"""
    print("\n" + "=" * 70)
    print("NAVIGATION DEAD RECKONING EVALUATION TOOL")
    print("Tests underwater navigation accuracy without GPS")
    print("=" * 70)
    
    evaluator = NavigationEvaluator()
    success = evaluator.run_full_evaluation()
    
    if success:
        print("\n✓ Evaluation completed successfully")
    else:
        print("\n✗ Evaluation failed or was interrupted")
    
    print("\nDone.")


if __name__ == "__main__":
    main()
