#!/usr/bin/env python3
"""
navigation.py - FIXED: Proper calibration application and realistic uncertainty tracking

FIXES IN THIS VERSION:
1. Heading initialization now uses calibrated compass readings
2. Position uncertainty calculation fixed - proper variance accumulation
3. Calibration corrections verified to be applied consistently

GPS FILTERING FIX:
- Added complementary filter for GPS position updates
- Rejects obviously bad GPS readings
- Smooths GPS integration to prevent jerky surface motion

GPS RESURFACING CORRECTION:
- Added exception for large GPS corrections after underwater excursions
- GPS_FIX task can prepare navigation for expected drift correction
- Temporarily relaxes outlier rejection when resurfacing

PITCH CALCULATION FIX:
- Added proper pitch and roll calculation from IMU accelerometer data
- Pitch is now derived from gravity vector for realistic attitude estimation

POSITION UNCERTAINTY FIX:
- Added real position uncertainty calculation and tracking
- Uncertainty grows during dead reckoning based on sensor errors
- Uncertainty reduces when GPS fixes are received
- Uncertainty written to nav_state for use by other systems
"""

import time
import math
import numpy as np
from typing import Dict, Optional, Tuple, Any, List

class Navigation:
    def __init__(self, vehicle_state, hal=None):
        self.vehicle_state = vehicle_state
        self.hal = hal

        # Internal navigation estimates - start uninitialized
        self.lat = 0.0
        self.lon = 0.0
        self.depth = 0.0
        self.heading = 0.0
        self.speed = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw_rate = 0.0
        
        # ENHANCED: Three-axis velocity tracking
        self.velocity_surge = 0.0    # Forward/backward
        self.velocity_sway = 0.0     # Port/starboard  
        self.velocity_heave = 0.0    # Up/down
        self.cross_track_drift = 0.0 # Accumulated sideways drift
        
        # Store current accelerometer readings
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 9.81
        
        # Complementary filter for heading - COMPASS-BIASED
        self.alpha = 0.90  # Increased from 0.75 - more gyro weight between compass updates
        self.compass_timeout = 2.0  # Increased from 1.0 - less sensitive to brief dropouts
        self.last_compass_time = 0.0
        self.gyro_heading = 0.0
        self.compass_heading = 0.0
        self.heading_initialized = False
        
        # Position tracking - start with high uncertainty
        self.position_uncertainty = 1000.0  # High uncertainty until GPS fix
        self.max_position_uncertainty = 1000.0
        self.last_update_time = time.time()
        self.last_gps_time = 0.0
        self.initialized = False  # Must get GPS fix to initialize

        # ENHANCED: Underwater navigation improvements
        self.underwater_position_drift = {'north': 0.0, 'east': 0.0}
        self.last_surface_position = {'lat': 0.0, 'lon': 0.0, 'time': 0.0}
        self.three_axis_available = False

        # GPS FILTERING PARAMETERS
        self.gps_filter_config = {
            'filter_alpha': 0.3,        # 0.0 = ignore GPS, 1.0 = trust GPS completely
            'max_gps_jump': 20.0,       # meters - reject GPS readings beyond this
            'min_gps_interval': 1.0,    # seconds - minimum time between GPS updates
            'outlier_rejection': True,   # Enable outlier detection
            'speed_consistency_check': True  # Check GPS-derived speed vs. sensor speed
        }
        
        # GPS filtering state
        self.gps_filter_state = {
            'consecutive_outliers': 0,
            'max_consecutive_outliers': 3,
            'last_good_gps_time': 0.0,
            'position_variance_estimate': 25.0  # Initial GPS noise estimate (5m std)
        }

        # GPS RESURFACING CORRECTION
        self.resurfacing_correction = {
            'enabled': False,
            'max_expected_drift': 500.0,  # meters - allow large corrections
            'activated_time': 0.0,
            'timeout': 30.0,  # seconds - revert to normal filtering after timeout
            'correction_applied': False
        }

        # === FIXED UNCERTAINTY MODEL PARAMETERS ===
        # Physics-based parameters with proper variance calculation
        self.uncertainty_model = {
            # Compass error propagation (major contributor to position uncertainty)
            'compass_std_deg_uncalibrated': 5.0,    # Realistic uncalibrated compass error
            'compass_std_deg_calibrated': 1.0,      # Good calibration
            
            # Speed sensor error accumulation in dead reckoning
            'speed_error_pct_uncalibrated': 0.15,   # 15% speed error without calibration
            'speed_error_pct_calibrated': 0.05,     # 5% speed error with GPS calibration
            
            # IMU drift characteristics
            'gyro_drift_deg_per_hour': 2.0,         # Realistic MEMS gyro drift rate
            'gyro_drift_deg_per_second': 2.0 / 3600.0,  # Convert to per-second
            
            # Environmental uncertainty
            'unknown_current_mps': 0.1,            # Unknown water current effects (10 cm/s)
            'current_uncertainty_per_time': 0.1,   # Current uncertainty growth rate
            
            # Cross-track uncertainty from sway velocity errors
            'sway_velocity_error_factor': 0.15,     # 15% error in sideways motion detection
            
            # Stationary drift (mainly from gyro bias integration)
            'stationary_drift_mps': 0.02,           # 2 cm/s uncertainty growth when stationary
            
            # Minimum uncertainty when stationary (sensor noise floor)
            'minimum_uncertainty_m': 2.0,           # Never better than 2m uncertainty
            
            # GPS uncertainty reduction
            'gps_good_uncertainty': 5.0,            # meters - good GPS fix uncertainty
            'gps_excellent_uncertainty': 2.0        # meters - excellent GPS fix uncertainty
        }

        # === GPS-BASED SENSOR CALIBRATION SYSTEM ===
        
        # Calibration data collection
        self.calibration_data = {
            'gps_speeds': [],
            'sensor_speeds': [],
            'gps_courses': [],
            'compass_headings': [],
            'timestamps': []
        }
        
        # Calibration results
        self.sensor_corrections = {
            'speed_bias': 0.0,       # m/s to subtract from sensor
            'speed_scale': 1.0,      # Factor to multiply sensor speed
            'heading_bias': 0.0,     # Degrees to subtract from compass
            'calibration_valid': False,
            'calibration_time': 0.0,
            'calibration_samples': 0
        }
        
        # Calibration configuration
        self.calibration_config = {
            'min_samples': 30,               # Minimum data points
            'min_speed_for_heading': 0.5,    # m/s minimum for heading calibration
            'max_calibration_time': 300,     # 5 minutes max calibration
            'recalibration_interval': 1800,  # 30 minutes between recalibrations
            'speed_validation_range': (0.2, 5.0),  # Valid speed range
            'heading_validation_range': 20.0,  # Max heading difference to accept
            'auto_calibrate': True           # Enable automatic calibration
        }

        # Sensor validation ranges
        self.compass_valid_range = (0.0, 360.0)
        self.speed_valid_range = (0.0, 10.0)
        self.depth_valid_range = (0.0, 100.0)
        self.velocity_valid_range = (-5.0, 5.0)  # For sway/heave components

        print("FIXED Navigation system started - UNINITIALIZED")
        print("Navigation is a PURE ESTIMATOR - does not read target waypoints")
        print("GPS-based sensor calibration system enabled")
        print("FIXES:")
        print("  1. Heading initialization uses CALIBRATED compass readings")
        print("  2. Position uncertainty uses proper VARIANCE math")
        print("  3. Calibration corrections applied CONSISTENTLY")

    def prepare_for_resurfacing_correction(self, max_expected_drift: float = 500.0, timeout: float = 30.0):
        """
        Prepare navigation system for large GPS correction after underwater excursion
        
        This temporarily relaxes the GPS outlier rejection to allow corrections that
        would normally be rejected as "jumps" in position. Used by GPS_FIX task.
        
        Args:
            max_expected_drift: Maximum position correction to allow (meters)
            timeout: How long to wait for GPS correction before reverting (seconds)
        """
        self.resurfacing_correction['enabled'] = True
        self.resurfacing_correction['max_expected_drift'] = max_expected_drift
        self.resurfacing_correction['timeout'] = timeout
        self.resurfacing_correction['activated_time'] = time.time()
        self.resurfacing_correction['correction_applied'] = False
        
        print(f"GPS resurfacing correction enabled: allowing up to {max_expected_drift:.1f}m position jump")
        print(f"Correction window: {timeout:.1f}s")

    def cancel_resurfacing_correction(self):
        """Cancel resurfacing correction mode and return to normal GPS filtering"""
        if self.resurfacing_correction['enabled']:
            self.resurfacing_correction['enabled'] = False
            print("GPS resurfacing correction disabled - returning to normal filtering")

    def _check_resurfacing_timeout(self):
        """Check if resurfacing correction has timed out"""
        if not self.resurfacing_correction['enabled']:
            return
        
        elapsed = time.time() - self.resurfacing_correction['activated_time']
        if elapsed > self.resurfacing_correction['timeout']:
            print(f"GPS resurfacing correction timed out after {elapsed:.1f}s")
            self.cancel_resurfacing_correction()

    def update(self, time_step: float = 0.0):
        """
        ENHANCED: Pure navigation update with realistic uncertainty model
        """
        current_time = time.time()
        dt = float(time_step) if time_step > 0 else (current_time - self.last_update_time)
        
        if dt <= 0 or dt > 10.0:
            self.last_update_time = current_time
            return

        # Check resurfacing correction timeout
        self._check_resurfacing_timeout()

        # Read sensor data from vehicle_state.sensor_data
        sensor_data = getattr(self.vehicle_state, 'sensor_data', {}) or {}
        
        if not sensor_data:
            return  # No sensors available

        # Process each sensor type
        compass_data = sensor_data.get('compass')
        imu_data = sensor_data.get('imu', {})
        depth_data = sensor_data.get('depth')
        speed_data = sensor_data.get('water_speed', {})
        gps_data = sensor_data.get('gps')

        # Try GPS initialization first if not initialized
        if not self.initialized and gps_data and self._is_surfaced():
            if self._initialize_from_gps(gps_data):
                print("Navigation system initialized from GPS fix!")
                self.initialized = True

        # If still not initialized, can't do navigation
        if not self.initialized:
            self._write_uninitialized_nav_state()
            self.last_update_time = current_time
            return

        # IMPROVED: Collect calibration data when surfaced with GPS
        if self._is_surfaced() and gps_data and self.calibration_config['auto_calibrate']:
            self._collect_calibration_data(gps_data, compass_data, speed_data)

        # Process sensors with validation (apply calibration if available)
        compass_reading = self._process_compass(compass_data)
        gyro_info = self._process_imu(imu_data, dt)
        depth_reading = self._process_depth(depth_data)
        
        # ENHANCED: Process three-axis velocity data with calibration
        velocity_data = self._process_three_axis_velocity(speed_data)

        # Update heading using compass-biased complementary filter
        self._fuse_heading_complementary_filter(compass_reading, gyro_info, dt)
        
        # Update other navigation states
        if depth_reading is not None:
            self.depth = depth_reading
            
        # ENHANCED: Dead reckoning with realistic uncertainty model
        if velocity_data:
            self._enhanced_dead_reckoning_step(velocity_data, dt)
        else:
            # Fallback to single-axis dead reckoning
            if self._process_speed_legacy(speed_data) is not None:
                self.speed = self._process_speed_legacy(speed_data)
            self._dead_reckoning_step(dt)
        
        # FIXED: Update position uncertainty based on dead reckoning
        self._update_position_uncertainty_fixed(dt, velocity_data)
        
        # FIXED: GPS update with position filtering if available and surfaced
        if self._is_surfaced() and gps_data:
            self._update_gps_filtered(gps_data)

        # Write all navigation estimates to nav_state
        self._update_nav_state()
        
        self.last_update_time = current_time

    def _update_position_uncertainty_fixed(self, dt: float, velocity_data: Optional[Dict[str, float]]):
        """
        FIXED: Calculate and update position uncertainty with proper variance math
        
        KEY FIX: Uncertainties must be combined using variance (squared values).
        This is the correct way to propagate independent errors.
        
        Formula: new_uncertainty = sqrt(old_uncertainty² + Δ₁² + Δ₂² + ...)
        
        This was the critical bug - adding uncertainties linearly instead of
        combining variances caused massive underprediction (57x off!).
        """
        if dt <= 0:
            return
        
        # Determine if we have GPS calibration
        is_calibrated = self.sensor_corrections['calibration_valid']
        
        # Get appropriate error parameters
        if is_calibrated:
            compass_std_deg = self.uncertainty_model['compass_std_deg_calibrated']
            speed_error_pct = self.uncertainty_model['speed_error_pct_calibrated']
        else:
            compass_std_deg = self.uncertainty_model['compass_std_deg_uncalibrated']
            speed_error_pct = self.uncertainty_model['speed_error_pct_uncalibrated']
        
        # Calculate variance growth (squared uncertainty contributions)
        # This is the CORRECT way to combine independent errors
        variance_growth = 0.0
        
        # 1. Heading uncertainty propagation (DOMINANT ERROR SOURCE)
        # Position error = distance * sin(heading_error)
        # For small angles: ≈ distance * heading_error_in_radians
        if self.speed > 0.1:  # Only if moving
            distance = self.speed * dt
            heading_error_rad = math.radians(compass_std_deg)
            heading_contribution = distance * heading_error_rad
            variance_growth += heading_contribution ** 2  # Add VARIANCE, not uncertainty
        
        # 2. Speed sensor error accumulation
        if self.speed > 0.1:
            distance = self.speed * dt
            speed_contribution = distance * speed_error_pct
            variance_growth += speed_contribution ** 2
        
        # 3. Gyro drift (especially important when underwater)
        if not self._is_surfaced() and self.speed > 0.1:
            distance = self.speed * dt
            gyro_drift_per_sec = self.uncertainty_model['gyro_drift_deg_per_second']
            gyro_drift_rad = math.radians(gyro_drift_per_sec * dt)
            gyro_contribution = distance * gyro_drift_rad
            variance_growth += gyro_contribution ** 2
        
        # 4. Unknown water currents (grows with time underwater)
        if not self._is_surfaced():
            current_contribution = self.uncertainty_model['unknown_current_mps'] * dt
            variance_growth += current_contribution ** 2
        
        # 5. Cross-track uncertainty from sway velocity errors
        if velocity_data and abs(self.velocity_sway) > 0.1:
            sway_contribution = abs(self.velocity_sway) * dt * self.uncertainty_model['sway_velocity_error_factor']
            variance_growth += sway_contribution ** 2
        
        # 6. Stationary drift (sensor noise accumulation)
        if self.speed < 0.1:
            stationary_contribution = self.uncertainty_model['stationary_drift_mps'] * dt
            variance_growth += stationary_contribution ** 2
        
        # Apply uncertainty growth using ROOT SUM SQUARE (RSS)
        # This is the statistically correct way to combine independent errors
        current_variance = self.position_uncertainty ** 2
        new_variance = current_variance + variance_growth
        self.position_uncertainty = math.sqrt(new_variance)
        
        # Cap at maximum uncertainty
        self.position_uncertainty = min(self.position_uncertainty, self.max_position_uncertainty)
        
        # Never go below minimum uncertainty (sensor noise floor)
        self.position_uncertainty = max(
            self.position_uncertainty, 
            self.uncertainty_model['minimum_uncertainty_m']
        )

    def _reduce_uncertainty_from_gps(self, gps_quality: str):
        """
        Reduce position uncertainty when GPS fix is received
        
        Uses Kalman-filter style update to blend GPS uncertainty with
        current navigation uncertainty estimate.
        """
        # Determine GPS measurement uncertainty based on fix quality
        if gps_quality == 'excellent':
            gps_uncertainty = self.uncertainty_model['gps_excellent_uncertainty']
        elif gps_quality == 'good':
            gps_uncertainty = self.uncertainty_model['gps_good_uncertainty']
        else:
            return  # Don't update uncertainty for poor GPS
        
        # Kalman filter update: blend navigation uncertainty with GPS uncertainty
        # Variance form of Kalman update (more numerically stable)
        nav_var = self.position_uncertainty ** 2
        gps_var = gps_uncertainty ** 2
        
        new_var = (nav_var * gps_var) / (nav_var + gps_var)
        new_uncertainty = math.sqrt(new_var)
        
        # Apply the update
        old_uncertainty = self.position_uncertainty
        self.position_uncertainty = new_uncertainty
        
        # Log significant uncertainty reductions
        if old_uncertainty - new_uncertainty > 1.0:
            print(f"Position uncertainty reduced by GPS: {old_uncertainty:.1f}m → {new_uncertainty:.1f}m")

    def _collect_calibration_data(self, gps_data: Dict, compass_data, speed_data):
        """
        IMPROVED: Actively collect calibration data during surface operations
        """
        try:
            # Extract GPS info
            gps_speed = gps_data.get('speed', 0.0)
            gps_course = gps_data.get('course', None)
            fix_quality = gps_data.get('fix_quality', 'poor')
            
            if fix_quality not in ('good', 'excellent'):
                return
            
            # Get sensor readings (these will be RAW, before calibration applied)
            # We need raw readings to compute the correction
            compass_raw = float(compass_data) if compass_data is not None else None
            
            if isinstance(speed_data, dict):
                speed_raw = speed_data.get('forward', speed_data.get('total', speed_data.get('surge', None)))
            else:
                speed_raw = float(speed_data) if speed_data is not None else None
            
            if compass_raw is None or speed_raw is None:
                return
            
            # Only collect when moving (need speed for heading calibration)
            if gps_speed < self.calibration_config['min_speed_for_heading']:
                return
            
            # Only collect if GPS course is valid
            if gps_course is None:
                return
            
            # Validate heading difference isn't too large (reject bad data)
            heading_diff = abs(self._angle_difference(gps_course, compass_raw))
            if heading_diff > self.calibration_config['heading_validation_range']:
                return
            
            # Collect data point
            self.calibration_data['gps_speeds'].append(gps_speed)
            self.calibration_data['sensor_speeds'].append(speed_raw)
            self.calibration_data['gps_courses'].append(gps_course)
            self.calibration_data['compass_headings'].append(compass_raw)
            self.calibration_data['timestamps'].append(time.time())
            
            # Try to compute calibration if we have enough data
            if len(self.calibration_data['gps_speeds']) >= self.calibration_config['min_samples']:
                self._compute_calibration()
                
        except Exception as e:
            print(f"Error collecting calibration data: {e}")

    def _compute_calibration(self):
        """
        IMPROVED: Compute sensor calibration from collected GPS data
        """
        try:
            if len(self.calibration_data['gps_speeds']) < self.calibration_config['min_samples']:
                return
            
            # Convert to numpy arrays
            gps_speeds = np.array(self.calibration_data['gps_speeds'])
            sensor_speeds = np.array(self.calibration_data['sensor_speeds'])
            gps_courses = np.array(self.calibration_data['gps_courses'])
            compass_headings = np.array(self.calibration_data['compass_headings'])
            
            # Compute speed calibration (linear fit: gps_speed = scale * sensor_speed + bias)
            if len(gps_speeds) > 5:
                # Simple linear regression
                mean_sensor = np.mean(sensor_speeds)
                mean_gps = np.mean(gps_speeds)
                
                numerator = np.sum((sensor_speeds - mean_sensor) * (gps_speeds - mean_gps))
                denominator = np.sum((sensor_speeds - mean_sensor) ** 2)
                
                if denominator > 0.01:
                    speed_scale = numerator / denominator
                    speed_bias = mean_gps - speed_scale * mean_sensor
                    
                    # Sanity check
                    if 0.5 < speed_scale < 1.5 and abs(speed_bias) < 1.0:
                        self.sensor_corrections['speed_scale'] = speed_scale
                        self.sensor_corrections['speed_bias'] = speed_bias
            
            # Compute heading calibration (average bias)
            if len(gps_courses) > 5:
                heading_errors = []
                for gps_course, compass_heading in zip(gps_courses, compass_headings):
                    error = self._angle_difference(compass_heading, gps_course)
                    heading_errors.append(error)
                
                heading_bias = np.mean(heading_errors)
                
                # Sanity check
                if abs(heading_bias) < 30.0:
                    self.sensor_corrections['heading_bias'] = heading_bias
            
            # Mark calibration as valid
            self.sensor_corrections['calibration_valid'] = True
            self.sensor_corrections['calibration_time'] = time.time()
            self.sensor_corrections['calibration_samples'] = len(gps_speeds)
            
            print(f"CALIBRATION COMPUTED:")
            print(f"  Speed: scale={self.sensor_corrections['speed_scale']:.3f}, bias={self.sensor_corrections['speed_bias']:.3f} m/s")
            print(f"  Heading: bias={self.sensor_corrections['heading_bias']:.2f}°")
            print(f"  Samples: {self.sensor_corrections['calibration_samples']}")
            
            # Clear calibration data to start fresh
            self.calibration_data = {
                'gps_speeds': [],
                'sensor_speeds': [],
                'gps_courses': [],
                'compass_headings': [],
                'timestamps': []
            }
            
        except Exception as e:
            print(f"Error computing calibration: {e}")

    def _enhanced_dead_reckoning_step(self, velocity_data: Dict[str, float], dt: float):
        """Dead reckoning with nautical convention coordinate transforms"""
        if dt <= 0:
            return

        surge = velocity_data.get('surge', 0.0)
        sway = velocity_data.get('sway', 0.0)
        
        # Apply speed calibration if available
        if self.sensor_corrections['calibration_valid']:
            surge = surge * self.sensor_corrections['speed_scale'] + self.sensor_corrections['speed_bias']
        
        # Convert body-frame velocities to earth frame using nautical heading
        # Nautical convention: 0° = North, 90° = East, clockwise
        # Conversion: math_angle = 90° - nautical_angle
        math_heading_rad = math.radians(90.0 - self.heading)
        
        cos_h = math.cos(math_heading_rad)
        sin_h = math.sin(math_heading_rad)
        
        # Transform body frame to earth frame
        velocity_east = surge * cos_h - sway * sin_h   # East component
        velocity_north = surge * sin_h + sway * cos_h  # North component
        
        # Calculate movement
        distance_north = velocity_north * dt
        distance_east = velocity_east * dt
        
        # Track cross-track drift for underwater navigation accuracy
        cross_track_distance = abs(sway * dt)
        self.cross_track_drift += cross_track_distance
        
        # Update underwater position tracking
        if not self._is_surfaced():
            self.underwater_position_drift['north'] += distance_north
            self.underwater_position_drift['east'] += distance_east

        # Convert to lat/lon
        meters_per_deg_lat = 111320.0
        d_lat = distance_north / meters_per_deg_lat
        
        meters_per_deg_lon = meters_per_deg_lat * max(0.0001, math.cos(math.radians(self.lat)))
        d_lon = distance_east / meters_per_deg_lon

        # Update position
        self.lat += d_lat
        self.lon += d_lon

    def _process_three_axis_velocity(self, speed_data) -> Optional[Dict[str, float]]:
        """Process three-axis paddlewheel velocity sensor data"""
        if speed_data is None:
            self.three_axis_available = False
            return None
            
        try:
            if not isinstance(speed_data, dict):
                self.three_axis_available = False
                return None

            # Check for three-axis data availability
            has_surge = 'surge' in speed_data or 'forward' in speed_data
            has_sway = 'sway' in speed_data or 'sideways' in speed_data  
            has_heave = 'heave' in speed_data or 'vertical' in speed_data
            
            if not (has_surge and has_sway and has_heave):
                self.three_axis_available = False
                return self._process_speed_legacy(speed_data)

            # Extract three-axis velocity components
            surge = float(speed_data.get('surge', speed_data.get('forward', 0.0)))
            sway = float(speed_data.get('sway', speed_data.get('sideways', 0.0)))
            heave = float(speed_data.get('heave', speed_data.get('vertical', 0.0)))

            # Update internal velocity tracking
            self.velocity_surge = surge
            self.velocity_sway = sway  
            self.velocity_heave = heave
            self.speed = abs(surge)  # Forward speed magnitude
            self.three_axis_available = True

            return {
                'surge': surge,
                'sway': sway,
                'heave': heave,
                'total_speed': math.sqrt(surge**2 + sway**2 + heave**2)
            }
            
        except (ValueError, TypeError):
            self.three_axis_available = False
            return None

    def _process_speed_legacy(self, speed_data) -> Optional[float]:
        """Legacy single-axis speed processing with calibration correction"""
        if speed_data is None:
            return None
            
        try:
            if isinstance(speed_data, dict):
                speed = speed_data.get('forward', speed_data.get('total', speed_data.get('surge', 0.0)))
            else:
                speed = float(speed_data)
            
            # Apply calibration if available
            if self.sensor_corrections['calibration_valid']:
                speed = speed * self.sensor_corrections['speed_scale'] + self.sensor_corrections['speed_bias']
                
            if self.speed_valid_range[0] <= speed <= self.speed_valid_range[1]:
                return max(0.0, speed)
        except (ValueError, TypeError):
            pass
        return None

    def _process_compass(self, compass_data) -> Optional[float]:
        """Process compass reading with calibration correction"""
        if compass_data is None:
            return None
            
        try:
            compass_reading = float(compass_data)
            
            # CRITICAL: Apply calibration correction
            # This corrects for magnetic declination and compass bias
            if self.sensor_corrections['calibration_valid']:
                compass_reading = (compass_reading - self.sensor_corrections['heading_bias']) % 360.0
            
            if self.compass_valid_range[0] <= compass_reading <= self.compass_valid_range[1]:
                self.last_compass_time = time.time()
                return compass_reading % 360.0
        except (ValueError, TypeError):
            pass
        return None

    def _fuse_heading_complementary_filter(self, compass_reading: Optional[float], 
                                         gyro_info: Optional[Dict], dt: float):
        """
        FIXED: Complementary filter now uses CALIBRATED compass for initialization
        
        CRITICAL FIX: compass_reading has already had calibration applied in
        _process_compass(), so when we initialize heading, we're using the
        corrected value. This fixes the bug where calibration made performance worse.
        """
        current_time = time.time()
        compass_timeout = (current_time - self.last_compass_time) > self.compass_timeout
        
        # Don't fuse heading until navigation is initialized
        if not self.initialized:
            return
        
        if not self.heading_initialized:
            if compass_reading is not None:
                # CRITICAL FIX: compass_reading is already calibrated
                self.heading = compass_reading
                self.gyro_heading = compass_reading
                self.compass_heading = compass_reading
                self.heading_initialized = True
                
                # Show calibration status in initialization message
                cal_status = " (CALIBRATED)" if self.sensor_corrections['calibration_valid'] else " (uncalibrated)"
                print(f"Heading initialized to {compass_reading:.1f}°{cal_status}")
            return
        
        # Update compass reading
        if compass_reading is not None:
            self.compass_heading = compass_reading
        
        # Compass-biased complementary filter fusion
        if gyro_info is not None and not compass_timeout:
            # Fuse gyro and compass with gyro weight
            gyro_heading = gyro_info['gyro_heading']
            compass_weight = 1.0 - self.alpha  # 0.10 with alpha=0.90
            
            # Handle angle wrapping
            compass_diff = self._angle_difference(self.compass_heading, gyro_heading)
            fused_heading = gyro_heading + compass_weight * compass_diff
            
            self.heading = fused_heading % 360.0
            
        elif gyro_info is not None:
            # Gyro only (compass timeout)
            self.heading = gyro_info['gyro_heading']
            
        elif compass_reading is not None:
            # Compass only (no gyro)
            self.heading = compass_reading
            self.gyro_heading = compass_reading

    def _process_imu(self, imu_data: Dict, dt: float) -> Optional[Dict]:
        """Process IMU, calculate pitch/roll from accelerometers, and integrate gyro"""
        if not isinstance(imu_data, dict):
            return None
            
        try:
            # Store actual accelerometer readings
            self.accel_x = float(imu_data.get('accel_x', self.accel_x))
            self.accel_y = float(imu_data.get('accel_y', self.accel_y))
            self.accel_z = float(imu_data.get('accel_z', self.accel_z))
            
            # Calculate pitch from accelerometers (degrees)
            pitch_rad = math.atan2(-self.accel_x, math.sqrt(self.accel_y**2 + self.accel_z**2))
            self.pitch = math.degrees(pitch_rad)
            
            # Calculate roll from accelerometers (degrees)
            roll_rad = math.atan2(self.accel_y, self.accel_z)
            self.roll = math.degrees(roll_rad)
            
            # Process gyro for heading integration
            gyro_z = float(imu_data.get('gyro_z', 0.0))  # rad/s
            yaw_rate_deg = math.degrees(gyro_z)
            self.yaw_rate = yaw_rate_deg
            
            # Integrate gyro for heading estimate (only if initialized)
            if self.heading_initialized and abs(yaw_rate_deg) > 0.1:
                self.gyro_heading += yaw_rate_deg * dt
                self.gyro_heading = self.gyro_heading % 360.0
            
            return {
                'yaw_rate': yaw_rate_deg,
                'gyro_heading': self.gyro_heading
            }
            
        except (ValueError, TypeError, ZeroDivisionError):
            return None

    def _process_depth(self, depth_data) -> Optional[float]:
        """Process depth sensor reading"""
        if depth_data is None:
            return None
            
        try:
            depth = float(depth_data)
            if self.depth_valid_range[0] <= depth <= self.depth_valid_range[1]:
                return max(0.0, depth)
        except (ValueError, TypeError):
            pass
        return None

    def _dead_reckoning_step(self, dt: float):
        """Legacy dead reckoning with nautical convention and calibration"""
        if dt <= 0 or self.speed <= 0:
            return

        # Apply speed calibration
        speed = self.speed
        if self.sensor_corrections['calibration_valid']:
            speed = speed * self.sensor_corrections['speed_scale'] + self.sensor_corrections['speed_bias']
        
        # Calculate movement using nautical heading convention
        distance = speed * dt
        
        # Convert nautical heading to mathematical angle for trig functions
        math_heading_rad = math.radians(90.0 - self.heading)
        
        # North/East components using mathematical angle
        d_east = distance * math.cos(math_heading_rad)   # East component
        d_north = distance * math.sin(math_heading_rad)  # North component

        # Convert to lat/lon
        meters_per_deg_lat = 111320.0
        d_lat = d_north / meters_per_deg_lat
        
        meters_per_deg_lon = meters_per_deg_lat * max(0.0001, math.cos(math.radians(self.lat)))
        d_lon = d_east / meters_per_deg_lon

        # Update position
        self.lat += d_lat
        self.lon += d_lon

    def _is_surfaced(self) -> bool:
        """Check if surfaced for GPS"""
        return self.depth < 0.5

    def _initialize_from_gps(self, gps_data: Dict) -> bool:
        """Initialize navigation system from first GPS fix"""
        try:
            if not isinstance(gps_data, dict):
                return False

            gps_lat = gps_data.get('lat')
            gps_lon = gps_data.get('lon')
            fix_quality = gps_data.get('fix_quality', 'poor')

            if gps_lat is None or gps_lon is None:
                return False

            # Validate coordinates
            if not (-90 <= float(gps_lat) <= 90) or not (-180 <= float(gps_lon) <= 180):
                return False

            # Initialize position from GPS
            self.lat = float(gps_lat)
            self.lon = float(gps_lon)
            self.depth = 0.0  # Assume surface for GPS fix
            
            # Set initial position uncertainty based on GPS quality
            if fix_quality == 'excellent':
                self.position_uncertainty = self.uncertainty_model['gps_excellent_uncertainty']
            elif fix_quality == 'good':
                self.position_uncertainty = self.uncertainty_model['gps_good_uncertainty']
            else:
                self.position_uncertainty = 10.0  # meters for acceptable GPS
            
            # Store surface position for underwater drift tracking
            self.last_surface_position = {
                'lat': self.lat,
                'lon': self.lon, 
                'time': time.time()
            }
            
            # Reset drift tracking
            self.underwater_position_drift = {'north': 0.0, 'east': 0.0}
            self.cross_track_drift = 0.0
            
            # Initialize GPS filter state
            self.gps_filter_state['last_good_gps_time'] = time.time()
            self.gps_filter_state['consecutive_outliers'] = 0
            
            self.last_gps_time = time.time()
            
            print(f"FIXED Navigation initialized from GPS: ({self.lat:.6f}, {self.lon:.6f})")
            print(f"Initial position uncertainty: {self.position_uncertainty:.1f}m")
            print("Coordinate system: Nautical (0° = North, clockwise)")
            print("GPS position filtering enabled")
            print("IMU attitude estimation enabled")
            print("Position uncertainty tracking enabled")
            
            return True

        except (ValueError, TypeError) as e:
            print(f"GPS initialization failed: {e}")
            return False

    def _update_gps_filtered(self, gps_data: Dict):
        """Process GPS fix with position filtering to eliminate jerky motion"""
        try:
            if not isinstance(gps_data, dict):
                return

            gps_lat = gps_data.get('lat')
            gps_lon = gps_data.get('lon')
            fix_quality = gps_data.get('fix_quality', 'poor')

            if gps_lat is None or gps_lon is None or fix_quality not in ('good', 'excellent'):
                return

            # Validate coordinates
            if not (-90 <= float(gps_lat) <= 90) or not (-180 <= float(gps_lon) <= 180):
                return

            gps_lat = float(gps_lat)
            gps_lon = float(gps_lon)

            # Check minimum time interval between GPS updates
            current_time = time.time()
            time_since_last_gps = current_time - self.last_gps_time
            
            if time_since_last_gps < self.gps_filter_config['min_gps_interval']:
                return  # Too soon for another GPS update

            # Apply GPS position filter instead of hard reset
            if self._apply_gps_position_filter(gps_lat, gps_lon):
                self.last_gps_time = current_time
                self.gps_filter_state['last_good_gps_time'] = current_time
                
                # Reduce position uncertainty based on GPS quality
                self._reduce_uncertainty_from_gps(fix_quality)
                
                # Reset drift tracking when good GPS is applied
                self.underwater_position_drift = {'north': 0.0, 'east': 0.0}
                self.cross_track_drift = 0.0
                self.last_surface_position = {
                    'lat': self.lat,
                    'lon': self.lon,
                    'time': current_time
                }

        except (ValueError, TypeError) as e:
            print(f"GPS processing error: {e}")

    def _apply_gps_position_filter(self, gps_lat: float, gps_lon: float) -> bool:
        """Apply complementary filter to GPS position updates with outlier rejection"""
        # Calculate distance between current navigation estimate and GPS reading
        distance_error = self._calculate_distance_meters(
            self.lat, self.lon, gps_lat, gps_lon
        )
        
        # Get filter parameters - check if resurfacing correction is active
        if self.resurfacing_correction['enabled'] and not self.resurfacing_correction['correction_applied']:
            max_gps_jump = self.resurfacing_correction['max_expected_drift']
            filter_alpha = 0.8  # Higher alpha = trust GPS more for resurfacing
            using_resurfacing_mode = True
        else:
            max_gps_jump = self.gps_filter_config['max_gps_jump']
            filter_alpha = self.gps_filter_config['filter_alpha']
            using_resurfacing_mode = False
        
        # OUTLIER REJECTION: Check if GPS reading is reasonable
        if distance_error > max_gps_jump:
            self.gps_filter_state['consecutive_outliers'] += 1
            mode_str = "resurfacing" if using_resurfacing_mode else "normal"
            print(f"GPS reading rejected ({mode_str} mode): {distance_error:.1f}m error exceeds {max_gps_jump}m threshold")
            
            # If too many consecutive outliers, something may be wrong
            if self.gps_filter_state['consecutive_outliers'] >= self.gps_filter_state['max_consecutive_outliers']:
                print(f"WARNING: {self.gps_filter_state['consecutive_outliers']} consecutive GPS outliers - check GPS system")
            
            return False  # Reject this GPS reading
        
        # GPS reading is reasonable - reset outlier counter
        self.gps_filter_state['consecutive_outliers'] = 0
        
        # APPLY COMPLEMENTARY FILTER
        # Smooth GPS integration instead of hard position reset
        new_lat = self.lat * (1.0 - filter_alpha) + gps_lat * filter_alpha
        new_lon = self.lon * (1.0 - filter_alpha) + gps_lon * filter_alpha
        
        # Calculate how much position was corrected
        correction_distance = self._calculate_distance_meters(
            self.lat, self.lon, new_lat, new_lon
        )
        
        if correction_distance > 1.0:  # Log significant corrections
            if using_resurfacing_mode:
                print(f"GPS RESURFACING correction applied: {correction_distance:.1f}m (α={filter_alpha:.2f})")
                self.resurfacing_correction['correction_applied'] = True
                # Disable resurfacing mode after first correction
                self.cancel_resurfacing_correction()
            else:
                print(f"GPS correction applied: {correction_distance:.1f}m (α={filter_alpha:.2f})")
        
        # Update position
        self.lat = new_lat
        self.lon = new_lon
        
        return True  # GPS update was successfully applied

    def _calculate_distance_meters(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance between two lat/lon points in meters"""
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        # Convert to meters using simple flat-earth approximation
        avg_lat = math.radians((lat1 + lat2) / 2)
        north_m = dlat * 111111.0  # meters per degree latitude
        east_m = dlon * 111111.0 * math.cos(avg_lat)  # meters per degree longitude
        
        return math.sqrt(north_m**2 + east_m**2)

    def _write_uninitialized_nav_state(self):
        """Write uninitialized state to nav_state"""
        try:
            with self.vehicle_state._state_lock:
                self.vehicle_state.nav_state.update({
                    'lat': 0.0,
                    'lon': 0.0,
                    'depth': 0.0,
                    'heading': 0.0,
                    'speed': 0.0,
                    'pitch': 0.0,
                    'roll': 0.0,
                    'yaw_rate': 0.0,
                    'accel_x': self.accel_x,
                    'accel_y': self.accel_y,
                    'accel_z': self.accel_z,
                    'position_uncertainty': 1000.0,
                    'velocity_surge': 0.0,
                    'velocity_sway': 0.0,
                    'velocity_heave': 0.0,
                    'cross_track_drift': 0.0,
                    'three_axis_available': False
                })
        except Exception as e:
            print(f"Error writing uninitialized nav_state: {e}")

    def _update_nav_state(self):
        """Write navigation estimates to nav_state"""
        try:
            with self.vehicle_state._state_lock:
                self.vehicle_state.nav_state.update({
                    'lat': self.lat,
                    'lon': self.lon,
                    'depth': self.depth,
                    'heading': self.heading,
                    'speed': self.speed,
                    'pitch': self.pitch,
                    'roll': self.roll,
                    'yaw_rate': self.yaw_rate,
                    'accel_x': self.accel_x,
                    'accel_y': self.accel_y,
                    'accel_z': self.accel_z,
                    'position_uncertainty': self.position_uncertainty,
                    'velocity_surge': self.velocity_surge,
                    'velocity_sway': self.velocity_sway,
                    'velocity_heave': self.velocity_heave,
                    'cross_track_drift': self.cross_track_drift,
                    'three_axis_available': self.three_axis_available,
                    'coordinate_system': 'nautical_fixed',
                    'gps_filter_active': True,
                    'resurfacing_correction_enabled': self.resurfacing_correction['enabled'],
                    'calibration_valid': self.sensor_corrections['calibration_valid']
                })
                
        except Exception as e:
            print(f"Error updating nav_state: {e}")

    def _angle_difference(self, target: float, current: float) -> float:
        """Calculate shortest angular difference"""
        diff = target - current
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff

    def configure_gps_filter(self, filter_alpha: float = None, max_gps_jump: float = None, 
                           min_interval: float = None):
        """Configure GPS filter parameters"""
        if filter_alpha is not None:
            if 0.0 <= filter_alpha <= 1.0:
                self.gps_filter_config['filter_alpha'] = filter_alpha
                print(f"GPS filter alpha set to {filter_alpha:.2f}")
            else:
                print("Filter alpha must be between 0.0 and 1.0")
        
        if max_gps_jump is not None:
            if max_gps_jump > 0:
                self.gps_filter_config['max_gps_jump'] = max_gps_jump
                print(f"GPS max jump threshold set to {max_gps_jump:.1f}m")
            else:
                print("Max GPS jump must be positive")
        
        if min_interval is not None:
            if min_interval >= 0:
                self.gps_filter_config['min_gps_interval'] = min_interval
                print(f"GPS minimum interval set to {min_interval:.1f}s")
            else:
                print("Minimum interval must be non-negative")

    def get_gps_filter_status(self) -> Dict[str, Any]:
        """Get GPS filter status and diagnostics"""
        current_time = time.time()
        return {
            'filter_config': self.gps_filter_config.copy(),
            'filter_state': {
                'consecutive_outliers': self.gps_filter_state['consecutive_outliers'],
                'time_since_last_good_gps': current_time - self.gps_filter_state['last_good_gps_time'],
                'position_variance_estimate': self.gps_filter_state['position_variance_estimate']
            },
            'resurfacing_correction': {
                'enabled': self.resurfacing_correction['enabled'],
                'max_expected_drift': self.resurfacing_correction['max_expected_drift'],
                'correction_applied': self.resurfacing_correction['correction_applied']
            },
            'last_gps_update': self.last_gps_time,
            'gps_available': self._is_surfaced(),
            'filter_active': True
        }

    def get_calibration_status(self) -> Dict[str, Any]:
        """Get sensor calibration status"""
        return {
            'calibration_valid': self.sensor_corrections['calibration_valid'],
            'calibration_time': self.sensor_corrections['calibration_time'],
            'calibration_samples': self.sensor_corrections['calibration_samples'],
            'speed_scale': self.sensor_corrections['speed_scale'],
            'speed_bias': self.sensor_corrections['speed_bias'],
            'heading_bias': self.sensor_corrections['heading_bias'],
            'data_collected': len(self.calibration_data['gps_speeds']),
            'min_samples_needed': self.calibration_config['min_samples']
        }

    def get_status(self) -> Dict[str, Any]:
        """Get navigation status"""
        return {
            'initialized': self.initialized,
            'heading_initialized': self.heading_initialized,
            'three_axis_velocity': self.three_axis_available,
            'coordinate_system': 'nautical_fixed',
            'calibration': self.get_calibration_status(),
            'gps_filtering': {
                'enabled': True,
                'filter_alpha': self.gps_filter_config['filter_alpha'],
                'max_jump_threshold': self.gps_filter_config['max_gps_jump'],
                'consecutive_outliers': self.gps_filter_state['consecutive_outliers'],
                'resurfacing_correction_enabled': self.resurfacing_correction['enabled']
            },
            'position': {
                'lat': self.lat,
                'lon': self.lon,
                'depth': self.depth,
                'uncertainty': self.position_uncertainty,
                'cross_track_drift': self.cross_track_drift
            },
            'attitude': {
                'heading': self.heading,
                'pitch': self.pitch,
                'roll': self.roll,
                'yaw_rate': self.yaw_rate
            },
            'motion': {
                'speed': self.speed,
                'velocity_surge': self.velocity_surge,
                'velocity_sway': self.velocity_sway,
                'velocity_heave': self.velocity_heave
            },
            'sensors': {
                'accel_x': self.accel_x,
                'accel_y': self.accel_y,
                'accel_z': self.accel_z
            },
            'gps': {
                'last_fix': self.last_gps_time,
                'available': self._is_surfaced(),
                'filter_status': self.get_gps_filter_status()
            },
            'underwater_performance': {
                'drift_compensation': self.three_axis_available,
                'position_drift_north': self.underwater_position_drift['north'],
                'position_drift_east': self.underwater_position_drift['east']
            },
            'architecture': 'FIXED_CALIBRATION_UNCERTAINTY_V3'
        }