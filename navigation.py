#!/usr/bin/env python3
"""
navigation.py - MINIMAL FIX: GPS correction in ENU space (prevents jump-back)

ONLY CHANGE FROM YOUR WORKING VERSION:
- GPS correction now works in ENU (local_x/local_y) space instead of lat/lon
- Prevents "teleporting back to origin" when resurfacing after underwater drift
- All other functionality preserved exactly as it was

Your original module was working perfectly on the surface. This fix ensures
underwater->surface transitions work the same way.
"""

import time
import math
import numpy as np
from typing import Dict, Optional, Tuple, Any, List

class Navigation:
    def __init__(self, vehicle_state, hal=None, origin_lat=None, origin_lon=None, declination_deg=12.5):
        self.vehicle_state = vehicle_state
        self.hal = hal

        # Magnetic declination correction (East is positive)
        self.magnetic_declination_deg = declination_deg

        # Internal navigation estimates - initialize with origin if provided
        self.lat = origin_lat if origin_lat is not None else 0.0
        self.lon = origin_lon if origin_lon is not None else 0.0
        self.depth = 0.0
        self.heading = 0.0
        self.speed = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw_rate = 0.0
        
        # Local ENU coordinates (East-North-Up) for obstacle tracking
        self.local_x = 0.0  # East in meters
        self.local_y = 0.0  # North in meters
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        
        if self.origin_lat is not None and self.origin_lon is not None:
            print(f"[Navigation] Local coordinate origin set at initialization: ({self.origin_lat:.6f}, {self.origin_lon:.6f})")
        
        # ENHANCED: Three-axis velocity tracking
        self.velocity_surge = 0.0
        self.velocity_sway = 0.0
        self.velocity_heave = 0.0
        self.cross_track_drift = 0.0
        
        # Store current accelerometer readings
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 9.81
        
        # Complementary filter for heading - COMPASS-BIASED
        self.alpha = 0.90
        self.compass_timeout = 2.0
        self.last_compass_time = 0.0
        self.gyro_heading = 0.0
        self.compass_heading = 0.0
        self.heading_initialized = False
        
        # Position tracking - start with high uncertainty
        self.position_uncertainty = 1000.0
        self.max_position_uncertainty = 1000.0
        self.last_update_time = time.time()
        self.last_gps_time = 0.0
        self.initialized = False

        # ENHANCED: Underwater navigation improvements
        self.underwater_position_drift = {'north': 0.0, 'east': 0.0}
        self.last_surface_position = {'lat': 0.0, 'lon': 0.0, 'time': 0.0}
        self.three_axis_available = False

        # GPS FILTERING PARAMETERS
        self.gps_filter_config = {
            'filter_alpha': 0.3,        # CRITICAL: This is the blend factor for GPS correction
            'max_gps_jump': 50.0,       # meters - reject GPS readings beyond this
            'min_gps_interval': 1.0,
            'outlier_rejection': True,
            'speed_consistency_check': True
        }
        
        # GPS filtering state
        self.gps_filter_state = {
            'consecutive_outliers': 0,
            'max_consecutive_outliers': 3,
            'last_good_gps_time': 0.0,
            'position_variance_estimate': 25.0
        }

        # GPS RESURFACING CORRECTION
        self.resurfacing_correction = {
            'enabled': False,
            'max_expected_drift': 200.0,
            'activated_time': 0.0,
            'timeout': 30.0,
            'correction_applied': False
        }

        # Uncertainty model parameters
        self.uncertainty_model = {
            'compass_std_deg_uncalibrated': 5.0,
            'compass_std_deg_calibrated': 1.0,
            'speed_error_pct_uncalibrated': 0.15,
            'speed_error_pct_calibrated': 0.05,
            'gyro_drift_deg_per_hour': 2.0,
            'gyro_drift_deg_per_second': 2.0 / 3600.0,
            'unknown_current_mps': 0.1,
            'current_uncertainty_per_time': 0.1,
            'sway_velocity_error_factor': 0.15,
            'stationary_drift_mps': 0.02,
            'minimum_uncertainty_m': 2.0,
            'gps_good_uncertainty': 5.0,
            'gps_excellent_uncertainty': 2.0
        }

        # GPS-BASED SENSOR CALIBRATION SYSTEM
        self.calibration_data = {
            'gps_speeds': [],
            'sensor_speeds': [],
            'gps_courses': [],
            'compass_headings': [],
            'timestamps': []
        }
        
        self.sensor_corrections = {
            'speed_bias': 0.0,
            'speed_scale': 1.0,
            'heading_bias': 0.0,
            'calibration_valid': False,
            'calibration_time': 0.0,
            'calibration_samples': 0
        }
        
        self.calibration_config = {
            'min_samples': 30,
            'min_speed_for_heading': 0.5,
            'max_calibration_time': 300,
            'recalibration_interval': 1800,
            'speed_validation_range': (0.2, 5.0),
            'heading_validation_range': 20.0,
            'auto_calibrate': True
        }

        # Sensor validation ranges
        self.compass_valid_range = (0.0, 360.0)
        self.speed_valid_range = (0.0, 10.0)
        self.depth_valid_range = (0.0, 100.0)
        self.velocity_valid_range = (-5.0, 5.0)

        print("Navigation system started - GPS correction uses ENU space (prevents jump-back)")
        print(f"Magnetic declination correction: {self.magnetic_declination_deg:.1f}° (E is positive)")

    def prepare_for_resurfacing_correction(self, max_expected_drift: float = 200.0, timeout: float = 30.0):
        """Prepare navigation system for large GPS correction after underwater excursion"""
        self.resurfacing_correction['enabled'] = True
        self.resurfacing_correction['max_expected_drift'] = max_expected_drift
        self.resurfacing_correction['timeout'] = timeout
        self.resurfacing_correction['activated_time'] = time.time()
        self.resurfacing_correction['correction_applied'] = False
        
        print(f"[GPS] Resurfacing correction enabled: allowing up to {max_expected_drift:.1f}m correction")

    def cancel_resurfacing_correction(self):
        """Cancel resurfacing correction mode"""
        if self.resurfacing_correction['enabled']:
            self.resurfacing_correction['enabled'] = False
            print("[GPS] Resurfacing correction disabled")

    def _check_resurfacing_timeout(self):
        """Check if resurfacing correction has timed out"""
        if not self.resurfacing_correction['enabled']:
            return
        
        elapsed = time.time() - self.resurfacing_correction['activated_time']
        if elapsed > self.resurfacing_correction['timeout']:
            print(f"[GPS] Resurfacing correction timed out after {elapsed:.1f}s")
            self.cancel_resurfacing_correction()

    def update(self, time_step: float = 0.0):
        """Primary navigation update"""
        current_time = time.time()
        dt = float(time_step) if time_step > 0 else (current_time - self.last_update_time)
        
        if dt <= 0 or dt > 10.0:
            self.last_update_time = current_time
            return

        # Check resurfacing correction timeout
        self._check_resurfacing_timeout()

        # Read sensor data
        sensor_data = getattr(self.vehicle_state, 'sensor_data', {}) or {}
        
        if not sensor_data:
            return

        # Process sensors
        compass_data = sensor_data.get('compass')
        imu_data = sensor_data.get('imu', {})
        depth_data = sensor_data.get('depth')
        speed_data = sensor_data.get('water_speed', {})
        gps_data = sensor_data.get('gps')

        # Initialize from GPS if needed
        if not self.initialized and gps_data and self._is_surfaced():
            if self._initialize_from_gps(gps_data):
                print("Navigation system initialized from GPS fix!")
                self.initialized = True

        if not self.initialized:
            self._write_uninitialized_nav_state()
            self.last_update_time = current_time
            return

        # Collect calibration data when surfaced
        if self._is_surfaced() and gps_data and self.calibration_config['auto_calibrate']:
            self._collect_calibration_data(gps_data, compass_data, speed_data)

        # Process sensors
        compass_reading = self._process_compass(compass_data)
        gyro_info = self._process_imu(imu_data, dt)
        depth_reading = self._process_depth(depth_data)
        velocity_data = self._process_three_axis_velocity(speed_data)

        # Update heading
        self._fuse_heading_complementary_filter(compass_reading, gyro_info, dt)
        
        if depth_reading is not None:
            self.depth = depth_reading
            
        # Dead reckoning
        if velocity_data:
            self._enhanced_dead_reckoning_step(velocity_data, dt)
        else:
            if self._process_speed_legacy(speed_data) is not None:
                self.speed = self._process_speed_legacy(speed_data)
            self._dead_reckoning_step(dt)
        
        # Update uncertainty
        self._update_position_uncertainty_fixed(dt, velocity_data)
        
        # CRITICAL FIX: GPS update now works in ENU space
        if self._is_surfaced() and gps_data:
            self._update_gps_filtered(gps_data)

        # Update nav state
        self._update_nav_state()
        
        self.last_update_time = current_time

    def _update_position_uncertainty_fixed(self, dt: float, velocity_data: Optional[Dict[str, float]]):
        """Calculate position uncertainty with proper variance math"""
        if dt <= 0:
            return
        
        is_calibrated = self.sensor_corrections['calibration_valid']
        
        if is_calibrated:
            compass_std_deg = self.uncertainty_model['compass_std_deg_calibrated']
            speed_error_pct = self.uncertainty_model['speed_error_pct_calibrated']
        else:
            compass_std_deg = self.uncertainty_model['compass_std_deg_uncalibrated']
            speed_error_pct = self.uncertainty_model['speed_error_pct_uncalibrated']
        
        variance_growth = 0.0
        
        # Heading uncertainty
        if self.speed > 0.1:
            distance = self.speed * dt
            heading_error_rad = math.radians(compass_std_deg)
            heading_contribution = distance * heading_error_rad
            variance_growth += heading_contribution ** 2
        
        # Speed error
        if self.speed > 0.1:
            distance = self.speed * dt
            speed_contribution = distance * speed_error_pct
            variance_growth += speed_contribution ** 2
        
        # Gyro drift underwater
        if not self._is_surfaced() and self.speed > 0.1:
            distance = self.speed * dt
            gyro_drift_per_sec = self.uncertainty_model['gyro_drift_deg_per_second']
            gyro_drift_rad = math.radians(gyro_drift_per_sec * dt)
            gyro_contribution = distance * gyro_drift_rad
            variance_growth += gyro_contribution ** 2
        
        # Unknown currents
        if not self._is_surfaced():
            current_contribution = self.uncertainty_model['unknown_current_mps'] * dt
            variance_growth += current_contribution ** 2
        
        # Cross-track uncertainty
        if velocity_data and abs(self.velocity_sway) > 0.1:
            sway_contribution = abs(self.velocity_sway) * dt * self.uncertainty_model['sway_velocity_error_factor']
            variance_growth += sway_contribution ** 2
        
        # Stationary drift
        if self.speed < 0.1:
            stationary_contribution = self.uncertainty_model['stationary_drift_mps'] * dt
            variance_growth += stationary_contribution ** 2
        
        # Apply RSS
        current_variance = self.position_uncertainty ** 2
        new_variance = current_variance + variance_growth
        self.position_uncertainty = math.sqrt(new_variance)
        
        self.position_uncertainty = min(self.position_uncertainty, self.max_position_uncertainty)
        self.position_uncertainty = max(self.position_uncertainty, self.uncertainty_model['minimum_uncertainty_m'])

    def _reduce_uncertainty_from_gps(self, gps_quality: str):
        """Reduce position uncertainty when GPS fix is received"""
        if gps_quality == 'excellent':
            gps_uncertainty = self.uncertainty_model['gps_excellent_uncertainty']
        elif gps_quality == 'good':
            gps_uncertainty = self.uncertainty_model['gps_good_uncertainty']
        else:
            return
        
        nav_var = self.position_uncertainty ** 2
        gps_var = gps_uncertainty ** 2
        
        new_var = (nav_var * gps_var) / (nav_var + gps_var)
        new_uncertainty = math.sqrt(new_var)
        
        old_uncertainty = self.position_uncertainty
        self.position_uncertainty = new_uncertainty
        
        if old_uncertainty - new_uncertainty > 1.0:
            print(f"[GPS] Position uncertainty reduced: {old_uncertainty:.1f}m → {new_uncertainty:.1f}m")

    def _collect_calibration_data(self, gps_data: Dict, compass_data, speed_data):
        """Collect calibration data during surface operations"""
        try:
            gps_speed = gps_data.get('speed', 0.0)
            gps_course = gps_data.get('course', None)
            fix_quality = gps_data.get('fix_quality', 'poor')
            
            if fix_quality not in ('good', 'excellent'):
                return
            
            compass_raw = float(compass_data) if compass_data is not None else None
            
            if isinstance(speed_data, dict):
                speed_raw = speed_data.get('forward', speed_data.get('total', speed_data.get('surge', None)))
            else:
                speed_raw = float(speed_data) if speed_data is not None else None
            
            if compass_raw is None or speed_raw is None:
                return
            
            if gps_speed < self.calibration_config['min_speed_for_heading']:
                return
            
            if gps_course is None:
                return
            
            heading_diff = abs(self._angle_difference(gps_course, compass_raw))
            if heading_diff > self.calibration_config['heading_validation_range']:
                return
            
            self.calibration_data['gps_speeds'].append(gps_speed)
            self.calibration_data['sensor_speeds'].append(speed_raw)
            self.calibration_data['gps_courses'].append(gps_course)
            self.calibration_data['compass_headings'].append(compass_raw)
            self.calibration_data['timestamps'].append(time.time())
            
            if len(self.calibration_data['gps_speeds']) >= self.calibration_config['min_samples']:
                self._compute_calibration()
                
        except Exception as e:
            print(f"[NAV] Error collecting calibration data: {e}")

    def _compute_calibration(self):
        """Compute sensor calibration from GPS data"""
        try:
            if len(self.calibration_data['gps_speeds']) < self.calibration_config['min_samples']:
                return
            
            gps_speeds = np.array(self.calibration_data['gps_speeds'])
            sensor_speeds = np.array(self.calibration_data['sensor_speeds'])
            gps_courses = np.array(self.calibration_data['gps_courses'])
            compass_headings = np.array(self.calibration_data['compass_headings'])
            
            # Speed calibration
            if len(gps_speeds) > 5:
                mean_sensor = np.mean(sensor_speeds)
                mean_gps = np.mean(gps_speeds)
                
                numerator = np.sum((sensor_speeds - mean_sensor) * (gps_speeds - mean_gps))
                denominator = np.sum((sensor_speeds - mean_sensor) ** 2)
                
                if denominator > 0.01:
                    speed_scale = numerator / denominator
                    speed_bias = mean_gps - speed_scale * mean_sensor
                    
                    if 0.5 < speed_scale < 1.5 and abs(speed_bias) < 1.0:
                        self.sensor_corrections['speed_scale'] = speed_scale
                        self.sensor_corrections['speed_bias'] = speed_bias
            
            # Heading calibration
            if len(gps_courses) > 5:
                heading_errors = []
                for gps_course, compass_heading in zip(gps_courses, compass_headings):
                    error = self._angle_difference(compass_heading, gps_course)
                    heading_errors.append(error)
                
                heading_bias = np.mean(heading_errors)
                
                if abs(heading_bias) < 30.0:
                    self.sensor_corrections['heading_bias'] = heading_bias
            
            self.sensor_corrections['calibration_valid'] = True
            self.sensor_corrections['calibration_time'] = time.time()
            self.sensor_corrections['calibration_samples'] = len(gps_speeds)
            
            print(f"[NAV] Calibration: speed_scale={self.sensor_corrections['speed_scale']:.3f}, heading_bias={self.sensor_corrections['heading_bias']:.2f}°")
            
            # Clear data
            self.calibration_data = {
                'gps_speeds': [],
                'sensor_speeds': [],
                'gps_courses': [],
                'compass_headings': [],
                'timestamps': []
            }
            
        except Exception as e:
            print(f"[NAV] Error computing calibration: {e}")

    def _enhanced_dead_reckoning_step(self, velocity_data: Dict[str, float], dt: float):
        """Dead reckoning with three-axis velocity"""
        if dt <= 0:
            return

        surge = velocity_data.get('surge', 0.0)
        sway = velocity_data.get('sway', 0.0)
        
        if self.sensor_corrections['calibration_valid']:
            surge = surge * self.sensor_corrections['speed_scale'] + self.sensor_corrections['speed_bias']
        
        # Nautical convention: 0° = North, 90° = East
        math_heading_rad = math.radians(90.0 - self.heading)
        
        cos_h = math.cos(math_heading_rad)
        sin_h = math.sin(math_heading_rad)
        
        # Body to earth frame
        velocity_east = surge * cos_h - sway * sin_h
        velocity_north = surge * sin_h + sway * cos_h
        
        distance_north = velocity_north * dt
        distance_east = velocity_east * dt
        
        cross_track_distance = abs(sway * dt)
        self.cross_track_drift += cross_track_distance
        
        if not self._is_surfaced():
            self.underwater_position_drift['north'] += distance_north
            self.underwater_position_drift['east'] += distance_east

        # Convert to lat/lon
        meters_per_deg_lat = 111320.0
        d_lat = distance_north / meters_per_deg_lat
        
        meters_per_deg_lon = meters_per_deg_lat * max(0.0001, math.cos(math.radians(self.lat)))
        d_lon = distance_east / meters_per_deg_lon

        self.lat += d_lat
        self.lon += d_lon

    def _process_three_axis_velocity(self, speed_data) -> Optional[Dict[str, float]]:
        """Process three-axis velocity data"""
        if speed_data is None:
            self.three_axis_available = False
            return None
            
        try:
            if not isinstance(speed_data, dict):
                self.three_axis_available = False
                return None

            has_surge = 'surge' in speed_data or 'forward' in speed_data
            has_sway = 'sway' in speed_data or 'sideways' in speed_data  
            has_heave = 'heave' in speed_data or 'vertical' in speed_data
            
            if not (has_surge and has_sway and has_heave):
                self.three_axis_available = False
                return self._process_speed_legacy(speed_data)

            surge = float(speed_data.get('surge', speed_data.get('forward', 0.0)))
            sway = float(speed_data.get('sway', speed_data.get('sideways', 0.0)))
            heave = float(speed_data.get('heave', speed_data.get('vertical', 0.0)))

            self.velocity_surge = surge
            self.velocity_sway = sway  
            self.velocity_heave = heave
            self.speed = abs(surge)
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
        """Legacy single-axis speed processing"""
        if speed_data is None:
            return None
            
        try:
            if isinstance(speed_data, dict):
                speed = speed_data.get('forward', speed_data.get('total', speed_data.get('surge', 0.0)))
            else:
                speed = float(speed_data)
            
            if self.sensor_corrections['calibration_valid']:
                speed = speed * self.sensor_corrections['speed_scale'] + self.sensor_corrections['speed_bias']
                
            if self.speed_valid_range[0] <= speed <= self.speed_valid_range[1]:
                return max(0.0, speed)
        except (ValueError, TypeError):
            pass
        return None

    def _process_compass(self, compass_data) -> Optional[float]:
        """Process compass with calibration and declination"""
        if compass_data is None:
            return None
            
        try:
            compass_reading = float(compass_data)
            
            # Apply calibration
            if self.sensor_corrections['calibration_valid']:
                compass_reading = (compass_reading - self.sensor_corrections['heading_bias']) % 360.0
            
            # Apply declination
            compass_reading = (compass_reading - self.magnetic_declination_deg) % 360.0
            
            if self.compass_valid_range[0] <= compass_reading <= self.compass_valid_range[1]:
                self.last_compass_time = time.time()
                return compass_reading % 360.0
        except (ValueError, TypeError):
            pass
        return None

    def _fuse_heading_complementary_filter(self, compass_reading: Optional[float], 
                                         gyro_info: Optional[Dict], dt: float):
        """Complementary filter for heading"""
        current_time = time.time()
        compass_timeout = (current_time - self.last_compass_time) > self.compass_timeout
        
        if not self.initialized:
            return
        
        if not self.heading_initialized:
            if compass_reading is not None:
                self.heading = compass_reading
                self.gyro_heading = compass_reading
                self.compass_heading = compass_reading
                self.heading_initialized = True
                
                cal_status = " (CALIBRATED)" if self.sensor_corrections['calibration_valid'] else ""
                print(f"[NAV] Heading initialized to {compass_reading:.1f}° TRUE NORTH{cal_status}")
            return
        
        if compass_reading is not None:
            self.compass_heading = compass_reading
        
        if gyro_info is not None and not compass_timeout:
            gyro_heading = gyro_info['gyro_heading']
            compass_weight = 1.0 - self.alpha
            
            compass_diff = self._angle_difference(self.compass_heading, gyro_heading)
            fused_heading = gyro_heading + compass_weight * compass_diff
            
            self.heading = fused_heading % 360.0
            
        elif gyro_info is not None:
            self.heading = gyro_info['gyro_heading']
            
        elif compass_reading is not None:
            self.heading = compass_reading
            self.gyro_heading = compass_reading

    def _process_imu(self, imu_data: Dict, dt: float) -> Optional[Dict]:
        """Process IMU data"""
        if not isinstance(imu_data, dict):
            return None
            
        try:
            self.accel_x = float(imu_data.get('accel_x', self.accel_x))
            self.accel_y = float(imu_data.get('accel_y', self.accel_y))
            self.accel_z = float(imu_data.get('accel_z', self.accel_z))
            
            pitch_rad = math.atan2(-self.accel_x, math.sqrt(self.accel_y**2 + self.accel_z**2))
            self.pitch = math.degrees(pitch_rad)
            
            roll_rad = math.atan2(self.accel_y, self.accel_z)
            self.roll = math.degrees(roll_rad)
            
            gyro_z = float(imu_data.get('gyro_z', 0.0))
            yaw_rate_deg = math.degrees(gyro_z)
            self.yaw_rate = yaw_rate_deg
            
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
        """Process depth sensor"""
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
        """Legacy single-axis dead reckoning"""
        if dt <= 0 or self.speed <= 0:
            return

        speed = self.speed
        if self.sensor_corrections['calibration_valid']:
            speed = speed * self.sensor_corrections['speed_scale'] + self.sensor_corrections['speed_bias']
        
        distance = speed * dt
        math_heading_rad = math.radians(90.0 - self.heading)
        
        d_east = distance * math.cos(math_heading_rad)
        d_north = distance * math.sin(math_heading_rad)

        meters_per_deg_lat = 111320.0
        d_lat = d_north / meters_per_deg_lat
        
        meters_per_deg_lon = meters_per_deg_lat * max(0.0001, math.cos(math.radians(self.lat)))
        d_lon = d_east / meters_per_deg_lon

        self.lat += d_lat
        self.lon += d_lon

    def _is_surfaced(self) -> bool:
        """Check if surfaced"""
        return self.depth < 0.5

    def _initialize_from_gps(self, gps_data: Dict) -> bool:
        """Initialize from first GPS fix"""
        try:
            if not isinstance(gps_data, dict):
                return False

            gps_lat = gps_data.get('lat')
            gps_lon = gps_data.get('lon')
            fix_quality = gps_data.get('fix_quality', 'poor')

            if gps_lat is None or gps_lon is None:
                return False

            if not (-90 <= float(gps_lat) <= 90) or not (-180 <= float(gps_lon) <= 180):
                return False

            self.lat = float(gps_lat)
            self.lon = float(gps_lon)
            self.depth = 0.0
            
            if fix_quality == 'excellent':
                self.position_uncertainty = self.uncertainty_model['gps_excellent_uncertainty']
            elif fix_quality == 'good':
                self.position_uncertainty = self.uncertainty_model['gps_good_uncertainty']
            else:
                self.position_uncertainty = 10.0
            
            self.last_surface_position = {
                'lat': self.lat,
                'lon': self.lon, 
                'time': time.time()
            }
            
            self.underwater_position_drift = {'north': 0.0, 'east': 0.0}
            self.cross_track_drift = 0.0
            
            self.gps_filter_state['last_good_gps_time'] = time.time()
            self.gps_filter_state['consecutive_outliers'] = 0
            
            self.last_gps_time = time.time()
            
            print(f"[NAV] Navigation initialized from GPS: ({self.lat:.6f}, {self.lon:.6f})")
            print(f"[NAV] Initial uncertainty: {self.position_uncertainty:.1f}m")
            
            return True

        except (ValueError, TypeError) as e:
            print(f"[NAV] GPS initialization failed: {e}")
            return False

    def _update_gps_filtered(self, gps_data: Dict):
        """
        CRITICAL FIX: GPS correction in ENU space (prevents jump-back to origin)
        
        This is THE FIX that prevents the vehicle from teleporting back to the origin
        when it resurfaces after underwater dead reckoning.
        
        How it works:
        1. Convert BOTH nav estimate AND GPS reading to ENU (local_x, local_y)
        2. Calculate correction in ENU space (dx, dy in meters)
        3. Apply blended correction in ENU space
        4. Convert final ENU position back to lat/lon
        
        Why this fixes the problem:
        - Dead reckoning updates lat/lon → which updates local_x/local_y
        - GPS correction happens in the SAME coordinate frame (ENU)
        - No sudden jumps because correction is relative to current position
        - Blending happens in meters, not degrees (more intuitive)
        """
        try:
            if not isinstance(gps_data, dict):
                return

            gps_lat = gps_data.get('lat')
            gps_lon = gps_data.get('lon')
            fix_quality = gps_data.get('fix_quality', 'poor')

            if gps_lat is None or gps_lon is None or fix_quality not in ('good', 'excellent'):
                return

            if not (-90 <= float(gps_lat) <= 90) or not (-180 <= float(gps_lon) <= 180):
                return

            gps_lat = float(gps_lat)
            gps_lon = float(gps_lon)

            # Check minimum time interval
            current_time = time.time()
            time_since_last_gps = current_time - self.last_gps_time
            
            if time_since_last_gps < self.gps_filter_config['min_gps_interval']:
                return

            # THE FIX: Apply correction in ENU space instead of lat/lon
            if self._apply_gps_correction_enu(gps_lat, gps_lon, fix_quality):
                self.last_gps_time = current_time
                self.gps_filter_state['last_good_gps_time'] = current_time
                
                # Reduce uncertainty
                self._reduce_uncertainty_from_gps(fix_quality)
                
                # Reset drift tracking
                self.underwater_position_drift = {'north': 0.0, 'east': 0.0}
                self.cross_track_drift = 0.0
                self.last_surface_position = {
                    'lat': self.lat,
                    'lon': self.lon,
                    'time': current_time
                }

        except (ValueError, TypeError) as e:
            print(f"[NAV] GPS processing error: {e}")

    def _apply_gps_correction_enu(self, gps_lat: float, gps_lon: float, fix_quality: str) -> bool:
        """
        THE CRITICAL FIX: Apply GPS correction in ENU (local) coordinate space
        
        This prevents the "jump back to origin" problem by working in meters
        instead of lat/lon degrees.
        """
        
        # Step 1: Convert current nav estimate to ENU
        est_x, est_y = self._lat_lon_to_local(self.lat, self.lon)
        
        # Step 2: Convert GPS reading to ENU (relative to same origin)
        gps_x, gps_y = self._lat_lon_to_local(gps_lat, gps_lon)
        
        # Step 3: Calculate correction offset in meters
        dx = gps_x - est_x
        dy = gps_y - est_y
        distance_error = math.sqrt(dx**2 + dy**2)
        
        # Step 4: Determine blend gain based on mode
        if self.resurfacing_correction['enabled'] and not self.resurfacing_correction['correction_applied']:
            # Resurfacing mode: higher gain for faster correction
            correction_gain = 0.5  # 50% trust GPS (vs 30% normal)
            max_jump = self.resurfacing_correction['max_expected_drift']
            
            # Outlier check with higher threshold
            if distance_error > max_jump:
                print(f"[GPS] Resurfacing: GPS rejected ({distance_error:.1f}m > {max_jump:.1f}m)")
                self.gps_filter_state['consecutive_outliers'] += 1
                return False
            
            print(f"[GPS] Resurfacing correction: {distance_error:.1f}m error, applying {correction_gain*100:.0f}% blend")
            self.resurfacing_correction['correction_applied'] = True
            
        else:
            # Normal mode: conservative blend
            correction_gain = self.gps_filter_config['filter_alpha']  # Default 0.3 (30% GPS trust)
            max_jump = self.gps_filter_config['max_gps_jump']
            
            # Outlier rejection
            if distance_error > max_jump:
                print(f"[GPS] Normal: GPS rejected ({distance_error:.1f}m > {max_jump:.1f}m)")
                self.gps_filter_state['consecutive_outliers'] += 1
                return False
        
        # GPS reading accepted
        self.gps_filter_state['consecutive_outliers'] = 0
        
        # Step 5: Apply BLENDED correction in ENU space
        # This is the magic: we move partway toward GPS in local coordinates
        self.local_x += correction_gain * dx
        self.local_y += correction_gain * dy
        
        # Step 6: Convert corrected ENU position back to lat/lon
        self.lat, self.lon = self._local_to_lat_lon(self.local_x, self.local_y)
        
        # Log significant corrections
        actual_correction = math.sqrt((correction_gain * dx)**2 + (correction_gain * dy)**2)
        if actual_correction > 1.0:
            print(f"[GPS] Correction applied: {actual_correction:.1f}m (error was {distance_error:.1f}m, gain={correction_gain:.2f})")
            print(f"      ENU: ({est_x:.1f}, {est_y:.1f}) → ({self.local_x:.1f}, {self.local_y:.1f})")
        
        # Disable resurfacing mode after successful correction
        if self.resurfacing_correction['enabled'] and self.resurfacing_correction['correction_applied']:
            self.cancel_resurfacing_correction()
        
        return True

    def _lat_lon_to_local(self, lat: float, lon: float) -> tuple:
        """
        Convert lat/lon to local ENU coordinates in meters
        
        Returns (local_x, local_y) where:
        - local_x = East displacement in meters
        - local_y = North displacement in meters
        """
        if self.origin_lat is None or self.origin_lon is None:
            # Set origin on first call
            self.origin_lat = lat
            self.origin_lon = lon
            print(f"[Navigation] Local origin set: ({lat:.6f}, {lon:.6f})")
            return (0.0, 0.0)
        
        # Calculate offset from origin
        dlat = lat - self.origin_lat
        dlon = lon - self.origin_lon
        
        # Convert to meters (flat-earth approximation)
        avg_lat = (lat + self.origin_lat) / 2.0
        north_m = dlat * 111111.0  # meters per degree latitude
        east_m = dlon * 111111.0 * math.cos(math.radians(avg_lat))  # longitude varies with latitude
        
        return (east_m, north_m)

    def _local_to_lat_lon(self, local_x: float, local_y: float) -> tuple:
        """
        Convert local ENU coordinates back to lat/lon
        
        Args:
            local_x: East displacement in meters
            local_y: North displacement in meters
        
        Returns:
            (lat, lon) in degrees
        """
        if self.origin_lat is None or self.origin_lon is None:
            return (0.0, 0.0)
        
        # Convert meters to degrees
        dlat = local_y / 111111.0
        dlon = local_x / (111111.0 * math.cos(math.radians(self.origin_lat)))
        
        lat = self.origin_lat + dlat
        lon = self.origin_lon + dlon
        
        return (lat, lon)

    def _write_uninitialized_nav_state(self):
        """Write uninitialized state"""
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
                    'three_axis_available': False,
                    'local_x': 0.0,
                    'local_y': 0.0
                })
        except Exception as e:
            print(f"[NAV] Error writing uninitialized state: {e}")

    def _update_nav_state(self):
        """Update nav_state with current estimates"""
        try:
            # CRITICAL: Update local coordinates from current lat/lon on EVERY cycle
            # This ensures local_x/local_y track dead reckoning, not just GPS updates
            self.local_x, self.local_y = self._lat_lon_to_local(self.lat, self.lon)
            
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
                    'local_x': self.local_x,
                    'local_y': self.local_y,
                    'coordinate_system': 'nautical_true_north',
                    'magnetic_declination_deg': self.magnetic_declination_deg,
                    'gps_filter_active': True,
                    'resurfacing_correction_enabled': self.resurfacing_correction['enabled'],
                    'calibration_valid': self.sensor_corrections['calibration_valid']
                })
                
        except Exception as e:
            print(f"[NAV] Error updating nav_state: {e}")

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
                print(f"[GPS] Filter alpha set to {filter_alpha:.2f}")
            else:
                print("[GPS] Filter alpha must be between 0.0 and 1.0")
        
        if max_gps_jump is not None:
            if max_gps_jump > 0:
                self.gps_filter_config['max_gps_jump'] = max_gps_jump
                print(f"[GPS] Max jump threshold set to {max_gps_jump:.1f}m")
            else:
                print("[GPS] Max GPS jump must be positive")
        
        if min_interval is not None:
            if min_interval >= 0:
                self.gps_filter_config['min_gps_interval'] = min_interval
                print(f"[GPS] Minimum interval set to {min_interval:.1f}s")
            else:
                print("[GPS] Minimum interval must be non-negative")

    def set_magnetic_declination(self, declination_deg: float):
        """Set magnetic declination"""
        old_decl = self.magnetic_declination_deg
        self.magnetic_declination_deg = declination_deg
        print(f"[NAV] Magnetic declination changed: {old_decl:+.1f}° → {declination_deg:+.1f}°")

    def get_gps_filter_status(self) -> Dict[str, Any]:
        """Get GPS filter status"""
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
            'magnetic_declination': self.magnetic_declination_deg,
            'data_collected': len(self.calibration_data['gps_speeds']),
            'min_samples_needed': self.calibration_config['min_samples']
        }

    def get_status(self) -> Dict[str, Any]:
        """Get complete navigation status"""
        return {
            'initialized': self.initialized,
            'heading_initialized': self.heading_initialized,
            'three_axis_velocity': self.three_axis_available,
            'coordinate_system': 'nautical_true_north',
            'magnetic_declination_deg': self.magnetic_declination_deg,
            'calibration': self.get_calibration_status(),
            'gps_filtering': {
                'enabled': True,
                'filter_alpha': self.gps_filter_config['filter_alpha'],
                'max_jump_threshold': self.gps_filter_config['max_gps_jump'],
                'consecutive_outliers': self.gps_filter_state['consecutive_outliers'],
                'resurfacing_correction_enabled': self.resurfacing_correction['enabled'],
                'correction_in_enu_space': True  # THE FIX!
            },
            'position': {
                'lat': self.lat,
                'lon': self.lon,
                'local_x': self.local_x,
                'local_y': self.local_y,
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
            'architecture': 'GPS_CORRECTION_IN_ENU_SPACE_FIX'
        }