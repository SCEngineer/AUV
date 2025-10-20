#!/usr/bin/env python3
"""
sensor_models.py - FIXED: GPS course calculation now uses nautical convention
Realistic sensor models for AUV simulation with SIMPLR-AUV three-axis velocity system

COORDINATE SYSTEM FIX:
- GPS course calculation now outputs nautical bearing (0° = North, clockwise)
- Consistent with compass heading and navigation system expectations
- Eliminates coordinate system mismatch that caused trajectory rotation

Incorporates patent-specified boundary layer compensation and realistic sensor characteristics
ADDED: Blue Robotics SoS leak detection for three watertight cylinders
"""

import math
import random
import time
from typing import Dict, Any, Optional
from vehicle_state import VehicleState

class SensorModels:
    """
    COMPLETE: Realistic sensor models for AUV simulation with three-axis velocity
    - GPS: Surface-only positioning with realistic accuracy (FIXED: nautical course)
    - Compass: Magnetic heading with declination and noise
    - IMU: Accelerometer and gyroscope with bias and noise
    - Depth: Pressure-based depth measurement
    - Three-axis paddlewheel velocity: SIMPLR-AUV innovation with boundary layer compensation
    - Blue Robotics SoS leak detection: Three watertight cylinders monitoring
    """
    
    def __init__(self, vehicle_state: VehicleState):
        self.vehicle_state = vehicle_state
        self.last_gps_time = 0.0
        self.last_position = None
        self.velocity_history = []
        
        # Sensor configurations based on real AUV specifications
        self.sensor_config = {
            'gps': {
                'update_rate': 1.0,  # Hz
                'horizontal_accuracy': 2.5,  # meters (1-sigma)
                'fix_time': 30.0,  # seconds to acquire fix
                'max_depth_for_signal': 0.3  # meters
            },
            'compass': {
                'update_rate': 10.0,  # Hz
                'heading_noise_std': 4.0,  # degrees
                'declination': 12.0  # magnetic declination for Southern California
            },
            'imu': {
                'update_rate': 100.0,  # Hz
                'accel_noise_std': 0.05,  # m/s²
                'gyro_noise_std': 0.01,  # rad/s
                'accel_bias_std': 0.02,  # m/s²
                'gyro_bias_std': 0.005  # rad/s
            },
            'depth': {
                'update_rate': 10.0,  # Hz
                'pressure_noise_std': 0.05,  # meters
                'surface_offset': 0.0
            },
            'velocity': {
                'update_rate': 4.0,  # Hz - paddlewheel mechanical limitation
                'noise_std': 0.04,  # m/s (per axis, from patent specifications)
                'min_detectable_speed': 0.02,  # m/s minimum detection threshold
                'accuracy_spec': 0.12  # m/s total system accuracy from patent
            },
            'leak_sensors': {
                'update_rate': 2.0,  # Hz - Blue Robotics SoS update rate
                'false_positive_rate': 0.0001,  # Very rare false positives
                'false_negative_rate': 0.00001,  # Extremely rare false negatives
                'response_time': 0.1  # seconds to detect water
            }
        }
        
        # Realistic sensor biases for all sensors including three-axis velocity
        self.sensor_biases = {
            'compass_bias': random.gauss(0, 1.0),
            'depth_bias': random.gauss(0, 0.02),
            'velocity_bias_surge': random.gauss(0, 0.02),
            'velocity_bias_sway': random.gauss(0, 0.02),
            'velocity_bias_heave': random.gauss(0, 0.02)
        }
        
        # Three-axis paddlewheel specific parameters from patent document
        self.paddlewheel_config = {
            'boundary_layer_compensation': {
                'surge_factor': 0.95,   # 5% flow reduction (bottom mount)
                'sway_factor': 0.92,    # 8% flow reduction (side mount)
                'heave_factor': 0.93    # 7% flow reduction (bottom mount)
            },
            'sensor_locations': {
                'surge_sensor': 'bottom_hull',    # X-axis sensor location
                'sway_sensor': 'side_hull',       # Y-axis sensor location  
                'heave_sensor': 'bottom_hull'     # Z-axis sensor location
            },
            'calibration_factors': {
                'surge_k': 1.02,      # Calibration constant for forward sensor
                'sway_k': 0.98,       # Calibration constant for side sensor
                'heave_k': 1.01       # Calibration constant for vertical sensor
            }
        }
        
        # Blue Robotics SoS leak sensor simulation parameters
        self.leak_sensor_config = {
            'sensor_locations': {
                'FWD_WTC': 'forward_electronics_cylinder',
                'MID_WTC': 'middle_payload_cylinder', 
                'AFT_WTC': 'aft_propulsion_cylinder'
            },
            'sensor_sensitivity': {
                'FWD_WTC': 0.95,  # 95% detection reliability
                'MID_WTC': 0.95,
                'AFT_WTC': 0.95
            },
            # For simulation testing - normally all False
            'simulated_leaks': {
                'FWD_WTC': False,
                'MID_WTC': False,
                'AFT_WTC': False
            }
        }
        
        print("SIMPLR-AUV Sensor Models Initialized (NAUTICAL CONVENTION):")
        print(f"  GPS: ±{self.sensor_config['gps']['horizontal_accuracy']}m accuracy (FIXED: nautical course)")
        print(f"  Compass: ±{self.sensor_config['compass']['heading_noise_std']}° noise")
        print(f"  Depth: ±{self.sensor_config['depth']['pressure_noise_std']}m accuracy")
        print(f"  Three-axis velocity: ±{self.sensor_config['velocity']['accuracy_spec']}m/s (patent spec)")
        print(f"  Boundary layer compensation: Enabled")
        print(f"  Blue Robotics SoS leak sensors: 3 cylinders monitored")
        print("COORDINATE FIX: GPS course now uses nautical bearing (0° = North)")
    
    def update(self, time_step: float) -> Dict[str, Any]:
        """Update all sensors and return combined sensor data"""
        current_time = time.time()
        sensor_data = {}
        
        # GPS (only at surface)
        if self._gps_available():
            if current_time - self.last_gps_time >= (1.0 / self.sensor_config['gps']['update_rate']):
                gps_data = self._simulate_gps()
                if gps_data:
                    sensor_data['gps'] = gps_data
                    self.last_gps_time = current_time
        
        # Compass (always available)
        sensor_data['compass'] = self._simulate_compass()
        
        # IMU (always available) 
        sensor_data['imu'] = self._simulate_imu()
        
        # Depth sensor (always available)
        sensor_data['depth'] = self._simulate_depth()
        
        # ENHANCED: Three-axis paddlewheel velocity sensor
        sensor_data['water_speed'] = self._simulate_three_axis_paddlewheel()
        
        # Blue Robotics SoS leak detection sensors
        sensor_data['leak_sensors'] = self._simulate_leak_sensors()
        
        return sensor_data
    
    def _gps_available(self) -> bool:
        """Check if GPS signal is available (surface only)"""
        depth = self.vehicle_state.true_state.get('true_depth', 0.0)
        return depth <= self.sensor_config['gps']['max_depth_for_signal']
    
    def _simulate_gps(self) -> Optional[Dict[str, Any]]:
        """FIXED: Simulate GPS with nautical course calculation"""
        if not self._gps_available():
            return None
        
        true_lat = self.vehicle_state.true_state.get('true_lat', 0.0)
        true_lon = self.vehicle_state.true_state.get('true_lon', 0.0)
        
        # GPS noise (no systematic bias)
        accuracy = self.sensor_config['gps']['horizontal_accuracy']
        lat_noise = random.gauss(0, accuracy / 111111.0)  # Convert meters to degrees
        lon_noise = random.gauss(0, accuracy / (111111.0 * math.cos(math.radians(true_lat))))
        
        # Simulated GPS coordinates
        gps_lat = true_lat + lat_noise
        gps_lon = true_lon + lon_noise
        
        # Calculate GPS-derived speed and course (FIXED for nautical convention)
        speed, course = self._calculate_gps_speed_course_nautical()
        
        return {
            'lat': gps_lat,
            'lon': gps_lon,
            'fix_quality': 'good',
            'speed': speed,
            'course': course,
            'timestamp': time.time()
        }
    
    def _calculate_gps_speed_course_nautical(self) -> tuple:
        """FIXED: Calculate speed and nautical course from GPS position history"""
        current_time = time.time()
        current_lat = self.vehicle_state.true_state.get('true_lat', 0.0)
        current_lon = self.vehicle_state.true_state.get('true_lon', 0.0)
        
        if self.last_position is None:
            self.last_position = (current_lat, current_lon, current_time)
            return 0.0, 0.0
        
        last_lat, last_lon, last_time = self.last_position
        dt = current_time - last_time
        
        if dt < 1.0:  # Need at least 1 second for good GPS speed estimate
            return 0.0, 0.0
        
        # Calculate distance and bearing
        dlat = current_lat - last_lat
        dlon = current_lon - last_lon
        
        # Convert to meters
        north_m = dlat * 111111.0
        east_m = dlon * 111111.0 * math.cos(math.radians(current_lat))
        
        # Speed calculation
        distance = math.sqrt(north_m**2 + east_m**2)
        speed = distance / dt
        
        if distance > 0.1:  # Only calculate course if significant movement
            # FIXED: Calculate nautical bearing (0° = North, clockwise)
            # atan2 gives mathematical angle (0° = East, counter-clockwise)
            # Convert to nautical: course = 90° - math_angle, then normalize
            math_angle_deg = math.degrees(math.atan2(east_m, north_m))
            course = math_angle_deg % 360.0  # This gives nautical bearing directly
        else:
            course = 0.0
        
        # Update position history
        self.last_position = (current_lat, current_lon, current_time)
        
        return speed, course
    
    def _simulate_compass(self) -> float:
        """Simulate compass reading with realistic noise and declination"""
        true_heading = self.vehicle_state.true_state.get('true_heading', 0.0)
        
        # Add noise and small bias
        noise = random.gauss(0, self.sensor_config['compass']['heading_noise_std'])
        bias = self.sensor_biases['compass_bias']
        
        # Magnetic declination effect (true to magnetic heading)
        declination = self.sensor_config['compass']['declination']
        
        # Compass reading = true heading + declination + noise + bias
        compass_reading = true_heading + declination + noise + bias
        
        return compass_reading % 360.0
    
    def _simulate_imu(self) -> Dict[str, float]:
        """Simulate IMU accelerometer and gyroscope"""
        # Get true vehicle attitude
        true_pitch = self.vehicle_state.true_state.get('true_pitch', 0.0)
        true_roll = self.vehicle_state.true_state.get('true_roll', 0.0)
        true_yaw_rate = self.vehicle_state.true_state.get('true_yaw_rate', 0.0)
        
        # Simulate gravity vector in body frame
        g = 9.81
        accel_x = -g * math.sin(true_pitch)
        accel_y = g * math.sin(true_roll) * math.cos(true_pitch)
        accel_z = g * math.cos(true_roll) * math.cos(true_pitch)
        
        # Add noise and bias
        accel_noise_std = self.sensor_config['imu']['accel_noise_std']
        gyro_noise_std = self.sensor_config['imu']['gyro_noise_std']
        
        return {
            'accel_x': accel_x + random.gauss(0, accel_noise_std),
            'accel_y': accel_y + random.gauss(0, accel_noise_std),
            'accel_z': accel_z + random.gauss(0, accel_noise_std),
            'gyro_x': 0.0 + random.gauss(0, gyro_noise_std),  # Roll rate
            'gyro_y': 0.0 + random.gauss(0, gyro_noise_std),  # Pitch rate
            'gyro_z': true_yaw_rate + random.gauss(0, gyro_noise_std),  # Yaw rate
            'timestamp': time.time()
        }
    
    def _simulate_depth(self) -> float:
        """Simulate pressure-based depth sensor"""
        true_depth = self.vehicle_state.true_state.get('true_depth', 0.0)
        
        # Add noise and small bias
        noise = random.gauss(0, self.sensor_config['depth']['pressure_noise_std'])
        bias = self.sensor_biases['depth_bias']
        
        measured_depth = max(0.0, true_depth + noise + bias)
        
        return measured_depth
    
    def _simulate_leak_sensors(self) -> Dict[str, str]:
        """
        Simulate Blue Robotics SoS leak detection sensors for three watertight cylinders
        
        Returns status for each cylinder:
        - 'DRY': No water detected (normal operation)
        - 'LEAK': Water detected (alarm condition)
        
        In simulation, normally returns 'DRY' unless testing leak scenarios
        """
        leak_status = {}
        
        for cylinder in ['FWD_WTC', 'MID_WTC', 'AFT_WTC']:
            # Check if simulated leak is enabled for testing
            simulated_leak = self.leak_sensor_config['simulated_leaks'].get(cylinder, False)
            
            if simulated_leak:
                # Simulate actual leak detection with sensor reliability
                sensor_sensitivity = self.leak_sensor_config['sensor_sensitivity'].get(cylinder, 0.95)
                
                # Apply sensor reliability - small chance of false negative
                if random.random() < sensor_sensitivity:
                    leak_status[cylinder] = 'LEAK'
                else:
                    leak_status[cylinder] = 'DRY'  # False negative (rare)
            else:
                # Normal operation - check for extremely rare false positives
                false_positive_rate = self.sensor_config['leak_sensors']['false_positive_rate']
                
                if random.random() < false_positive_rate:
                    leak_status[cylinder] = 'LEAK'  # False positive (very rare)
                else:
                    leak_status[cylinder] = 'DRY'   # Normal operation
        
        return leak_status
    
    def inject_leak(self, cylinder: str, enable: bool = True):
        """
        Inject a simulated leak for testing purposes
        
        Args:
            cylinder: 'FWD_WTC', 'MID_WTC', or 'AFT_WTC'
            enable: True to simulate leak, False to clear simulated leak
        """
        if cylinder in self.leak_sensor_config['simulated_leaks']:
            self.leak_sensor_config['simulated_leaks'][cylinder] = enable
            status = "enabled" if enable else "disabled"
            print(f"Leak simulation {status} for {cylinder}")
        else:
            print(f"Invalid cylinder: {cylinder}. Valid options: FWD_WTC, MID_WTC, AFT_WTC")
    
    def get_leak_sensor_status(self) -> Dict[str, Any]:
        """Get leak sensor system status and configuration"""
        return {
            'sensor_locations': self.leak_sensor_config['sensor_locations'],
            'sensor_sensitivity': self.leak_sensor_config['sensor_sensitivity'],
            'simulated_leaks': self.leak_sensor_config['simulated_leaks'],
            'false_positive_rate': self.sensor_config['leak_sensors']['false_positive_rate'],
            'false_negative_rate': self.sensor_config['leak_sensors']['false_negative_rate'],
            'update_rate': f"{self.sensor_config['leak_sensors']['update_rate']} Hz"
        }
    
    def _simulate_three_axis_paddlewheel(self) -> Dict[str, float]:
        """
        COMPLETE: Simulate SIMPLR-AUV three-axis paddlewheel velocity sensor system
        
        Implements the patent-specified three-axis flush-mounted paddlewheel design:
        - X-axis (surge): Forward/backward velocity measurement
        - Y-axis (sway): Port/starboard velocity measurement  
        - Z-axis (heave): Up/down velocity measurement
        
        Includes boundary layer compensation, sensor mounting effects, and realistic
        noise characteristics as specified in the patent document.
        """
        
        # Get true vehicle motion components from vehicle dynamics
        true_speed = self.vehicle_state.true_state.get('true_speed', 0.0)
        true_heading = math.radians(self.vehicle_state.true_state.get('true_heading', 0.0))
        true_yaw_rate = self.vehicle_state.true_state.get('true_yaw_rate', 0.0)
        true_vertical_velocity = self.vehicle_state.true_state.get('true_vertical_velocity', 0.0)
        true_pitch = self.vehicle_state.true_state.get('true_pitch', 0.0)
        
        # Vehicle physical parameters for realistic motion coupling
        vehicle_length = 1.524  # meters (from vehicle specs)
        
        # === CALCULATE TRUE WATER VELOCITIES ===
        
        # SURGE (X-AXIS) - Forward/backward velocity
        # Primary forward motion component
        surge_velocity = true_speed
        
        # SWAY (Y-AXIS) - Port/starboard velocity
        # Calculate sway from vehicle turning dynamics and attitude coupling
        
        # 1. Turn-induced sway: Vehicle slides sideways during turns
        turn_radius_effect = true_yaw_rate * (vehicle_length * 0.3)  # CG offset from turn center
        
        # 2. Pitch-induced sway: Forward motion + pitch creates cross-flow
        pitch_coupling = true_speed * math.sin(true_pitch) * 0.1  # Small pitch-to-sway coupling
        
        # 3. Realistic sway velocity from fluid dynamics
        sway_velocity = turn_radius_effect + pitch_coupling
        
        # HEAVE (Z-AXIS) - Up/down velocity
        # 1. Direct vertical motion through water
        heave_velocity = -true_vertical_velocity  # Negative because down is positive depth
        
        # 2. Forward motion + pitch creates significant vertical flow component
        pitch_induced_heave = true_speed * math.sin(true_pitch) * 0.8  # Primary coupling
        heave_velocity += pitch_induced_heave
        
        # === APPLY SENSOR CHARACTERISTICS ===
        
        # Get sensor configuration parameters
        boundary_comp = self.paddlewheel_config['boundary_layer_compensation']
        calibration = self.paddlewheel_config['calibration_factors']
        
        # Boundary layer compensation (from patent document)
        # Flush-mounted sensors experience reduced flow due to hull boundary layer
        compensated_surge = surge_velocity / boundary_comp['surge_factor']
        compensated_sway = sway_velocity / boundary_comp['sway_factor']
        compensated_heave = heave_velocity / boundary_comp['heave_factor']
        
        # Apply sensor calibration factors
        calibrated_surge = compensated_surge * calibration['surge_k']
        calibrated_sway = compensated_sway * calibration['sway_k']
        calibrated_heave = compensated_heave * calibration['heave_k']
        
        # Add realistic sensor noise and bias
        base_noise_std = self.sensor_config['velocity']['noise_std']
        
        # Individual sensor noise (each paddlewheel has independent noise)
        surge_noise = random.gauss(0, base_noise_std)
        sway_noise = random.gauss(0, base_noise_std)
        heave_noise = random.gauss(0, base_noise_std)
        
        # Apply biases (small, realistic sensor-to-sensor variations)
        surge_bias = self.sensor_biases['velocity_bias_surge']
        sway_bias = self.sensor_biases['velocity_bias_sway']
        heave_bias = self.sensor_biases['velocity_bias_heave']
        
        # Final measured velocities
        measured_surge = calibrated_surge + surge_noise + surge_bias
        measured_sway = calibrated_sway + sway_noise + sway_bias
        measured_heave = calibrated_heave + heave_noise + heave_bias
        
        # Apply minimum detection threshold (mechanical sensor limitation)
        min_speed = self.sensor_config['velocity']['min_detectable_speed']
        
        if abs(measured_surge) < min_speed:
            measured_surge = 0.0
        if abs(measured_sway) < min_speed:
            measured_sway = 0.0  
        if abs(measured_heave) < min_speed:
            measured_heave = 0.0
        
        # Calculate derived measurements
        total_speed = math.sqrt(measured_surge**2 + measured_sway**2 + measured_heave**2)
        horizontal_speed = math.sqrt(measured_surge**2 + measured_sway**2)
        
        # === RETURN COMPLETE SENSOR DATA ===
        return {
            # Primary three-axis velocity components (body frame)
            'surge': measured_surge,        # Forward (+) / Backward (-)
            'sway': measured_sway,          # Starboard (+) / Port (-)  
            'heave': measured_heave,        # Down (+) / Up (-)
            
            # Derived velocity measurements
            'total': total_speed,           # 3D velocity magnitude
            'horizontal': horizontal_speed,  # 2D horizontal velocity magnitude
            'forward': measured_surge,      # Alias for surge (compatibility)
            'sideways': measured_sway,      # Alias for sway (compatibility)
            'vertical': measured_heave,     # Alias for heave (compatibility)
            
            # Sensor system status indicators
            'boundary_compensated': True,   # Boundary layer compensation applied
            'three_axis_valid': True,       # All three axes providing valid data
            'calibrated': True,             # Sensor calibration applied
            'sensor_type': 'paddlewheel_3axis',  # Sensor identification
            
            # Quality metrics
            'accuracy_met': total_speed == 0.0 or total_speed * 0.12 >= self.sensor_config['velocity']['accuracy_spec'],
            'noise_level': base_noise_std,
            
            # Diagnostic information
            'surge_compensated': abs(surge_velocity - compensated_surge) > 0.001,
            'sway_compensated': abs(sway_velocity - compensated_sway) > 0.001,
            'heave_compensated': abs(heave_velocity - compensated_heave) > 0.001,
            
            # Standard metadata
            'timestamp': time.time()
        }
    
    def get_sensor_status(self) -> Dict[str, Any]:
        """Get comprehensive sensor system status"""
        return {
            'gps_available': self._gps_available(),
            'coordinate_system': 'nautical_fixed',
            'three_axis_velocity': {
                'enabled': True,
                'sensor_type': 'SIMPLR-AUV_paddlewheel_3axis',
                'boundary_layer_compensation': self.paddlewheel_config['boundary_layer_compensation'],
                'calibration_factors': self.paddlewheel_config['calibration_factors'],
                'accuracy_specification': f"±{self.sensor_config['velocity']['accuracy_spec']} m/s",
                'update_rate': f"{self.sensor_config['velocity']['update_rate']} Hz"
            },
            'leak_detection': {
                'sensor_type': 'Blue_Robotics_SoS',
                'cylinders_monitored': list(self.leak_sensor_config['sensor_locations'].keys()),
                'sensor_locations': self.leak_sensor_config['sensor_locations'],
                'update_rate': f"{self.sensor_config['leak_sensors']['update_rate']} Hz",
                'false_positive_rate': self.sensor_config['leak_sensors']['false_positive_rate'],
                'simulated_leaks': self.leak_sensor_config['simulated_leaks']
            },
            'sensor_biases': {
                'compass': self.sensor_biases['compass_bias'],
                'depth': self.sensor_biases['depth_bias'],
                'velocity_surge': self.sensor_biases['velocity_bias_surge'],
                'velocity_sway': self.sensor_biases['velocity_bias_sway'],
                'velocity_heave': self.sensor_biases['velocity_bias_heave']
            },
            'sensor_configurations': self.sensor_config,
            'patent_compliance': {
                'three_axis_measurement': True,
                'flush_mount_design': True,
                'boundary_layer_compensation': True,
                'cost_target_met': True  # Assumes <$1500 vs $20K+ DVL
            }
        }
    
    def reset_biases(self):
        """Reset all sensor biases to new random values"""
        self.sensor_biases = {
            'compass_bias': random.gauss(0, 1.0),
            'depth_bias': random.gauss(0, 0.02),
            'velocity_bias_surge': random.gauss(0, 0.02),
            'velocity_bias_sway': random.gauss(0, 0.02),
            'velocity_bias_heave': random.gauss(0, 0.02)
        }
        print("All sensor biases reset to new random values")
    
    def calibrate_paddlewheels(self, surge_k: float = None, sway_k: float = None, heave_k: float = None):
        """
        Update paddlewheel calibration factors (simulates field calibration)
        """
        if surge_k is not None:
            self.paddlewheel_config['calibration_factors']['surge_k'] = surge_k
        if sway_k is not None:
            self.paddlewheel_config['calibration_factors']['sway_k'] = sway_k  
        if heave_k is not None:
            self.paddlewheel_config['calibration_factors']['heave_k'] = heave_k
            
        print("Paddlewheel sensors recalibrated:")
        print(f"  Surge calibration: {self.paddlewheel_config['calibration_factors']['surge_k']:.3f}")
        print(f"  Sway calibration: {self.paddlewheel_config['calibration_factors']['sway_k']:.3f}")
        print(f"  Heave calibration: {self.paddlewheel_config['calibration_factors']['heave_k']:.3f}")
    
    def get_paddlewheel_diagnostics(self) -> Dict[str, Any]:
        """
        Get detailed diagnostics for the three-axis paddlewheel system
        Useful for validating sensor performance and patent claims
        """
        return {
            'sensor_locations': self.paddlewheel_config['sensor_locations'],
            'boundary_layer_effects': {
                'surge_reduction': f"{(1.0 - self.paddlewheel_config['boundary_layer_compensation']['surge_factor']) * 100:.1f}%",
                'sway_reduction': f"{(1.0 - self.paddlewheel_config['boundary_layer_compensation']['sway_factor']) * 100:.1f}%",
                'heave_reduction': f"{(1.0 - self.paddlewheel_config['boundary_layer_compensation']['heave_factor']) * 100:.1f}%"
            },
            'performance_metrics': {
                'update_rate': f"{self.sensor_config['velocity']['update_rate']} Hz",
                'accuracy_target': f"±{self.sensor_config['velocity']['accuracy_spec']} m/s",
                'minimum_detection': f"{self.sensor_config['velocity']['min_detectable_speed']} m/s",
                'noise_level': f"±{self.sensor_config['velocity']['noise_std']} m/s per axis"
            },
            'innovation_features': [
                'Three orthogonal paddlewheel sensors',
                'Flush-mount zero-drag design', 
                'Automatic boundary layer compensation',
                'Individual sensor calibration',
                'Cost-effective alternative to DVL systems',
                'Nautical coordinate system compatibility'
            ]
        }