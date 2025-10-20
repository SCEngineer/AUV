#!/usr/bin/env python3
"""
hardware/simulated_sensors.py - Simulated sensor interface for HAL
"""

import time
from typing import Dict, Any

class SimulatedSensorInterface:
    """Simulated sensor interface for testing and simulation"""
    
    def __init__(self, vehicle_state, config: Dict[str, Any] = None):
        self.vehicle_state = vehicle_state
        self.config = config or {}
        self.available = True
        
        # Try to use existing sensor models if available
        try:
            from sensor_models import SensorModels
            self.sensor_models = SensorModels(vehicle_state)
        except ImportError:
            self.sensor_models = None
        
        print("Simulated sensors initialized")
    
    def read_all_sensors(self) -> Dict[str, Any]:
        """Read all simulated sensors"""
        if not self.available:
            return {}
        
        try:
            if self.sensor_models:
                # Use existing sensor models
                sensor_data = self.sensor_models.update(0.1)
                return self._standardize_sensor_data(sensor_data)
            else:
                # Basic simulation
                nav = self.vehicle_state.nav_state
                return {
                    'gps': {
                        'lat': nav.get('lat', 33.6094),
                        'lon': nav.get('lon', -117.7070),
                        'available': nav.get('depth', 0.0) < 0.5
                    },
                    'compass': {
                        'heading': nav.get('heading', 0.0),
                        'available': True
                    },
                    'depth': {
                        'depth': nav.get('depth', 0.0),
                        'available': True
                    },
                    'imu': {
                        'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 9.81,
                        'available': True
                    },
                    'leak': {'leak_detected': False, 'available': True}
                }
        except Exception as e:
            print(f"Simulated sensor error: {e}")
            return {}
    
    def _standardize_sensor_data(self, raw_data):
        """Convert sensor model output to standard format"""
        standardized = {}
        
        if 'gps' in raw_data and raw_data['gps']:
            gps_data = raw_data['gps']
            standardized['gps'] = {
                'lat': gps_data.get('lat', 0.0),
                'lon': gps_data.get('lon', 0.0),
                'available': True
            }
        else:
            standardized['gps'] = {'available': False}
        
        if 'compass' in raw_data:
            standardized['compass'] = {
                'heading': raw_data['compass'],
                'available': True
            }
        
        if 'depth' in raw_data:
            standardized['depth'] = {
                'depth': raw_data['depth'],
                'available': True
            }
        
        if 'imu' in raw_data:
            imu_data = raw_data['imu']
            standardized['imu'] = {
                'accel_x': imu_data.get('accel_x', 0.0),
                'accel_y': imu_data.get('accel_y', 0.0),
                'accel_z': imu_data.get('accel_z', 9.81),
                'available': True
            }
        
        standardized['leak'] = {'leak_detected': False, 'available': True}
        
        return standardized
    
    def is_available(self) -> bool:
        return self.available
    
    def close(self):
        """Cleanup simulated sensors"""
        pass
