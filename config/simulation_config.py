#!/usr/bin/env python3
"""
config/simulation_config.py - Simulation configuration
"""

SIMULATION_CONFIG = {
    'mode': 'simulation',
    'sensor_update_rate': 10.0,  # Hz
    'enable_x_tail': True,
    'debug': False,
    
    # Physics simulation parameters
    'physics': {
        'simulation_rate': 10.0,        # Hz
        'enable_current': False,        # Water current simulation
        'current_speed': 0.0,           # m/s
        'current_direction': 0.0        # degrees
    },
    
    # Simulated sensor parameters
    'sensors': {
        'gps_noise_std': 2.5,           # meters (1-sigma)
        'compass_noise_std': 4.0,        # degrees (1-sigma)
        'depth_noise_std': 0.05,        # meters (1-sigma)
        'imu_noise_std': 0.05,          # m/s² for accelerometer
        'velocity_noise_std': 0.05,     # m/s (1-sigma)
        'gps_max_depth': 0.3,           # meters (no GPS below this depth)
    },
    
    # Safety limits (same as hardware)
    'fin_limits': {
        'min': -45.0,
        'max': 45.0
    },
    
    'thruster_limits': {
        'min': 0.0,
        'max': 100.0
    }
}
