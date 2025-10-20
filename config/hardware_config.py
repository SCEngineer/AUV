#!/usr/bin/env python3
"""
config/hardware_config.py - Hardware configuration for Raspberry Pi deployment
"""

HARDWARE_CONFIG = {
    'mode': 'hardware',
    'sensor_update_rate': 10.0,  # Hz
    'enable_x_tail': True,
    'debug': False,
    
    # Sensor configurations
    'sensors': {
        'gps': {
            'device': '/dev/ttyUSB0',  # USB GPS dongle
            'baudrate': 9600,
            'timeout': 1.0
        },
        'compass': {
            'i2c_address': 0x1E,  # HMC5883L magnetometer
            'bus': 1,
            'declination': 12.0  # Magnetic declination for your location
        },
        'imu': {
            'i2c_address': 0x68,  # MPU6050 IMU
            'bus': 1
        },
        'depth_sensor': {
            'i2c_address': 0x76,  # MS5837 pressure sensor
            'bus': 1
        }
    },
    
    # Actuator configurations
    'actuators': {
        'servo_controller': {
            'i2c_address': 0x40,  # PCA9685 servo controller
            'bus': 1,
            'frequency': 50,  # 50Hz for servos
            
            # Servo channel assignments for X-tail
            'fin_upper_left_channel': 0,
            'fin_upper_right_channel': 1,
            'fin_lower_right_channel': 2,
            'fin_lower_left_channel': 3,
        },
        
        'thruster_esc': {
            'gpio_pin': 18,     # PWM pin for ESC
            'frequency': 50,    # 50Hz for ESC
        },
        
        'ballast_system': {
            'pump_relay_pin': 23,    # GPIO pin for pump relay
            'vent_relay_pin': 24,    # GPIO pin for vent valve relay
        }
    },
    
    # Safety limits
    'fin_limits': {
        'min': -45.0,  # degrees
        'max': 45.0    # degrees
    },
    
    'thruster_limits': {
        'min': 0.0,    # percent
        'max': 100.0   # percent
    }
}
