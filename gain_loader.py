#!/usr/bin/env python3
"""
gain_loader.py - Load PID control gains from configuration file
UPDATED: Support for cascaded control structure (depth/pitch/heading/speed)
Converts from config file format to vehicle_control.py expected structure
"""

import os
from typing import Dict, Any

def load_gains(gain_file: str = "auv_pid_gains.txt") -> Dict[str, Dict[str, Dict[str, float]]]:
    """
    Load PID control gains from configuration file for cascaded control structure
    Reads auv_pid_gains.txt format with [sections] and returns nested structure
    expected by the cascaded vehicle_control.py
    
    Returns:
        Dictionary with structure: {'gains': {'depth': {'kp': val, 'ki': val, 'kd': val}, ...}}
    """
    
    # Default gains for cascaded control structure
    default_gains = {
        'depth': {'kp': 2.5, 'ki': 0.15, 'kd': 0.6},    # Outer loop: depth → pitch
        'pitch': {'kp': 2.0, 'ki': 0.05, 'kd': 0.4},    # Inner loop: pitch → fins
        'heading': {'kp': 1.59, 'ki': 0.026, 'kd': 0.06}, # Heading → yaw
        'speed': {'kp': 21.1, 'ki': 0.15, 'kd': 0.218}   # Speed → thrust
    }
    
    gains = default_gains.copy()
    gains_loaded = False
    
    # Try to load from file
    if os.path.exists(gain_file):
        try:
            current_section = None
            section_gains = {}
            
            with open(gain_file, 'r') as f:
                for line_num, line in enumerate(f, 1):
                    line = line.strip()
                    
                    # Skip comments and empty lines
                    if not line or line.startswith('#'):
                        continue
                    
                    # Parse section headers [depth], [pitch], [heading], [speed]
                    if line.startswith('[') and line.endswith(']'):
                        # Save previous section if it exists and has gains
                        if current_section and section_gains:
                            if current_section in gains:
                                # Update gains with loaded values
                                for key in ['Kp', 'Ki', 'Kd']:
                                    if key in section_gains:
                                        gains[current_section][key.lower()] = section_gains[key]
                        
                        current_section = line[1:-1].strip().lower()  # Remove brackets, normalize case
                        section_gains = {}
                        continue
                    
                    # Parse gain entries (format: Kp=1.0)
                    if '=' in line and current_section:
                        try:
                            key, value = line.split('=', 1)
                            key = key.strip()
                            value = value.strip()
                            
                            # Validate key is one of Kp, Ki, Kd
                            if key in ['Kp', 'Ki', 'Kd']:
                                section_gains[key] = float(value)
                            else:
                                print(f"Warning: Unknown gain parameter '{key}' in section [{current_section}] at line {line_num}")
                                
                        except ValueError as e:
                            print(f"Warning: Invalid gain value at line {line_num}: {line} ({e})")
                            continue
                        except Exception as e:
                            print(f"Warning: Error parsing line {line_num}: {line} ({e})")
                            continue
                
                # Save the last section
                if current_section and section_gains:
                    if current_section in gains:
                        for key in ['Kp', 'Ki', 'Kd']:
                            if key in section_gains:
                                gains[current_section][key.lower()] = section_gains[key]
            
            gains_loaded = True
            print(f"Cascaded control gains loaded from {gain_file}:")
            for axis, gain_dict in gains.items():
                print(f"  {axis}: Kp={gain_dict['kp']:.3f}, Ki={gain_dict['ki']:.3f}, Kd={gain_dict['kd']:.3f}")
            
            # Validate cascaded structure
            required_sections = ['depth', 'pitch', 'heading', 'speed']
            missing_sections = [section for section in required_sections if section not in gains]
            if missing_sections:
                print(f"Warning: Missing sections in gains file: {missing_sections}")
                print("Using default values for missing sections")
            
        except Exception as e:
            print(f"Error loading gains from {gain_file}: {e}")
            print("Using default cascaded gains")
            gains_loaded = False
    
    else:
        print(f"Gains file {gain_file} not found, using defaults")
        
        # Create a template file for cascaded control
        try:
            _create_cascaded_gain_file_template(gain_file, gains)
            print(f"Created cascaded control gains template: {gain_file}")
        except Exception as e:
            print(f"Could not create gain file template: {e}")
    
    # Return in format expected by cascaded vehicle_control.py
    return {'gains': gains, 'gains_loaded': gains_loaded, 'gains_file': gain_file}


def _create_cascaded_gain_file_template(filename: str, gains: Dict[str, Dict[str, float]]) -> None:
    """Create a template gain file for cascaded control structure"""
    
    template_content = """# AUV PID Gains - Cascaded Control Structure
# CASCADED CONTROL ARCHITECTURE:
# Outer Loop: Depth Error → Pitch Command (depth gains)
# Inner Loop: Pitch Error → Fin Commands (pitch gains) 
# Independent: Heading → Yaw Commands, Speed → Thrust Commands

[depth]
Kp = {depth_kp}
Ki = {depth_ki}
Kd = {depth_kd}

[pitch]
Kp = {pitch_kp}
Ki = {pitch_ki}
Kd = {pitch_kd}

[heading]
Kp = {heading_kp}
Ki = {heading_ki}
Kd = {heading_kd}

[speed]
Kp = {speed_kp}
Ki = {speed_ki}
Kd = {speed_kd}
""".format(
        depth_kp=gains['depth']['kp'], depth_ki=gains['depth']['ki'], depth_kd=gains['depth']['kd'],
        pitch_kp=gains['pitch']['kp'], pitch_ki=gains['pitch']['ki'], pitch_kd=gains['pitch']['kd'],
        heading_kp=gains['heading']['kp'], heading_ki=gains['heading']['ki'], heading_kd=gains['heading']['kd'],
        speed_kp=gains['speed']['kp'], speed_ki=gains['speed']['ki'], speed_kd=gains['speed']['kd']
    )

    with open(filename, 'w') as f:
        f.write(template_content)


def validate_cascaded_gains(gains: Dict[str, Dict[str, float]]) -> bool:
    """Validate that all required gains are present for cascaded control"""
    required_axes = ['depth', 'pitch', 'heading', 'speed']
    required_params = ['kp', 'ki', 'kd']
    
    for axis in required_axes:
        if axis not in gains:
            print(f"Error: Missing {axis} gains")
            return False
        
        for param in required_params:
            if param not in gains[axis]:
                print(f"Error: Missing {param} parameter in {axis} gains")
                return False
            
            if not isinstance(gains[axis][param], (int, float)):
                print(f"Error: Invalid {param} value in {axis} gains: {gains[axis][param]}")
                return False
    
    return True


if __name__ == "__main__":
    # Test the loader
    print("Testing cascaded gains loader...")
    gains_data = load_gains()
    
    if validate_cascaded_gains(gains_data['gains']):
        print("✓ Cascaded gains structure is valid")
    else:
        print("✗ Cascaded gains structure has errors")
    
    print(f"Final cascaded gains structure:")
    for axis, gain_dict in gains_data['gains'].items():
        print(f"  {axis}: {gain_dict}")