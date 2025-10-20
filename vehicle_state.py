#!/usr/bin/env python3
"""
vehicle_state.py - UPDATED: Added task_params and leak detection system for three watertight cylinders
- set_initial_position() now only sets true_state (physics reality)
- Navigation system must initialize itself through GPS or other sensors
- Added leak_detection for FWD_WTC, MID_WTC, AFT_WTC monitoring
- Added task_params to separate task configuration from navigation targets
"""

import time
import threading
from enum import Enum
from typing import Dict, Any

class SystemState(Enum):
    INITIALIZED = "INITIALIZED"
    RUNNING = "RUNNING"
    SURFACED_TRIM = "SURFACED_TRIM"
    SUBMERGED_TRIM = "SUBMERGED_TRIM"
    FAULT = "FAULT"
    FAILSAFE = "FAILSAFE"
    EMERGENCY = "EMERGENCY"
    MISSION_COMPLETE = "MISSION_COMPLETE"

class TaskType(Enum):
    NONE = "NONE"
    GO_TO_SURFACED_TRIM = "GO_TO_SURFACED_TRIM"
    GO_TO_SUBMERGED_TRIM = "GO_TO_SUBMERGED_TRIM"
    DIVE = "DIVE"
    CLIMB = "CLIMB"
    GPS_FIX = "GPS_FIX"
    SWIM_TO_WAYPOINT = "SWIM_TO_WAYPOINT"
    FOLLOW_TRACK = "FOLLOW_TRACK"
    TELEMETRY = "TELEMETRY"
    EMERGENCY_SURFACE = "EMERGENCY_SURFACE"
    MISSION_COMPLETE = "MISSION_COMPLETE"

class VehicleState:
    def __init__(self):
        self._state_lock = threading.RLock()
        self.current_state = SystemState.INITIALIZED
        self.current_task = TaskType.NONE.value
        self.task_complete = False
        self.task_status = ""
        self.mission_time = 0.0
        self.faults = []
        self.failsafe_triggered = False
        self.failsafe_reason = ""
        
        try:
            from gain_loader import load_gains
            self.control_gains = load_gains()
        except ImportError:
            self.control_gains = {}  # Fallback if gain_loader is unavailable
        
        self.ballast_state = {
            'tank_volume': 0.0,
            'tank_capacity': 0.001982,
            'tank_status': 'EMPTY',
            'pump_running': False,
            'vent_open': False
        }
        self.buoyancy_state = "POSITIVE"
        
        self.true_state = {
            'true_lat': 0.0,
            'true_lon': 0.0,
            'true_depth': 0.0,
            'true_heading': 0.0,
            'true_speed': 0.0,
            'true_pitch': 0.0,
            'true_yaw_rate': 0.0,
            'true_roll': 0.0,
            'true_vertical_velocity': 0.0
        }
        
        # Navigation state starts uninitialized - navigation system must initialize through sensors
        self.nav_state = {
            'lat': 0.0,
            'lon': 0.0,
            'depth': 0.0,
            'heading': 0.0,
            'speed': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw_rate': 0.0,
            'accel_x': 0.0,
            'accel_y': 0.0,
            'accel_z': 9.81,
            'position_uncertainty': 1000.0,  # Start with high uncertainty
            'distance_to_waypoint': 0.0
        }
        
        # Target state - navigation/control targets only
        self.target_state = {
            'target_speed': 0.0,
            'target_heading': 0.0,
            'target_depth': 0.0,
            'target_lat': 0.0,
            'target_lon': 0.0,
            'ballast_cmd': 'OFF',
            'distance_to_waypoint': 0.0
        }
        
        # Task parameters - configuration data for guidance plugins
        self.task_params = {}
        
        self.control_errors = {
            'speed_error': 0.0,
            'heading_error': 0.0,
            'depth_error': 0.0,
            'position_error': 0.0
        }
        
        self.actuator_commands = {
            'thruster_cmd': 0.0,
            'UL': 0.0,
            'UR': 0.0,
            'LR': 0.0,
            'LL': 0.0,
            'rudder_cmd': 0.0,
            'elevator_cmd': 0.0,
            'ballast_cmd': 'OFF',
            'vertical_force': 0.0,
            'side_force': 0.0,
            'yaw_moment': 0.0,
            'pitch_moment': 0.0,
            'roll_moment': 0.0
        }
        
        self.energy_state = {
            'voltage': 24.0,
            'capacity_remaining': 100.0,
            'power_draw': 0.0,
            'current_draw': 0.0
        }
        
        self.sensor_data = {}
        
        # Leak detection system - three watertight cylinders with Blue Robotics SoS sensors
        self.leak_detection = {
            'FWD_WTC': 'DRY',  # Forward watertight cylinder
            'MID_WTC': 'DRY',  # Mid watertight cylinder  
            'AFT_WTC': 'DRY'   # Aft watertight cylinder
        }
        
        self._initial_position_set = False
        self._initial_position = None
    
    def get_time(self) -> float:
        """Return current mission time, or system time if mission_time is not set."""
        with self._state_lock:
            return self.mission_time if self.mission_time > 0.0 else time.time()
    
    def set_initial_position(self, lat: float, lon: float, depth: float, heading: float):
        """Set initial TRUE position only - navigation system initializes separately through sensors."""
        with self._state_lock:
            # Only set the actual physics position (what the vehicle really is)
            self.true_state.update({
                'true_lat': lat,
                'true_lon': lon,
                'true_depth': max(0.0, depth),
                'true_heading': heading % 360.0,
                'true_speed': 0.0,
                'true_pitch': 0.0,
                'true_yaw_rate': 0.0,
                'true_roll': 0.0,
                'true_vertical_velocity': 0.0
            })
            
            # DON'T initialize nav_state - navigation system must determine position through sensors
            # This creates realistic uncertainty and forces GPS_FIX to actually matter
            
            self._initial_position = {'lat': lat, 'lon': lon, 'depth': depth, 'heading': heading}
            self._initial_position_set = True
            
            print(f"Vehicle true position set to ({lat:.6f}, {lon:.6f}) at {depth:.1f}m, heading {heading:.1f}°")
            print("Navigation system must initialize through sensor readings (GPS_FIX)")
    
    def is_initial_position_set(self) -> bool:
        """Check if initial true position has been set"""
        return self._initial_position_set
    
    def get_initial_position(self) -> Dict[str, float]:
        """Get the initial true position (for reference only)"""
        return self._initial_position.copy() if self._initial_position else {'lat': 0.0, 'lon': 0.0, 'depth': 0.0, 'heading': 0.0}
    
    def get_position_dict(self) -> Dict[str, float]:
        """Get current navigation position as dictionary"""
        with self._state_lock:
            return {
                'lat': self.nav_state['lat'],
                'lon': self.nav_state['lon'],
                'depth': self.nav_state['depth'],
                'heading': self.nav_state['heading']
            }
    
    def update_true_state(self, **kwargs):
        """Update true vehicle state (physics simulation only)."""
        with self._state_lock:
            self.true_state.update(kwargs)
            if 'true_depth' in kwargs:
                self.true_state['true_depth'] = max(0.0, kwargs['true_depth'])
            if 'true_heading' in kwargs:
                self.true_state['true_heading'] = kwargs['true_heading'] % 360.0
    
    def update_nav_state(self, **kwargs):
        """Update navigation state (navigation system only)."""
        with self._state_lock:
            self.nav_state.update(kwargs)
            if 'depth' in kwargs:
                self.nav_state['depth'] = max(0.0, kwargs['depth'])
            if 'heading' in kwargs:
                self.nav_state['heading'] = kwargs['heading'] % 360.0
    
    def update_target_state(self, **kwargs):
        """Update target state (navigation/control targets only)."""
        with self._state_lock:
            # Only allow known target state fields to prevent pollution
            allowed_fields = {
                'target_speed', 'target_heading', 'target_depth', 
                'target_lat', 'target_lon', 'ballast_cmd', 'distance_to_waypoint'
            }
            
            for key, value in kwargs.items():
                if key in allowed_fields:
                    self.target_state[key] = value
                else:
                    print(f"Warning: Attempted to set unknown target_state field '{key}' - use task_params instead")
    
    def update_task_params(self, **kwargs):
        """Update task parameters (configuration data for guidance plugins)."""
        with self._state_lock:
            self.task_params.update(kwargs)
    
    def clear_task_params(self):
        """Clear all task parameters (called when starting new task)."""
        with self._state_lock:
            self.task_params = {}
    
    def get_task_params(self) -> Dict[str, Any]:
        """Get copy of current task parameters."""
        with self._state_lock:
            return self.task_params.copy()
    
    def update_control_errors(self, **kwargs):
        """Update control error values."""
        with self._state_lock:
            self.control_errors.update(kwargs)
    
    def update_actuator_commands(self, **kwargs):
        """Update actuator commands and buoyancy state."""
        with self._state_lock:
            self.actuator_commands.update(kwargs)
            self.update_buoyancy_state()
    
    def update_buoyancy_state(self):
        """Update buoyancy state."""
        with self._state_lock:
            tank_volume = self.ballast_state.get('tank_volume', 0.0)
            tank_capacity = self.ballast_state.get('tank_capacity', 0.001982)
            ballast_cmd = self.actuator_commands.get('ballast_cmd', 'OFF')
            
            if ballast_cmd == 'FILL' and tank_volume < tank_capacity * 0.99:
                self.ballast_state['tank_status'] = 'FILLING'
                self.ballast_state['pump_running'] = True
            elif ballast_cmd == 'EMPTY' and tank_volume > 0.0:
                self.ballast_state['tank_status'] = 'EMPTYING'
                self.ballast_state['pump_running'] = True
            elif tank_volume >= tank_capacity * 0.99:
                self.ballast_state['tank_status'] = 'FULL'
                self.ballast_state['pump_running'] = False
                self.ballast_state['vent_open'] = False
            elif tank_volume <= 0.0:
                self.ballast_state['tank_status'] = 'EMPTY'
                self.ballast_state['pump_running'] = False
                self.ballast_state['vent_open'] = False
            else:
                self.ballast_state['tank_status'] = self.ballast_state.get('tank_status', 'EMPTY')
                self.ballast_state['pump_running'] = False
                self.ballast_state['vent_open'] = False
            
            self.buoyancy_state = "NEUTRAL" if tank_volume >= tank_capacity * 0.99 else "POSITIVE"
    
    def update_leak_detection(self, **kwargs):
        """Update leak detection status for watertight cylinders."""
        with self._state_lock:
            self.leak_detection.update(kwargs)
            # Log any leak detections
            for cylinder, status in kwargs.items():
                if status == 'LEAK':
                    self.add_fault(f"Water leak detected in {cylinder}")
                    print(f"LEAK ALERT: Water detected in {cylinder}")
    
    def get_leak_status(self) -> Dict[str, str]:
        """Get current leak detection status for all cylinders."""
        with self._state_lock:
            return self.leak_detection.copy()
    
    def has_leak(self) -> bool:
        """Check if any watertight cylinder has a leak."""
        with self._state_lock:
            return any(status == 'LEAK' for status in self.leak_detection.values())
    
    def get_leaking_cylinders(self) -> list:
        """Get list of cylinders that have leaks."""
        with self._state_lock:
            return [cylinder for cylinder, status in self.leak_detection.items() if status == 'LEAK']
    
    def set_system_state(self, state: SystemState):
        """Set system state."""
        with self._state_lock:
            self.current_state = state
    
    def set_task_complete(self, complete: bool):
        """Set task completion status."""
        with self._state_lock:
            self.task_complete = complete
    
    def add_fault(self, fault: str):
        """Add a fault to the fault list - does NOT trigger failsafe automatically"""
        with self._state_lock:
            if fault not in self.faults:  # Avoid duplicates
                self.faults.append(fault)
                print(f"FAULT ADDED: {fault}")
    
    def clear_faults(self):
        """Clear all faults - does NOT reset failsafe state"""
        with self._state_lock:
            self.faults = []
            print("FAULTS CLEARED")
    
    def has_fault(self, fault: str = None) -> bool:
        """Check if a specific fault exists, or if any faults exist."""
        with self._state_lock:
            if fault is None:
                return len(self.faults) > 0
            else:
                return fault in self.faults
    
    def get_faults(self) -> list:
        """Get a copy of all current faults."""
        with self._state_lock:
            return self.faults.copy()
    
    def clear_fault(self, fault: str):
        """Clear a specific fault."""
        with self._state_lock:
            if fault in self.faults:
                self.faults.remove(fault)
                print(f"FAULT CLEARED: {fault}")
    
    def trigger_failsafe(self, reason: str):
        """Trigger failsafe with reason - should only be called by failsafe module"""
        with self._state_lock:
            self.failsafe_triggered = True
            self.failsafe_reason = reason
            self.current_state = SystemState.FAILSAFE
            print(f"FAILSAFE TRIGGERED: {reason}")
    
    def reset_failsafe(self):
        """Reset failsafe state - should only be called by failsafe module"""
        with self._state_lock:
            self.failsafe_triggered = False
            self.failsafe_reason = ""
            if self.current_state == SystemState.FAILSAFE:
                self.current_state = SystemState.RUNNING
            print("FAILSAFE RESET")
    
    def get_status(self) -> Dict[str, Any]:
        """Get current vehicle state."""
        with self._state_lock:
            return {
                'system_state': self.current_state.value,
                'current_task': self.current_task,
                'task_complete': self.task_complete,
                'task_status': self.task_status,
                'mission_time': self.mission_time,
                'failsafe_triggered': self.failsafe_triggered,
                'failsafe_reason': self.failsafe_reason,
                'faults': self.faults.copy(),
                'true_state': self.true_state.copy(),
                'nav_state': self.nav_state.copy(),
                'target_state': self.target_state.copy(),
                'task_params': self.task_params.copy(),
                'control_errors': self.control_errors.copy(),
                'actuator_commands': self.actuator_commands.copy(),
                'ballast_state': self.ballast_state.copy(),
                'buoyancy_state': self.buoyancy_state,
                'energy_state': self.energy_state.copy(),
                'leak_detection': self.leak_detection.copy()
            }
    
    def get_status_summary(self) -> Dict[str, Any]:
        """Get simplified status summary for telemetry."""
        with self._state_lock:
            return {
                'system_state': self.current_state.value,
                'current_task': self.current_task,
                'position': f"{self.nav_state['lat']:.6f}, {self.nav_state['lon']:.6f}",
                'depth': f"{self.nav_state['depth']:.1f}m",
                'heading': f"{self.nav_state['heading']:.1f}°",
                'speed': f"{self.nav_state['speed']:.2f}m/s",
                'battery': f"{self.energy_state['capacity_remaining']:.0f}%",
                'buoyancy': self.buoyancy_state,
                'mission_time': f"{self.mission_time:.0f}s",
                'faults': len(self.faults),
                'leak_status': self.leak_detection.copy()
            }