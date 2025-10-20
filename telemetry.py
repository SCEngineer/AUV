#!/usr/bin/env python3
"""
telemetry.py - Simple JSON telemetry module to replace MAVLINK
Sends vehicle state, target parameters, and control data as JSON messages over TCP
"""

import time
import threading
import queue
from typing import Dict, Any, Optional
import json
import math

class TelemetryMessage:
    """Simple telemetry message container"""
    def __init__(self, msg_type: str, payload: Dict[str, Any]):
        self.msg_type = msg_type
        self.payload = payload
        self.timestamp = time.time()

class TelemetryModule:
    def __init__(self, vehicle_state, update_rate: float = 2.0):
        """
        Initialize telemetry module
        
        Args:
            vehicle_state: Reference to vehicle state object
            update_rate: Rate in Hz to send telemetry updates
        """
        self.vehicle_state = vehicle_state
        self.update_rate = update_rate
        
        # Threading control
        self.running = False
        self.thread = None
        self.message_queue = queue.Queue(maxsize=50)
        
        # Timing
        self.last_update_time = 0.0
        self.update_interval = 1.0 / update_rate
        
        print(f"Telemetry Module initialized - update rate: {update_rate} Hz")
    
    def start(self):
        """Start the telemetry generation thread"""
        if self.running:
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.thread.start()
        print("Telemetry generation started")
    
    def stop(self):
        """Stop the telemetry generation"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        print("Telemetry generation stopped")
    
    def get_message(self, timeout: float = 0.1) -> Optional[TelemetryMessage]:
        """Get next telemetry message from queue"""
        try:
            return self.message_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def get_all_pending_messages(self) -> list:
        """Get all pending messages from queue"""
        messages = []
        while True:
            try:
                msg = self.message_queue.get_nowait()
                messages.append(msg)
            except queue.Empty:
                break
        return messages
    
    def _safe_get_attr(self, obj, attr_name: str, default=None):
        """Safely get attribute from object with fallback"""
        try:
            value = getattr(obj, attr_name, default)
            return value if value is not None else default
        except:
            return default
    
    def _safe_get_dict(self, dictionary: dict, key: str, default=0.0):
        """Safely get value from dictionary with null checking"""
        try:
            if not isinstance(dictionary, dict):
                return default
            value = dictionary.get(key, default)
            return value if value is not None else default
        except:
            return default
    
    def _safe_float(self, value, default=0.0):
        """Safely convert to float with null checking"""
        try:
            if value is None:
                return default
            return float(value)
        except (TypeError, ValueError):
            return default
    
    def _safe_int(self, value, default=0):
        """Safely convert to int with null checking"""
        try:
            if value is None:
                return default
            return int(value)
        except (TypeError, ValueError):
            return default
    
    def _telemetry_loop(self):
        """Main telemetry generation loop"""
        while self.running:
            try:
                current_time = time.time()
                
                # Check if it's time to send an update
                if current_time - self.last_update_time >= self.update_interval:
                    # Generate comprehensive telemetry message
                    message = self._generate_telemetry_update(current_time)
                    if message:
                        try:
                            self.message_queue.put_nowait(message)
                            self.last_update_time = current_time
                        except queue.Full:
                            # Drop oldest message to make room
                            try:
                                self.message_queue.get_nowait()
                                self.message_queue.put_nowait(message)
                                self.last_update_time = current_time
                            except queue.Empty:
                                pass
                
                # Sleep briefly to avoid excessive CPU usage
                time.sleep(0.1)  # 10Hz loop for checking
                
            except Exception as e:
                print(f"Telemetry generation error: {e}")
                time.sleep(0.5)
    
    def _generate_telemetry_update(self, timestamp: float) -> Optional[TelemetryMessage]:
        """Generate a comprehensive telemetry update message"""
        try:
            # Get current vehicle state data
            nav_state = self._safe_get_attr(self.vehicle_state, 'nav_state', {})
            target_state = self._safe_get_attr(self.vehicle_state, 'target_state', {})
            control_errors = self._safe_get_attr(self.vehicle_state, 'control_errors', {})
            energy_state = self._safe_get_attr(self.vehicle_state, 'energy_state', {})
            ballast_state = self._safe_get_attr(self.vehicle_state, 'ballast_state', {})
            actuator_commands = self._safe_get_attr(self.vehicle_state, 'actuator_commands', {})
            
            # Get system state
            try:
                system_state = self.vehicle_state.current_state.value
            except:
                system_state = 'UNKNOWN'
            
            # Get current task
            current_task = self._safe_get_attr(self.vehicle_state, 'current_task', 'NONE')
            
            # Get mission time
            mission_time = self._safe_get_attr(self.vehicle_state, 'mission_time', 0.0)
            
            # Get leak status
            try:
                leak_status = self.vehicle_state.get_leak_status()
                has_leak = self.vehicle_state.has_leak()
            except:
                leak_status = {}
                has_leak = False
            
            # Extract position data
            lat = self._safe_float(self._safe_get_dict(nav_state, 'lat', 0.0))
            lon = self._safe_float(self._safe_get_dict(nav_state, 'lon', 0.0))
            depth = self._safe_float(self._safe_get_dict(nav_state, 'depth', 0.0))
            heading = self._safe_float(self._safe_get_dict(nav_state, 'heading', 0.0))
            speed = self._safe_float(self._safe_get_dict(nav_state, 'speed', 0.0))
            
            # Extract target data
            target_depth = self._safe_float(self._safe_get_dict(target_state, 'target_depth', 0.0))
            target_heading = self._safe_float(self._safe_get_dict(target_state, 'target_heading', 0.0))
            target_speed = self._safe_float(self._safe_get_dict(target_state, 'target_speed', 0.0))
            
            # Extract control errors
            depth_error = self._safe_float(self._safe_get_dict(control_errors, 'depth_error', 0.0))
            heading_error = self._safe_float(self._safe_get_dict(control_errors, 'heading_error', 0.0))
            speed_error = self._safe_float(self._safe_get_dict(control_errors, 'speed_error', 0.0))
            
            # Extract energy data
            battery_remaining = self._safe_float(self._safe_get_dict(energy_state, 'capacity_remaining', 100.0))
            voltage = self._safe_float(self._safe_get_dict(energy_state, 'voltage', 24.0))
            current_draw = self._safe_float(self._safe_get_dict(energy_state, 'current_draw', 0.0))
            power_draw = self._safe_float(self._safe_get_dict(energy_state, 'power_draw', 0.0))
            
            # Extract actuator commands
            thruster_cmd = self._safe_float(self._safe_get_dict(actuator_commands, 'thruster_cmd', 0.0))
            ballast_cmd = self._safe_get_dict(actuator_commands, 'ballast_cmd', 'OFF')
            
            # Extract ballast state
            tank_volume = self._safe_float(self._safe_get_dict(ballast_state, 'tank_volume', 0.0))
            tank_capacity = self._safe_float(self._safe_get_dict(ballast_state, 'tank_capacity', 0.001982))
            tank_status = self._safe_get_dict(ballast_state, 'tank_status', 'UNKNOWN')
            
            # Build comprehensive payload
            payload = {
                # Position and navigation
                'position': {
                    'lat': lat,
                    'lon': lon,
                    'depth': depth,
                    'heading': heading,
                    'speed': speed
                },
                
                # Target parameters
                'targets': {
                    'depth': target_depth,
                    'heading': target_heading,
                    'speed': target_speed
                },
                
                # Control errors
                'errors': {
                    'depth_error': depth_error,
                    'heading_error': heading_error,
                    'speed_error': speed_error
                },
                
                # System status
                'system': {
                    'state': system_state,
                    'current_task': str(current_task),
                    'mission_time': mission_time,
                    'has_leak': has_leak,
                    'leak_status': leak_status
                },
                
                # Energy system
                'energy': {
                    'battery_remaining': battery_remaining,
                    'voltage': voltage,
                    'current_draw': current_draw,
                    'power_draw': power_draw
                },
                
                # Ballast system
                'ballast': {
                    'tank_volume': tank_volume,
                    'tank_capacity': tank_capacity,
                    'tank_status': tank_status,
                    'fill_percentage': (tank_volume / tank_capacity * 100.0) if tank_capacity > 0 else 0.0
                },
                
                # Actuator commands
                'actuators': {
                    'thruster_cmd': thruster_cmd,
                    'ballast_cmd': ballast_cmd
                },
                
                # Timestamp
                'timestamp': timestamp
            }
            
            return TelemetryMessage('VEHICLE_STATUS', payload)
            
        except Exception as e:
            print(f"Error generating telemetry update: {e}")
            return None
    
    def set_update_rate(self, rate_hz: float):
        """Set the telemetry update rate"""
        self.update_rate = max(0.1, min(10.0, rate_hz))  # Clamp to reasonable range
        self.update_interval = 1.0 / self.update_rate
        print(f"Telemetry: Set update rate to {rate_hz} Hz")
    
    def get_status(self) -> Dict[str, Any]:
        """Get telemetry module status"""
        return {
            'running': self.running,
            'update_rate': self.update_rate,
            'queue_size': self.message_queue.qsize(),
            'last_update_time': self.last_update_time
        }
    
    def __del__(self):
        """Cleanup when module is destroyed"""
        self.stop()