#!/usr/bin/env python3
"""
hal_commands.py - Command handling for SIMPLR AUV
Processes incoming commands from GCS and manages mission state
"""

import time
from typing import Any, Dict, Optional


class CommandHandler:
    """Handles incoming commands from GCS"""
    
    def __init__(self, hal_instance, vehicle_state, config: Dict[str, Any]):
        self.hal = hal_instance
        self.vehicle_state = vehicle_state
        self.config = config
        
        # Mission control flags
        self.mission_start_requested = False
        self.mission_stop_requested = False
        self.mission_pause_requested = False
        self.emergency_stop_requested = False
        self.shutdown_requested = False
        
        # Mission data
        self.uploaded_mission_data: Optional[Dict[str, Any]] = None
    
    def handle_incoming_message(self, message: Dict[str, Any]):
        """Route incoming message to appropriate handler"""
        try:
            msg_type = message.get('msg_type', '')
            payload = message.get('payload', {})
            timestamp = message.get('timestamp', time.time())
            
            if self.hal.verbose:
                print(f"Commands: Received {msg_type} from GCS")
            
            handlers = {
                'mission_upload': self._handle_mission_upload,
                'mission_download': self._handle_mission_download,
                'mission_start': self._handle_mission_start,
                'mission_stop': self._handle_mission_stop,
                'mission_pause': self._handle_mission_pause,
                'emergency_stop': self._handle_emergency_stop,
                'ping': self._handle_ping,
                'shutdown': self._handle_shutdown
            }
            
            handler = handlers.get(msg_type)
            if handler:
                handler(payload, timestamp)
            else:
                if self.hal.verbose:
                    print(f"Commands: Unhandled message type: {msg_type}")
        
        except Exception as e:
            if self.hal.verbose:
                print(f"Commands: Error handling message: {e}")
    
    def _handle_mission_upload(self, payload: Dict[str, Any], timestamp: float):
        """Handle mission upload from GCS"""
        try:
            mission_name = payload.get('name', 'uploaded_mission')
            mission_data = payload.get('mission', {})
            
            if not mission_data:
                self._send_ack('mission_upload_ack', False, "No mission data received")
                return
            
            # Validate mission structure
            required = ['mission_info', 'vehicle_config', 'tasks']
            for field in required:
                if field not in mission_data:
                    self._send_ack('mission_upload_ack', False, f"Missing required field: {field}")
                    return
            
            self.uploaded_mission_data = mission_data
            self.hal.mark_mission_loaded(True)
            
            # Seed initial position if present
            try:
                init_pos = mission_data.get('vehicle_config', {}).get('initial_position', {})
                lat0 = float(init_pos.get('lat', 0.0))
                lon0 = float(init_pos.get('lon', 0.0))
            except Exception:
                lat0, lon0 = 0.0, 0.0
            
            if self._coords_valid(lat0, lon0):
                self.vehicle_state.nav_state['lat'] = lat0
                self.vehicle_state.nav_state['lon'] = lon0
                print(f"Commands: Initialized nav origin from mission file ({lat0:.6f}, {lon0:.6f})")
            
            task_count = len(mission_data.get('tasks', []))
            print(f"Commands: ✓ Received mission upload: '{mission_name}' with {task_count} tasks")
            self._send_ack('mission_upload_ack', True, 
                          f"Mission {mission_name} received and validated successfully")
        
        except Exception as e:
            error_msg = f"Mission upload error: {e}"
            print(f"Commands: {error_msg}")
            self._send_ack('mission_upload_ack', False, error_msg)
    
    def _handle_mission_download(self, payload: Dict[str, Any], timestamp: float):
        """Handle mission download request from GCS"""
        try:
            response_payload = {
                'name': 'current_mission.json',
                'mission': {
                    'mission_info': {
                        'name': 'Current Mission',
                        'description': 'Downloaded from vehicle'
                    },
                    'vehicle_config': {
                        'initial_position': {'lat': 0.0, 'lon': 0.0, 'depth': 0.0}
                    },
                    'tasks': []
                },
                'download_time': time.time()
            }
            response = {
                'timestamp': time.time(),
                'msg_type': 'mission_download_response',
                'payload': response_payload
            }
            self.hal.network_mgr.send_raw_json(response)
            print("Commands: Mission download response sent")
        except Exception as e:
            print(f"Commands: Mission download error: {e}")
    
    def _handle_mission_start(self, payload: Dict[str, Any], timestamp: float):
        """Handle mission start command from GCS"""
        try:
            execution_mode = payload.get('execution_mode', 'SIMULATION')
            mission_name = payload.get('mission_name', 'Unknown')
            print(f"Commands: ✓ Mission start command received - {mission_name} in {execution_mode} mode")
            self.mission_start_requested = True
            self._send_ack('mission_start_ack', True, f"Mission {mission_name} start request queued")
        except Exception as e:
            error_msg = f"Mission start error: {e}"
            print(f"Commands: {error_msg}")
            self._send_ack('mission_start_ack', False, error_msg)
    
    def _handle_mission_stop(self, payload: Dict[str, Any], timestamp: float):
        """Handle mission stop command from GCS"""
        try:
            print("Commands: Mission stop command received")
            self.mission_stop_requested = True
            self._send_ack('mission_stop_ack', True, "Mission stop request queued")
        except Exception as e:
            error_msg = f"Mission stop error: {e}"
            print(f"Commands: {error_msg}")
            self._send_ack('mission_stop_ack', False, error_msg)
    
    def _handle_mission_pause(self, payload: Dict[str, Any], timestamp: float):
        """Handle mission pause command from GCS"""
        try:
            print("Commands: Mission pause command received")
            self.mission_pause_requested = True
            self._send_ack('mission_pause_ack', True, "Mission pause request queued")
        except Exception as e:
            error_msg = f"Mission pause error: {e}"
            print(f"Commands: {error_msg}")
            self._send_ack('mission_pause_ack', False, error_msg)
    
    def _handle_emergency_stop(self, payload: Dict[str, Any], timestamp: float):
        """Handle emergency stop command from GCS"""
        try:
            print("Commands: EMERGENCY STOP command received")
            self.emergency_stop_requested = True
            self.hal.emergency_stop()
            self._send_ack('emergency_stop_ack', True, "Emergency stop executed successfully")
        except Exception as e:
            error_msg = f"Emergency stop error: {e}"
            print(f"Commands: {error_msg}")
            self._send_ack('emergency_stop_ack', False, error_msg)
    
    def _handle_shutdown(self, payload: Dict[str, Any], timestamp: float):
        """Handle shutdown command from GCS"""
        try:
            print("Commands: Shutdown command received")
            self.shutdown_requested = True
            self._send_ack('shutdown_ack', True, "Vehicle shutdown requested by GCS")
        except Exception as e:
            print(f"Commands: Shutdown error: {e}")
    
    def _handle_ping(self, payload: Dict[str, Any], timestamp: float):
        """Handle ping from GCS"""
        try:
            ping_time = payload.get('ping_time', timestamp)
            response = {
                'timestamp': time.time(),
                'msg_type': 'pong',
                'payload': {
                    'ping_time': ping_time,
                    'pong_time': time.time(),
                    'vehicle_id': self.hal.vehicle_id,
                    'telemetry_active': self.hal._telemetry_activated
                }
            }
            self.hal.network_mgr.send_raw_json(response)
            if self.hal.verbose:
                print("Commands: Pong sent in response to ping")
        except Exception as e:
            print(f"Commands: Ping response error: {e}")
    
    def _send_ack(self, ack_type: str, success: bool, message: str = ""):
        """Send acknowledgment message to GCS"""
        try:
            ack_payload = {
                'success': success,
                'message': message,
                'timestamp': time.time(),
                'vehicle_id': self.hal.vehicle_id,
                'telemetry_active': self.hal._telemetry_activated
            }
            if not success:
                ack_payload['error'] = message
            
            response = {
                'timestamp': time.time(),
                'msg_type': ack_type,
                'payload': ack_payload
            }
            self.hal.network_mgr.send_raw_json(response)
            
            if self.hal.verbose:
                status = "SUCCESS" if success else "FAILED"
                print(f"Commands: Sent {ack_type} - {status}: {message}")
        except Exception as e:
            print(f"Commands: Error sending ack: {e}")
    
    @staticmethod
    def _coords_valid(lat: float, lon: float) -> bool:
        """Validate coordinates"""
        if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
            return False
        if abs(lat) < 1e-6 and abs(lon) < 1e-6:
            return False
        if abs(lat) > 90 or abs(lon) > 180:
            return False
        return True
    
    def clear_mission_start_flag(self):
        """Clear mission start flag"""
        self.mission_start_requested = False
    
    def clear_mission_stop_flag(self):
        """Clear mission stop flag"""
        self.mission_stop_requested = False
    
    def clear_emergency_stop_flag(self):
        """Clear emergency stop flag"""
        self.emergency_stop_requested = False
