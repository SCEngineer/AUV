#!/usr/bin/env python3
"""
hal_telemetry.py - Telemetry management for SIMPLR AUV
Handles telemetry formatting, transmission, and lifecycle
"""

import time
import traceback
from typing import Any, Dict, Optional


class TelemetryManager:
    """Manages telemetry data formatting and transmission"""
    
    def __init__(self, hal_instance, vehicle_state, config: Dict[str, Any]):
        self.hal = hal_instance
        self.vehicle_state = vehicle_state
        self.config = config
        
        # Telemetry module instance
        self.telemetry_module = None
        self.telemetry_enabled = False
        
        # Telemetry rate
        self.update_rate = float(config.get('telemetry_rate_hz', 1.0))
    
    def initialize_telemetry(self):
        """Initialize telemetry module - called when all conditions are met"""
        if self.hal._telemetry_activated:
            self.hal._log("Telemetry: Already activated, skipping reinitialization")
            return
        
        # Check prerequisites
        if not self.hal._mission_loaded:
            self.hal._log("Telemetry: Init blocked - mission not loaded yet")
            return
        if not self.hal.initialized:
            self.hal._log("Telemetry: Init blocked - vehicle not initialized yet")
            return
        if not self.hal._mission_started:
            self.hal._log("Telemetry: Init blocked - mission not started yet")
            return
        
        try:
            from telemetry import TelemetryModule
            self.telemetry_module = TelemetryModule(self.vehicle_state, update_rate=self.update_rate)
            self.telemetry_module.start()
            self.telemetry_enabled = True
            self.hal._telemetry_activated = True
            
            conn_type = "WiFi" if self.hal.network_mgr.wifi_enabled else "local"
            self.hal._log(f"Telemetry: ✓ ACTIVATED for {conn_type} connection to "
                         f"{self.hal.network_mgr.gcs_host}:{self.hal.network_mgr.gcs_port}")
            self.hal._log(f"Telemetry: Will send vehicle_status at {self.update_rate} Hz")
        except ImportError:
            print("Telemetry: Module not available")
            self.telemetry_enabled = False
        except Exception as e:
            print(f"Telemetry: Failed to initialize: {e}")
            if self.hal.verbose:
                traceback.print_exc()
            self.telemetry_enabled = False
    
    def process_telemetry_messages(self, network_mgr):
        """Send outbound telemetry frames - ONLY if telemetry is activated"""
        if not self.hal._telemetry_activated or not self.telemetry_enabled:
            return
        
        nav = self.vehicle_state.nav_state
        lat, lon = nav.get('lat', 0.0), nav.get('lon', 0.0)
        if not self._coords_valid(lat, lon):
            return
        
        try:
            if self.telemetry_module:
                messages = self.telemetry_module.get_all_pending_messages()
                for message in messages:
                    if not self._send_message_to_gcs(message.timestamp, message.payload, network_mgr):
                        break
            else:
                # Fallback: create telemetry payload directly
                fallback_payload = self.convert_to_telemetry_format({})
                if fallback_payload:
                    self._send_message_to_gcs(time.time(), fallback_payload, network_mgr)
        except Exception as e:
            if self.hal.verbose:
                print(f"Telemetry: Processing error: {e}")
            network_mgr.connected_to_gcs = False
    
    def _send_message_to_gcs(self, timestamp: float, payload: Dict[str, Any], network_mgr) -> bool:
        """Send a single telemetry message to GCS"""
        if not network_mgr.gcs_socket or not network_mgr.connected_to_gcs:
            return False
        
        try:
            pos = payload.get('position', {})
            if not self._coords_valid(pos.get('lat', 0.0), pos.get('lon', 0.0)):
                return True
            
            message_data = {
                'timestamp': timestamp,
                'msg_type': 'VEHICLE_STATUS',
                'payload': payload
            }
            network_mgr.send_raw_json(message_data)
            network_mgr.network_errors = 0
            network_mgr.connection_quality = 'GOOD'
            return True
        except Exception as e:
            if self.hal.verbose:
                print(f"Telemetry: Error sending message: {e}")
            return False
    
    def convert_to_telemetry_format(self, _payload: Dict[str, Any]) -> Dict[str, Any]:
        """Convert vehicle state to telemetry format"""
        try:
            nav_state = self.vehicle_state.nav_state
            system_state = getattr(self.vehicle_state.current_state, 'value', 'UNKNOWN')
            current_task = getattr(self.vehicle_state, 'current_task', 'UNKNOWN')
            
            mission_time = getattr(self.vehicle_state, 'mission_time', 0.0)
            if mission_time <= 0 and getattr(self.vehicle_state, 'mission_start_time', None):
                mission_time = time.time() - self.vehicle_state.mission_start_time
            
            depth = nav_state.get('depth', 0.0)
            nav_mode = self._determine_nav_mode(depth, system_state)
            
            payload = {
                'nav_mode': nav_mode,
                'position': {
                    'lat': nav_state.get('lat', 0.0),
                    'lon': nav_state.get('lon', 0.0),
                    'depth': depth,
                    'heading': nav_state.get('heading', 0.0),
                    'speed': nav_state.get('speed', 0.0)
                },
                'system': {
                    'state': system_state,
                    'current_task': current_task,
                    'mission_time': mission_time
                },
                'navigation': {
                    'uncertainty': nav_state.get('position_uncertainty', 1.0)
                },
                'energy': {
                    'battery_remaining': self.vehicle_state.energy_state.get('capacity_remaining', 100.0),
                    'voltage': self.vehicle_state.energy_state.get('voltage', 12.0),
                    'power_draw': self.vehicle_state.energy_state.get('power_draw', 0.0),
                    'current_draw': self.vehicle_state.energy_state.get('current_draw', 0.0)
                },
                'targets': {
                    'depth': nav_state.get('target_depth', depth),
                    'heading': nav_state.get('target_heading', nav_state.get('heading', 0.0)),
                    'speed': nav_state.get('target_speed', nav_state.get('speed', 0.0))
                },
                'errors': {
                    'depth_error': nav_state.get('depth_error', 0.0),
                    'heading_error': nav_state.get('heading_error', 0.0),
                    'speed_error': nav_state.get('speed_error', 0.0)
                }
            }
            return payload
        except Exception as e:
            if self.hal.verbose:
                print(f"Telemetry: Error converting payload format: {e}")
            return {}
    
    def _determine_nav_mode(self, depth: float, system_state: str) -> str:
        """Determine navigation mode based on depth"""
        return "GPS" if depth < 1.0 else "INERTIAL"
    
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
    
    def get_status(self) -> Dict[str, Any]:
        """Get telemetry system status"""
        status = {
            'enabled': self.telemetry_enabled,
            'activated': self.hal._telemetry_activated,
            'wifi_mode': self.hal.network_mgr.wifi_enabled,
            'vehicle_id': self.hal.vehicle_id,
            'mission_loaded': self.hal._mission_loaded,
            'mission_started': self.hal._mission_started,
            'initialized': self.hal.initialized,
            'transport': {
                'type': 'tcp',
                'gcs_host': self.hal.network_mgr.gcs_host,
                'gcs_port': self.hal.network_mgr.gcs_port,
                'connected': self.hal.network_mgr.connected_to_gcs,
                'connection_quality': self.hal.network_mgr.connection_quality
            }
        }
        if self.telemetry_module:
            try:
                status['module'] = self.telemetry_module.get_status()
            except Exception:
                status['module'] = {'error': 'Module status unavailable'}
        return status
    
    def stop(self):
        """Stop telemetry module"""
        if self.telemetry_module:
            try:
                self.telemetry_module.stop()
            except Exception:
                pass