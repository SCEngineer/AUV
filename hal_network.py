#!/usr/bin/env python3
"""
hal_network.py - Network and GCS communication management for SIMPLR AUV
Handles TCP connections, depth-based connectivity, and message transport
"""

import json
import platform
import socket
import subprocess
import time
from typing import Any, Dict, Optional


class NetworkManager:
    """Manages network connectivity and GCS communication"""
    
    def __init__(self, hal_instance, config: Dict[str, Any], vehicle_state):
        self.hal = hal_instance
        self.config = config
        self.vehicle_state = vehicle_state
        
        # Network configuration
        self.wifi_enabled = bool(config.get('wifi_enabled', False))
        self.gcs_host = config.get('tcp_host', '0.0.0.0') # changed this from 127.0.0.1
        self.gcs_port = int(config.get('tcp_port', 15000))
        self.gcs_socket: Optional[socket.socket] = None
        
        # Connection state
        self.connected_to_gcs = False
        self.connection_attempts = 0
        self.last_connection_attempt = 0.0
        self.messages_sent = 0
        self.network_errors = 0
        self.max_network_errors = 10
        self.connection_quality = 'UNKNOWN'
        
        # Timing
        self.wifi_timeout = 15.0 if self.wifi_enabled else 5.0
        self.wifi_keepalive_interval = 30.0
        self.last_keepalive = 0.0
        
        # Depth-aware reconnection
        self.last_surface_time = time.time()
        self.surface_reconnect_delay = 2.0
        self._was_at_surface = False
        self._last_depth_log_time = time.time()
        
        # RX buffer for newline-delimited JSON
        self._rx_buffer = ""
        
        # Setup
        if self.wifi_enabled:
            self._configure_firewall()
    
    def _configure_firewall(self):
        """Configure Windows firewall if needed"""
        if platform.system() != "Windows":
            return
        try:
            subprocess.run(
                ["netsh", "advfirewall", "firewall", "delete", "rule",
                 f"name=SIMPLR-AUV Port {self.gcs_port}"],
                capture_output=True, timeout=10
            )
            subprocess.run(
                ["netsh", "advfirewall", "firewall", "add", "rule",
                 f"name=SIMPLR-AUV Port {self.gcs_port} Outbound",
                 "dir=out", "action=allow", "protocol=TCP",
                 f"remoteport={self.gcs_port}", "enable=yes", "profile=any"],
                check=True, timeout=10
            )
            self.hal._log(f"Network: Configured Windows Firewall for port {self.gcs_port}")
        except Exception as e:
            self.hal._log(f"Network: Firewall configuration failed: {e}")
    
    def is_communication_disabled(self) -> bool:
        """Check if communication is disabled due to depth or telemetry state"""
        if not self.hal._telemetry_activated:
            return True
        
        try:
            current_depth = self.vehicle_state.nav_state.get('depth', 0.0)
            disabled = current_depth >= 1.0
            
            if time.time() - self._last_depth_log_time > 5.0:
                mode = "WiFi" if self.wifi_enabled else "localhost"
                status = "DISABLED" if disabled else "ENABLED"
                self.hal._log(
                    f"Network: Telemetry depth check - depth={current_depth:.3f}m, "
                    f"status={status} (threshold=1.0m), mode={mode}"
                )
                self._last_depth_log_time = time.time()
            return disabled
        except Exception as e:
            self.hal._log(f"Network: Error checking communication disabled: {e}")
            return True
    
    def is_at_surface(self) -> bool:
        """Check if vehicle is at surface (depth < 1.0m)"""
        try:
            current_depth = self.vehicle_state.nav_state.get('depth', 0.0)
            return current_depth < 1.0
        except Exception:
            return True
    
    def handle_depth_based_connection(self):
        """Manage connection based on depth and telemetry state"""
        if not self.hal._telemetry_activated:
            if self.connected_to_gcs:
                pass  # Keep command connection alive
            return
        
        comm_disabled = self.is_communication_disabled()
        
        if comm_disabled:
            if self.connected_to_gcs:
                depth = self.vehicle_state.nav_state.get('depth', 0.0)
                mode = "WiFi" if self.wifi_enabled else "localhost"
                self.hal._log(f"Network: {mode} telemetry DISABLED - depth {depth:.3f}m (>= 1.0m)")
                reason = f"Submerged at {depth:.1f}m"
                self.disconnect_from_gcs(reason)
            self._was_at_surface = False
            return
        
        if self.is_at_surface():
            now = time.time()
            if not self.connected_to_gcs:
                if not self._was_at_surface:
                    self.last_surface_time = now
                    mode = "WiFi" if self.wifi_enabled else "localhost"
                    depth = self.vehicle_state.nav_state.get('depth', 0.0)
                    self.hal._log(f"Network: {mode} telemetry RE-ENABLED - depth {depth:.3f}m (< 1.0m)")
                self._was_at_surface = True
                if now - self.last_surface_time >= self.surface_reconnect_delay:
                    self.connect_to_gcs()
            else:
                self._was_at_surface = True
        else:
            self._was_at_surface = False
    
    def connect_to_gcs(self) -> bool:
        """Attempt to connect to GCS"""
        if self.hal._telemetry_activated:
            if self.is_communication_disabled():
                return False
            if self.is_at_surface() and (time.time() - self.last_surface_time) < self.surface_reconnect_delay:
                return False
        
        now = time.time()
        retry_interval = min(30.0, max(5.0, self.connection_attempts * 2.0))
        if now - self.last_connection_attempt < retry_interval:
            return False
        
        self.last_connection_attempt = now
        self.connection_attempts += 1
        
        try:
            if self.gcs_socket:
                try:
                    self.gcs_socket.close()
                except Exception:
                    pass
                self.gcs_socket = None
            
            self.gcs_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._configure_socket()
            
            self.gcs_socket.connect((self.gcs_host, self.gcs_port))
            self.gcs_socket.setblocking(False)
            
            self.connected_to_gcs = True
            self.network_errors = 0
            self.connection_quality = 'GOOD'
            
            conn_type = "WiFi" if self.wifi_enabled else "localhost"
            depth = self.vehicle_state.nav_state.get('depth', 0.0)
            telemetry_status = "ACTIVE" if self.hal._telemetry_activated else "INACTIVE"
            self.hal._log(f"Network: ✓ Connected to {conn_type} GCS (telemetry: {telemetry_status}, depth: {depth:.3f}m)")
            
            if self.hal._telemetry_activated:
                self.hal._log("Network: Telemetry active - sending vehicle_status messages")
            else:
                self.hal._log("Network: Command connection - ready to receive commands")
            
            return True
        
        except ConnectionRefusedError:
            self.connection_quality = 'REFUSED'
        except socket.timeout:
            self.connection_quality = 'TIMEOUT'
        except Exception as e:
            self.connection_quality = 'ERROR'
            if self.connection_attempts % 10 == 1:
                self.hal._log(f"Network: Connection error: {e}")
        
        if self.gcs_socket:
            try:
                self.gcs_socket.close()
            except Exception:
                pass
            self.gcs_socket = None
        
        self.connected_to_gcs = False
        return False
    
    def _configure_socket(self):
        """Configure socket options"""
        if not self.gcs_socket:
            return
        try:
            self.gcs_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            self.gcs_socket.settimeout(self.wifi_timeout)
            if self.wifi_enabled and platform.system() == "Windows":
                self.gcs_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 16384)
                self.gcs_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception as e:
            self.hal._log(f"Network: Socket configuration error: {e}")
    
    def disconnect_from_gcs(self, reason: str):
        """Disconnect from GCS"""
        try:
            if self.gcs_socket:
                try:
                    self.gcs_socket.close()
                except Exception:
                    pass
                self.gcs_socket = None
            self.connected_to_gcs = False
            self.hal._log(f"Network: Disconnected from GCS - {reason}")
        except Exception as e:
            self.hal._log(f"Network: Error during disconnect: {e}")
    
    def send_raw_json(self, message: dict) -> bool:
        """Send a JSON message to GCS"""
        try:
            if self.gcs_socket and self.connected_to_gcs:
                json_str = json.dumps(message) + '\n'
                self.gcs_socket.sendall(json_str.encode('utf-8'))
                self.messages_sent += 1
                return True
            return False
        except Exception as e:
            print(f"Network: Error sending message to GCS: {e}")
            self.connected_to_gcs = False
            return False
    
    def process_incoming_messages(self, message_handler_callback):
        """Process incoming messages from GCS"""
        if not self.gcs_socket:
            return
        
        try:
            try:
                chunk = self.gcs_socket.recv(4096)
            except (BlockingIOError, socket.timeout):
                return
            
            if not chunk:
                if self.hal.verbose:
                    print("Network: GCS closed connection")
                self.disconnect_from_gcs("Remote closed")
                return
            
            self._rx_buffer += chunk.decode('utf-8', errors='ignore')
            lines = self._rx_buffer.split('\n')
            self._rx_buffer = lines[-1]
            
            for line in lines[:-1]:
                line = line.strip()
                if not line:
                    continue
                try:
                    message = json.loads(line)
                    message_handler_callback(message)
                    if self.hal.verbose:
                        print(f"Network: Processed {message.get('msg_type', 'unknown')} from GCS")
                except json.JSONDecodeError:
                    if self.hal.verbose:
                        print(f"Network: Invalid JSON from GCS: {line}")
        
        except Exception as e:
            if self.hal.verbose:
                print(f"Network: Incoming message processing error: {e}")
            self.network_errors += 1
            if self.network_errors >= self.max_network_errors:
                self.disconnect_from_gcs("Too many RX errors")
    
    def send_keepalive(self, vehicle_id: str):
        """Send keepalive message to GCS"""
        current_time = time.time()
        if current_time - self.last_keepalive < self.wifi_keepalive_interval:
            return
        
        try:
            keepalive_msg = {
                'timestamp': current_time,
                'msg_type': 'keepalive',
                'payload': {
                    'vehicle_id': vehicle_id,
                    'status': 'alive',
                    'telemetry_active': self.hal._telemetry_activated
                }
            }
            self.send_raw_json(keepalive_msg)
            self.last_keepalive = current_time
        except Exception as e:
            if self.hal.verbose:
                print(f"Network: Keepalive failed: {e}")
            self.connected_to_gcs = False
    
    def get_status(self) -> Dict[str, Any]:
        """Get network connection status"""
        current_depth = self.vehicle_state.nav_state.get('depth', 0.0)
        return {
            'connected_to_gcs': self.connected_to_gcs,
            'gcs_host': self.gcs_host,
            'gcs_port': self.gcs_port,
            'connection_attempts': self.connection_attempts,
            'messages_sent': self.messages_sent,
            'wifi_enabled': self.wifi_enabled,
            'connection_quality': self.connection_quality,
            'network_errors': self.network_errors,
            'current_depth': current_depth,
            'wifi_available_at_depth': (not self.is_communication_disabled()) if self.wifi_enabled else True
        }
