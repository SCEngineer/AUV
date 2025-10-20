#!/usr/bin/env python3
"""
enhanced_hal.py - Enhanced Hardware Abstraction Layer for SIMPLR AUV
WITH RESTORED CONTINUOUS BALLAST SIMULATION
"""

from __future__ import annotations

import json
import logging
import platform
import socket
import subprocess
import time
import traceback
from typing import Any, Dict, Optional

# NOTE: VehicleState is expected to be available in your project.
try:
    from vehicle_state import VehicleState
except Exception:
    class VehicleState:  # type: ignore
        def __init__(self):
            self.nav_state: Dict[str, Any] = {}
            self.energy_state: Dict[str, Any] = {}
            self.ballast_state: Dict[str, Any] = {}
            self.actuator_commands: Dict[str, Any] = {}
            self.sensor_data: Dict[str, Any] = {}
            self.current_state = type("S", (), {"value": "UNKNOWN"})()
            self.mission_start_time = None
            self._state_lock = type("L", (), {"__enter__":lambda s:None, "__exit__":lambda s,a,b,c:None})()

        def add_fault(self, msg: str):
            print("VehicleState FAULT:", msg)


class EnhancedHAL:
    """Enhanced Hardware Abstraction Layer with restored continuous ballast simulation"""

    def __init__(self, config: Dict[str, Any], vehicle_state: VehicleState):
        self.config = config
        self.vehicle_state = vehicle_state
        self.mode = config.get('mode', 'simulation')
        self.verbose = bool(config.get('verbose', False))

        # ---- Logging ----
        self.hal_logger = None
        if self.verbose:
            self.hal_logger = logging.getLogger('HAL_Telemetry')
            self.hal_logger.handlers.clear()
            handler = logging.FileHandler('hal_telemetry.log', mode='a', encoding='utf-8')
            handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
            self.hal_logger.addHandler(handler)
            self.hal_logger.setLevel(logging.INFO)
            self._log("=".ljust(80, "="))
            self._log(f"HAL Telemetry Logger Started - Mode: {self.mode}")
            self._log("=".ljust(80, "="))

        # ---- Core timing ----
        self.last_update_time = time.time()
        self._last_ballast_log = 0.0

        # ---- Identity ----
        self.vehicle_id = config.get('vehicle_id', f"SIMPLR-AUV-{int(time.time())}")

        # ---- Mission / control flags ----
        self.mission_start_requested = False
        self.mission_stop_requested = False
        self.mission_pause_requested = False
        self.emergency_stop_requested = False
        self.uploaded_mission_data: Optional[Dict[str, Any]] = None

        # ---- Telemetry state management ----
        self.initialized = False
        self._telemetry_activated = False
        self._mission_loaded = False

        # ---- Network / Telemetry config ----
        self._setup_network_config()

        # ---- Component handles ----
        self.vehicle_dynamics = None
        self.sensor_models = None
        self.telemetry_module = None

        # ---- RX buffer for newline-delimited JSON ----
        self._rx_buffer = ""

        # ---- Initialize per mode (no telemetry yet) ----
        self._initialize_components()

        self._log(f"Enhanced HAL initialized in {self.mode} mode with vehicle ID: {self.vehicle_id}")

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    def _log(self, message: str):
        if self.verbose:
            print(message)
            if self.hal_logger:
                self.hal_logger.info(message)

    # ------------------------------------------------------------------
    # Network setup
    # ------------------------------------------------------------------
    def _setup_network_config(self):
        """Configure network and telemetry settings"""
        self.wifi_enabled = bool(self.config.get('wifi_enabled', False))
        self.telemetry_enabled = False  # Start disabled

        # TCP GCS
        self.gcs_host = self.config.get('tcp_host', '127.0.0.1')
        self.gcs_port = int(self.config.get('tcp_port', 15000))
        self.gcs_socket: Optional[socket.socket] = None

        # Connection state
        self.connected_to_gcs = False
        self.connection_attempts = 0
        self.last_connection_attempt = 0.0
        self.messages_sent = 0
        self.network_errors = 0
        self.max_network_errors = 10
        self.connection_quality = 'UNKNOWN'

        # Comms timing
        self.wifi_timeout = 15.0 if self.wifi_enabled else 5.0
        self.wifi_keepalive_interval = 30.0
        self.last_keepalive = 0.0

        # Depth-aware reconnection tracking
        self.last_surface_time = time.time()
        self.surface_reconnect_delay = 2.0
        self._was_at_surface = False

        # Throttle depth logs
        self._last_depth_log_time = time.time()

        # Optional firewall (Windows)
        if self.wifi_enabled:
            self._configure_firewall()

    def _configure_firewall(self):
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
            self._log(f"HAL: Configured Windows Firewall for port {self.gcs_port}")
        except Exception as e:
            self._log(f"HAL: Firewall configuration failed: {e}")

    # ------------------------------------------------------------------
    # Component initialization
    # ------------------------------------------------------------------
    def _initialize_components(self):
        if self.mode == 'simulation':
            self._initialize_simulation()
        elif self.mode == 'hardware':
            self._initialize_hardware()

    def _initialize_simulation(self):
        try:
            from vehicle_dynamics import VehicleDynamics
            from sensor_models import SensorModels
            self.vehicle_dynamics = VehicleDynamics(self.vehicle_state, dt=0.1)
            self.sensor_models = SensorModels(self.vehicle_state)
            self._log("HAL: Simulation components initialized")
        except ImportError as e:
            print(f"HAL: Could not import simulation modules: {e}")
        except Exception as e:
            print(f"HAL: Failed to initialize simulation: {e}")
            try:
                self.vehicle_state.add_fault(f"Simulation initialization error: {e}")
            except Exception:
                pass

    def _initialize_hardware(self):
        self._log("HAL: Hardware mode initialization - placeholder")

    # ------------------------------------------------------------------
    # RESTORED: Continuous Ballast Simulation
    # ------------------------------------------------------------------
    def _update_ballast_simulation(self, dt: float):
        """Update ballast tank simulation - CONTINUOUS FILL/EMPTY"""
        try:
            with self.vehicle_state._state_lock:
                tank_volume = self.vehicle_state.ballast_state.get('tank_volume', 0.0)
                tank_capacity = self.vehicle_state.ballast_state.get('tank_capacity', 0.001982)
                cmd = self.vehicle_state.actuator_commands.get('ballast_cmd', 'OFF')
                pump_rate = 0.00005  # m³/s - realistic fill rate
                
                # Log ballast commands
                if cmd != getattr(self, '_last_ballast_cmd', 'OFF'):
                    self._log(f"HAL: Ballast command received: {cmd}")
                    self._last_ballast_cmd = cmd
                
                if cmd == 'FILL' and tank_volume < tank_capacity:
                    new_volume = min(tank_capacity, tank_volume + pump_rate * dt)
                    self._update_ballast_state(new_volume, tank_capacity, 'FILLING', True)
                    
                    # Log progress every 2 seconds
                    current_time = time.time()
                    if current_time - self._last_ballast_log > 2.0:
                        progress = (new_volume / tank_capacity) * 100
                        self._log(f"HAL: Ballast filling - {progress:.1f}% complete ({new_volume:.6f}/{tank_capacity:.6f} m³)")
                        self._last_ballast_log = current_time
                    
                    if new_volume >= tank_capacity - 1e-12:
                        self._stop_ballast_operation('FULL')
                        self._log("HAL: Ballast fill COMPLETE - tank FULL, buoyancy NEUTRAL")
                        
                elif cmd == 'VENT' and tank_volume > 0.0:
                    # Empty faster than fill (2x rate)
                    new_volume = max(0.0, tank_volume - pump_rate * dt * 2.0)
                    self._update_ballast_state(new_volume, tank_capacity, 'EMPTYING', True)
                    
                    # Log progress every 2 seconds
                    current_time = time.time()
                    if current_time - self._last_ballast_log > 2.0:
                        progress = (1.0 - (new_volume / tank_capacity)) * 100
                        self._log(f"HAL: Ballast emptying - {progress:.1f}% complete ({new_volume:.6f}/{tank_capacity:.6f} m³)")
                        self._last_ballast_log = current_time
                    
                    if new_volume <= 1e-12:
                        self._stop_ballast_operation('EMPTY')
                        self._log("HAL: Ballast empty COMPLETE - tank EMPTY, buoyancy POSITIVE")
                        
                else:
                    # OFF command or invalid
                    status = self._get_ballast_status_from_volume(tank_volume, tank_capacity)
                    self._update_ballast_state(tank_volume, tank_capacity, status, False)
                    
        except Exception as e:
            self._log(f"HAL: Error updating ballast: {e}")

    def _update_ballast_state(self, volume: float, capacity: float, status: str, pump_running: bool):
        """Update ballast state parameters"""
        self.vehicle_state.ballast_state.update({
            'tank_volume': volume,
            'tank_status': status,
            'pump_running': pump_running,
            'vent_open': pump_running  # Vent open when pump is running
        })
        
        # Update buoyancy based on new volume
        self._update_buoyancy_from_ballast()

    def _stop_ballast_operation(self, final_status: str):
        """Stop ballast operation and set final status"""
        self.vehicle_state.actuator_commands['ballast_cmd'] = 'OFF'
        self.vehicle_state.ballast_state.update({
            'tank_status': final_status,
            'pump_running': False,
            'vent_open': False
        })

    def _get_ballast_status_from_volume(self, volume: float, capacity: float) -> str:
        """Get ballast status based on current volume"""
        if volume <= 1e-12:
            return 'EMPTY'
        elif volume >= capacity - 1e-12:
            return 'FULL'
        else:
            return 'PARTIAL'

    def _update_buoyancy_from_ballast(self):
        """Update buoyancy state based on ballast volume"""
        try:
            if hasattr(self.vehicle_state, 'update_buoyancy_state'):
                self.vehicle_state.update_buoyancy_state()
            else:
                # Fallback calculation
                with self.vehicle_state._state_lock:
                    vol = self.vehicle_state.ballast_state.get('tank_volume', 0.0)
                    cap = self.vehicle_state.ballast_state.get('tank_capacity', 0.001982)
                    
                    if vol <= 1e-12:
                        self.vehicle_state.buoyancy_state = 'POSITIVE'
                    elif vol >= cap - 1e-12:
                        self.vehicle_state.buoyancy_state = 'NEUTRAL'
                    else:
                        self.vehicle_state.buoyancy_state = 'PARTIAL'
                        
        except Exception as e:
            self._log(f"HAL: Error updating buoyancy: {e}")

    # ------------------------------------------------------------------
    # Telemetry lifecycle
    # ------------------------------------------------------------------
    def _initialize_telemetry(self):
        """Start telemetry module (only if mission loaded AND vehicle initialized)."""
        if not self._mission_loaded:
            self._log("HAL: Telemetry init blocked - mission not loaded yet.")
            return
        if not self.initialized:
            self._log("HAL: Telemetry init blocked - vehicle not initialized yet.")
            return
            
        try:
            from telemetry import TelemetryModule
            update_rate = float(self.config.get('telemetry_rate_hz', 1.0))
            self.telemetry_module = TelemetryModule(self.vehicle_state, update_rate=update_rate)
            self.telemetry_module.start()
            self.telemetry_enabled = True
            self._telemetry_activated = True
            conn_type = "WiFi" if self.wifi_enabled else "local"
            self._log(f"HAL: Telemetry ACTIVATED for {conn_type} connection to {self.gcs_host}:{self.gcs_port}")
        except ImportError:
            print("HAL: Telemetry module not available")
            self.telemetry_enabled = False
        except Exception as e:
            print(f"HAL: Failed to initialize telemetry: {e}")
            self.telemetry_enabled = False

    def activate(self):
        """Public activation hook: call AFTER mission load + vehicle init."""
        if self._telemetry_activated:
            return
            
        if self._mission_loaded and self.initialized:
            self._log("[HAL] Mission loaded AND vehicle initialized - ACTIVATING telemetry")
            self._initialize_telemetry()
        else:
            self._log(f"[HAL] Activation deferred - mission_loaded: {self._mission_loaded}, initialized: {self.initialized}")

    def mark_initialized(self, value: bool = True):
        """Mark vehicle as initialized and activate telemetry if mission is loaded."""
        prev = self.initialized
        self.initialized = bool(value)
        if self.initialized and not prev:
            self._log("[HAL] Vehicle marked as initialized")
            self.activate()

    def mark_mission_loaded(self, value: bool = True):
        """Mark mission as loaded and activate telemetry if vehicle is initialized."""
        prev = self._mission_loaded
        self._mission_loaded = bool(value)
        if self._mission_loaded and not prev:
            self._log("[HAL] Mission marked as loaded")
            self.activate()

    # ------------------------------------------------------------------
    # GCS connection / mission IO
    # ------------------------------------------------------------------
    def await_gcs_connection(self, timeout: int = 60) -> bool:
        start = time.time()
        print(f"[HAL] Waiting for GCS connection (timeout: {timeout}s)...")
        print(f"[HAL] Listening on {self.gcs_host}:{self.gcs_port}")

        while time.time() - start < timeout:
            try:
                if not self.connected_to_gcs:
                    self._connect_to_gcs()

                if self.connected_to_gcs:
                    print(f"[HAL] Connected to GCS at {self.gcs_host}:{self.gcs_port}")
                    return True

                time.sleep(0.5)

            except KeyboardInterrupt:
                print("\n[HAL] Connection wait cancelled by user")
                return False
            except Exception as e:
                self._log(f"[HAL] Discovery error: {e}")
                time.sleep(1)

        print(f"[HAL] No GCS connection established within {timeout}s timeout")
        return False

    def listen_for_commands(self) -> Optional[Dict[str, Any]]:
        """Receive mission upload & mission_start from GCS."""
        mission_uploaded = False
        last_status_time = time.time()
        status_interval = 10.0

        print("[HAL] Listening for mission upload and start command from GCS...")

        while True:
            try:
                if self.connected_to_gcs:
                    self._process_incoming_gcs_messages()
                else:
                    self._connect_to_gcs()

                if self.uploaded_mission_data is not None and not mission_uploaded:
                    mission_uploaded = True
                    self.mark_mission_loaded(True)
                    mission_name = self.uploaded_mission_data.get('mission_info', {}).get('name', 'Unknown')
                    task_count = len(self.uploaded_mission_data.get('tasks', []))
                    print(f"[HAL] Mission received: '{mission_name}' with {task_count} tasks")
                    print("[HAL] Waiting for mission start command...")

                if self.mission_start_requested and mission_uploaded:
                    print("[HAL] Mission start command received - ready to begin execution")
                    self.clear_mission_start_flag()
                    return self.uploaded_mission_data

                now = time.time()
                if now - last_status_time >= status_interval:
                    conn_status = "CONNECTED" if self.connected_to_gcs else "DISCONNECTED"
                    mission_status = "LOADED" if mission_uploaded else "WAITING"
                    telemetry_status = "ACTIVE" if self.telemetry_enabled else "INACTIVE"
                    print(f"[HAL] Status: GCS={conn_status} | Mission={mission_status} | Telemetry={telemetry_status}")
                    last_status_time = now

                time.sleep(0.25)

            except KeyboardInterrupt:
                print("\n[HAL] Shutdown requested")
                raise
            except Exception as e:
                print(f"[HAL] Error in listen loop: {e}")
                if self.verbose:
                    traceback.print_exc()
                time.sleep(0.5)

    # ------------------------------------------------------------------
    # Main update
    # ------------------------------------------------------------------
    def update(self, dt: float = None):
        """Update vehicle systems; handle comms; auto-activate when ready."""
        try:
            current_time = time.time()
            timestep = self._calculate_timestep(current_time, dt)
            self.last_update_time = current_time

            # Update simulation/hardware state
            self._update_vehicle_systems(timestep)

            # Auto-detect initialization when valid coordinates appear
            if not self.initialized:
                nav = self.vehicle_state.nav_state
                lat, lon = nav.get('lat', 0.0), nav.get('lon', 0.0)
                if self._coords_valid(lat, lon):
                    self._log(f"[HAL] Auto-detected initialization with valid coordinates: ({lat:.6f}, {lon:.6f})")
                    self.mark_initialized(True)

            # Handle comms (only if telemetry is activated)
            if self._telemetry_activated:
                self._handle_communication()

        except Exception as e:
            print(f"HAL: Update error: {e}")
            if self.verbose:
                traceback.print_exc()
            try:
                self.vehicle_state.add_fault(f"HAL update error: {e}")
            except Exception:
                pass

    def _calculate_timestep(self, current_time: float, dt: Optional[float]) -> float:
        if dt is None:
            calc = current_time - self.last_update_time
            return max(0.001, min(1.0, calc))
        return max(0.001, min(1.0, dt))

    # ------------------------------------------------------------------
    # Vehicle & sensor updates
    # ------------------------------------------------------------------
    def _update_vehicle_systems(self, timestep: float):
        if self.mode == 'simulation':
            # RESTORED: Continuous ballast simulation
            self._update_ballast_simulation(timestep)

            try:
                if self.sensor_models:
                    sensor_data = self.sensor_models.update(timestep)
                    self.vehicle_state.sensor_data = sensor_data
                    self._update_nav_from_sensors(sensor_data)
            except Exception as e:
                self._log(f"HAL: Simulation update error: {e}")

        elif self.mode == 'hardware':
            self._read_hardware_sensors()

    def _update_nav_from_sensors(self, sensor_data: Dict[str, Any]):
        try:
            nav_state = self.vehicle_state.nav_state

            if 'gps' in sensor_data and sensor_data['gps']:
                gps = sensor_data['gps']
                if 'lat' in gps and 'lon' in gps:
                    nav_state['lat'] = gps['lat']
                    nav_state['lon'] = gps['lon']
                if gps.get('speed', 0.0) > 0.1:
                    nav_state['speed'] = gps.get('speed', 0.0)

            if 'compass' in sensor_data:
                nav_state['heading'] = sensor_data['compass']

            if 'depth' in sensor_data:
                nav_state['depth'] = max(0.0, sensor_data['depth'])

        except Exception as e:
            self._log(f"HAL: Error updating nav from sensors: {e}")

    def _read_hardware_sensors(self):
        try:
            # Replace with your actual hardware reads
            pass
        except Exception as e:
            self._log(f"HAL: Error reading hardware sensors: {e}")
            try:
                self.vehicle_state.add_fault(f"Hardware sensor error: {e}")
            except Exception:
                pass

    def _determine_nav_mode(self, depth: float, system_state: str) -> str:
        return "GPS" if depth < 1.0 else "INERTIAL"

    # ------------------------------------------------------------------
    # Comms core (keep your existing comms methods)
    # ------------------------------------------------------------------
    def _is_communication_disabled(self) -> bool:
        """Disable comms when depth >= 1.0 m (WiFi cannot reach)."""
        if not self._telemetry_activated:
            return True
            
        try:
            current_depth = self.vehicle_state.nav_state.get('depth', 0.0)
            disabled = current_depth >= 1.0
            if time.time() - self._last_depth_log_time > 5.0:
                mode = "WiFi" if self.wifi_enabled else "localhost"
                status = "DISABLED" if disabled else "ENABLED"
                telemetry_status = "ACTIVE" if self._telemetry_activated else "INACTIVE"
                self._log(
                    f"HAL: Telemetry depth check - telemetry={telemetry_status}, "
                    f"depth={current_depth:.3f}m, status={status} (threshold=1.0m), mode={mode}"
                )
                self._last_depth_log_time = time.time()
            return disabled
        except Exception as e:
            self._log(f"HAL: Error checking communication disabled: {e}")
            return True

    def _is_at_surface(self) -> bool:
        try:
            current_depth = self.vehicle_state.nav_state.get('depth', 0.0)
            return current_depth < 1.0
        except Exception:
            return True

    def _handle_depth_based_connection(self):
        if not self._telemetry_activated:
            if self.connected_to_gcs:
                self._disconnect_from_gcs("Telemetry not activated")
            return

        comm_disabled = self._is_communication_disabled()

        if comm_disabled:
            if self.connected_to_gcs:
                depth = self.vehicle_state.nav_state.get('depth', 0.0)
                mode = "WiFi" if self.wifi_enabled else "localhost"
                self._log(f"HAL: {mode} communication DISABLED - depth {depth:.3f}m (>= 1.0m)")
                reason = f"Deep at {depth:.1f}m"
                self._disconnect_from_gcs(reason)
            self._was_at_surface = False
            return

        if self._is_at_surface():
            now = time.time()
            if not self.connected_to_gcs:
                if not self._was_at_surface:
                    self.last_surface_time = now
                    mode = "WiFi" if self.wifi_enabled else "localhost"
                    depth = self.vehicle_state.nav_state.get('depth', 0.0)
                    self._log(f"HAL: {mode} communication RE-ENABLED - depth {depth:.3f}m (< 1.0m)")
                self._was_at_surface = True
                if now - self.last_surface_time >= self.surface_reconnect_delay:
                    self._connect_to_gcs()
            else:
                self._was_at_surface = True
        else:
            self._was_at_surface = False

    def _connect_to_gcs(self) -> bool:
        """Attempt to connect to GCS."""
        if self._telemetry_activated and self._is_communication_disabled():
            return False

        if self._telemetry_activated and self._is_at_surface() and (time.time() - self.last_surface_time) < self.surface_reconnect_delay:
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
            telemetry_status = "ACTIVE" if self._telemetry_activated else "INACTIVE"
            self._log(f"HAL: Connected to {conn_type} GCS (telemetry: {telemetry_status}, depth: {depth:.3f}m)")
            
            if self._telemetry_activated:
                self._log("HAL: Telemetry active - ready to send vehicle status")
            else:
                self._log("HAL: Connection established - ready to receive commands")

            return True

        except ConnectionRefusedError:
            self.connection_quality = 'REFUSED'
        except socket.timeout:
            self.connection_quality = 'TIMEOUT'
        except Exception as e:
            self.connection_quality = 'ERROR'
            if self.connection_attempts % 10 == 1:
                self._log(f"HAL: Connection error: {e}")

        if self.gcs_socket:
            try:
                self.gcs_socket.close()
            except Exception:
                pass
            self.gcs_socket = None

        self.connected_to_gcs = False
        return False

    def _configure_socket(self):
        if not self.gcs_socket:
            return
        try:
            self.gcs_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            self.gcs_socket.settimeout(self.wifi_timeout)
            if self.wifi_enabled and platform.system() == "Windows":
                self.gcs_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 16384)
                self.gcs_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception as e:
            self._log(f"HAL: Socket configuration error: {e}")

    def _disconnect_from_gcs(self, reason: str):
        try:
            if self.gcs_socket:
                try:
                    self.gcs_socket.close()
                except Exception:
                    pass
                self.gcs_socket = None
            self.connected_to_gcs = False
            self._log(f"HAL: Disconnected from GCS - {reason}")
        except Exception as e:
            self._log(f"HAL: Error during disconnect: {e}")

    def _send_raw_json(self, message: dict) -> bool:
        try:
            if self.gcs_socket and self.connected_to_gcs:
                json_str = json.dumps(message) + '\n'
                self.gcs_socket.sendall(json_str.encode('utf-8'))
                return True
            return False
        except Exception as e:
            print(f"HAL: Error sending message to GCS: {e}")
            self.connected_to_gcs = False
            return False

    # ------------------------------------------------------------------
    # Telemetry send path
    # ------------------------------------------------------------------
    def _handle_communication(self):
        self._handle_depth_based_connection()
        if self.connected_to_gcs:
            self._process_incoming_gcs_messages()
        self._process_telemetry_messages()
        if self.wifi_enabled and self.connected_to_gcs:
            self._send_keepalive()

    def _process_telemetry_messages(self):
        """Send outbound telemetry frames."""
        if not self._telemetry_activated or not self.telemetry_enabled:
            return

        nav = self.vehicle_state.nav_state
        lat, lon = nav.get('lat', 0.0), nav.get('lon', 0.0)
        if not self._coords_valid(lat, lon):
            return

        try:
            if self.telemetry_module:
                messages = self.telemetry_module.get_all_pending_messages()
                for message in messages:
                    if not self._send_message_to_gcs(message.timestamp, message.payload):
                        break
            else:
                fallback_payload = self._convert_to_telemetry_format({})
                if fallback_payload:
                    self._send_message_to_gcs(time.time(), fallback_payload)
        except Exception as e:
            if self.verbose:
                print(f"HAL: Telemetry processing error: {e}")
            self.connected_to_gcs = False

    def _send_message_to_gcs(self, timestamp: float, payload: Dict[str, Any]) -> bool:
        if not self.gcs_socket or not self.connected_to_gcs:
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
            self._send_raw_json(message_data)
            self.messages_sent += 1
            self.network_errors = 0
            self.connection_quality = 'GOOD'
            return True
        except Exception as e:
            if self.verbose:
                print(f"HAL: Error sending message: {e}")
            return False

    def _convert_to_telemetry_format(self, _payload: Dict[str, Any]) -> Dict[str, Any]:
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
            if self.verbose:
                print(f"HAL: Error converting payload format: {e}")
            return {}

    def _send_keepalive(self):
        current_time = time.time()
        if current_time - self.last_keepalive < self.wifi_keepalive_interval:
            return
        try:
            keepalive_msg = {
                'timestamp': current_time,
                'msg_type': 'keepalive',
                'payload': {
                    'vehicle_id': self.vehicle_id, 
                    'status': 'alive',
                    'telemetry_active': self._telemetry_activated
                }
            }
            self._send_raw_json(keepalive_msg)
            self.last_keepalive = current_time
        except Exception as e:
            if self.verbose:
                print(f"HAL: Keepalive failed: {e}")
            self.connected_to_gcs = False

    # ------------------------------------------------------------------
    # Inbound GCS messages (keep your existing message handlers)
    # ------------------------------------------------------------------
    def _process_incoming_gcs_messages(self):
        if not self.gcs_socket:
            return
        try:
            try:
                chunk = self.gcs_socket.recv(4096)
            except (BlockingIOError, socket.timeout):
                return

            if not chunk:
                if self.verbose:
                    print("HAL: GCS closed connection")
                self._disconnect_from_gcs("Remote closed")
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
                    self._handle_incoming_message(message)
                    if self.verbose:
                        print(f"HAL: Processed {message.get('msg_type', 'unknown')} from GCS")
                except json.JSONDecodeError:
                    if self.verbose:
                        print(f"HAL: Invalid JSON from GCS: {line}")

        except Exception as e:
            if self.verbose:
                print(f"HAL: Incoming message processing error: {e}")
            self.network_errors += 1
            if self.network_errors >= self.max_network_errors:
                self._disconnect_from_gcs("Too many RX errors")

    def _handle_incoming_message(self, message: Dict[str, Any]):
        try:
            msg_type = message.get('msg_type', '')
            payload = message.get('payload', {})
            timestamp = message.get('timestamp', time.time())

            if self.verbose:
                print(f"HAL: Received {msg_type} from GCS")

            if msg_type == 'mission_upload':
                self._handle_mission_upload(payload, timestamp)
            elif msg_type == 'mission_download':
                self._handle_mission_download(payload, timestamp)
            elif msg_type == 'mission_start':
                self._handle_mission_start(payload, timestamp)
            elif msg_type == 'mission_stop':
                self._handle_mission_stop(payload, timestamp)
            elif msg_type == 'mission_pause':
                self._handle_mission_pause(payload, timestamp)
            elif msg_type == 'emergency_stop':
                self._handle_emergency_stop(payload, timestamp)
            elif msg_type == 'ping':
                self._handle_ping(payload, timestamp)
            else:
                if self.verbose:
                    print(f"HAL: Unhandled message type: {msg_type}")

        except Exception as e:
            if self.verbose:
                print(f"HAL: Error handling message: {e}")

    def _handle_mission_upload(self, payload: Dict[str, Any], timestamp: float):
        """Accept a mission from GCS and mark mission as loaded."""
        try:
            mission_name = payload.get('name', 'uploaded_mission')
            mission_data = payload.get('mission', {})
            if not mission_data:
                self._send_ack('mission_upload_ack', False, "No mission data received")
                return

            required = ['mission_info', 'vehicle_config', 'tasks']
            for field in required:
                if field not in mission_data:
                    self._send_ack('mission_upload_ack', False, f"Missing required field: {field}")
                    return

            self.uploaded_mission_data = mission_data
            self.mark_mission_loaded(True)

            # Seed initial lat/lon if present in mission
            try:
                init_pos = mission_data.get('vehicle_config', {}).get('initial_position', {})
                lat0 = float(init_pos.get('lat', 0.0))
                lon0 = float(init_pos.get('lon', 0.0))
            except Exception:
                lat0, lon0 = 0.0, 0.0

            if self._coords_valid(lat0, lon0):
                self.vehicle_state.nav_state['lat'] = lat0
                self.vehicle_state.nav_state['lon'] = lon0
                print(f"HAL: Initialized nav origin from mission file ({lat0:.6f}, {lon0:.6f})")

            print(f"HAL: Received mission upload: '{mission_name}' with {len(mission_data.get('tasks', []))} tasks")
            self._send_ack('mission_upload_ack', True, f"Mission {mission_name} received and validated successfully")

        except Exception as e:
            error_msg = f"Mission upload error: {e}"
            print(f"HAL: {error_msg}")
            self._send_ack('mission_upload_ack', False, error_msg)

    def _handle_mission_download(self, payload: Dict[str, Any], timestamp: float):
        try:
            response_payload = {
                'name': 'current_mission.json',
                'mission': {
                    'mission_info': {'name': 'Current Mission', 'description': 'Downloaded from vehicle'},
                    'vehicle_config': {'initial_position': {'lat': 0.0, 'lon': 0.0, 'depth': 0.0}},
                    'tasks': []
                },
                'download_time': time.time()
            }
            response = {
                'timestamp': time.time(),
                'msg_type': 'mission_download_response',
                'payload': response_payload
            }
            self._send_raw_json(response)
            print("HAL: Mission download response sent")
        except Exception as e:
            print(f"HAL: Mission download error: {e}")

    def _handle_mission_start(self, payload: Dict[str, Any], timestamp: float):
        try:
            execution_mode = payload.get('execution_mode', 'SIMULATION')
            mission_name = payload.get('mission_name', 'Unknown')
            print(f"HAL: Mission start command received - {mission_name} in {execution_mode} mode")
            self.mission_start_requested = True
            self._send_ack('mission_start_ack', True, f"Mission {mission_name} start request queued")
        except Exception as e:
            error_msg = f"Mission start error: {e}"
            print(f"HAL: {error_msg}")
            self._send_ack('mission_start_ack', False, error_msg)

    def _handle_mission_stop(self, payload: Dict[str, Any], timestamp: float):
        try:
            print("HAL: Mission stop command received")
            self.mission_stop_requested = True
            self._send_ack('mission_stop_ack', True, "Mission stop request queued")
        except Exception as e:
            error_msg = f"Mission stop error: {e}"
            print(f"HAL: {error_msg}")
            self._send_ack('mission_stop_ack', False, error_msg)

    def _handle_mission_pause(self, payload: Dict[str, Any], timestamp: float):
        try:
            print("HAL: Mission pause command received")
            self.mission_pause_requested = True
            self._send_ack('mission_pause_ack', True, "Mission pause request queued")
        except Exception as e:
            error_msg = f"Mission pause error: {e}"
            print(f"HAL: {error_msg}")
            self._send_ack('mission_pause_ack', False, error_msg)

    def _handle_emergency_stop(self, payload: Dict[str, Any], timestamp: float):
        try:
            print("HAL: EMERGENCY STOP command received")
            self.emergency_stop_requested = True
            self.emergency_stop()
            self._send_ack('emergency_stop_ack', True, "Emergency stop executed successfully")
        except Exception as e:
            error_msg = f"Emergency stop error: {e}"
            print(f"HAL: {error_msg}")
            self._send_ack('emergency_stop_ack', False, error_msg)

    # ------------------------------------------------------------------
    # Public control / status
    # ------------------------------------------------------------------
    def clear_mission_start_flag(self):
        self.mission_start_requested = False

    def reconnect(self) -> bool:
        if not self.telemetry_enabled:
            return False
        self._disconnect_from_gcs("Manual reconnect")
        self.network_errors = 0
        return self._connect_to_gcs()

    def get_connection_status(self) -> Dict[str, Any]:
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
            'wifi_available_at_depth': (not self._is_communication_disabled()) if self.wifi_enabled else True,
            'vehicle_id': self.vehicle_id,
            'telemetry_activated': self._telemetry_activated,
            'mission_loaded': self._mission_loaded,
            'initialized': self.initialized
        }

    def get_telemetry_status(self) -> Dict[str, Any]:
        status = {
            'enabled': self.telemetry_enabled,
            'activated': self._telemetry_activated,
            'wifi_mode': self.wifi_enabled,
            'vehicle_id': self.vehicle_id,
            'mission_loaded': self._mission_loaded,
            'initialized': self.initialized,
            'transport': {
                'type': 'tcp',
                'gcs_host': self.gcs_host,
                'gcs_port': self.gcs_port,
                'connected': self.connected_to_gcs,
                'connection_quality': self.connection_quality
            }
        }
        if self.telemetry_module:
            try:
                status['module'] = self.telemetry_module.get_status()
            except Exception:
                status['module'] = {'error': 'Module status unavailable'}
        return status

    # ------------------------------------------------------------------
    # Emergency / shutdown
    # ------------------------------------------------------------------
    def emergency_stop(self):
        try:
            with self.vehicle_state._state_lock:
                self.vehicle_state.actuator_commands.update({
                    'ballast_cmd': 'OFF',
                    'thruster_cmd': 0.0,
                    'UL': 0.0, 'UR': 0.0, 'LR': 0.0, 'LL': 0.0,
                    'rudder_cmd': 0.0,
                    'elevator_cmd': 0.0
                })
                self.vehicle_state.ballast_state.update({
                    'pump_running': False,
                    'vent_open': False
                })
            if self.verbose:
                print("HAL: Emergency stop executed")
        except Exception as e:
            print(f"HAL: Error during emergency stop: {e}")

    def shutdown(self):
        try:
            self._log("HAL: Shutting down...")
            if self.telemetry_module:
                try:
                    self.telemetry_module.stop()
                except Exception:
                    pass
            self._disconnect_from_gcs("Shutdown")
            self.emergency_stop()
            self._log("HAL: Shutdown complete")
            if self.hal_logger:
                for handler in list(self.hal_logger.handlers):
                    try:
                        handler.close()
                    except Exception:
                        pass
        except Exception as e:
            print(f"HAL: Error during shutdown: {e}")

    def __del__(self):
        try:
            self.shutdown()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Ping / ACK helpers
    # ------------------------------------------------------------------
    def _handle_ping(self, payload: Dict[str, Any], timestamp: float):
        try:
            ping_time = payload.get('ping_time', timestamp)
            response = {
                'timestamp': time.time(),
                'msg_type': 'pong',
                'payload': {
                    'ping_time': ping_time,
                    'pong_time': time.time(),
                    'vehicle_id': self.vehicle_id,
                    'telemetry_active': self._telemetry_activated
                }
            }
            self._send_raw_json(response)
            if self.verbose:
                print("HAL: Pong sent in response to ping")
        except Exception as e:
            print(f"HAL: Ping response error: {e}")

    def _send_ack(self, ack_type: str, success: bool, message: str = ""):
        try:
            ack_payload = {
                'success': success,
                'message': message,
                'timestamp': time.time(),
                'vehicle_id': self.vehicle_id,
                'telemetry_active': self._telemetry_activated
            }
            if not success:
                ack_payload['error'] = message
            response = {
                'timestamp': time.time(),
                'msg_type': ack_type,
                'payload': ack_payload
            }
            self._send_raw_json(response)
            if self.verbose:
                status = "SUCCESS" if success else "FAILED"
                print(f"HAL: Sent {ack_type} - {status}: {message}")
        except Exception as e:
            print(f"HAL: Error sending ack: {e}")

    # ------------------------------------------------------------------
    # Validation helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _coords_valid(lat: float, lon: float) -> bool:
        if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
            return False
        if abs(lat) < 1e-6 and abs(lon) < 1e-6:
            return False
        if abs(lat) > 90 or abs(lon) > 180:
            return False
        return True