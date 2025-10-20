#!/usr/bin/env python3
"""
enhanced_hal.py - Enhanced Hardware Abstraction Layer for SIMPLR AUV
Main orchestrator module - coordinates network, telemetry, commands, and simulation

ARCHITECTURE: HAL is a ROUTER/MANAGER, not a controller
- Routes sensor data from simulation/hardware to vehicle_state
- Manages network communication and telemetry
- Does NOT control obstacle avoidance (that's in auv_main.py)
"""

from __future__ import annotations

import logging
import time
import traceback
from typing import Any, Dict, Optional

# Import manager modules
from hal_network import NetworkManager
from hal_telemetry import TelemetryManager
from hal_commands import CommandHandler
from hal_simulation import SimulationManager

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
    """Enhanced Hardware Abstraction Layer - Main orchestrator (ROUTER ONLY)"""

    def __init__(self, config: Dict[str, Any], vehicle_state: VehicleState):
        self.config = config
        self.vehicle_state = vehicle_state
        self.mode = config.get('mode', 'simulation')
        self.verbose = bool(config.get('verbose', False))

        # Setup logging
        self.hal_logger = None
        self._setup_logging()

        # Core timing
        self.last_update_time = time.time()

        # Vehicle identity
        self.vehicle_id = config.get('vehicle_id', f"SIMPLR-AUV-{int(time.time())}")

        # Telemetry state management
        self.initialized = False
        self._telemetry_activated = False 
        self._mission_loaded = False
        self._mission_started = False

        # Initialize manager modules (pass self as hal_instance)
        self.network_mgr = NetworkManager(self, config, vehicle_state)
        self.telemetry_mgr = TelemetryManager(self, vehicle_state, config)
        self.command_handler = CommandHandler(self, vehicle_state, config)
        self.sim_mgr = SimulationManager(vehicle_state, config, self)

        self._log(f"Enhanced HAL initialized in {self.mode} mode with vehicle ID: {self.vehicle_id}")
        self._log("HAL Architecture: ROUTER/MANAGER (obstacle avoidance handled by main controller)")

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------
    def _setup_logging(self):
        """Setup HAL logging"""
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

    def _log(self, message: str):
        """Log message to console and file"""
        if self.verbose:
            print(message)
            if self.hal_logger:
                self.hal_logger.info(message)

    # ------------------------------------------------------------------
    # Telemetry lifecycle
    # ------------------------------------------------------------------
    def activate(self):
        """Public activation hook - checks conditions and activates if ready"""
        if self._telemetry_activated:
            return

        if self._mission_loaded and self.initialized and self._mission_started:
            self._log("[HAL] All conditions met - ACTIVATING telemetry")
            self._log(f"[HAL]   Mission loaded: {self._mission_loaded}")
            self._log(f"[HAL]   Vehicle initialized: {self.initialized}")
            self._log(f"[HAL]   Mission started: {self._mission_started}")
            self.telemetry_mgr.initialize_telemetry()
        else:
            self._log(f"[HAL] Activation deferred - mission_loaded: {self._mission_loaded}, "
                     f"initialized: {self.initialized}, mission_started: {self._mission_started}")

    def mark_initialized(self, value: bool = True):
        """Mark vehicle as initialized and try to activate telemetry"""
        prev = self.initialized
        self.initialized = bool(value)
        if self.initialized and not prev:
            self._log("[HAL] ✓ Vehicle marked as initialized")
            self.activate()

    def mark_mission_loaded(self, value: bool = True):
        """Mark mission as loaded and try to activate telemetry"""
        prev = self._mission_loaded
        self._mission_loaded = bool(value)
        if self._mission_loaded and not prev:
            self._log("[HAL] ✓ Mission marked as loaded")
            self.activate()

    def mark_mission_started(self, value: bool = True):
        """Mark mission execution as started and try to activate telemetry"""
        prev = self._mission_started
        self._mission_started = bool(value)
        if self._mission_started and not prev:
            self._log("[HAL] ✓ Mission marked as STARTED - execution beginning")
            self.activate()

    # ------------------------------------------------------------------
    # GCS connection / mission IO
    # ------------------------------------------------------------------
    def await_gcs_connection(self, timeout: int = 60) -> bool:
        """Wait for GCS connection"""
        start = time.time()
        print(f"[HAL] Waiting for GCS connection (timeout: {timeout}s)...")
        print(f"[HAL] Listening on {self.network_mgr.gcs_host}:{self.network_mgr.gcs_port}")

        while time.time() - start < timeout:
            try:
                if not self.network_mgr.connected_to_gcs:
                    self.network_mgr.connect_to_gcs()

                if self.network_mgr.connected_to_gcs:
                    print(f"[HAL] ✓ Connected to GCS at {self.network_mgr.gcs_host}:{self.network_mgr.gcs_port}")
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
        """Receive mission upload & mission_start from GCS"""
        mission_uploaded = False
        last_status_time = time.time()
        status_interval = 10.0

        print("[HAL] Listening for mission upload and start command from GCS...")

        while True:
            try:
                if self.network_mgr.connected_to_gcs:
                    self.network_mgr.process_incoming_messages(self.command_handler.handle_incoming_message)
                else:
                    self.network_mgr.connect_to_gcs()

                if self.command_handler.uploaded_mission_data is not None and not mission_uploaded:
                    mission_uploaded = True
                    self.mark_mission_loaded(True)
                    mission_info = self.command_handler.uploaded_mission_data.get('mission_info', {})
                    mission_name = mission_info.get('name', 'Unknown')
                    task_count = len(self.command_handler.uploaded_mission_data.get('tasks', []))
                    print(f"[HAL] ✓ Mission received: '{mission_name}' with {task_count} tasks")
                    print("[HAL] Waiting for mission start command...")

                if self.command_handler.mission_start_requested and mission_uploaded:
                    print("[HAL] ✓ Mission start command received - ready to begin execution")
                    self.command_handler.clear_mission_start_flag()
                    return self.command_handler.uploaded_mission_data

                now = time.time()
                if now - last_status_time >= status_interval:
                    conn_status = "CONNECTED" if self.network_mgr.connected_to_gcs else "DISCONNECTED"
                    mission_status = "LOADED" if mission_uploaded else "WAITING"
                    telemetry_status = "ACTIVE" if self._telemetry_activated else "INACTIVE"
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
        """Update vehicle systems; handle comms; auto-activate when ready"""
        try:
            current_time = time.time()
            timestep = self._calculate_timestep(current_time, dt)
            self.last_update_time = current_time

            # Update simulation/hardware state
            # This populates sensor_data with sonar, GPS, IMU, etc.
            self.sim_mgr.update_systems(timestep)

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
        """Calculate timestep for update"""
        if dt is None:
            calc = current_time - self.last_update_time
            return max(0.001, min(1.0, calc))
        return max(0.001, min(1.0, dt))

    def _handle_communication(self):
        """Handle network communication"""
        self.network_mgr.handle_depth_based_connection()
        if self.network_mgr.connected_to_gcs:
            self.network_mgr.process_incoming_messages(self.command_handler.handle_incoming_message)
        self.telemetry_mgr.process_telemetry_messages(self.network_mgr)
        if self.network_mgr.wifi_enabled and self.network_mgr.connected_to_gcs:
            self.network_mgr.send_keepalive(self.vehicle_id)

    # ------------------------------------------------------------------
    # Public control / status
    # ------------------------------------------------------------------
    def clear_mission_start_flag(self):
        """Clear mission start flag"""
        self.command_handler.clear_mission_start_flag()

    def clear_mission_stop_flag(self):
        """Clear mission stop flag"""
        self.command_handler.clear_mission_stop_flag()

    def clear_emergency_stop_flag(self):
        """Clear emergency stop flag"""
        self.command_handler.clear_emergency_stop_flag()

    def reconnect(self) -> bool:
        """Reconnect to GCS"""
        if not self.telemetry_mgr.telemetry_enabled:
            return False
        self.network_mgr.disconnect_from_gcs("Manual reconnect")
        self.network_mgr.network_errors = 0
        return self.network_mgr.connect_to_gcs()

    def get_connection_status(self) -> Dict[str, Any]:
        """Get connection status"""
        status = self.network_mgr.get_status()
        status.update({
            'vehicle_id': self.vehicle_id,
            'telemetry_activated': self._telemetry_activated,
            'mission_loaded': self._mission_loaded,
            'mission_started': self._mission_started,
            'initialized': self.initialized
        })
        return status

    def get_telemetry_status(self) -> Dict[str, Any]:
        """Get telemetry system status"""
        return self.telemetry_mgr.get_status()

    # ------------------------------------------------------------------
    # Properties for backward compatibility
    # ------------------------------------------------------------------
    @property
    def mission_start_requested(self) -> bool:
        """Check if mission start was requested"""
        return self.command_handler.mission_start_requested

    @property
    def mission_stop_requested(self) -> bool:
        """Check if mission stop was requested"""
        return self.command_handler.mission_stop_requested

    @property
    def mission_pause_requested(self) -> bool:
        """Check if mission pause was requested"""
        return self.command_handler.mission_pause_requested

    @property
    def emergency_stop_requested(self) -> bool:
        """Check if emergency stop was requested"""
        return self.command_handler.emergency_stop_requested

    @property
    def shutdown_requested(self) -> bool:
        """Check if shutdown was requested"""
        return self.command_handler.shutdown_requested

    @property
    def uploaded_mission_data(self) -> Optional[Dict[str, Any]]:
        """Get uploaded mission data"""
        return self.command_handler.uploaded_mission_data

    @property
    def connected_to_gcs(self) -> bool:
        """Check if connected to GCS"""
        return self.network_mgr.connected_to_gcs

    @property
    def telemetry_enabled(self) -> bool:
        """Check if telemetry is enabled"""
        return self.telemetry_mgr.telemetry_enabled

    @property
    def wifi_enabled(self) -> bool:
        """Check if WiFi is enabled"""
        return self.network_mgr.wifi_enabled

    # ------------------------------------------------------------------
    # Emergency / shutdown
    # ------------------------------------------------------------------
    def emergency_stop(self):
        """Execute emergency stop"""
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
        """Shutdown HAL and all components"""
        try:
            self._log("HAL: Shutting down...")
            
            # Stop telemetry
            self.telemetry_mgr.stop()
            
            # Disconnect from GCS
            self.network_mgr.disconnect_from_gcs("Shutdown")
            
            # Stop all actuators
            self.emergency_stop()
            
            # Shutdown hardware if in hardware mode
            if self.mode == 'hardware' and hasattr(self.sim_mgr, 'hardware_interface'):
                if self.sim_mgr.hardware_interface:
                    self.sim_mgr.hardware_interface.shutdown()
            
            self._log("HAL: Shutdown complete")
            
            # Close log handlers
            if self.hal_logger:
                for handler in list(self.hal_logger.handlers):
                    try:
                        handler.close()
                    except Exception:
                        pass
        except Exception as e:
            print(f"HAL: Error during shutdown: {e}")

    def __del__(self):
        """Destructor"""
        try:
            self.shutdown()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Utility methods
    # ------------------------------------------------------------------
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
