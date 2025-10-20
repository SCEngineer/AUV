#!/usr/bin/env python3
"""
hal_simulation.py - Simulation Manager for SIMPLR AUV

FIXED VERSION: Proper coordinate conversion for sonar simulation
- Converts GPS lat/lon to local ENU (East-North-Up) coordinates
- Enables realistic sonar obstacle detection as vehicle moves
- Real hardware sonar doesn't need this (it measures range directly)
- NEW: Ping1D-style sonar packet with confidence

This version standardizes the forward sonar output to a minimal packet:
    vehicle_state.sensor_data['sonar'] = {
        'range': <float>,          # meters, finite
        'confidence': <float>,     # 0-100%
        'timestamp': <epoch seconds>
    }

It also forwards that packet to self.hal.obstacle_avoidance.update_sonar(...)
if present, so your OA module can remain device-agnostic.

Constructor signature matches EnhancedHAL expectations:
    SimulationManager(vehicle_state, config, hal_instance)
"""

import time
import math
import traceback
from typing import Dict, Any, Optional

try:
    # Rich simulated sonar with obstacle ray-casting
    from simulated_sonar import SimulatedSonar
except Exception as _e:
    SimulatedSonar = None  # type: ignore

try:
    # Optional simulation components
    from vehicle_dynamics import VehicleDynamics  # type: ignore
except Exception:
    VehicleDynamics = None  # type: ignore

try:
    from sensor_models import SensorModels  # type: ignore
except Exception:
    SensorModels = None  # type: ignore


class SimulationManager:
    """Manages simulation components, sensors, and the bridge to obstacle avoidance."""

    def __init__(self, vehicle_state, config: Optional[Dict[str, Any]], hal_instance):
        self.vehicle_state = vehicle_state
        self.config = config or {}
        self.hal = hal_instance

        # Mode selection
        self.mode = (self.config.get("mode") or "simulation").lower()

        # Components (simulation mode)
        self.vehicle_dynamics = None
        self.sensor_models = None
        self.simulated_sonar = None

        # Sonar update cadence
        sonar_cfg = self.config.get("sonar", {})
        self.sonar_update_interval = float(self.config.get("sonar_update_interval", 0.5))
        self._last_sonar_update = 0.0

        # Ballast sim bookkeeping
        self._last_ballast_log = 0.0
        self._last_ballast_cmd = "OFF"

        # Initialize per mode
        if self.mode == "simulation":
            self._init_simulation(sonar_cfg)
        else:
            # Hardware mode is a no-op here (this file is primarily for simulation)
            self._log("SimulationManager initialized in HARDWARE mode (sonar bridge disabled)")

    # -------------------------------------------------------------------------
    # Initialization
    # -------------------------------------------------------------------------
    def _init_simulation(self, sonar_cfg: Dict[str, Any]):
        # Sonar (optional but recommended)
        if SimulatedSonar is not None:
            try:
                default_sonar = {
                    "max_range": 100.0,
                    "min_range": 0.3,
                    "beam_width": 25.0,
                    "vertical_beam_width": 25.0,
                    "mount_angle": 0.0,
                    "frequency": 115,
                    "debug": False,
                }
                cfg = {**default_sonar, **sonar_cfg}
                self.simulated_sonar = SimulatedSonar(cfg)
                self._log("Simulation: Enhanced simulated sonar initialized")
            except Exception as e:
                self._log(f"Simulation: Failed to init SimulatedSonar: {e}")
                self.simulated_sonar = None
        else:
            self._log("Simulation: SimulatedSonar module unavailable")

        # Vehicle dynamics and sensor models (optional)
        if VehicleDynamics is not None:
            try:
                # 10 Hz default timestep inside vehicle_dynamics if it uses internal dt
                self.vehicle_dynamics = VehicleDynamics(self.vehicle_state, dt=0.1)
                self._log("Simulation: VehicleDynamics initialized")
            except Exception as e:
                self._log(f"Simulation: VehicleDynamics init error: {e}")
                self.vehicle_dynamics = None

        if SensorModels is not None:
            try:
                self.sensor_models = SensorModels(self.vehicle_state)
                self._log("Simulation: SensorModels initialized")
            except Exception as e:
                self._log(f"Simulation: SensorModels init error: {e}")
                self.sensor_models = None

        self._log("SimulationManager initialized (mode=SIMULATION, Ping1D-style sonar interface active)")
        self._log("FIXED: GPS to ENU coordinate conversion enabled for sonar simulation")

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------
    def load_mission_obstacles(self, mission_json: Dict[str, Any]):
        """
        Load obstacles into the simulated sonar from mission JSON (if sonar active).
        Expects mission_json to have an 'obstacles' array of shapes.
        """
        try:
            if self.simulated_sonar is None:
                self._log("Simulation: Cannot load obstacles (sonar not initialized)")
                return
            obstacles = mission_json.get("obstacles", [])
            self.simulated_sonar.load_obstacles(obstacles)
            self._log(f"Simulation: Loaded {len(obstacles)} obstacle(s) into sonar")
        except Exception as e:
            self._log(f"Simulation: Error loading obstacles: {e}")

    def update_systems(self, timestep: float = 0.1):
        """Main entry point called by HAL each tick."""
        if self.mode != "simulation":
            return
        self._update_simulation(timestep)

    # -------------------------------------------------------------------------
    # Internal update steps (simulation mode)
    # -------------------------------------------------------------------------
    def _update_simulation(self, dt: float):
        # 1) Ballast simulation (continuous; keeps tank/trim state coherent)
        self._update_ballast_simulation(dt)

        # 2) Sonar (range + confidence + timestamp)
        now = time.time()
        if self.simulated_sonar and (now - self._last_sonar_update >= self.sonar_update_interval):
            self._update_simulated_sonar()
            self._last_sonar_update = now

        # 3) Vehicle dynamics (optional)
        try:
            if self.vehicle_dynamics is not None:
                self.vehicle_dynamics.update(dt)
        except Exception as e:
            self._log(f"Simulation: Vehicle dynamics update error: {e}")
            traceback.print_exc()

        # 4) Sensor models (optional -> updates nav_state, gps, imu, etc.)
        try:
            if self.sensor_models is not None:
                sensor_data = self.sensor_models.update(dt)
                # Merge any existing sensor entries (like sonar) with new data
                if isinstance(sensor_data, dict):
                    existing = self.vehicle_state.sensor_data
                    existing.update(sensor_data)
                    self.vehicle_state.sensor_data = existing
                # Simple nav update from sensors (GPS, etc.)
                self._update_nav_from_sensors(self.vehicle_state.sensor_data)
        except Exception as e:
            self._log(f"Simulation: Sensor model update error: {e}")
            traceback.print_exc()

        # 5) Forward sonar packet to OA if available
        try:
            sonar_pkt = self.vehicle_state.sensor_data.get("sonar")
            if sonar_pkt and hasattr(self.hal, "obstacle_avoidance") and self.hal.obstacle_avoidance:
                self.hal.obstacle_avoidance.update_sonar(sonar_pkt)
        except Exception as e:
            self._log(f"Simulation: OA sonar forward error: {e}")
            traceback.print_exc()

    def _update_simulated_sonar(self):
        """
        FIXED: Compute/update the Ping1D-style sonar packet with confidence.
        
        Converts GPS lat/lon to local ENU (East-North-Up) coordinates for ray-casting.
        Real hardware sonar doesn't need this - it measures range directly.
        """
        try:
            nav = self.vehicle_state.nav_state
            
            # Get initial position as local coordinate origin
            init_pos = self.vehicle_state.get_initial_position()
            
            # Convert current GPS lat/lon to local ENU coordinates (meters)
            vehicle_x, vehicle_y = self._latlon_to_local_enu(
                nav.get("lat", 0.0),
                nav.get("lon", 0.0),
                init_pos['lat'],
                init_pos['lon']
            )
            
            vehicle_pos = {
                "x": vehicle_x,
                "y": vehicle_y,
                "depth": nav.get("depth", 0.0),
            }
            heading = nav.get("heading", 0.0)

            distance, info = self.simulated_sonar.get_distance_with_info(vehicle_pos, heading)
            timestamp = time.time()

            # Construct Ping1D-style sonar packet
            sonar_pkt = {
                "range": distance,
                "confidence": info.get("confidence", 0.0),
                "timestamp": timestamp
            }

            # Update sensor data
            if "sensor_data" not in self.vehicle_state.__dict__:
                self.vehicle_state.sensor_data = {}
            self.vehicle_state.sensor_data["sonar"] = sonar_pkt
            # Backward compatibility
            self.vehicle_state.sensor_data["sonar_distance"] = distance

            # Debug output when confidence is high
            if sonar_pkt["confidence"] > 50.0:
                print(f"[SONAR] range={distance:.1f}m conf={sonar_pkt['confidence']:.1f}% | "
                      f"Vehicle ENU: ({vehicle_x:.1f}, {vehicle_y:.1f}) | Heading: {heading:.1f}°")
        except Exception as e:
            self._log(f"Simulation: Sonar update error: {e}")
            traceback.print_exc()

    def _latlon_to_local_enu(self, lat: float, lon: float, origin_lat: float, origin_lon: float) -> tuple:
        """
        Convert GPS lat/lon to local ENU (East-North-Up) coordinates in meters.
        
        This is ONLY needed for simulation. Real hardware sonar measures range directly
        and doesn't need to know vehicle position or obstacle locations.
        
        Args:
            lat, lon: Current position (degrees)
            origin_lat, origin_lon: Local coordinate origin (degrees) - typically initial position
        
        Returns:
            (x, y) tuple in meters where:
                x = East displacement (meters)
                y = North displacement (meters)
        
        Uses flat-earth approximation (accurate for distances < 10km):
            - 1 degree latitude ≈ 111,320 meters (constant)
            - 1 degree longitude ≈ 111,320 * cos(latitude) meters (varies with latitude)
        """
        # Difference in degrees
        dlat = lat - origin_lat
        dlon = lon - origin_lon
        
        # Convert latitude difference to meters (constant conversion)
        meters_per_deg_lat = 111320.0  # meters per degree latitude
        
        # Convert longitude difference to meters (varies with latitude)
        avg_lat_rad = math.radians((lat + origin_lat) / 2.0)
        meters_per_deg_lon = 111320.0 * math.cos(avg_lat_rad)
        
        # Calculate ENU coordinates
        x = dlon * meters_per_deg_lon  # East displacement (meters)
        y = dlat * meters_per_deg_lat  # North displacement (meters)
        
        return (x, y)

    # -------------------------------------------------------------------------
    # Ballast simulation helpers
    # -------------------------------------------------------------------------
    def _update_ballast_simulation(self, dt: float):
        """Very simple ballast fill/empty simulation to keep tank/buoyancy coherent."""
        try:
            with self.vehicle_state._state_lock:
                tank_volume = self.vehicle_state.ballast_state.get("tank_volume", 0.0)
                tank_capacity = self.vehicle_state.ballast_state.get("tank_capacity", 0.001982)
                cmd = self.vehicle_state.actuator_commands.get("ballast_cmd", "OFF")
                pump_rate = 0.00005  # m^3/s

                if cmd != self._last_ballast_cmd:
                    self._log(f"Simulation: Ballast command received: {cmd}")
                    self._last_ballast_cmd = cmd

                if cmd == "FILL" and tank_volume < tank_capacity:
                    new_volume = min(tank_capacity, tank_volume + pump_rate * dt)
                    self._apply_ballast_state(new_volume, tank_capacity, "FILLING", True, True)
                    self._progress_log(new_volume, tank_capacity, filling=True)
                    if new_volume >= tank_capacity - 1e-12:
                        self._finalize_ballast("FULL")

                elif cmd == "EMPTY" and tank_volume > 0.0:
                    new_volume = max(0.0, tank_volume - pump_rate * dt * 2.0)
                    self._apply_ballast_state(new_volume, tank_capacity, "EMPTYING", True, True)
                    self._progress_log(new_volume, tank_capacity, filling=False)
                    if new_volume <= 1e-12:
                        self._finalize_ballast("EMPTY")

                else:
                    status = self._status_from_volume(tank_volume, tank_capacity)
                    self._apply_ballast_state(tank_volume, tank_capacity, status, False, False)
        except Exception as e:
            self._log(f"Simulation: Error in ballast sim: {e}")
            traceback.print_exc()

    def _apply_ballast_state(self, vol: float, cap: float, status: str, pump: bool, vent: bool):
        self.vehicle_state.ballast_state.update({
            "tank_volume": vol,
            "tank_status": status,
            "pump_running": pump,
            "vent_open": vent,
        })
        # Keep buoyancy coherent using the vehicle state's own method
        try:
            self.vehicle_state.update_buoyancy_state()
        except Exception:
            pass

    def _finalize_ballast(self, final_status: str):
        self.vehicle_state.actuator_commands["ballast_cmd"] = "OFF"
        self.vehicle_state.ballast_state.update({
            "tank_status": final_status,
            "pump_running": False,
            "vent_open": False,
        })
        self._log(f"Simulation: Ballast {final_status.lower()} complete")

    def _status_from_volume(self, vol: float, cap: float) -> str:
        if vol <= 1e-12:
            return "EMPTY"
        if vol >= cap - 1e-12:
            return "FULL"
        return "PARTIAL"

    def _progress_log(self, new_volume: float, capacity: float, filling: bool):
        now = time.time()
        if now - self._last_ballast_log > 2.0:
            if filling:
                progress = (new_volume / capacity) * 100.0
                self._log(f"Simulation: Ballast filling - {progress:.1f}%")
            else:
                progress = (1.0 - (new_volume / capacity)) * 100.0
                self._log(f"Simulation: Ballast emptying - {progress:.1f}%")
            self._last_ballast_log = now

    # -------------------------------------------------------------------------
    # Minimal nav-from-sensors helper
    # -------------------------------------------------------------------------
    def _update_nav_from_sensors(self, sensors: Dict[str, Any]):
        """
        Very light nav update (keeps nav_state coherent if SensorModels provides GPS).
        """
        try:
            nav = self.vehicle_state.nav_state
            gps = sensors.get("gps")
            if isinstance(gps, dict):
                lat = gps.get("lat")
                lon = gps.get("lon")
                if isinstance(lat, (int, float)) and isinstance(lon, (int, float)):
                    nav["lat"] = lat
                    nav["lon"] = lon
                spd = gps.get("speed")
                if isinstance(spd, (int, float)) and spd >= 0.0:
                    nav["speed"] = spd
        except Exception as e:
            self._log(f"Simulation: Nav update error: {e}")

    # -------------------------------------------------------------------------
    # Logging helper
    # -------------------------------------------------------------------------
    def _log(self, msg: str):
        try:
            if hasattr(self.hal, "_log") and callable(self.hal._log):
                self.hal._log(msg)
            else:
                print(msg)
        except Exception:
            print(msg)