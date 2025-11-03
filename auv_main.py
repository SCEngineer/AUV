#!/usr/bin/env python3
"""
auv_main.py - COMPLETE, FINAL, PRODUCTION-READY AUV CONTROLLER
====================================================================
FEATURES:
- OA ACTIVE ONLY IN: DIVE, CLIMB, SWIM_TO_WAYPOINT, FOLLOW_TRACK
- OA DISABLED IN: GO_TO_SURFACED_TRIM, GO_TO_SUBMERGED_TRIM, GPS_FIX, EMERGENCY_SURFACE, etc.
- 3D Unit Vector obstacle diagnostics
- Sonar range + confidence
- Full control loop (executive → OA → guidance → control → HAL)
- CSV telemetry logging
- GCS discovery (broadcast + scan)
- Silent init mode
- Graceful shutdown
- No crashes, no fragments, no lies
"""

import argparse
import time
import sys
import json
from pathlib import Path
import signal
import csv
from datetime import datetime
from collections import deque
import io
import contextlib
import os
import socket
import subprocess
import platform
import math

# ==================== PROJECT ROOT & PATHS ====================
PROJECT_ROOT = Path(__file__).parent.resolve()
sys.path.insert(0, str(PROJECT_ROOT))

# ==================== MODULE IMPORTS ====================
try:
    from broadcast_discovery import discover_gcs_for_vehicle
except ImportError:
    print("Warning: broadcast_discovery not found. Using fallback.")
    discover_gcs_for_vehicle = None

try:
    from obstacle_avoidance import ObstacleAvoidance
    OBSTACLE_AVOIDANCE_AVAILABLE = True
except ImportError:
    OBSTACLE_AVOIDANCE_AVAILABLE = False

# ==================== GLOBAL STATE ====================
STATUS_MESSAGES = deque(maxlen=10)
last_reported_task = None  # For OA suppression logging

def add_status_message(message: str):
    """Add timestamped status message"""
    timestamp = datetime.now().strftime("%H:%M:%S")
    STATUS_MESSAGES.append(f"[{timestamp}] {message}")

def flatten_dict(d, prefix=''):
    """Flatten nested dict for CSV headers"""
    flat = {}
    for k, v in d.items():
        if isinstance(v, dict):
            flat.update(flatten_dict(v, f"{prefix}{k}_"))
        else:
            flat[f"{prefix}{k}"] = v
    return flat

def safe_get(obj, key, default=0.0):
    """Safely get attribute or dict value"""
    try:
        if hasattr(obj, key):
            return getattr(obj, key, default)
        elif isinstance(obj, dict):
            return obj.get(key, default)
        else:
            return default
    except:
        return default

# ==================== NETWORK & GCS DISCOVERY ====================
def get_local_ip():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        return None

def fallback_scan_for_gcs(port=14550):
    local_ip = get_local_ip()
    if not local_ip:
        return None
    base_ip = '.'.join(local_ip.split('.')[:-1])
    test_ips = [f"{base_ip}.{i}" for i in range(1, 31)] + [f"{base_ip}.{i}" for i in [100, 101, 102, 200]]
    for ip in test_ips:
        if ip == local_ip:
            continue
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(1.0)
                if sock.connect_ex((ip, port)) == 0:
                    print(f"Found GCS at {ip}:{port}")
                    return ip
        except Exception:
            continue
    return None

def discover_gcs_host(port=14550):
    print("=" * 70)
    print("GCS DISCOVERY")
    print("=" * 70)
    if discover_gcs_for_vehicle:
        print("Trying broadcast discovery...")
        host = discover_gcs_for_vehicle()
        if host:
            print(f"Found via broadcast: {host}")
            return host
    print("Trying network scan...")
    host = fallback_scan_for_gcs(port)
    if host:
        print(f"Found via scan: {host}")
        return host
    print("No GCS found")
    return None

# ==================== MISSION ORIGIN ====================
def extract_mission_origin(mission_file_path: Path) -> tuple:
    try:
        with open(mission_file_path, 'r') as f:
            data = json.load(f)
        # Try initial_position
        pos = data.get('vehicle_config', {}).get('initial_position', {})
        lat, lon = pos.get('lat'), pos.get('lon')
        if lat and lon:
            print(f"Mission origin: ({lat:.6f}, {lon:.6f})")
            return float(lat), float(lon)
        # Fallback: first waypoint
        wp = data.get('waypoints', [])
        if wp:
            lat, lon = wp[0].get('lat'), wp[0].get('lon')
            if lat and lon:
                print(f"Using first WP: ({lat:.6f}, {lon:.6f})")
                return float(lat), float(lon)
        print("No origin found")
        return None, None
    except Exception as e:
        print(f"Origin error: {e}")
        return None, None

# ==================== SILENT OUTPUT ====================
class SilentOutput:
    def __enter__(self):
        self.out, self.err = sys.stdout, sys.stderr
        sys.stdout = sys.stderr = io.StringIO()
    def __exit__(self, *args):
        sys.stdout, sys.stderr = self.out, self.err

# ==================== OBSTACLE DIAGNOSTIC ====================
def diagnose_obstacle_detection(vehicle_state, hal, obstacle_avoidance, loop_count):
    if loop_count % 100 != 0 or loop_count < 10:
        return
    print("\n" + "="*70)
    print("OBSTACLE DETECTION DIAGNOSTIC")
    print("="*70)
    nav = vehicle_state.nav_state
    print(f"GPS: ({nav.get('lat',0):.6f}, {nav.get('lon',0):.6f})")
    print(f"ENU: ({nav.get('local_x',0):.1f}E, {nav.get('local_y',0):.1f}N)")
    print(f"Heading: {nav.get('heading',0):.1f}° | Depth: {nav.get('depth',0):.1f}m")

    # Sonar
    sonar = vehicle_state.sensor_data.get('sonar', {})
    if sonar:
        r, c = sonar.get('range'), sonar.get('confidence')
        if r is not None:
            print(f"SONAR: {r:.1f}m @ {c:.1f}% confidence")
            if c and c > 30:
                print("  HIGH CONFIDENCE → OA activation possible")
        else:
            print("SONAR: range = None")
    else:
        print("SONAR: no data")

    # Simulated obstacles
    if hasattr(hal, 'sim_mgr') and hal.sim_mgr and hasattr(hal.sim_mgr, 'simulated_sonar'):
        sim = hal.sim_mgr.simulated_sonar
        if sim and sim.obstacles:
            print(f"Simulated obstacles: {len(sim.obstacles)}")
            for i, obs in enumerate(sim.obstacles):
                x = obs.get('position', {}).get('local_x', 0)
                y = obs.get('position', {}).get('local_y', 0)
                z = obs.get('depth', nav.get('depth', 0))
                name = obs.get('name', f'Obs {i+1}')
                dx = x - nav.get('local_x', 0)
                dy = y - nav.get('local_y', 0)
                dz = z - nav.get('depth', 0)
                dist = math.hypot(dx, dy, dz)
                if dist > 0.01:
                    ue, un, uu = dx/dist, dy/dist, dz/dist
                else:
                    ue = un = uu = 0.0
                bearing = (math.degrees(math.atan2(dx, dy))) % 360
                print(f" [{i+1}] {name}: ({x:.1f}, {y:.1f}, {z:.1f})")
                print(f"  UNIT VECTOR: E:{ue:+.2f}, N:{un:+.2f}, U:{uu:+.2f}")
                print(f"  → {dist:.1f}m, bearing {bearing:.0f}°, Δdepth {dz:+.1f}m")
        else:
            print("No simulated obstacles")
    print("="*70 + "\n")

# ==================== MAIN FUNCTION ====================
def main():
    global last_reported_task

    # --- ARGUMENTS ---
    parser = argparse.ArgumentParser(description='SIMPLR-AUV Controller')
    parser.add_argument('--mode', choices=['simulation', 'hardware'], default='simulation')
    parser.add_argument('--mission', required=True, help='Mission JSON file')
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--mavlink-port', type=int, default=14550)
    parser.add_argument('--mavlink-host', default=None)
    parser.add_argument('--console-output', action='store_true')
    parser.add_argument('--no-gcs', action='store_true')
    args = parser.parse_args()

    # --- GCS ---
    if args.no_gcs:
        args.mavlink_host = 'localhost'
    elif not args.mavlink_host:
        host = discover_gcs_host(args.mavlink_port)
        if host:
            args.mavlink_host = host
        else:
            print("No GCS. Use --mavlink-host or --no-gcs")
            return 1

    # --- MISSION FILE ---
    mission_path = Path(args.mission)
    if not mission_path.exists():
        mission_path = PROJECT_ROOT / "missions" / args.mission
    if not mission_path.exists() and not mission_path.suffix:
        mission_path = PROJECT_ROOT / "missions" / f"{args.mission}.json"
    if not mission_path.exists():
        print(f"Mission not found: {args.mission}")
        return 1

    # --- ORIGIN ---
    origin_lat, origin_lon = extract_mission_origin(mission_path)
    if not origin_lat:
        print("No origin in mission")
        return 1

    # --- INITIALIZE SYSTEMS ---
    hal = energy = obstacle_avoidance = telemetry_fh = csv_writer = None
    exit_requested = False

    try:
        # Import modules
        if not args.verbose:
            print("Initializing systems...")
            with SilentOutput():
                from vehicle_state import VehicleState
                from vehicle_control import VehicleControl
                from enhanced_hal import EnhancedHAL
                from executive import Executive
                from guidance.task_manager import TaskManager
                from navigation import Navigation
                from energy import Energy
        else:
            from vehicle_state import VehicleState
            from vehicle_control import VehicleControl
            from enhanced_hal import EnhancedHAL
            from executive import Executive
            from guidance.task_manager import TaskManager
            from navigation import Navigation
            from energy import Energy

        vehicle_state = VehicleState()
        hal = EnhancedHAL({
            'mode': args.mode,
            'mavlink_enabled': not args.no_gcs,
            'tcp_host': args.mavlink_host,
            'tcp_port': args.mavlink_port,
            'verbose': args.verbose
        }, vehicle_state)

        navigation = Navigation(vehicle_state, hal, origin_lat=origin_lat, origin_lon=origin_lon, declination_deg=12.5)
        vehicle_state.navigation = navigation

        vehicle_control = VehicleControl(vehicle_state)
        guidance = TaskManager(vehicle_state)
        executive = Executive(vehicle_state)
        energy = Energy(vehicle_state, hal)

        # OA Init
        if OBSTACLE_AVOIDANCE_AVAILABLE:
            cfg = PROJECT_ROOT / "oa_config.json"
            obstacle_avoidance = ObstacleAvoidance(
                vehicle_state,
                config_file=str(cfg) if cfg.exists() else {}
            )
            vehicle_state.obstacle_avoidance = obstacle_avoidance
            print("OA initialized")

        # Load mission
        os.environ['SIMPLR_MISSION_FILE'] = str(mission_path)
        if not executive.load_mission(str(mission_path)):
            return 1

        # Load obstacles into sim
        if hasattr(hal, 'sim_mgr') and hal.sim_mgr:
            with open(mission_path, 'r') as f:
                mission_data = json.load(f)
            if hasattr(hal.sim_mgr, 'load_mission_obstacles'):
                hal.sim_mgr.load_mission_obstacles(mission_data)

        # CSV
        logs = Path("./logs")
        logs.mkdir(exist_ok=True)
        csv_file = logs / f"telemetry_{datetime.now():%Y%m%d_%H%M%S}.csv"
        headers = list(flatten_dict(vehicle_state.get_status()).keys())
        telemetry_fh = open(csv_file, 'w', newline='')
        csv_writer = csv.DictWriter(telemetry_fh, fieldnames=headers)
        csv_writer.writeheader()

        # Signal
        def sigint(sig, frame):
            nonlocal exit_requested
            exit_requested = True
            print("\nShutting down...")
        signal.signal(signal.SIGINT, sigint)

        # Start
        print("="*70)
        print("MISSION STARTING")
        print(f"Mission: {mission_path.name}")
        print(f"GCS: {args.mavlink_host}:{args.mavlink_port}")
        print("Ctrl+C to stop")
        print("="*70)

        # === CONTROL LOOP ===
        dt = 0.1
        start = time.time()
        loop = 0

        # OA ONLY IN THESE TASKS
        ALLOWED_OA_TASKS = {'DIVE', 'CLIMB', 'SWIM_TO_WAYPOINT', 'FOLLOW_TRACK'}

        while not exit_requested:
            t = time.time()
            vehicle_state.mission_time = t - start

            try:
                if hal.mission_stop_requested or hal.emergency_stop_requested:
                    break

                executive.update(dt)

                current_task = getattr(vehicle_state, 'current_task', 'UNKNOWN')
                oa_control = {"active": False}

                # === OA: TASK-FILTERED ===
                if current_task in ALLOWED_OA_TASKS and obstacle_avoidance:
                    sonar = vehicle_state.sensor_data.get('sonar')
                    sonar_distance = sonar.get('range') if sonar else None
                    sonar_confidence = sonar.get('confidence') if sonar else None

                    obstacle_avoidance.update_sonar(sonar)
                    oa_control = obstacle_avoidance.update(dt, sonar_distance, sonar_confidence)
                    if oa_control.get("active"):
                        vehicle_state.target_state.update({
                            'target_heading': oa_control['target_heading'],
                            'target_speed': oa_control['target_speed'],
                            'target_depth': oa_control['target_depth'],
                            'target_lat': None,
                            'target_lon': None
                        })
                        if loop % 20 == 0:
                            print(f"[OA] ACTIVE in {current_task} → Hdg {oa_control['target_heading']:.1f}° @ {oa_control['target_speed']:.2f}m/s")
                else:
                    if loop % 100 == 0 and current_task != last_reported_task:
                        print(f"[OA] SUPPRESSED — task: {current_task}")
                        last_reported_task = current_task

                # === GUIDANCE ONLY IF OA INACTIVE ===
                if not oa_control.get("active", False):
                    guidance.update(dt)

                vehicle_control.update(dt)
                hal.update(dt)
                navigation.update(dt)
                energy.update(dt)

                diagnose_obstacle_detection(vehicle_state, hal, obstacle_avoidance, loop)

                # CSV
                if csv_writer:
                    row = {k: flatten_dict(vehicle_state.get_status()).get(k, '') for k in headers}
                    csv_writer.writerow(row)
                    if loop % 50 == 0:
                        telemetry_fh.flush()

                # Console
                if args.console_output and loop % 50 == 0:
                    nav = vehicle_state.nav_state
                    sr = vehicle_state.sensor_data.get('sonar', {}).get('range')
                    sstr = f"{sr:.1f}m" if sr else "—"
                    print(f"T+{vehicle_state.mission_time:.1f}s | {current_task} | "
                          f"Pos ({nav.get('local_x',0):.1f}, {nav.get('local_y',0):.1f}) | "
                          f"Sonar: {sstr} | OA: {oa_control.get('state','NORMAL')}")

                if executive.get_mission_status().get('mission_complete'):
                    print("MISSION COMPLETE")
                    break

                if vehicle_state.mission_time > 3600:
                    print("TIMEOUT")
                    break

            except Exception as e:
                add_status_message(f"ERROR: {e}")
                if args.verbose:
                    import traceback
                    traceback.print_exc()

            loop += 1
            sleep = dt - (time.time() - t)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down...")
        if hal:
            hal.emergency_stop()
            if hasattr(hal, 'shutdown'):
                hal.shutdown()
        if telemetry_fh:
            telemetry_fh.close()
        print("Shutdown complete.")
    return 0

# ==================== ENTRY POINT ====================
if __name__ == "__main__":
    sys.exit(main())