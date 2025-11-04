#!/usr/bin/env python3
"""
auv_main_unified.py - Combined Standalone & GCS Controller
================================================================
MODES:
1. Standalone: python auv_main_unified.py --mission file.json --no-gcs
2. Remote GCS: python auv_main_unified.py --wifi --gcs-host 192.168.1.100

FEATURES:
- OA ACTIVE ONLY IN: DIVE, CLIMB, SWIM_TO_WAYPOINT, FOLLOW_TRACK
- OA DISABLED IN: GO_TO_SURFACED_TRIM, GO_TO_SUBMERGED_TRIM, GPS_FIX, EMERGENCY_SURFACE, etc.
- 3D Unit Vector obstacle diagnostics
- Full control loop (executive → OA → guidance → control → HAL)
- CSV telemetry logging
- GCS discovery (broadcast + scan)
- Remote mission upload and start commands
- Graceful shutdown
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
import os
import socket
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
last_reported_task = None

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

def fallback_scan_for_gcs(port=15000):
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

def discover_gcs_host(port=15000):
    """Try broadcast discovery first, then fallback to scan"""
    print("=" * 70)
    print("GCS DISCOVERY")
    print("=" * 70)
    if discover_gcs_for_vehicle:
        print("Trying broadcast discovery...")
        gcs_addr = discover_gcs_for_vehicle(timeout=10.0)
        if gcs_addr:
            host = gcs_addr[0]
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
def extract_mission_origin(mission_source) -> tuple:
    """Extract origin from mission data (file path or dict)"""
    try:
        # If it's a file path, load it
        if isinstance(mission_source, (str, Path)):
            with open(mission_source, 'r') as f:
                mission_data = json.load(f)
        else:
            mission_data = mission_source
        
        # Try initial_position
        pos = mission_data.get('vehicle_config', {}).get('initial_position', {})
        lat, lon = pos.get('lat'), pos.get('lon')
        if lat and lon:
            print(f"Mission origin: ({lat:.6f}, {lon:.6f})")
            return float(lat), float(lon)
        
        # Fallback: first waypoint
        wp = mission_data.get('waypoints', [])
        if wp:
            lat, lon = wp[0].get('lat'), wp[0].get('lon')
            if lat and lon:
                print(f"Using first WP: ({lat:.6f}, {lon:.6f})")
                return float(lat), float(lon)
        
        print("No origin found in mission")
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
    """Periodic obstacle detection diagnostic output"""
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

# ==================== WAIT FOR MISSION FROM GCS ====================
def wait_for_mission_and_start(hal, executive, timeout=300):
    """Wait for mission upload and start command from GCS"""
    print("\n" + "="*70)
    print("WAITING FOR MISSION FROM GCS")
    print("="*70)
    
    mission_received = False
    start_received = False
    start_time = time.time()
    last_status_time = 0
    
    while time.time() - start_time < timeout:
        elapsed = time.time() - start_time
        
        # CRITICAL: Process incoming messages from GCS
        if hal.network_mgr.connected_to_gcs:
            hal.network_mgr.process_incoming_messages(hal.command_handler.handle_incoming_message)
        
        # Check if mission has been uploaded
        if not mission_received:
            if hal.command_handler.uploaded_mission_data:
                mission_received = True
                mission_name = hal.command_handler.uploaded_mission_data.get('mission_info', {}).get('name', 'Unknown')
                print(f"\n✓ Mission received: {mission_name}")
                print("Waiting for START command from GCS...")
        
        # Check if start command has been received
        if mission_received and not start_received:
            if hal.command_handler.mission_start_requested:
                start_received = True
                print("\n✓ START command received from GCS")
                hal.command_handler.clear_mission_start_flag()
                break
        
        # Status update every 5 seconds
        if int(elapsed) - last_status_time >= 5 and elapsed > 0:
            if not mission_received:
                print(f"[{int(elapsed)}s] Waiting for mission upload...")
            elif not start_received:
                print(f"[{int(elapsed)}s] Mission ready, waiting for START command...")
            last_status_time = int(elapsed)
        
        # Check for emergency stop
        if hal.emergency_stop_requested:
            print("\n✗ Emergency stop requested during wait")
            return False, None
        
        time.sleep(0.1)
    
    # Timeout check
    if not mission_received:
        print(f"\n✗ Timeout: No mission received after {timeout}s")
        return False, None
    
    if not start_received:
        print(f"\n✗ Timeout: No START command received after {timeout}s")
        return False, None
    
    # Mission received and started - extract and save it
    mission_data = hal.command_handler.uploaded_mission_data
    
    # Save mission to temporary file for executive to load
    temp_mission_path = PROJECT_ROOT / "temp_remote_mission.json"
    try:
        with open(temp_mission_path, 'w') as f:
            json.dump(mission_data, f, indent=2)
        print(f"\n✓ Mission saved to {temp_mission_path}")
    except Exception as e:
        print(f"\n✗ Failed to save mission file: {e}")
        return False, None
    
    # Load mission into executive
    os.environ['SIMPLR_MISSION_FILE'] = str(temp_mission_path)
    if not executive.load_mission(str(temp_mission_path)):
        print("\n✗ Failed to load mission into executive")
        return False, None
    
    print("✓ Mission loaded into executive")
    print("=" * 70)
    
    return True, mission_data

# ==================== MAIN FUNCTION ====================
def main():
    global last_reported_task

    # --- ARGUMENTS ---
    parser = argparse.ArgumentParser(description='SIMPLR-AUV Unified Controller')
    parser.add_argument('--mode', choices=['simulation', 'hardware'], default='simulation')
    
    # Standalone mode
    parser.add_argument('--mission', type=str, help='Mission JSON file (for standalone mode)')
    parser.add_argument('--no-gcs', action='store_true', help='Run standalone without GCS')
    
    # GCS mode
    parser.add_argument('--wifi', action='store_true', help='Enable WiFi connection to remote GCS')
    parser.add_argument('--gcs-host', type=str, help='GCS host IP address')
    parser.add_argument('--gcs-port', type=int, default=15000, help='GCS TCP port (default: 15000)')
    parser.add_argument('--wait-timeout', type=int, default=300, help='Timeout for waiting for mission (seconds)')
    
    # Common
    parser.add_argument('--verbose', action='store_true', help='Verbose output')
    parser.add_argument('--console-output', action='store_true', help='Enable console telemetry output')
    
    args = parser.parse_args()

    # --- MODE VALIDATION ---
    if args.no_gcs and not args.mission:
        print("ERROR: --no-gcs requires --mission <file>")
        return 1
    
    if not args.no_gcs and not args.wifi:
        print("ERROR: Must specify either --no-gcs (standalone) or --wifi (GCS mode)")
        return 1

    # --- STANDALONE MODE: Load mission from file ---
    if args.no_gcs:
        print("="*70)
        print("STANDALONE MODE - No GCS Connection")
        print("="*70)
        
        mission_path = Path(args.mission)
        if not mission_path.exists():
            mission_path = PROJECT_ROOT / "missions" / args.mission
        if not mission_path.exists() and not mission_path.suffix:
            mission_path = PROJECT_ROOT / "missions" / f"{args.mission}.json"
        if not mission_path.exists():
            print(f"Mission not found: {args.mission}")
            return 1
        
        origin_lat, origin_lon = extract_mission_origin(mission_path)
        if not origin_lat:
            print("No origin in mission")
            return 1
        
        args.gcs_host = 'localhost'  # Dummy value for HAL config
        remote_mission_mode = False
        mission_file_path = mission_path

    # --- GCS MODE: Discover GCS and wait for mission ---
    else:
        print("="*70)
        print("REMOTE GCS MODE - Waiting for Connection")
        print("="*70)
        
        if not args.gcs_host:
            host = discover_gcs_host(args.gcs_port)
            if host:
                args.gcs_host = host
            else:
                print("ERROR: No GCS found. Specify --gcs-host or ensure GCS is running.")
                return 1

        print(f"GCS Target: {args.gcs_host}:{args.gcs_port}")
        remote_mission_mode = True
        mission_file_path = None  # Will be set after upload

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

        # Initialize vehicle state first
        vehicle_state = VehicleState()
        
        # Initialize HAL with network config
        # Note: wifi_enabled controls network connectivity to GCS
        hal_config = {
            'mode': args.mode,
            'wifi_enabled': not args.no_gcs,  # Enable WiFi only in GCS mode
            'tcp_host': args.gcs_host,
            'tcp_port': args.gcs_port,
            'verbose': args.verbose,
            'vehicle_id': f"SIMPLR-AUV-{int(time.time())}"
        }
        hal = EnhancedHAL(hal_config, vehicle_state)
        
        # Initialize other systems (navigation comes later after we have origin)
        vehicle_control = VehicleControl(vehicle_state)
        guidance = TaskManager(vehicle_state)
        executive = Executive(vehicle_state)
        energy = Energy(vehicle_state, hal)

        # --- STANDALONE MODE: Load mission now ---
        if args.no_gcs:
            os.environ['SIMPLR_MISSION_FILE'] = str(mission_file_path)
            if not executive.load_mission(str(mission_file_path)):
                print("Failed to load mission")
                return 1
            print(f"✓ Mission loaded: {mission_file_path}")
            
            # Load obstacles into sim
            if hasattr(hal, 'sim_mgr') and hal.sim_mgr:
                with open(mission_file_path, 'r') as f:
                    mission_data = json.load(f)
                if hasattr(hal.sim_mgr, 'load_mission_obstacles'):
                    hal.sim_mgr.load_mission_obstacles(mission_data)

        # --- GCS MODE: Wait for connection, mission upload, and start ---
        else:
            # Wait for HAL to connect to GCS
            print(f"\n[Main] Connecting to GCS at {args.gcs_host}:{args.gcs_port}...")
            if not hal.await_gcs_connection(timeout=30):
                print("✗ Failed to connect to GCS")
                return 1
            
            print("[Main] Connected to GCS, waiting for mission...")

            # Wait for mission upload and start command
            success, mission_data = wait_for_mission_and_start(hal, executive, args.wait_timeout)
            if not success:
                print("Failed to receive mission or start command")
                return 1

            # Extract mission origin from uploaded data
            origin_lat, origin_lon = extract_mission_origin(mission_data)
            if not origin_lat:
                print("ERROR: No origin in mission data")
                return 1
            
            # Load obstacles into sim
            if hasattr(hal, 'sim_mgr') and hal.sim_mgr:
                if hasattr(hal.sim_mgr, 'load_mission_obstacles'):
                    print("[Main] Loading mission obstacles into simulation...")
                    hal.sim_mgr.load_mission_obstacles(mission_data)

        # Initialize navigation with mission origin
        navigation = Navigation(vehicle_state, hal, origin_lat=origin_lat, origin_lon=origin_lon, declination_deg=12.5)
        vehicle_state.navigation = navigation

        # Mark mission as started in HAL (enables telemetry)
        hal.mark_mission_started(True)

        # OA Init
        if OBSTACLE_AVOIDANCE_AVAILABLE:
            cfg = PROJECT_ROOT / "oa_config.json"
            obstacle_avoidance = ObstacleAvoidance(
                vehicle_state,
                config_file=str(cfg) if cfg.exists() else {}
            )
            vehicle_state.obstacle_avoidance = obstacle_avoidance
            print("✓ Obstacle avoidance initialized")
        else:
            print("⚠ Obstacle avoidance not available")

        # CSV Telemetry
        logs = Path("./logs")
        logs.mkdir(exist_ok=True)
        csv_file = logs / f"telemetry_{datetime.now():%Y%m%d_%H%M%S}.csv"
        headers = list(flatten_dict(vehicle_state.get_status()).keys())
        telemetry_fh = open(csv_file, 'w', newline='')
        csv_writer = csv.DictWriter(telemetry_fh, fieldnames=headers)
        csv_writer.writeheader()
        print(f"✓ Telemetry logging to {csv_file}")

        # Signal handlers
        def signal_handler(sig, frame):
            nonlocal exit_requested
            print("\n\nShutdown requested...")
            exit_requested = True

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        # Mission execution loop
        print("\n" + "="*70)
        print("STARTING MISSION EXECUTION")
        print("="*70)
        
        dt = 0.1
        start_time = time.time()
        loop_count = 0

        # OA ONLY IN THESE TASKS
        ALLOWED_OA_TASKS = {'DIVE', 'CLIMB', 'SWIM_TO_WAYPOINT', 'FOLLOW_TRACK'}

        while not exit_requested:
            loop_start = time.time()
            vehicle_state.mission_time = time.time() - start_time

            try:
                # Check for stop commands
                if hal.mission_stop_requested or hal.emergency_stop_requested:
                    print("\n✗ Stop command received")
                    break

                # Update executive (returns False when mission complete)
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
                        if loop_count % 20 == 0:
                            print(f"[OA] ACTIVE in {current_task} → Hdg {oa_control['target_heading']:.1f}° @ {oa_control['target_speed']:.2f}m/s")
                else:
                    if loop_count % 100 == 0 and current_task != last_reported_task:
                        print(f"[OA] SUPPRESSED — task: {current_task}")
                        last_reported_task = current_task

                # === GUIDANCE ONLY IF OA INACTIVE ===
                if not oa_control.get("active", False):
                    guidance.update(dt)

                # Update other systems
                vehicle_control.update(dt)
                hal.update(dt)
                navigation.update(dt)
                energy.update(dt)

                # Obstacle diagnostics
                diagnose_obstacle_detection(vehicle_state, hal, obstacle_avoidance, loop_count)

                # CSV logging
                if csv_writer:
                    row = {k: flatten_dict(vehicle_state.get_status()).get(k, '') for k in headers}
                    csv_writer.writerow(row)
                    if loop_count % 50 == 0:
                        telemetry_fh.flush()

                # Console output
                if args.console_output and loop_count % 50 == 0:
                    nav = vehicle_state.nav_state
                    sr = vehicle_state.sensor_data.get('sonar', {}).get('range')
                    sstr = f"{sr:.1f}m" if sr else "—"
                    print(f"T+{vehicle_state.mission_time:.1f}s | {current_task} | "
                          f"Pos ({nav.get('local_x',0):.1f}, {nav.get('local_y',0):.1f}) | "
                          f"Sonar: {sstr} | OA: {oa_control.get('state','NORMAL')}")

                # Mission complete check (use status dict instead of return value)
                mission_status = executive.get_mission_status()
                if mission_status.get('mission_complete', False):
                    print("\n" + "="*70)
                    print("MISSION COMPLETE")
                    print("="*70)
                    break

                # Timeout check
                if vehicle_state.mission_time > 3600:
                    print("\n✗ MISSION TIMEOUT (1 hour)")
                    break

            except Exception as e:
                add_status_message(f"ERROR: {e}")
                if args.verbose:
                    import traceback
                    traceback.print_exc()

            loop_count += 1
            
            # Loop timing
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print("\n\nKeyboard interrupt received")
    except Exception as e:
        print(f"\n\nFATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        print("\nShutdown sequence...")
        
        if telemetry_fh:
            telemetry_fh.close()
            print("✓ Telemetry file closed")
        
        if hal:
            hal.emergency_stop()
            if hasattr(hal, 'shutdown'):
                hal.shutdown()
            print("✓ HAL shutdown")
        
        print("✓ Shutdown complete")

    return 0

if __name__ == "__main__":
    sys.exit(main())
