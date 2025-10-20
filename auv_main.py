#!/usr/bin/env python3
"""
auv_main.py - Complete SIMPLR-AUV Vehicle Controller with WiFi support  
Integrates existing functionality with WiFi telemetry for separate machine deployment
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

PROJECT_ROOT = Path(__file__).parent.resolve()
sys.path.insert(0, str(PROJECT_ROOT))

# Import broadcast discovery
try:
    from broadcast_discovery import discover_gcs_for_vehicle
except ImportError:
    print("Warning: broadcast_discovery module not found. Using fallback discovery.")
    discover_gcs_for_vehicle = None

# Import obstacle avoidance
try:
    from obstacle_avoidance import ObstacleAvoidance
    OBSTACLE_AVOIDANCE_AVAILABLE = True
except ImportError:
    OBSTACLE_AVOIDANCE_AVAILABLE = False

# Global message buffer for status messages
STATUS_MESSAGES = deque(maxlen=10)

def add_status_message(message):
    """Add a message to the status display"""
    timestamp = datetime.now().strftime("%H:%M:%S")
    STATUS_MESSAGES.append(f"[{timestamp}] {message}")

def flatten_dict(d, prefix=''):
    """Flatten a nested dictionary into one level with prefix keys."""
    flat = {}
    for k, v in d.items():
        if isinstance(v, dict):
            flat.update(flatten_dict(v, prefix=f"{prefix}{k}_"))
        else:
            flat[f"{prefix}{k}"] = v
    return flat

def safe_get(obj, key, default=0.0):
    """Safely get a value from a dict/object with fallback"""
    try:
        if hasattr(obj, key):
            return getattr(obj, key, default)
        elif isinstance(obj, dict):
            return obj.get(key, default)
        else:
            return default
    except:
        return default

def get_local_ip():
    """Get the local IP address on the WiFi network"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
        return local_ip
    except Exception:
        return None

def fallback_scan_for_gcs(port=14550, timeout=5):
    """Fallback scan when broadcast discovery is not available"""
    local_ip = get_local_ip()
    if not local_ip:
        return None
    
    print(f"Scanning for GCS on network {local_ip}/24...")
    
    # Get network range and test common IPs
    base_ip = '.'.join(local_ip.split('.')[:-1])
    test_ips = [f"{base_ip}.{i}" for i in [1, 10, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 100, 101, 102, 200]]
    
    found_servers = []
    for ip in test_ips:
        if str(ip) == local_ip:
            continue
            
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(1.0)
                result = sock.connect_ex((str(ip), port))
                if result == 0:
                    found_servers.append(str(ip))
                    print(f"Found GCS server at {ip}:{port}")
        except Exception:
            continue
    
    return found_servers[0] if found_servers else None

def discover_gcs_host(port=14550):
    """
    Discover GCS host on network using broadcast discovery or fallback scan.
    Returns the discovered IP address or None.
    """
    print("=" * 70)
    print("GCS DISCOVERY")
    print("=" * 70)
    
    # Try broadcast discovery first
    if discover_gcs_for_vehicle:
        print("Attempting broadcast discovery...")
        discovered_host = discover_gcs_for_vehicle()
        if discovered_host:
            print(f"Found GCS via broadcast: {discovered_host}")
            print("=" * 70)
            return discovered_host
        else:
            print("No GCS found via broadcast discovery")
    
    # Fallback to network scan
    print("Attempting network scan...")
    discovered_host = fallback_scan_for_gcs(port)
    if discovered_host:
        print(f"Found GCS via scan: {discovered_host}")
        print("=" * 70)
        return discovered_host
    
    print("No GCS found on network")
    print("=" * 70)
    return None

def configure_windows_firewall_for_network(port):
    """Configure Windows Firewall for network communication"""
    if platform.system() != "Windows":
        return True
    
    try:
        # Remove any existing rules first
        subprocess.run([
            "netsh", "advfirewall", "firewall", "delete", "rule",
            f"name=SIMPLR-AUV Port {port}"
        ], capture_output=True, timeout=10)
        
        # Add inbound rule for all profiles
        subprocess.run([
            "netsh", "advfirewall", "firewall", "add", "rule",
            f"name=SIMPLR-AUV Port {port}",
            "dir=in",
            "action=allow",
            "protocol=TCP",
            f"localport={port}",
            "enable=yes",
            "profile=any"
        ], check=True, timeout=10)
        
        # Add outbound rule
        subprocess.run([
            "netsh", "advfirewall", "firewall", "add", "rule",
            f"name=SIMPLR-AUV Port {port} Outbound",
            "dir=out",
            "action=allow",
            "protocol=TCP",
            f"remoteport={port}",
            "enable=yes",
            "profile=any"
        ], check=True, timeout=10)
        
        print(f"Configured Windows Firewall for network communication on port {port}")
        return True
        
    except subprocess.CalledProcessError as e:
        print(f"Failed to configure Windows Firewall: {e}")
        return False
    except Exception as e:
        print(f"Firewall configuration error: {e}")
        return False

def test_network_connectivity(host, port, timeout=5):
    """Test network connectivity to GCS"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(timeout)
            result = sock.connect_ex((host, port))
            if result == 0:
                print(f"Successfully connected to {host}:{port}")
                return True
            else:
                print(f"Failed to connect to {host}:{port} (error: {result})")
                return False
    except Exception as e:
        print(f"Network connectivity test failed: {e}")
        return False

def check_admin_privileges():
    """Check if running with administrator privileges on Windows"""
    if sys.platform == "win32":
        try:
            import ctypes
            return ctypes.windll.shell32.IsUserAnAdmin()
        except:
            return False
    return True

class SilentOutput:
    """Context manager to suppress all output"""
    def __enter__(self):
        self._original_stdout = sys.stdout
        self._original_stderr = sys.stderr
        sys.stdout = io.StringIO()
        sys.stderr = io.StringIO()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        sys.stdout = self._original_stdout
        sys.stderr = self._original_stderr

def run_no_mission_workflow(args):
    """
    Complete workflow for no-mission mode: connect to GCS, receive mission, execute it.
    Returns exit code.
    """
    from enhanced_hal import EnhancedHAL
    from vehicle_state import VehicleState
    from vehicle_control import VehicleControl
    from executive import Executive
    from guidance.task_manager import TaskManager
    from navigation import Navigation
    from energy import Energy

    obstacle_avoidance = None
    if OBSTACLE_AVOIDANCE_AVAILABLE:
        try:
            from obstacle_avoidance import ObstacleAvoidance
        except ImportError:
            pass

    hal = None
    energy = None
    telemetry_fh = None
    csv_writer = None
    exit_requested = False

    try:
        print("\nInitializing vehicle systems...")
        vehicle_state = VehicleState()

        wifi_deployment = args.mavlink_host not in ['localhost', '127.0.0.1']
        hal_config = {
            'mode': args.mode,
            'mavlink_enabled': False,
            'tcp_host': args.mavlink_host,
            'tcp_port': args.mavlink_port,
            'verbose': args.verbose,
            'wifi_enabled': wifi_deployment
        }
        
        hal = EnhancedHAL(hal_config, vehicle_state)
        
        print("\nWaiting for GCS connection...")
        if not hal.await_gcs_connection(timeout=args.connection_timeout):
            print("ERROR: Could not connect to GCS")
            print(f"Make sure GCS is running and listening on {args.mavlink_host}:{args.mavlink_port}")
            hal.shutdown()
            return 1
        
        print("Connected to GCS")
        
        print("\nWaiting for mission upload and START command from GCS...")
        mission_data = hal.listen_for_commands()
        if not mission_data:
            print("ERROR: No mission data received")
            hal.shutdown()
            return 1
        
        print("Mission received and START command confirmed")
        
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump(mission_data, f, indent=2)
            temp_mission_file = f.name
        
        print(f"Mission saved to: {temp_mission_file}")
        print("\n" + "=" * 70)
        print("STARTING MISSION EXECUTION")
        print("=" * 70)
        
        print("Loading mission execution subsystems...")
        navigation = Navigation(vehicle_state, hal)
        vehicle_control = VehicleControl(vehicle_state)
        guidance = TaskManager(vehicle_state)
        executive = Executive(vehicle_state)
        energy = Energy(vehicle_state, hal)
        
        if OBSTACLE_AVOIDANCE_AVAILABLE:
            avoidance_config = {
                'enabled': True,
                'detection_threshold': 25.0,              # CHANGED: was 15.0
                'clear_threshold': 35.0,                  # CHANGED: was 25.0
                'turn_rate': 9.0,
                'speed_reduction_factor': 0.75,
                'avoidance_mode': '3d',
                'depth_avoidance_offset': 3.0,
                'min_avoidance_depth': 0.5,
                'max_avoidance_depth': 15.0,
                # NEW MULTI-OBSTACLE PARAMETERS:
                'obstacle_cooldown': 10.0,
                'min_obstacle_separation': 5.0,
                'confidence_threshold': 60.0,             # CHANGED: was 30.0
                'out_of_beam_confidence': 10.0,
                'enable_adaptive_turning': True,
                'turn_reversal_threshold': 0.5            # CHANGED: was -0.3
            }
            obstacle_avoidance = ObstacleAvoidance(vehicle_state, avoidance_config)
            vehicle_state.obstacle_avoidance = obstacle_avoidance
            print("Multi-obstacle avoidance initialized")

        os.environ['SIMPLR_MISSION_FILE'] = str(temp_mission_file)
        mission_loaded = executive.load_mission(temp_mission_file)
        if not mission_loaded:
            print("ERROR: Failed to load mission")
            faults = vehicle_state.get_faults()
            for fault in faults:
                print(f"FAULT: {fault}")
            hal.shutdown()
            return 1
        
        print("Mission loaded and validated")
        
        if hasattr(hal, 'sim_mgr') and hal.sim_mgr:
            with open(temp_mission_file, 'r') as mf:
                mission_data_reload = json.load(mf)
            if hasattr(hal.sim_mgr, 'load_mission_obstacles'):
                hal.sim_mgr.load_mission_obstacles(mission_data_reload)

        print("\n" + "=" * 70)
        print("SONAR STATUS CHECK")
        print("=" * 70)
        if hasattr(hal, 'sim_mgr') and hal.sim_mgr:
            if hasattr(hal.sim_mgr, 'simulated_sonar') and hal.sim_mgr.simulated_sonar:
                sonar_status = hal.sim_mgr.simulated_sonar.get_status()
                print(f"Simulated sonar ACTIVE")
                print(f"  Max range: {sonar_status['max_range']}m")
                print(f"  Obstacles loaded: {sonar_status['obstacles_loaded']}")
                
                if hal.sim_mgr.simulated_sonar.obstacles:
                    print(f"  Obstacle details:")
                    for i, obs in enumerate(hal.sim_mgr.simulated_sonar.obstacles):
                        name = obs.get('name', f'Obstacle {i+1}')
                        pos = obs.get('position', {})
                        x = pos.get('local_x', 0)
                        y = pos.get('local_y', 0)
                        print(f"    [{i+1}] {name} at ({x:.1f}, {y:.1f})")
            else:
                print("WARNING: Simulated sonar NOT initialized!")
        else:
            print("No simulation manager found")
        print("=" * 70 + "\n")
        
        print("Marking mission as started...")
        hal.mark_mission_started(True)
        print("Telemetry should now be active")
        
        logs_dir = Path("./logs")
        logs_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_file = logs_dir / f"telemetry_{timestamp}.csv"
        
        try:
            status_snapshot = vehicle_state.get_status()
            flat_status = flatten_dict(status_snapshot)
            csv_headers = list(flat_status.keys())
            
            telemetry_fh = open(csv_file, 'w', newline='')
            csv_writer = csv.DictWriter(telemetry_fh, fieldnames=csv_headers)
            csv_writer.writeheader()
            add_status_message(f"Telemetry logging: {csv_file.name}")
        except Exception as e:
            print(f"CSV setup failed: {e}")
            csv_writer = None

        def handle_sigint(sig, frame):
            nonlocal exit_requested
            if not exit_requested:
                exit_requested = True
                print("\nShutdown requested...")
            else:
                print("\nForced exit!")
                sys.exit(1)

        if hasattr(signal, 'SIGINT'):
            signal.signal(signal.SIGINT, handle_sigint)

        print("=" * 70)
        print("SIMPLR-AUV VEHICLE CONTROLLER - MISSION EXECUTION")
        print("=" * 70)
        print(f"Mode: {args.mode.upper()}")
        print(f"GCS Connection: {args.transport.upper()} to {args.mavlink_host}:{args.mavlink_port}")
        print(f"Console Dashboard: {'ENABLED' if args.console_output else 'DISABLED'}")
        print(f"Telemetry Log: {csv_file.name}")
        if wifi_deployment:
            local_ip = get_local_ip()
            print(f"WiFi Mode: ENABLED (Local IP: {local_ip})")
        print("=" * 70)
        print("Press Ctrl+C to shutdown gracefully")
        print("=" * 70)

        add_status_message("Starting main control loop")
        control_start_time = time.time()
        dt = 0.1
        dashboard_interval = 5.0
        last_dashboard_time = control_start_time
        loop_count = 0

        previous_task = None
        previous_system_state = None

        while not exit_requested:
            loop_start = time.time()
            vehicle_state.mission_time = loop_start - control_start_time
            
            try:
                if hal.mission_stop_requested:
                    hal.clear_mission_stop_flag()
                    print("Mission stop command received")
                    break
                
                if hal.mission_pause_requested:
                    hal.clear_mission_pause_flag()
                    print("Mission pause command received")
                
                if hal.emergency_stop_requested:
                    hal.clear_emergency_stop_flag()
                    print("Emergency stop command received")
                    break

                if not args.verbose:
                    with SilentOutput():
                        executive.update(dt)
                        guidance.update(dt)
                        hal.update(dt)  # HAL generates sonar data FIRST
                        
                        # Now obstacle avoidance can use fresh sonar data
                        if obstacle_avoidance:
                            sonar_pkt = vehicle_state.sensor_data.get('sonar', None)
                            if sonar_pkt is not None:
                                obstacle_avoidance.update_sonar(sonar_pkt)
                            obstacle_avoidance.update(dt)
                            
                        navigation.update(dt)
                        vehicle_control.update(dt)
                        energy.update(dt)
                else:
                    executive.update(dt)
                    guidance.update(dt)
                    hal.update(dt)  # HAL generates sonar data FIRST
                    
                    # Now obstacle avoidance can use fresh sonar data
                    if obstacle_avoidance:
                        sonar_pkt = vehicle_state.sensor_data.get('sonar', None)
                        if sonar_pkt is not None:
                            obstacle_avoidance.update_sonar(sonar_pkt)
                        obstacle_avoidance.update(dt)
                        
                    navigation.update(dt)
                    vehicle_control.update(dt)
                    energy.update(dt)

                current_task = safe_get(vehicle_state, 'current_task', 'UNKNOWN')
                try:
                    current_system_state = vehicle_state.current_state.value
                except:
                    current_system_state = 'UNKNOWN'
                
                if current_task != previous_task:
                    add_status_message(f"Task: {current_task}")
                    previous_task = current_task
                
                if current_system_state != previous_system_state:
                    add_status_message(f"State: {current_system_state}")
                    previous_system_state = current_system_state

                if csv_writer:
                    try:
                        status_snapshot = vehicle_state.get_status()
                        flat_status = flatten_dict(status_snapshot)
                        row_to_write = {k: flat_status.get(k, '') for k in csv_headers}
                        csv_writer.writerow(row_to_write)
                        
                        if loop_count % 50 == 0:
                            telemetry_fh.flush()
                    except Exception as e:
                        if loop_count % 100 == 0 and args.verbose:
                            add_status_message(f"CSV logging error: {e}")

                if args.console_output and (loop_start - last_dashboard_time >= dashboard_interval):
                    try:
                        mission_time = safe_get(vehicle_state, 'mission_time', 0.0)
                        nav_state = safe_get(vehicle_state, 'nav_state', {})
                        current_depth = safe_get(nav_state, 'depth', 0.0)
                        current_speed = safe_get(nav_state, 'speed', 0.0)
                        
                        gcs_connected = False
                        if hasattr(hal, 'get_connection_status'):
                            conn_status = hal.get_connection_status()
                            gcs_connected = conn_status.get('connected_to_gcs', False)
                        
                        gcs_status = "GCS:ON" if gcs_connected else "GCS:OFF"
                        if wifi_deployment:
                            gcs_status = f"WiFi-{gcs_status}"
                        
                        print(f"Loop {loop_count}: {current_task} | {current_system_state} | {gcs_status} | "
                              f"Time: {mission_time:.1f}s | Depth: {current_depth:.1f}m | Speed: {current_speed:.2f}m/s")
                        
                        if STATUS_MESSAGES:
                            latest_msg = list(STATUS_MESSAGES)[-1]
                            print(f"  Last: {latest_msg}")
                            
                    except Exception:
                        gcs_connected = False
                        if hasattr(hal, 'get_connection_status'):
                            conn_status = hal.get_connection_status()
                            gcs_connected = conn_status.get('connected_to_gcs', False)
                        
                        gcs_status = "GCS:ON" if gcs_connected else "GCS:OFF"
                        if wifi_deployment:
                            gcs_status = f"WiFi-{gcs_status}"
                        print(f"Loop {loop_count}: Running | {gcs_status} | Time: {vehicle_state.mission_time:.1f}s")
                        
                    last_dashboard_time = loop_start

                try:
                    mission_status = executive.get_mission_status()
                    if mission_status.get('mission_complete', False):
                        add_status_message(f"Mission completed in {vehicle_state.mission_time:.1f}s")
                        break
                except:
                    pass
                    
                if vehicle_state.mission_time > 3600:
                    add_status_message(f"Mission timeout after {vehicle_state.mission_time:.1f}s")
                    break

                try:
                    if vehicle_state.current_state.value == 'FAULT':
                        add_status_message("System in FAULT state")
                        break
                except:
                    pass

            except Exception as e:
                add_status_message(f"ERROR in main loop: {e}")
                if args.verbose:
                    import traceback
                    traceback.print_exc()
                if loop_count < 5:
                    print(f"CRITICAL: Early loop error on iteration {loop_count}: {e}")
                    import traceback
                    traceback.print_exc()
                    break

            loop_count += 1
            loop_time = time.time() - loop_start
            if loop_time < dt:
                time.sleep(dt - loop_time)

    except KeyboardInterrupt:
        add_status_message("Keyboard interrupt received")
    except Exception as e:
        add_status_message(f"Fatal error: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1
    finally:
        print("\nShutting down vehicle systems...")
        
        if hal:
            try:
                hal.emergency_stop()
                if hasattr(hal, 'shutdown'):
                    hal.shutdown()
                print("HAL stopped")
            except Exception as e:
                if args.verbose:
                    print(f"HAL shutdown error: {e}")
                
        if telemetry_fh:
            try:
                telemetry_fh.close()
                print("Telemetry CSV closed")
            except Exception as e:
                if args.verbose:
                    print(f"CSV close error: {e}")
        
        if STATUS_MESSAGES:
            print("\nFinal Status Messages:")
            for msg in STATUS_MESSAGES:
                print(f"  {msg}")
        
        print("Vehicle shutdown complete")
        time.sleep(1)

    return 0

def main():
    parser = argparse.ArgumentParser(description='SIMPLR-AUV Vehicle Controller (WiFi-Enabled)')
    parser.add_argument('--mode', choices=['simulation', 'hardware'], default='simulation')
    parser.add_argument('--mission', required=False, help="Path to mission file (optional)")
    parser.add_argument('--verbose', action='store_true', help='Show all initialization output')
    parser.add_argument('--mavlink-port', type=int, default=14550, help='GCS server port to connect to')
    parser.add_argument('--mavlink-host', default=None, help='GCS server host IP (auto-discover if not specified)')
    parser.add_argument('--transport', choices=['tcp', 'rf'], default='tcp', help='Telemetry transport type')
    parser.add_argument('--console-output', action='store_true', help='Show console dashboard output')
    parser.add_argument('--connection-timeout', type=int, default=60, help='Seconds to wait for initial GCS connection')
    parser.add_argument('--no-firewall', action='store_true', help='Skip Windows firewall configuration')
    parser.add_argument('--no-discovery', action='store_true', help='Skip network discovery of GCS')
    parser.add_argument('--auto-start', action='store_true', help='Start mission automatically')
    parser.add_argument('--no-gcs', action='store_true', help='Run without GCS (standalone simulation mode)')
    args = parser.parse_args()

    if args.no_gcs:
        print("=" * 70)
        print("STANDALONE SIMULATION MODE (No GCS)")
        print("=" * 70)
        
        if not args.mission:
            print("ERROR: --mission is required when using --no-gcs")
            return 1
        
        args.mavlink_host = 'localhost'
        args.no_discovery = True
        args.auto_start = True
        
        print(f"Mode: {args.mode.upper()}")
        print(f"Mission: {args.mission}")
        print(f"GCS: DISABLED (Standalone)")
        print("=" * 70)
        print()
    
    elif not args.mavlink_host and not args.no_discovery:
        discovered_host = discover_gcs_host(args.mavlink_port)
        if discovered_host:
            args.mavlink_host = discovered_host
        else:
            print("=" * 70)
            print("WARNING: No GCS found on network")
            print("=" * 70)
            print("Options:")
            print("  1. Specify GCS IP manually: --mavlink-host 192.168.1.100")
            print("  2. Run GCS first, then restart this vehicle software")
            print("  3. Use --no-gcs for standalone simulation without GCS")
            print("=" * 70)
            return 1
    elif not args.mavlink_host:
        args.mavlink_host = 'localhost'

    if not args.mission and not args.no_gcs:
        print("=" * 70)
        print("NO MISSION MODE - Waiting for GCS")
        print("=" * 70)
        return run_no_mission_workflow(args)

    local_ip = get_local_ip()
    if not args.no_gcs:
        print("=" * 70)
        print("SIMPLR-AUV VEHICLE CONTROLLER")
        print("=" * 70)
        print(f"Local Machine IP: {local_ip}")
        print(f"Target GCS: {args.mavlink_host}:{args.mavlink_port}")
        
        wifi_deployment = args.mavlink_host != 'localhost' and args.mavlink_host != '127.0.0.1'
        if wifi_deployment:
            print("WiFi Deployment Mode: ENABLED")
        else:
            print("Local Deployment Mode")
        print("=" * 70)

        if sys.platform == "win32" and not args.no_firewall and wifi_deployment:
            if not check_admin_privileges():
                print("Warning: Not running as Administrator")
            if not configure_windows_firewall_for_network(args.mavlink_port):
                print("Warning: Firewall configuration may have failed")

        if wifi_deployment:
            if not test_network_connectivity(args.mavlink_host, args.mavlink_port, timeout=5):
                print("Warning: Cannot connect to GCS")
    else:
        wifi_deployment = False

    hal = None
    energy = None
    obstacle_avoidance = None
    telemetry_fh = None
    csv_writer = None
    exit_requested = False

    try:
        if not args.verbose:
            print("Initializing SIMPLR-AUV systems...")
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

        if not args.verbose:
            with SilentOutput():
                vehicle_state = VehicleState()
        else:
            vehicle_state = VehicleState()

        hal_config = {
            'mode': args.mode, 
            'sensor_update_rate': 10.0,
            'mavlink_enabled': not args.no_gcs,
            'transport_type': args.transport,
            'tcp_host': args.mavlink_host,
            'tcp_port': args.mavlink_port,
            'verbose': args.verbose,
            'wifi_enabled': wifi_deployment and not args.no_gcs
        }

        if args.no_gcs:
            print("Loading subsystems WITHOUT GCS integration...")
        else:
            print("Loading subsystems with GCS integration...")
            
        if not args.verbose:
            try:
                with SilentOutput():
                    hal = EnhancedHAL(hal_config, vehicle_state)
                    navigation = Navigation(vehicle_state, hal)
                    vehicle_control = VehicleControl(vehicle_state)
                    guidance = TaskManager(vehicle_state)
                    executive = Executive(vehicle_state)
                    energy = Energy(vehicle_state, hal)
                print("Subsystems loaded")
            except Exception as e:
                print(f"Error during subsystem initialization: {e}")
                hal = EnhancedHAL(hal_config, vehicle_state)
                navigation = Navigation(vehicle_state, hal)
                vehicle_control = VehicleControl(vehicle_state)
                guidance = TaskManager(vehicle_state)
                executive = Executive(vehicle_state)
                energy = Energy(vehicle_state, hal)
        else:
            hal = EnhancedHAL(hal_config, vehicle_state)
            navigation = Navigation(vehicle_state, hal)
            vehicle_control = VehicleControl(vehicle_state)
            guidance = TaskManager(vehicle_state)
            executive = Executive(vehicle_state)
            energy = Energy(vehicle_state, hal)
        
        if OBSTACLE_AVOIDANCE_AVAILABLE:
            avoidance_config = {
                'enabled': True,
                'detection_threshold': 25.0,              # CHANGED: was 15.0
                'clear_threshold': 35.0,                  # CHANGED: was 25.0
                'turn_rate': 9.0,
                'speed_reduction_factor': 0.75,
                'avoidance_mode': '3d',              
                'depth_avoidance_offset': 3.0,       
                'min_avoidance_depth': 0.5,          
                'max_avoidance_depth': 15.0,
                # NEW MULTI-OBSTACLE PARAMETERS:
                'obstacle_cooldown': 10.0,
                'min_obstacle_separation': 5.0,
                'confidence_threshold': 60.0,             # CHANGED: was 30.0
                'out_of_beam_confidence': 10.0,
                'enable_adaptive_turning': True,
                'turn_reversal_threshold': 0.5            # CHANGED: was -0.3
            }
            obstacle_avoidance = ObstacleAvoidance(vehicle_state, avoidance_config)
            vehicle_state.obstacle_avoidance = obstacle_avoidance
            print("Multi-obstacle avoidance initialized")

        mission_path = Path(args.mission)
        if not mission_path.exists():
            mission_path = PROJECT_ROOT / "missions" / args.mission
        if not mission_path.exists() and not mission_path.suffix:
            mission_path = PROJECT_ROOT / "missions" / f"{args.mission}.json"

        if not mission_path.exists():
            print(f"ERROR: Mission file not found: {args.mission}")
            return 1

        os.environ['SIMPLR_MISSION_FILE'] = str(mission_path)
        mission_loaded = executive.load_mission(str(mission_path))
        if not mission_loaded:
            faults = vehicle_state.get_faults()
            for fault in faults:
                print(f"FAULT: {fault}")
            return 1
        
        if hasattr(hal, 'sim_mgr') and hal.sim_mgr:
            with open(str(mission_path), 'r') as mf:
                mission_data = json.load(mf)
            if hasattr(hal.sim_mgr, 'load_mission_obstacles'):
                obstacles = mission_data.get('obstacles', None)
                hal.sim_mgr.load_mission_obstacles(mission_data)

        print("\n" + "=" * 70)
        print("SONAR STATUS CHECK")
        print("=" * 70)
        if hasattr(hal, 'sim_mgr') and hal.sim_mgr:
            if hasattr(hal.sim_mgr, 'simulated_sonar') and hal.sim_mgr.simulated_sonar:
                sonar_status = hal.sim_mgr.simulated_sonar.get_status()
                print(f"Simulated sonar ACTIVE")
                print(f"  Max range: {sonar_status['max_range']}m")
                print(f"  Obstacles loaded: {sonar_status['obstacles_loaded']}")
                
                if hal.sim_mgr.simulated_sonar.obstacles:
                    print(f"  Obstacle details:")
                    for i, obs in enumerate(hal.sim_mgr.simulated_sonar.obstacles):
                        name = obs.get('name', f'Obstacle {i+1}')
                        pos = obs.get('position', {})
                        x = pos.get('local_x', 0)
                        y = pos.get('local_y', 0)
                        print(f"    [{i+1}] {name} at ({x:.1f}, {y:.1f})")
            else:
                print("WARNING: Simulated sonar NOT initialized!")
        else:
            print("No simulation manager found")
        print("=" * 70 + "\n")

        if not args.auto_start and not args.no_gcs:
            connection_type = "WiFi" if wifi_deployment else "local"
            print(f"Waiting for mission_start command from GCS...")
            print(f"HAL will establish {connection_type} connection to {args.mavlink_host}:{args.mavlink_port}")
            
            wait_start_time = time.time()
            last_status_time = wait_start_time
            connection_status_shown = False
            
            while not hal.mission_start_requested and not exit_requested:
                try:
                    hal.update(0.1)
                    
                    current_time = time.time()
                    if current_time - last_status_time >= 5.0:
                        if hasattr(hal, 'get_connection_status'):
                            conn_status = hal.get_connection_status()
                            connected = conn_status.get('connected_to_gcs', False)
                            
                            if connected and not connection_status_shown:
                                print(f"GCS connection established - ready for mission_start command")
                                connection_status_shown = True
                            elif not connected:
                                elapsed = current_time - wait_start_time
                                print(f"  Waiting for GCS connection and mission_start command... ({elapsed:.0f}s elapsed)")
                                connection_status_shown = False
                        
                        last_status_time = current_time
                    
                    time.sleep(0.5)
                    
                except KeyboardInterrupt:
                    print(f"\nMission start wait cancelled by user")
                    return 1
                except Exception as e:
                    if args.verbose:
                        print(f"Error during mission start wait: {e}")
                    time.sleep(1)

            if exit_requested:
                print("Shutdown requested before mission start")
                return 0

            hal.clear_mission_start_flag()
            print("Mission start command received - beginning execution")
        else:
            if args.no_gcs:
                print("No-GCS mode - beginning mission execution immediately")
            else:
                print("Auto-start enabled - beginning mission execution immediately")

        print("Setting up telemetry logging...")
        logs_dir = Path("./logs")
        logs_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_file = logs_dir / f"telemetry_{timestamp}.csv"

        try:
            status_snapshot = vehicle_state.get_status()
            flat_status = flatten_dict(status_snapshot)
            csv_headers = list(flat_status.keys())
            
            telemetry_fh = open(csv_file, 'w', newline='')
            csv_writer = csv.DictWriter(telemetry_fh, fieldnames=csv_headers)
            csv_writer.writeheader()
        except Exception as e:
            print(f"CSV setup failed: {e}")
            csv_writer = None

        def handle_sigint(sig, frame):
            nonlocal exit_requested
            if not exit_requested:
                exit_requested = True
                print("\nShutdown requested...")
            else:
                print("\nForced exit!")
                sys.exit(1)

        if hasattr(signal, 'SIGINT'):
            signal.signal(signal.SIGINT, handle_sigint)

        print("=" * 70)
        print("SIMPLR-AUV VEHICLE CONTROLLER - MISSION EXECUTION")
        print("=" * 70)
        print(f"Mode: {args.mode.upper()}")
        print(f"Mission: {mission_path.name}")
        if not args.no_gcs:
            print(f"GCS Connection: {args.transport.upper()} to {args.mavlink_host}:{args.mavlink_port}")
        else:
            print("GCS Connection: DISABLED (Standalone)")
        print(f"Console Dashboard: {'ENABLED' if args.console_output else 'DISABLED'}")
        print(f"Telemetry Log: {csv_file.name}")
        if wifi_deployment and not args.no_gcs:
            print(f"WiFi Mode: ENABLED (Local IP: {local_ip})")
        print("=" * 70)
        print("Press Ctrl+C to shutdown gracefully")
        print("=" * 70)

        add_status_message("Starting main control loop")
        control_start_time = time.time()
        dt = 0.1
        dashboard_interval = 5.0
        last_dashboard_time = control_start_time
        loop_count = 0

        previous_task = None
        previous_system_state = None

        while not exit_requested:
            loop_start = time.time()
            vehicle_state.mission_time = loop_start - control_start_time
            
            try:
                if hal.mission_stop_requested:
                    hal.clear_mission_stop_flag()
                    print("Mission stop command received")
                    break
                
                if hal.mission_pause_requested:
                    hal.clear_mission_pause_flag()
                    print("Mission pause command received")
                
                if hal.emergency_stop_requested:
                    hal.clear_emergency_stop_flag()
                    print("Emergency stop command received")
                    break

                if not args.verbose:
                    with SilentOutput():
                        executive.update(dt)
                        guidance.update(dt)
                        hal.update(dt)  # HAL generates sonar data FIRST
                        
                        # Now obstacle avoidance can use fresh sonar data
                        if obstacle_avoidance:
                            sonar_pkt = vehicle_state.sensor_data.get('sonar', None)
                            if sonar_pkt is not None:
                                obstacle_avoidance.update_sonar(sonar_pkt)
                            obstacle_avoidance.update(dt)
                            
                        navigation.update(dt)
                        vehicle_control.update(dt)
                        energy.update(dt)
                else:
                    executive.update(dt)
                    guidance.update(dt)
                    hal.update(dt)  # HAL generates sonar data FIRST
                    
                    # Now obstacle avoidance can use fresh sonar data
                    if obstacle_avoidance:
                        sonar_pkt = vehicle_state.sensor_data.get('sonar', None)
                        if sonar_pkt is not None:
                            obstacle_avoidance.update_sonar(sonar_pkt)
                        obstacle_avoidance.update(dt)
                        
                    navigation.update(dt)
                    vehicle_control.update(dt)
                    energy.update(dt)

                current_task = safe_get(vehicle_state, 'current_task', 'UNKNOWN')
                try:
                    current_system_state = vehicle_state.current_state.value
                except:
                    current_system_state = 'UNKNOWN'
                
                if current_task != previous_task:
                    add_status_message(f"Task: {current_task}")
                    previous_task = current_task
                
                if current_system_state != previous_system_state:
                    add_status_message(f"State: {current_system_state}")
                    previous_system_state = current_system_state

                if csv_writer:
                    try:
                        status_snapshot = vehicle_state.get_status()
                        flat_status = flatten_dict(status_snapshot)
                        row_to_write = {k: flat_status.get(k, '') for k in csv_headers}
                        csv_writer.writerow(row_to_write)
                        
                        if loop_count % 50 == 0:
                            telemetry_fh.flush()
                    except Exception as e:
                        if loop_count % 100 == 0 and args.verbose:
                            add_status_message(f"CSV logging error: {e}")

                if args.console_output and (loop_start - last_dashboard_time >= dashboard_interval):
                    try:
                        mission_time = safe_get(vehicle_state, 'mission_time', 0.0)
                        nav_state = safe_get(vehicle_state, 'nav_state', {})
                        current_depth = safe_get(nav_state, 'depth', 0.0)
                        current_speed = safe_get(nav_state, 'speed', 0.0)
                        
                        gcs_connected = False
                        if not args.no_gcs and hasattr(hal, 'get_connection_status'):
                            conn_status = hal.get_connection_status()
                            gcs_connected = conn_status.get('connected_to_gcs', False)
                        
                        gcs_status = "GCS:ON" if gcs_connected else "GCS:OFF"
                        if wifi_deployment and not args.no_gcs:
                            gcs_status = f"WiFi-{gcs_status}"
                        
                        print(f"Loop {loop_count}: {current_task} | {current_system_state} | {gcs_status} | "
                              f"Time: {mission_time:.1f}s | Depth: {current_depth:.1f}m | Speed: {current_speed:.2f}m/s")
                        
                        if STATUS_MESSAGES:
                            latest_msg = list(STATUS_MESSAGES)[-1]
                            print(f"  Last: {latest_msg}")
                            
                    except Exception:
                        gcs_connected = False
                        if not args.no_gcs and hasattr(hal, 'get_connection_status'):
                            conn_status = hal.get_connection_status()
                            gcs_connected = conn_status.get('connected_to_gcs', False)
                        
                        gcs_status = "GCS:ON" if gcs_connected else "GCS:OFF"
                        if wifi_deployment and not args.no_gcs:
                            gcs_status = f"WiFi-{gcs_status}"
                        print(f"Loop {loop_count}: Running | {gcs_status} | Time: {vehicle_state.mission_time:.1f}s")
                        
                    last_dashboard_time = loop_start

                try:
                    mission_status = executive.get_mission_status()
                    if mission_status.get('mission_complete', False):
                        add_status_message(f"Mission completed in {vehicle_state.mission_time:.1f}s")
                        break
                except:
                    pass
                    
                if vehicle_state.mission_time > 3600:
                    add_status_message(f"Mission timeout after {vehicle_state.mission_time:.1f}s")
                    break

                try:
                    if vehicle_state.current_state.value == 'FAULT':
                        add_status_message("System in FAULT state")
                        break
                except:
                    pass

            except Exception as e:
                add_status_message(f"ERROR in main loop: {e}")
                if args.verbose:
                    import traceback
                    traceback.print_exc()
                if loop_count < 5:
                    print(f"CRITICAL: Early loop error on iteration {loop_count}: {e}")
                    import traceback
                    traceback.print_exc()
                    break

            loop_count += 1
            loop_time = time.time() - loop_start
            if loop_time < dt:
                time.sleep(dt - loop_time)

    except KeyboardInterrupt:
        add_status_message("Keyboard interrupt received")
    except Exception as e:
        add_status_message(f"Fatal error: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1
    finally:
        print("\nShutting down vehicle systems...")
        
        if hal:
            try:
                hal.emergency_stop()
                if hasattr(hal, 'shutdown'):
                    hal.shutdown()
                print("HAL stopped")
            except Exception as e:
                if args.verbose:
                    print(f"HAL shutdown error: {e}")
                
        if telemetry_fh:
            try:
                telemetry_fh.close()
                print("Telemetry CSV closed")
            except Exception as e:
                if args.verbose:
                    print(f"CSV close error: {e}")
        
        if STATUS_MESSAGES:
            print("\nFinal Status Messages:")
            for msg in STATUS_MESSAGES:
                print(f"  {msg}")
        
        print("Vehicle shutdown complete")
        time.sleep(1)

    return 0

if __name__ == "__main__":
    sys.exit(main())