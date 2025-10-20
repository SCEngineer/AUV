#!/usr/bin/env python3
"""
auv_main.py - Complete SIMPLR-AUV Vehicle Controller with WiFi support
Integrates existing functionality with WiFi telemetry for separate machine deployment
UPDATED: Now waits for mission_start command from GCS before beginning execution
UPDATED: Added Energy module initialization and update calls
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

def main():
    parser = argparse.ArgumentParser(description='SIMPLR-AUV Vehicle Controller (WiFi-Enabled)')
    parser.add_argument('--mode', choices=['simulation', 'hardware'], default='simulation')
    parser.add_argument('--mission', required=True, help='Mission file path')
    parser.add_argument('--verbose', action='store_true', help='Show all initialization output')
    parser.add_argument('--mavlink-port', type=int, default=14550, help='GCS server port to connect to')
    parser.add_argument('--mavlink-host', default=None, help='GCS server host IP (auto-discover if not specified)')
    parser.add_argument('--transport', choices=['tcp', 'rf'], default='tcp', help='Telemetry transport type')
    parser.add_argument('--console-output', action='store_true', help='Show console dashboard output')
    parser.add_argument('--connection-timeout', type=int, default=60, help='Seconds to wait for initial GCS connection')
    parser.add_argument('--no-firewall', action='store_true', help='Skip Windows firewall configuration')
    parser.add_argument('--no-discovery', action='store_true', help='Skip network discovery of GCS')
    parser.add_argument('--auto-start', action='store_true', help='Start mission automatically (skip GCS start command wait)')
    args = parser.parse_args()

    # Network discovery if host not specified
    if not args.mavlink_host and not args.no_discovery:
        print("Discovering GCS servers on network...")
        
        # Try broadcast discovery first
        if discover_gcs_for_vehicle:
            discovered_host = discover_gcs_for_vehicle()
            if discovered_host:
                args.mavlink_host = discovered_host
                print(f"Found GCS server via broadcast: {args.mavlink_host}")
            else:
                print("No GCS found via broadcast discovery. Trying fallback scan...")
                discovered_host = fallback_scan_for_gcs(args.mavlink_port)
                if discovered_host:
                    args.mavlink_host = discovered_host
                    print(f"Found GCS server via scan: {args.mavlink_host}")
        else:
            # Use fallback scan if broadcast discovery not available
            discovered_host = fallback_scan_for_gcs(args.mavlink_port)
            if discovered_host:
                args.mavlink_host = discovered_host
                print(f"Found GCS server: {args.mavlink_host}")
        
        if not args.mavlink_host:
            print("No GCS found via discovery. Please specify --mavlink-host manually")
            print("Example: --mavlink-host 192.168.86.25")
            return 1
    elif not args.mavlink_host:
        # Default to localhost for backwards compatibility
        args.mavlink_host = 'localhost'
        print("No GCS host specified, defaulting to localhost")

    # Display network configuration
    local_ip = get_local_ip()
    print("=" * 70)
    print("SIMPLR-AUV VEHICLE CONTROLLER")
    print("=" * 70)
    print(f"Local Machine IP: {local_ip}")
    print(f"Target GCS: {args.mavlink_host}:{args.mavlink_port}")
    
    # Check if this is a WiFi deployment
    wifi_deployment = args.mavlink_host != 'localhost' and args.mavlink_host != '127.0.0.1'
    if wifi_deployment:
        print("WiFi Deployment Mode: ENABLED")
    else:
        print("Local Deployment Mode")
    print("=" * 70)

    # Check Windows firewall configuration for network communication
    if sys.platform == "win32" and not args.no_firewall and wifi_deployment:
        if not check_admin_privileges():
            print("Warning: Not running as Administrator. Some network operations may be restricted.")
        if not configure_windows_firewall_for_network(args.mavlink_port):
            print("Warning: Firewall configuration may have failed. Connection issues may occur.")

    # Test initial connectivity if WiFi deployment
    if wifi_deployment:
        print(f"Testing connectivity to GCS at {args.mavlink_host}:{args.mavlink_port}...")
        if not test_network_connectivity(args.mavlink_host, args.mavlink_port, timeout=5):
            print("Warning: Cannot connect to GCS. Vehicle will start anyway and HAL will handle reconnection.")

    # Initialize variables for cleanup
    hal = None
    energy = None
    telemetry_fh = None
    csv_writer = None
    exit_requested = False
    start_time = time.time()

    try:
        # Import modules with output suppression unless verbose
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

        # Initialize vehicle state
        if not args.verbose:
            with SilentOutput():
                vehicle_state = VehicleState()
        else:
            add_status_message("Creating vehicle state...")
            vehicle_state = VehicleState()

        # Configure HAL with GCS connection settings
        hal_config = {
            'mode': args.mode, 
            'sensor_update_rate': 10.0,
            'mavlink_enabled': True,
            'transport_type': args.transport,
            'tcp_host': args.mavlink_host,
            'tcp_port': args.mavlink_port,
            'verbose': args.verbose,
            'wifi_enabled': wifi_deployment
        }

        # Initialize subsystems
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
                print("Retrying with verbose output...")
                hal = EnhancedHAL(hal_config, vehicle_state)
                navigation = Navigation(vehicle_state, hal)
                vehicle_control = VehicleControl(vehicle_state)
                guidance = TaskManager(vehicle_state)
                executive = Executive(vehicle_state)
                energy = Energy(vehicle_state, hal)
        else:
            add_status_message("Initializing subsystems...")
            hal = EnhancedHAL(hal_config, vehicle_state)
            navigation = Navigation(vehicle_state, hal)
            vehicle_control = VehicleControl(vehicle_state)
            guidance = TaskManager(vehicle_state)
            executive = Executive(vehicle_state)
            energy = Energy(vehicle_state, hal)

        # Load and validate mission file
        mission_path = Path(args.mission)
        if not mission_path.exists():
            mission_path = PROJECT_ROOT / "missions" / args.mission
        if not mission_path.exists() and not mission_path.suffix:
            mission_path = PROJECT_ROOT / "missions" / f"{args.mission}.json"

        if not mission_path.exists():
            add_status_message(f"ERROR: Mission file not found: {args.mission}")
            print(f"ERROR: Mission file not found: {args.mission}")
            return 1

        # Set mission file environment variable
        os.environ['SIMPLR_MISSION_FILE'] = str(mission_path)
        add_status_message(f"Loading mission: {mission_path.name}")
        mission_loaded = executive.load_mission(str(mission_path))
        if not mission_loaded:
            add_status_message("ERROR: Failed to load mission")
            faults = vehicle_state.get_faults()
            for fault in faults:
                print(f"FAULT: {fault}")
            return 1

        add_status_message("Mission loaded successfully")

        # NEW: Wait for mission_start command from GCS (unless auto-start enabled)
        if not args.auto_start:
            connection_type = "WiFi" if wifi_deployment else "local"
            print(f"Waiting for mission_start command from GCS...")
            print(f"HAL will establish {connection_type} connection to {args.mavlink_host}:{args.mavlink_port}")
            print("Send mission_start via Mission Control tab in GCS to begin execution")
            
            wait_start_time = time.time()
            last_status_time = wait_start_time
            connection_status_shown = False
            
            while not hal.mission_start_requested and not exit_requested:
                try:
                    # Keep HAL responsive to handle incoming messages
                    hal.update(0.1)
                    
                    # Check connection status periodically
                    current_time = time.time()
                    if current_time - last_status_time >= 5.0:  # Every 5 seconds
                        if hasattr(hal, 'get_connection_status'):
                            conn_status = hal.get_connection_status()
                            connected = conn_status.get('connected_to_gcs', False)
                            
                            if connected and not connection_status_shown:
                                add_status_message(f"GCS connection established - ready for mission_start command")
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

            # Clear the flag and acknowledge mission start
            hal.clear_mission_start_flag()
            add_status_message("Mission start command received - beginning execution")
            print("Mission start command received - beginning execution")
        else:
            add_status_message("Auto-start enabled - beginning mission execution immediately")
            print("Auto-start enabled - beginning mission execution immediately")

        # Wait for initial GCS connection establishment (for telemetry)
        connection_type = "WiFi" if wifi_deployment else "local"
        print(f"Establishing {connection_type} telemetry connection...")
        connection_start = time.time()
        initial_connection_found = False
        
        # Shorter timeout since we already handled mission start
        connection_timeout = 30 if not args.auto_start else args.connection_timeout
        
        while time.time() - connection_start < connection_timeout:
            try:
                # Let HAL attempt connection
                hal.update(0.1)
                
                # Check HAL connection status
                if hasattr(hal, 'get_connection_status'):
                    conn_status = hal.get_connection_status()
                    if conn_status.get('connected_to_gcs', False):
                        initial_connection_found = True
                        add_status_message(f"Telemetry connection established after {time.time() - connection_start:.1f}s")
                        break
                
                time.sleep(0.5)
                
                # Show progress every 5 seconds
                if int(time.time() - connection_start) % 5 == 0:
                    elapsed = time.time() - connection_start
                    remaining = connection_timeout - elapsed
                    print(f"  Establishing telemetry connection... ({remaining:.0f}s remaining)")
                    
            except KeyboardInterrupt:
                print(f"\nTelemetry connection wait cancelled by user")
                break
            except Exception as e:
                if args.verbose:
                    print(f"Telemetry connection error: {e}")
                time.sleep(1)

        if not initial_connection_found:
            print(f"No telemetry connection established within {connection_timeout}s")
            print("Vehicle will continue - HAL will connect when appropriate")
            add_status_message("No telemetry connection - HAL will connect when vehicle state allows")
        else:
            print(f"Telemetry connection established successfully")

        # Check telemetry status
        if hasattr(hal, 'get_telemetry_status'):
            telemetry_status = hal.get_telemetry_status()
            if telemetry_status.get('enabled', False):
                conn_info = telemetry_status.get('transport', {})
                if conn_info.get('connected', False):
                    add_status_message(f"Telemetry active: {args.transport.upper()} to {args.mavlink_host}:{args.mavlink_port}")
                else:
                    add_status_message(f"Telemetry enabled - HAL will connect when appropriate")
            else:
                add_status_message("WARNING: Telemetry not available")

        # Setup telemetry CSV logging
        print("Setting up telemetry logging...")
        logs_dir = Path("./logs")
        logs_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_file = logs_dir / f"telemetry_{timestamp}.csv"

        # Initialize CSV with headers
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
            add_status_message(f"Warning: CSV setup failed: {e}")
            csv_writer = None

        # Setup graceful shutdown handler
        def handle_sigint(sig, frame):
            nonlocal exit_requested
            if not exit_requested:
                exit_requested = True
                add_status_message("Shutdown requested...")
                print("\nShutdown requested - finishing current operations...")
            else:
                print("\nForced exit!")
                sys.exit(1)

        if hasattr(signal, 'SIGINT'):
            signal.signal(signal.SIGINT, handle_sigint)

        # Display startup information
        print("=" * 70)
        print("SIMPLR-AUV VEHICLE CONTROLLER")
        print("=" * 70)
        print(f"Mode: {args.mode.upper()}")
        print(f"Mission: {mission_path.name}")
        print(f"GCS Connection: {args.transport.upper()} to {args.mavlink_host}:{args.mavlink_port}")
        print(f"Mission Start: {'AUTO' if args.auto_start else 'GCS COMMAND'}")
        print(f"Telemetry Status: {'CONNECTED' if initial_connection_found else 'NOT CONNECTED'}")
        print(f"Console Dashboard: {'ENABLED' if args.console_output else 'DISABLED'}")
        print(f"Telemetry Log: {csv_file.name}")
        if wifi_deployment:
            print(f"WiFi Mode: ENABLED (Local IP: {local_ip})")
        if sys.platform == "win32":
            print(f"Windows Firewall: {'CONFIGURED' if not args.no_firewall else 'SKIPPED'}")
        print("=" * 70)
        print("Mission execution starting")
        print("HAL manages all GCS connections based on vehicle state and depth")
        print("Press Ctrl+C to shutdown gracefully")
        print("=" * 70)

        # Main control loop
        add_status_message("Starting main control loop")
        control_start_time = time.time()
        dt = 0.1  # 10 Hz main loop
        dashboard_interval = 5.0  # 5 second console updates (if enabled)
        last_dashboard_time = control_start_time
        loop_count = 0
        connection_check_interval = 10.0  # Check GCS connection every 10 seconds
        last_connection_check = control_start_time

        # Store previous values to detect changes
        previous_task = None
        previous_system_state = None
        previous_gcs_connected = initial_connection_found

        while not exit_requested:
            loop_start = time.time()
            vehicle_state.mission_time = loop_start - control_start_time
            
            try:
                # Check for mission control commands from GCS - NEW
                if hal.mission_stop_requested:
                    hal.clear_mission_stop_flag()
                    add_status_message("Mission stop command received")
                    print("Mission stop command received - stopping execution")
                    break
                
                if hal.mission_pause_requested:
                    hal.clear_mission_pause_flag()
                    add_status_message("Mission pause command received")
                    print("Mission pause command received - pausing execution")
                    # You could implement pause logic here if needed
                    # For now, we'll just log it and continue
                
                if hal.emergency_stop_requested:
                    hal.clear_emergency_stop_flag()
                    add_status_message("Emergency stop command received")
                    print("Emergency stop command received - halting immediately")
                    break

                # Update all subsystems (HAL handles its own WiFi connection management)
                if not args.verbose:
                    with SilentOutput():
                        executive.update(dt)
                        guidance.update(dt)
                        hal.update(dt)  # HAL manages GCS connection internally
                        navigation.update(dt)
                        vehicle_control.update(dt)
                        energy.update(dt)  # Energy module updates vehicle_state.energy_state
                else:
                    executive.update(dt)
                    guidance.update(dt)
                    hal.update(dt)
                    navigation.update(dt)
                    vehicle_control.update(dt)
                    energy.update(dt)  # Energy module updates vehicle_state.energy_state

                # Periodic GCS connection status check (for logging only)
                current_gcs_connected = False
                if loop_start - last_connection_check >= connection_check_interval:
                    if hasattr(hal, 'get_connection_status'):
                        conn_status = hal.get_connection_status()
                        current_gcs_connected = conn_status.get('connected_to_gcs', False)
                        
                        if current_gcs_connected != previous_gcs_connected:
                            if current_gcs_connected:
                                add_status_message("GCS connection established")
                            else:
                                add_status_message("GCS connection lost")
                            previous_gcs_connected = current_gcs_connected
                    
                    last_connection_check = loop_start

                # Detect and log significant events
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

                # Log telemetry data
                if csv_writer:
                    try:
                        status_snapshot = vehicle_state.get_status()
                        flat_status = flatten_dict(status_snapshot)
                        row_to_write = {k: flat_status.get(k, '') for k in csv_headers}
                        csv_writer.writerow(row_to_write)
                        
                        # Flush CSV periodically
                        if loop_count % 50 == 0:  # Every 5 seconds
                            telemetry_fh.flush()
                    except Exception as e:
                        if loop_count % 100 == 0 and args.verbose:
                            add_status_message(f"CSV logging error: {e}")

                # Console dashboard (optional, minimal)
                if args.console_output and (loop_start - last_dashboard_time >= dashboard_interval):
                    try:
                        mission_time = safe_get(vehicle_state, 'mission_time', 0.0)
                        nav_state = safe_get(vehicle_state, 'nav_state', {})
                        current_depth = safe_get(nav_state, 'depth', 0.0)
                        current_speed = safe_get(nav_state, 'speed', 0.0)
                        
                        # Get actual GCS connection status from HAL
                        gcs_connected = False
                        if hasattr(hal, 'get_connection_status'):
                            conn_status = hal.get_connection_status()
                            gcs_connected = conn_status.get('connected_to_gcs', False)
                        
                        # Connection status indicator
                        gcs_status = "GCS:ON" if gcs_connected else "GCS:OFF"
                        if wifi_deployment:
                            gcs_status = f"WiFi-{gcs_status}"
                        
                        # Simple one-line status
                        print(f"Loop {loop_count}: {current_task} | {current_system_state} | {gcs_status} | "
                              f"Time: {mission_time:.1f}s | Depth: {current_depth:.1f}m | Speed: {current_speed:.2f}m/s")
                        
                        # Show recent messages
                        if STATUS_MESSAGES:
                            latest_msg = list(STATUS_MESSAGES)[-1]
                            print(f"  Last: {latest_msg}")
                            
                    except Exception:
                        # Fallback status display
                        gcs_connected = False
                        if hasattr(hal, 'get_connection_status'):
                            conn_status = hal.get_connection_status()
                            gcs_connected = conn_status.get('connected_to_gcs', False)
                        
                        gcs_status = "GCS:ON" if gcs_connected else "GCS:OFF"
                        if wifi_deployment:
                            gcs_status = f"WiFi-{gcs_status}"
                        print(f"Loop {loop_count}: Running | {gcs_status} | Time: {vehicle_state.mission_time:.1f}s")
                        
                    last_dashboard_time = loop_start

                # Check mission completion
                try:
                    mission_status = executive.get_mission_status()
                    if mission_status.get('mission_complete', False):
                        add_status_message(f"Mission completed in {vehicle_state.mission_time:.1f}s")
                        break
                except:
                    pass
                    
                # Check for mission timeout
                if vehicle_state.mission_time > 3600:  # 1 hour timeout
                    add_status_message(f"Mission timeout after {vehicle_state.mission_time:.1f}s")
                    break

                # Check for critical faults
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
                # Continue running unless it's a critical early error
                if loop_count < 5:
                    print(f"CRITICAL: Early loop error on iteration {loop_count}: {e}")
                    import traceback
                    traceback.print_exc()
                    break

            # Maintain loop timing
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
        # Cleanup sequence
        print("\nShutting down vehicle systems...")
        
        if hal:
            try:
                hal.emergency_stop()
                if hasattr(hal, 'shutdown'):
                    hal.shutdown()  # This will close GCS connection
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
        
        # Show final status messages
        if STATUS_MESSAGES:
            print("\nFinal Status Messages:")
            for msg in STATUS_MESSAGES:
                print(f"  {msg}")
        
        print("Vehicle shutdown complete")
        time.sleep(1)  # Brief pause

    return 0

if __name__ == "__main__":
    sys.exit(main())