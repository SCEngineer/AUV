#!/usr/bin/env python3
"""
analyze_telemetry.py - COMPLETE TELEMETRY ANALYSIS
Now supports UPDATED obstacle_avoidance_log_*.csv format
- Columns: timestamp, state, sonar_distance, sonar_confidence, 
          veh_x, veh_y, veh_heading, veh_speed,
          tgt_heading, tgt_speed, carrot_x, carrot_y, 
          dist_to_carrot, position_uncertainty, adaptive_radius, avoidance_id
- Uses timestamp → mission_time conversion
- All analysis tools restored and IMPLEMENTED
- Optional depth maneuver fields (gracefully handled if missing)
- UPDATED: Only plots first encounter per obstacle (not every update)
- UPDATED: Complete mission replay animation implementation
- UPDATED: Plots obstacles from mission file
"""

import os
import glob
import csv
import json
import math
import time
import bisect
import matplotlib

# Try to use an interactive backend for displaying plots
# Try different backends in order of preference
backends_to_try = ['TkAgg', 'Qt5Agg', 'Qt4Agg', 'WXAgg']
backend_set = False

for backend in backends_to_try:
    try:
        matplotlib.use(backend)
        import matplotlib.pyplot as plt
        backend_set = True
        print(f"Using matplotlib backend: {backend}")
        break
    except (ImportError, ValueError):
        continue

if not backend_set:
    import matplotlib.pyplot as plt
    print(f"Warning: Using default matplotlib backend: {matplotlib.get_backend()}")
    print("If plots don't display, install tkinter: 'sudo apt-get install python3-tk'")

import matplotlib.animation as animation
from datetime import datetime

# ----------------------------
# Utility Functions
# ----------------------------

def safe_float(value, default=float('nan')):
    if value is None or value == "":
        return default
    try:
        return float(value)
    except (ValueError, TypeError):
        return default

# ----------------------------
# Automatic File Detection
# ----------------------------

def find_latest_csv_file():
    folder = "."
    csv_files = sorted(glob.glob(os.path.join(folder, "telemetry_*.csv")), key=os.path.getmtime, reverse=True)
    if csv_files:
        print(f"Found latest telemetry CSV: {os.path.basename(csv_files[0])}")
        return csv_files[0]
    else:
        print("No telemetry CSV files found")
        return None

def find_latest_mission_file():
    folder = "."
    
    # First, check for box_route_with_obstacles.json specifically
    preferred_file = os.path.join(folder, "box_route_with_obstacles.json")
    if os.path.exists(preferred_file):
        print(f"Found preferred mission: {os.path.basename(preferred_file)}")
        return preferred_file
    
    # Otherwise, find the latest JSON file by modification time
    mission_files = sorted(glob.glob(os.path.join(folder, "*.json")), key=os.path.getmtime, reverse=True)
    if mission_files:
        print(f"Found latest mission: {os.path.basename(mission_files[0])}")
        return mission_files[0]
    else:
        print("No mission files found")
        return None

def find_latest_oa_file():
    folder = "."
    oa_files = sorted(glob.glob(os.path.join(folder, "obstacle_avoidance_log_*.csv")), key=os.path.getmtime, reverse=True)
    if oa_files:
        print(f"Found latest OA log: {os.path.basename(oa_files[0])}")
        return oa_files[0]
    print("No obstacle avoidance log found (obstacle_avoidance_log_*.csv)")
    return None

def find_all_csv_files():
    folder = "."
    return sorted(glob.glob(os.path.join(folder, "telemetry_*.csv")))

def find_all_mission_files():
    folder = "."
    return sorted(glob.glob(os.path.join(folder, "*.json")))

def pick_file_from_list(files, prompt):
    if not files:
        print("No files available.")
        return None
    print(f"\n{prompt}")
    for i, f in enumerate(files):
        print(f"  {i+1}. {os.path.basename(f)}")
    while True:
        choice = input(f"Enter number (1-{len(files)}): ").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(files):
            return files[int(choice)-1]
        print("Invalid choice.")

# ----------------------------
# Data Loaders
# ----------------------------

def load_csv(csv_file):
    numeric_columns = [
        "mission_time", "nav_state_lat", "nav_state_lon", "nav_state_heading", "nav_state_depth",
        "nav_state_local_x", "nav_state_local_y", "nav_state_speed",
        "true_state_true_lat", "true_state_true_lon", "true_state_true_heading", "true_state_true_depth",
        "energy_state_voltage", "energy_state_capacity_remaining",
        "control_errors_speed_error", "control_errors_heading_error", "control_errors_depth_error"
    ]
    data = {}
    with open(csv_file, newline='') as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            print("Error: CSV file is empty or invalid")
            return {}
        for key in reader.fieldnames:
            data[key] = []
        for row in reader:
            for key, value in row.items():
                if key in numeric_columns:
                    try:
                        data[key].append(float(value) if value else float('nan'))
                    except (ValueError, TypeError):
                        data[key].append(float('nan'))
                else:
                    data[key].append(value if value else '')
    print(f"Loaded telemetry CSV with {len(data.get('mission_time', []))} rows")
    return data

def load_oa_csv(oa_csv):
    """Load obstacle_avoidance_log_*.csv format"""
    if not oa_csv:
        return {}
    data = {}
    try:
        with open(oa_csv, newline='') as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                print("OA CSV missing headers.")
                return {}
            
            print(f"OA CSV columns: {reader.fieldnames}")
            
            for k in reader.fieldnames:
                data[k] = []
            
            # Initialize mission_time list
            data['mission_time'] = []
            
            first_timestamp = None
            for row in reader:
                # Process timestamp first to compute mission_time for this row
                timestamp_value = row.get('timestamp', '')
                mission_time_for_row = float('nan')
                
                if timestamp_value and timestamp_value != "":
                    try:
                        ts = float(timestamp_value)
                        if first_timestamp is None:
                            first_timestamp = ts
                            mission_time_for_row = 0.0
                        else:
                            mission_time_for_row = ts - first_timestamp
                    except (ValueError, TypeError):
                        mission_time_for_row = float('nan')
                
                # Add mission_time for this row
                data['mission_time'].append(mission_time_for_row)
                
                # Now process all columns
                for k, v in row.items():
                    if v == "" or v is None:
                        # String columns
                        if k in ['state']:
                            data[k].append("")
                        else:
                            data[k].append(float('nan'))
                    else:
                        if k == 'timestamp':
                            try:
                                data[k].append(float(v))
                            except:
                                data[k].append(float('nan'))
                        elif k in ['sonar_distance', 'sonar_confidence', 
                                 'veh_x', 'veh_y', 'veh_heading', 'veh_speed',
                                 'tgt_heading', 'tgt_speed',
                                 'carrot_x', 'carrot_y', 'dist_to_carrot',
                                 'position_uncertainty', 'adaptive_radius',
                                 'avoidance_id']:
                            data[k].append(safe_float(v))
                        elif k == 'state':
                            data[k].append(v)
                        else:
                            data[k].append(v)
            
            data['obstacle_detected'] = [
                not math.isnan(d) and d < 100.0 
                for d in data.get('sonar_distance', [])
            ]
        
    except Exception as e:
        print(f"Error loading OA CSV: {e}")
        import traceback
        traceback.print_exc()
        return {}
    
    return data

# ----------------------------
# ENU Conversion
# ----------------------------

def latlon_to_enu(lat, lon, lat0, lon0):
    R = 6378137.0
    x = [R * math.radians(lon_i - lon0) * math.cos(math.radians(lat0)) for lon_i in lon]
    y = [R * math.radians(lat_i - lat0) for lat_i in lat]
    return x, y

# ----------------------------
# Mission Parsing
# ----------------------------

def extract_mission_waypoints(mission):
    waypoints = []
    if 'vehicle_config' in mission and 'initial_position' in mission['vehicle_config']:
        initial = mission['vehicle_config']['initial_position']
        if 'lat' in initial and 'lon' in initial:
            try:
                lat = float(initial['lat'])
                lon = float(initial['lon'])
                waypoints.append({'lat': lat, 'lon': lon, 'description': 'START', 'type': 'start'})
            except (ValueError, TypeError):
                pass
    for i, task in enumerate(mission.get('tasks', [])):
        task_type = task.get('type', '')
        if task_type == 'SWIM_TO_WAYPOINT' and 'lat' in task and 'lon' in task:
            try:
                lat = float(task['lat'])
                lon = float(task['lon'])
                waypoints.append({
                    'lat': lat, 'lon': lon,
                    'description': f"WP{len([w for w in waypoints if w['type'] != 'start'])+1}",
                    'type': 'waypoint'
                })
            except (ValueError, TypeError):
                pass
        elif task_type == 'FOLLOW_TRACK' and 'waypoints' in task:
            track_waypoints = task['waypoints']
            for j, wp in enumerate(track_waypoints):
                try:
                    lat = float(wp['lat'])
                    lon = float(wp['lon'])
                    waypoints.append({
                        'lat': lat, 'lon': lon,
                        'description': f"TRACK_WP{j+1}",
                        'type': 'track_waypoint'
                    })
                except (ValueError, TypeError, KeyError):
                    pass
    return waypoints, []

# ----------------------------
# OA Merging
# ----------------------------

def merge_oa_with_telemetry_optimized(telemetry, oa_data):
    merged = []
    if not oa_data or 'mission_time' not in oa_data:
        return merged

    t_mis = telemetry.get("mission_time", [])
    if not t_mis:
        print("Telemetry has no mission_time field.")
        return merged

    oa_times = oa_data.get("mission_time", [])
    if not oa_times:
        return merged

    nav_lat = telemetry.get("nav_state_lat", [])
    nav_lon = telemetry.get("nav_state_lon", [])
    nav_depth = telemetry.get("nav_state_depth", [])
    cur_task = telemetry.get("current_task", [])
    
    oa_avoidance_id = oa_data.get("avoidance_id", [])
    oa_state = oa_data.get("state", [])
    oa_sonar_range = oa_data.get("sonar_distance", [])
    oa_confidence = oa_data.get("sonar_confidence", [])
    
    # Optional columns (may not exist in all CSV formats)
    oa_turn_dir = oa_data.get("turn_direction", [])
    oa_depth_dir = oa_data.get("depth_direction", [])
    oa_using_depth = oa_data.get("using_depth", [])
    
    # Validate that all OA arrays have the same length
    # Only validate required fields
    oa_arrays = {
        'mission_time': oa_times,
        'avoidance_id': oa_avoidance_id,
        'state': oa_state,
        'sonar_distance': oa_sonar_range,
        'sonar_confidence': oa_confidence,
    }
    
    # Add optional fields if they exist
    if oa_turn_dir:
        oa_arrays['turn_direction'] = oa_turn_dir
    if oa_depth_dir:
        oa_arrays['depth_direction'] = oa_depth_dir
    if oa_using_depth:
        oa_arrays['using_depth'] = oa_using_depth
    
    lengths = {name: len(arr) for name, arr in oa_arrays.items()}
    if len(set(lengths.values())) > 1:
        print("WARNING: OA data arrays have mismatched lengths:")
        for name, length in lengths.items():
            print(f"  {name}: {length} rows")
        # Use the minimum length to avoid index errors
        min_length = min(lengths.values())
        print(f"Using minimum length: {min_length}")
        oa_times = oa_times[:min_length]
    
    print(f"Merging {len(oa_times)} OA samples with telemetry...")
    start_time = time.time()
    
    for i in range(len(oa_times)):
        mission_time = oa_times[i]
        # Convert to float if it's a string or other type
        if isinstance(mission_time, str):
            try:
                mission_time = float(mission_time)
            except (ValueError, TypeError):
                continue
        if not isinstance(mission_time, (int, float)) or math.isnan(mission_time):
            continue
        
        idx = bisect.bisect_left(t_mis, mission_time)
        if idx >= len(t_mis):
            idx = len(t_mis) - 1
        
        # Get telemetry values at this index
        def get_value(arr, idx_val):
            if not arr or idx_val >= len(arr):
                return float('nan')
            return arr[idx_val]
        
        entry = {
            "mission_time": mission_time,
            "sonar_distance": oa_sonar_range[i] if i < len(oa_sonar_range) else float('nan'),
            "sonar_confidence": oa_confidence[i] if i < len(oa_confidence) else float('nan'),
            "avoidance_id": oa_avoidance_id[i] if i < len(oa_avoidance_id) else float('nan'),
            "state": oa_state[i] if i < len(oa_state) else "",
            "nav_lat": get_value(nav_lat, idx),
            "nav_lon": get_value(nav_lon, idx),
            "nav_depth": get_value(nav_depth, idx),
            "current_task": get_value(cur_task, idx) if cur_task else "",
        }
        
        # Add optional fields if present
        if oa_turn_dir and i < len(oa_turn_dir):
            entry["turn_direction"] = oa_turn_dir[i]
        else:
            entry["turn_direction"] = ""
            
        if oa_depth_dir and i < len(oa_depth_dir):
            entry["depth_direction"] = oa_depth_dir[i]
        else:
            entry["depth_direction"] = ""
            
        if oa_using_depth and i < len(oa_using_depth):
            entry["using_depth"] = oa_using_depth[i]
        else:
            entry["using_depth"] = "N"
        
        merged.append(entry)
    
    elapsed = time.time() - start_time
    print(f"Merge completed in {elapsed:.2f}s, {len(merged)} samples merged.")
    return merged

# Rest of the plotting functions remain the same as before...
# [Continuing from line 400]

def plot_coordinate_system_validation(data, mission_file):
    """Validate coordinate system transformations"""
    try:
        with open(mission_file, 'r') as f:
            mission = json.load(f)
    except:
        mission = {}
    
    waypoints, _ = extract_mission_waypoints(mission)
    
    if not waypoints:
        print("No waypoints found in mission.")
        return
    
    # Get trajectory data
    nav_lat = data.get("nav_state_lat", [])
    nav_lon = data.get("nav_state_lon", [])
    local_x = data.get("nav_state_local_x", [])
    local_y = data.get("nav_state_local_y", [])
    
    if not nav_lat or not nav_lon or not local_x or not local_y:
        print("Missing coordinate data")
        return
    
    # Use first waypoint as reference
    lat0 = waypoints[0]["lat"]
    lon0 = waypoints[0]["lon"]
    
    # Filter valid data
    valid_indices = [i for i in range(len(nav_lat)) 
                    if not math.isnan(nav_lat[i]) and not math.isnan(nav_lon[i]) 
                    and not math.isnan(local_x[i]) and not math.isnan(local_y[i])
                    and nav_lat[i] != 0 and nav_lon[i] != 0]
    
    if not valid_indices:
        print("No valid coordinate data")
        return
    
    # Extract valid coordinates
    valid_lat = [nav_lat[i] for i in valid_indices]
    valid_lon = [nav_lon[i] for i in valid_indices]
    valid_x = [local_x[i] for i in valid_indices]
    valid_y = [local_y[i] for i in valid_indices]
    
    # Convert lat/lon to ENU
    enu_x, enu_y = latlon_to_enu(valid_lat, valid_lon, lat0, lon0)
    
    # Create plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))
    
    # Plot 1: Direct comparison
    ax1.scatter(valid_x, valid_y, c='blue', s=20, alpha=0.6, label='Vehicle local_x/y')
    ax1.scatter(enu_x, enu_y, c='red', s=20, alpha=0.6, label='ENU from lat/lon')
    
    # Plot waypoints
    wp_enu_x, wp_enu_y = latlon_to_enu([wp['lat'] for wp in waypoints], 
                                        [wp['lon'] for wp in waypoints], lat0, lon0)
    ax1.scatter(wp_enu_x, wp_enu_y, c='green', s=200, marker='*', 
               edgecolors='black', linewidths=2, label='Waypoints', zorder=10)
    
    for i, wp in enumerate(waypoints):
        ax1.annotate(wp['description'], (wp_enu_x[i], wp_enu_y[i]), 
                    xytext=(5, 5), textcoords='offset points', fontsize=10, fontweight='bold')
    
    ax1.set_xlabel('East (m)', fontsize=12)
    ax1.set_ylabel('North (m)', fontsize=12)
    ax1.set_title('Coordinate System Validation', fontsize=14, fontweight='bold')
    ax1.legend(loc='best')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Plot 2: Error analysis
    errors_x = [enu_x[i] - valid_x[i] for i in range(len(enu_x))]
    errors_y = [enu_y[i] - valid_y[i] for i in range(len(enu_y))]
    error_magnitude = [math.sqrt(ex**2 + ey**2) for ex, ey in zip(errors_x, errors_y)]
    
    sample_indices = range(len(error_magnitude))
    ax2.plot(sample_indices, error_magnitude, 'b-', linewidth=2, label='Position Error')
    ax2.axhline(y=1.0, color='orange', linestyle='--', label='1m threshold')
    ax2.axhline(y=5.0, color='red', linestyle='--', label='5m threshold')
    
    ax2.set_xlabel('Sample Index', fontsize=12)
    ax2.set_ylabel('Error Magnitude (m)', fontsize=12)
    ax2.set_title('Coordinate Transformation Error', fontsize=14, fontweight='bold')
    ax2.legend(loc='best')
    ax2.grid(True, alpha=0.3)
    
    # Print statistics
    mean_error = sum(error_magnitude) / len(error_magnitude)
    max_error = max(error_magnitude)
    print(f"\nCoordinate System Validation:")
    print(f"  Mean error: {mean_error:.3f}m")
    print(f"  Max error: {max_error:.3f}m")
    print(f"  Samples analyzed: {len(error_magnitude)}")
    
    plt.tight_layout()
    plt.show()

def plot_follow_track_navigation_analysis(data, mission_file):
    """Enhanced FOLLOW_TRACK analysis with cross-track error"""
    try:
        with open(mission_file, 'r') as f:
            mission = json.load(f)
    except:
        mission = {}
    
    waypoints, _ = extract_mission_waypoints(mission)
    
    if not waypoints:
        print("No waypoints in mission")
        return
    
    # Extract data
    times = data.get("mission_time", [])
    nav_lat = data.get("nav_state_lat", [])
    nav_lon = data.get("nav_state_lon", [])
    local_x = data.get("nav_state_local_x", [])
    local_y = data.get("nav_state_local_y", [])
    heading = data.get("nav_state_heading", [])
    speed = data.get("nav_state_speed", [])
    current_task = data.get("current_task", [])
    
    if not times or not local_x or not local_y:
        print("Missing required data")
        return
    
    # Find FOLLOW_TRACK periods
    track_periods = []
    in_track = False
    track_start = None
    
    for i, task in enumerate(current_task):
        if "FOLLOW_TRACK" in str(task) and not in_track:
            track_start = i
            in_track = True
        elif "FOLLOW_TRACK" not in str(task) and in_track:
            track_periods.append((track_start, i))
            in_track = False
    
    if in_track and track_start is not None:
        track_periods.append((track_start, len(current_task)))
    
    if not track_periods:
        print("No FOLLOW_TRACK periods found")
        return
    
    print(f"Found {len(track_periods)} FOLLOW_TRACK period(s)")
    
    # Get reference point
    lat0 = waypoints[0]["lat"]
    lon0 = waypoints[0]["lon"]
    
    # Convert waypoints to ENU
    wp_enu_x, wp_enu_y = latlon_to_enu([wp['lat'] for wp in waypoints], 
                                        [wp['lon'] for wp in waypoints], lat0, lon0)
    
    # Create figure
    fig = plt.figure(figsize=(16, 10))
    
    # Plot 1: Trajectory overview
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.set_title('Track Following Trajectory', fontsize=14, fontweight='bold')
    
    # Plot full trajectory
    valid_x = [x for x in local_x if not math.isnan(x)]
    valid_y = [y for y in local_y if not math.isnan(y)]
    ax1.plot(valid_x, valid_y, 'b-', linewidth=1, alpha=0.4, label='Full Trajectory')
    
    # Highlight FOLLOW_TRACK periods
    for start, end in track_periods:
        track_x = [local_x[i] for i in range(start, end) if not math.isnan(local_x[i])]
        track_y = [local_y[i] for i in range(start, end) if not math.isnan(local_y[i])]
        ax1.plot(track_x, track_y, 'r-', linewidth=2.5, alpha=0.8, label='FOLLOW_TRACK')
    
    # Plot waypoints
    ax1.scatter(wp_enu_x, wp_enu_y, c='green', s=200, marker='*', 
               edgecolors='black', linewidths=2, zorder=10, label='Waypoints')
    
    # Draw track segments
    for i in range(len(wp_enu_x) - 1):
        ax1.plot([wp_enu_x[i], wp_enu_x[i+1]], [wp_enu_y[i], wp_enu_y[i+1]], 
                'g--', linewidth=2, alpha=0.5)
    
    ax1.set_xlabel('East (m)', fontsize=11)
    ax1.set_ylabel('North (m)', fontsize=11)
    ax1.legend(loc='best', fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Plot 2: Cross-track error
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.set_title('Cross-Track Error', fontsize=14, fontweight='bold')
    
    # Calculate cross-track error for each FOLLOW_TRACK period
    for period_idx, (start, end) in enumerate(track_periods):
        period_times = [times[i] for i in range(start, end) if not math.isnan(times[i])]
        
        # Simple cross-track: distance from track line
        if len(wp_enu_x) >= 2:
            # Use first two waypoints to define track
            track_errors = []
            for i in range(start, end):
                if not math.isnan(local_x[i]) and not math.isnan(local_y[i]):
                    # Point-to-line distance
                    x0, y0 = local_x[i], local_y[i]
                    x1, y1 = wp_enu_x[0], wp_enu_y[0]
                    x2, y2 = wp_enu_x[1], wp_enu_y[1]
                    
                    # Cross-track error
                    num = abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1)
                    den = math.sqrt((y2-y1)**2 + (x2-x1)**2)
                    cte = num / den if den > 0 else 0
                    track_errors.append(cte)
                else:
                    track_errors.append(float('nan'))
            
            valid_errors = [e for e in track_errors if not math.isnan(e)]
            if valid_errors and period_times:
                ax2.plot(period_times[:len(valid_errors)], valid_errors, 
                        linewidth=2, label=f'Period {period_idx+1}')
    
    ax2.axhline(y=5, color='orange', linestyle='--', alpha=0.5, label='5m threshold')
    ax2.axhline(y=10, color='red', linestyle='--', alpha=0.5, label='10m threshold')
    ax2.set_xlabel('Mission Time (s)', fontsize=11)
    ax2.set_ylabel('Cross-Track Error (m)', fontsize=11)
    ax2.legend(loc='best', fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Speed profile
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.set_title('Speed Profile', fontsize=14, fontweight='bold')
    
    valid_times = [times[i] for i in range(len(times)) if not math.isnan(times[i]) and not math.isnan(speed[i])]
    valid_speeds = [speed[i] for i in range(len(speed)) if not math.isnan(times[i]) and not math.isnan(speed[i])]
    
    if valid_times and valid_speeds:
        ax3.plot(valid_times, valid_speeds, 'b-', linewidth=2, alpha=0.7)
        
        # Highlight FOLLOW_TRACK periods
        for start, end in track_periods:
            track_times = [times[i] for i in range(start, end) if not math.isnan(times[i])]
            if track_times:
                ax3.axvspan(track_times[0], track_times[-1], alpha=0.2, color='red')
    
    ax3.set_xlabel('Mission Time (s)', fontsize=11)
    ax3.set_ylabel('Speed (m/s)', fontsize=11)
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Heading
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.set_title('Heading Profile', fontsize=14, fontweight='bold')
    
    valid_heading_times = [times[i] for i in range(len(times)) if not math.isnan(times[i]) and not math.isnan(heading[i])]
    valid_headings = [heading[i] for i in range(len(heading)) if not math.isnan(times[i]) and not math.isnan(heading[i])]
    
    if valid_heading_times and valid_headings:
        ax4.plot(valid_heading_times, valid_headings, 'g-', linewidth=2, alpha=0.7)
        
        # Highlight FOLLOW_TRACK periods
        for start, end in track_periods:
            track_times = [times[i] for i in range(start, end) if not math.isnan(times[i])]
            if track_times:
                ax4.axvspan(track_times[0], track_times[-1], alpha=0.2, color='red')
    
    ax4.set_xlabel('Mission Time (s)', fontsize=11)
    ax4.set_ylabel('Heading (°)', fontsize=11)
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def plot_telemetry_overview(data, mission_file):
    """Comprehensive telemetry overview"""
    try:
        with open(mission_file, 'r') as f:
            mission = json.load(f)
    except:
        mission = {}
    
    waypoints, _ = extract_mission_waypoints(mission)
    
    times = data.get("mission_time", [])
    nav_lat = data.get("nav_state_lat", [])
    nav_lon = data.get("nav_state_lon", [])
    local_x = data.get("nav_state_local_x", [])
    local_y = data.get("nav_state_local_y", [])
    depth = data.get("nav_state_depth", [])
    heading = data.get("nav_state_heading", [])
    speed = data.get("nav_state_speed", [])
    battery = data.get("energy_state_capacity_remaining", [])
    current_task = data.get("current_task", [])
    
    if not times:
        print("No data to plot")
        return
    
    fig = plt.figure(figsize=(18, 12))
    
    # Plot 1: Trajectory
    ax1 = fig.add_subplot(3, 2, 1)
    ax1.set_title('Vehicle Trajectory', fontsize=14, fontweight='bold')
    
    valid_x = [x for x in local_x if not math.isnan(x)]
    valid_y = [y for y in local_y if not math.isnan(y)]
    
    if valid_x and valid_y:
        ax1.plot(valid_x, valid_y, 'b-', linewidth=2, alpha=0.7, label='Trajectory')
        
        # Plot waypoints
        if waypoints:
            lat0 = waypoints[0]["lat"]
            lon0 = waypoints[0]["lon"]
            wp_enu_x, wp_enu_y = latlon_to_enu([wp['lat'] for wp in waypoints], 
                                                [wp['lon'] for wp in waypoints], lat0, lon0)
            ax1.scatter(wp_enu_x, wp_enu_y, c='red', s=200, marker='*', 
                       edgecolors='black', linewidths=2, zorder=10, label='Waypoints')
    
    ax1.set_xlabel('East (m)', fontsize=11)
    ax1.set_ylabel('North (m)', fontsize=11)
    ax1.legend(loc='best')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Plot 2: Depth
    ax2 = fig.add_subplot(3, 2, 2)
    ax2.set_title('Depth Profile', fontsize=14, fontweight='bold')
    
    valid_depth_times = [times[i] for i in range(len(times)) if not math.isnan(times[i]) and not math.isnan(depth[i])]
    valid_depths = [depth[i] for i in range(len(depth)) if not math.isnan(times[i]) and not math.isnan(depth[i])]
    
    if valid_depth_times and valid_depths:
        ax2.plot(valid_depth_times, valid_depths, 'b-', linewidth=2)
        ax2.invert_yaxis()
    
    ax2.set_xlabel('Mission Time (s)', fontsize=11)
    ax2.set_ylabel('Depth (m)', fontsize=11)
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Speed
    ax3 = fig.add_subplot(3, 2, 3)
    ax3.set_title('Speed Profile', fontsize=14, fontweight='bold')
    
    valid_speed_times = [times[i] for i in range(len(times)) if not math.isnan(times[i]) and not math.isnan(speed[i])]
    valid_speeds = [speed[i] for i in range(len(speed)) if not math.isnan(times[i]) and not math.isnan(speed[i])]
    
    if valid_speed_times and valid_speeds:
        ax3.plot(valid_speed_times, valid_speeds, 'g-', linewidth=2)
    
    ax3.set_xlabel('Mission Time (s)', fontsize=11)
    ax3.set_ylabel('Speed (m/s)', fontsize=11)
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Heading
    ax4 = fig.add_subplot(3, 2, 4)
    ax4.set_title('Heading Profile', fontsize=14, fontweight='bold')
    
    valid_heading_times = [times[i] for i in range(len(times)) if not math.isnan(times[i]) and not math.isnan(heading[i])]
    valid_headings = [heading[i] for i in range(len(heading)) if not math.isnan(times[i]) and not math.isnan(heading[i])]
    
    if valid_heading_times and valid_headings:
        ax4.plot(valid_heading_times, valid_headings, 'r-', linewidth=2)
    
    ax4.set_xlabel('Mission Time (s)', fontsize=11)
    ax4.set_ylabel('Heading (°)', fontsize=11)
    ax4.grid(True, alpha=0.3)
    
    # Plot 5: Battery
    ax5 = fig.add_subplot(3, 2, 5)
    ax5.set_title('Battery Level', fontsize=14, fontweight='bold')
    
    valid_battery_times = [times[i] for i in range(len(times)) if not math.isnan(times[i]) and not math.isnan(battery[i])]
    valid_battery = [battery[i] for i in range(len(battery)) if not math.isnan(times[i]) and not math.isnan(battery[i])]
    
    if valid_battery_times and valid_battery:
        ax5.plot(valid_battery_times, valid_battery, 'orange', linewidth=2)
    
    ax5.set_xlabel('Mission Time (s)', fontsize=11)
    ax5.set_ylabel('Battery (%)', fontsize=11)
    ax5.grid(True, alpha=0.3)
    
    # Plot 6: Task timeline
    ax6 = fig.add_subplot(3, 2, 6)
    ax6.set_title('Mission Tasks', fontsize=14, fontweight='bold')
    
    # Extract unique tasks
    task_changes = []
    prev_task = None
    for i, task in enumerate(current_task):
        if task != prev_task:
            task_changes.append((times[i] if i < len(times) else 0, str(task)))
            prev_task = task
    
    for i, (t, task) in enumerate(task_changes):
        next_t = task_changes[i+1][0] if i+1 < len(task_changes) else (times[-1] if times else t)
        ax6.barh(0, next_t - t, left=t, height=0.5, alpha=0.6, 
                label=task if task not in [tc[1] for tc in task_changes[:i]] else "")
        ax6.text((t + next_t) / 2, 0, task, ha='center', va='center', fontsize=8)
    
    ax6.set_xlabel('Mission Time (s)', fontsize=11)
    ax6.set_yticks([])
    ax6.set_ylim(-0.5, 0.5)
    handles, labels = ax6.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax6.legend(by_label.values(), by_label.keys(), loc='upper left', bbox_to_anchor=(1, 1), fontsize=8)
    
    plt.tight_layout()
    plt.show()

def plot_navigation_analysis(data, mission_file):
    """Navigation system analysis"""
    try:
        with open(mission_file, 'r') as f:
            mission = json.load(f)
    except:
        mission = {}
    
    times = data.get("mission_time", [])
    speed_error = data.get("control_errors_speed_error", [])
    heading_error = data.get("control_errors_heading_error", [])
    depth_error = data.get("control_errors_depth_error", [])
    
    if not times:
        print("No data to plot")
        return
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 10))
    
    # Plot 1: Speed error
    ax1.set_title('Speed Control Error', fontsize=14, fontweight='bold')
    valid_times = [times[i] for i in range(len(times)) if not math.isnan(times[i]) and not math.isnan(speed_error[i])]
    valid_errors = [speed_error[i] for i in range(len(speed_error)) if not math.isnan(times[i]) and not math.isnan(speed_error[i])]
    if valid_times and valid_errors:
        ax1.plot(valid_times, valid_errors, 'b-', linewidth=2)
        ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax1.set_xlabel('Mission Time (s)', fontsize=11)
    ax1.set_ylabel('Speed Error (m/s)', fontsize=11)
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Heading error
    ax2.set_title('Heading Control Error', fontsize=14, fontweight='bold')
    valid_times = [times[i] for i in range(len(times)) if not math.isnan(times[i]) and not math.isnan(heading_error[i])]
    valid_errors = [heading_error[i] for i in range(len(heading_error)) if not math.isnan(times[i]) and not math.isnan(heading_error[i])]
    if valid_times and valid_errors:
        ax2.plot(valid_times, valid_errors, 'r-', linewidth=2)
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_xlabel('Mission Time (s)', fontsize=11)
    ax2.set_ylabel('Heading Error (°)', fontsize=11)
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Depth error
    ax3.set_title('Depth Control Error', fontsize=14, fontweight='bold')
    valid_times = [times[i] for i in range(len(times)) if not math.isnan(times[i]) and not math.isnan(depth_error[i])]
    valid_errors = [depth_error[i] for i in range(len(depth_error)) if not math.isnan(times[i]) and not math.isnan(depth_error[i])]
    if valid_times and valid_errors:
        ax3.plot(valid_times, valid_errors, 'g-', linewidth=2)
        ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax3.set_xlabel('Mission Time (s)', fontsize=11)
    ax3.set_ylabel('Depth Error (m)', fontsize=11)
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Combined RMS error
    ax4.set_title('RMS Control Errors', fontsize=14, fontweight='bold')
    
    # Calculate RMS errors over time windows
    window_size = 50
    rms_times = []
    rms_speed = []
    rms_heading = []
    rms_depth = []
    
    for i in range(0, len(times) - window_size, window_size // 2):
        window_time = times[i:i+window_size]
        window_speed_err = speed_error[i:i+window_size]
        window_heading_err = heading_error[i:i+window_size]
        window_depth_err = depth_error[i:i+window_size]
        
        # Filter NaN
        valid_speed = [e for e in window_speed_err if not math.isnan(e)]
        valid_heading = [e for e in window_heading_err if not math.isnan(e)]
        valid_depth = [e for e in window_depth_err if not math.isnan(e)]
        
        if valid_speed:
            rms_speed.append(math.sqrt(sum(e**2 for e in valid_speed) / len(valid_speed)))
            rms_times.append(window_time[len(window_time)//2])
        
        if valid_heading and len(rms_heading) < len(rms_speed):
            rms_heading.append(math.sqrt(sum(e**2 for e in valid_heading) / len(valid_heading)))
        
        if valid_depth and len(rms_depth) < len(rms_speed):
            rms_depth.append(math.sqrt(sum(e**2 for e in valid_depth) / len(valid_depth)))
    
    if rms_times:
        if rms_speed:
            ax4.plot(rms_times[:len(rms_speed)], rms_speed, 'b-', linewidth=2, label='Speed RMS')
        if rms_heading:
            ax4.plot(rms_times[:len(rms_heading)], rms_heading, 'r-', linewidth=2, label='Heading RMS')
        if rms_depth:
            ax4.plot(rms_times[:len(rms_depth)], rms_depth, 'g-', linewidth=2, label='Depth RMS')
    
    ax4.set_xlabel('Mission Time (s)', fontsize=11)
    ax4.set_ylabel('RMS Error', fontsize=11)
    ax4.legend(loc='best')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def plot_energy_analysis(data):
    """Energy consumption analysis"""
    times = data.get("mission_time", [])
    voltage = data.get("energy_state_voltage", [])
    capacity = data.get("energy_state_capacity_remaining", [])
    speed = data.get("nav_state_speed", [])
    
    if not times or not capacity:
        print("No energy data to plot")
        return
    
    fig = plt.figure(figsize=(16, 10))
    
    # Plot 1: Battery capacity
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.set_title('Battery Capacity', fontsize=14, fontweight='bold')
    
    valid_times = [times[i] for i in range(len(times)) if not math.isnan(times[i]) and not math.isnan(capacity[i])]
    valid_capacity = [capacity[i] for i in range(len(capacity)) if not math.isnan(times[i]) and not math.isnan(capacity[i])]
    
    if valid_times and valid_capacity:
        ax1.plot(valid_times, valid_capacity, 'b-', linewidth=2)
        ax1.axhline(y=20, color='red', linestyle='--', alpha=0.5, label='20% Critical')
    
    ax1.set_xlabel('Mission Time (s)', fontsize=11)
    ax1.set_ylabel('Capacity (%)', fontsize=11)
    ax1.legend(loc='best')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Voltage
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.set_title('Battery Voltage', fontsize=14, fontweight='bold')
    
    valid_voltage_times = [times[i] for i in range(len(times)) if not math.isnan(times[i]) and not math.isnan(voltage[i])]
    valid_voltage = [voltage[i] for i in range(len(voltage)) if not math.isnan(times[i]) and not math.isnan(voltage[i])]
    
    if valid_voltage_times and valid_voltage:
        ax2.plot(valid_voltage_times, valid_voltage, 'r-', linewidth=2)
    
    ax2.set_xlabel('Mission Time (s)', fontsize=11)
    ax2.set_ylabel('Voltage (V)', fontsize=11)
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Consumption rate
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.set_title('Energy Consumption Rate', fontsize=14, fontweight='bold')
    
    # Calculate consumption rate
    capacity_filt = [c for c in capacity if not math.isnan(c)]
    times_filt = [times[i] for i in range(len(capacity)) if not math.isnan(capacity[i])]
    
    if len(capacity_filt) > 1:
        consumption_rate = []
        rate_times = []
        for i in range(1, len(capacity_filt)):
            dt = times_filt[i] - times_filt[i-1]
            if dt > 0:
                dc = capacity_filt[i-1] - capacity_filt[i]
                rate = (dc / dt) * 3600  # Convert to %/hour
                consumption_rate.append(rate)
                rate_times.append(times_filt[i])
        if consumption_rate:
            ax3.plot(rate_times, consumption_rate, 'orange', linewidth=2)
    
    ax3.set_xlabel('Mission Time (s)', fontsize=11)
    ax3.set_ylabel('Consumption Rate (%/hour)', fontsize=11)
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Power vs Speed
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.set_title('Power Consumption vs Speed', fontsize=14, fontweight='bold')
    
    speed_filt = [s for s in speed if not math.isnan(s)]
    if len(consumption_rate) > 0 and len(speed_filt) >= len(consumption_rate):
        # Align speed with consumption rate
        speed_for_power = speed_filt[:len(consumption_rate)]
        ax4.scatter(speed_for_power, consumption_rate, alpha=0.5)
    
    ax4.set_xlabel('Speed (m/s)', fontsize=11)
    ax4.set_ylabel('Consumption Rate (%/hour)', fontsize=11)
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

# ----------------------------
# Obstacle Avoidance Analysis - UPDATED TO ONLY PLOT FIRST ENCOUNTER
# ----------------------------

def plot_obstacle_avoidance_analysis(telemetry, oa_data, mission_file):
    """Obstacle Avoidance Analysis - Only plots first detection of each encounter"""
    if not oa_data:
        print("No obstacle avoidance data to analyze.")
        return
    try:
        with open(mission_file, 'r') as f:
            mission = json.load(f)
    except:
        mission = {}
    
    waypoints, _ = extract_mission_waypoints(mission)
    merged = merge_oa_with_telemetry_optimized(telemetry, oa_data)
    
    if not merged:
        print("No valid OA samples after merge.")
        return

    # Extract data
    times = [e["mission_time"] for e in merged]
    dist = [e["sonar_distance"] for e in merged]
    conf = [e["sonar_confidence"] for e in merged]
    states = [e["state"] for e in merged]
    using_depth = [e["using_depth"] for e in merged]
    nav_lat_pts = [e["nav_lat"] for e in merged]
    nav_lon_pts = [e["nav_lon"] for e in merged]
    nav_depth_pts = [e["nav_depth"] for e in merged]
    
    # Get full trajectory from telemetry
    traj_lat = telemetry.get("nav_state_lat", [])
    traj_lon = telemetry.get("nav_state_lon", [])
    traj_times = telemetry.get("mission_time", [])
    
    if waypoints:
        lat0 = waypoints[0]["lat"]
        lon0 = waypoints[0]["lon"]
    else:
        lat0 = next((v for v in nav_lat_pts if not math.isnan(v) and v != 0.0), 0.0)
        lon0 = next((v for v in nav_lon_pts if not math.isnan(v) and v != 0.0), 0.0)

    # Convert full trajectory to ENU
    traj_x, traj_y = [], []
    if traj_lat and traj_lon:
        filtered_traj_lat = [lat for lat in traj_lat if not math.isnan(lat) and lat != 0.0]
        filtered_traj_lon = [lon for lon in traj_lon if not math.isnan(lon) and lon != 0.0]
        if filtered_traj_lat and filtered_traj_lon:
            traj_x, traj_y = latlon_to_enu(filtered_traj_lat, filtered_traj_lon, lat0, lon0)

    # Group data by avoidance events - ONLY KEEP FIRST DETECTION PER ENCOUNTER
    obstacles = {}
    for i, e in enumerate(merged):
        aid = e["avoidance_id"]
        if aid == 0 or math.isnan(aid):
            continue
        aid = int(aid)
        t = times[i]
        if math.isnan(t):
            continue
        
        # Only create new obstacle entry if this is the first time we see this ID
        if aid not in obstacles:
            obstacles[aid] = {
                'times': [t],
                'distances': [dist[i]],
                'states': [states[i]],
                'confidences': [conf[i]],
                'nav_lat': [nav_lat_pts[i]],
                'nav_lon': [nav_lon_pts[i]],
                'nav_depth': [nav_depth_pts[i]],
                'first_detection_only': True  # Flag to indicate we only stored first detection
            }
            if using_depth and i < len(using_depth):
                obstacles[aid]['using_depth'] = [using_depth[i]]
        else:
            # Continue tracking all data for statistics, just won't plot every point
            obstacles[aid]['times'].append(t)
            obstacles[aid]['distances'].append(dist[i])
            obstacles[aid]['states'].append(states[i])
            obstacles[aid]['confidences'].append(conf[i])
            obstacles[aid]['nav_lat'].append(nav_lat_pts[i])
            obstacles[aid]['nav_lon'].append(nav_lon_pts[i])
            obstacles[aid]['nav_depth'].append(nav_depth_pts[i])
            if 'using_depth' in obstacles[aid]:
                obstacles[aid]['using_depth'].append(using_depth[i])
    
    if not obstacles:
        print("No valid obstacle avoidance events found.")
        return
    
    print(f"Found {len(obstacles)} obstacle encounter(s): IDs {list(obstacles.keys())}")
    
    # Create comprehensive figure
    fig = plt.figure(figsize=(18, 14))
    
    # Define colors for each obstacle
    colors = plt.cm.Set1(range(len(obstacles)))
    
    # ========== PLOT 1: Timeline Overview ==========
    ax1 = fig.add_subplot(3, 2, 1)
    ax1.set_title('Obstacle Avoidance Timeline', fontsize=14, fontweight='bold')
    
    y_pos = 0
    for idx, (aid, obs) in enumerate(obstacles.items()):
        # Find avoidance periods
        in_avoid = False
        avoid_start = None
        
        for i, (t, state) in enumerate(zip(obs['times'], obs['states'])):
            if state in ['AVOIDING', 'CLEARING'] and not in_avoid:
                avoid_start = t
                in_avoid = True
            elif state not in ['AVOIDING', 'CLEARING'] and in_avoid:
                # End of avoidance
                ax1.barh(y_pos, t - avoid_start, left=avoid_start, height=0.8, 
                        color=colors[idx], alpha=0.6, edgecolor='black')
                in_avoid = False
        
        # Handle case where avoidance continues to end
        if in_avoid:
            ax1.barh(y_pos, obs['times'][-1] - avoid_start, left=avoid_start, 
                    height=0.8, color=colors[idx], alpha=0.6, edgecolor='black')
        
        # Mark FIRST detection only with a larger marker
        ax1.plot(obs['times'][0], y_pos, 'o', color='red', markersize=12, 
                markeredgecolor='black', markeredgewidth=2, label=f'First Detection {aid}' if idx == 0 else '', zorder=10)
        
        y_pos += 1
    
    ax1.set_yticks(range(len(obstacles)))
    ax1.set_yticklabels([f'Obstacle {aid}' for aid in obstacles.keys()])
    ax1.set_xlabel('Mission Time (s)', fontsize=11)
    ax1.set_ylabel('Obstacle ID', fontsize=11)
    ax1.grid(True, alpha=0.3, axis='x')
    
    # Add legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='red', alpha=0.6, label='First Detection'),
        Patch(facecolor=colors[0], alpha=0.6, label='Avoidance Period')
    ]
    ax1.legend(handles=legend_elements, loc='upper right', fontsize=9)
    
    # ========== PLOT 2: Sonar Range Over Time ==========
    ax2 = fig.add_subplot(3, 2, 2)
    ax2.set_title('Sonar Distance to Obstacles', fontsize=14, fontweight='bold')
    
    for idx, (aid, obs) in enumerate(obstacles.items()):
        valid_times = []
        valid_dists = []
        for t, d in zip(obs['times'], obs['distances']):
            if not math.isnan(t) and not math.isnan(d) and d < 100.0:
                valid_times.append(t)
                valid_dists.append(d)
        
        if valid_times:
            ax2.plot(valid_times, valid_dists, linewidth=2.5, 
                    label=f'Obstacle {aid}', color=colors[idx], alpha=0.8)
            
            # Mark FIRST detection with larger marker
            ax2.plot(valid_times[0], valid_dists[0], 'o', color='red', markersize=15,
                    markeredgecolor='black', markeredgewidth=2, zorder=10)
            
            # Mark minimum distance
            min_dist = min(valid_dists)
            min_idx = valid_dists.index(min_dist)
            ax2.plot(valid_times[min_idx], min_dist, '*', color='yellow', markersize=20, 
                    markeredgecolor='black', markeredgewidth=2, zorder=9)
    
    ax2.axhline(y=20, color='r', linestyle='--', linewidth=1, alpha=0.5, label='Detection Threshold')
    ax2.set_xlabel('Mission Time (s)', fontsize=11)
    ax2.set_ylabel('Range (m)', fontsize=11)
    ax2.legend(loc='upper right', fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    # ========== PLOT 3: Avoidance States ==========
    ax3 = fig.add_subplot(3, 2, 3)
    ax3.set_title('Avoidance State Transitions', fontsize=14, fontweight='bold')
    
    # Create state value mapping for better visualization
    state_values = {'NORMAL': 0, 'APPROACHING': 0.33, 'CLEARING': 0.66, 'AVOIDING': 1.0}
    
    for idx, (aid, obs) in enumerate(obstacles.items()):
        state_vals = []
        state_times = []
        for t, s in zip(obs['times'], obs['states']):
            if not math.isnan(t) and s in state_values:
                state_times.append(t)
                state_vals.append(state_values[s])
        
        if state_times:
            ax3.plot(state_times, state_vals, drawstyle='steps-post', 
                    linewidth=2.5, label=f'Obstacle {aid}', color=colors[idx], alpha=0.8)
            
            # Mark first detection
            ax3.plot(state_times[0], state_vals[0], 'o', color='red', markersize=12,
                    markeredgecolor='black', markeredgewidth=2, zorder=10)
    
    ax3.set_yticks([0, 0.33, 0.66, 1.0])
    ax3.set_yticklabels(['NORMAL', 'APPROACHING', 'CLEARING', 'AVOIDING'], fontsize=10)
    ax3.set_xlabel('Mission Time (s)', fontsize=11)
    ax3.set_ylabel('Avoidance State', fontsize=11)
    ax3.legend(loc='best', fontsize=9)
    ax3.grid(True, alpha=0.3)
    
    # ========== PLOT 4: Depth Avoidance Usage (if available) ==========
    ax4 = fig.add_subplot(3, 2, 4)
    ax4.set_title('Depth Maneuver Usage', fontsize=14, fontweight='bold')
    
    # Check if depth data is available
    if using_depth and any(u != "N" and u != "" for u in using_depth):
        # Show depth usage for all OA samples
        depth_usage_times = []
        depth_usage_vals = []
        for t, u in zip(times, using_depth):
            if not math.isnan(t):
                depth_usage_times.append(t)
                depth_usage_vals.append(1 if u == 'Y' else 0)
        
        if depth_usage_times:
            ax4.fill_between(depth_usage_times, 0, depth_usage_vals, 
                            color='purple', alpha=0.5, step='post', label='Depth Maneuver Active')
            ax4.plot(depth_usage_times, depth_usage_vals, 'k-', linewidth=1, alpha=0.6, drawstyle='steps-post')
        ax4.set_yticks([0, 1])
        ax4.set_yticklabels(['Not Using', 'Using Depth'], fontsize=10)
        ax4.legend(loc='upper right', fontsize=9)
    else:
        # No depth data available
        ax4.text(0.5, 0.5, 'Depth Maneuver Data Not Available', 
                ha='center', va='center', transform=ax4.transAxes, fontsize=12, style='italic')
    
    ax4.set_xlabel('Mission Time (s)', fontsize=11)
    ax4.set_ylabel('Depth Avoidance', fontsize=11)
    ax4.grid(True, alpha=0.3)
    
    # ========== PLOT 5: Trajectory with Avoidance Zones and Mission Obstacles ==========
    ax5 = fig.add_subplot(3, 2, (5, 6))
    ax5.set_title('Vehicle Trajectory with Obstacle Encounters and Mission Obstacles', fontsize=14, fontweight='bold')
    
    # Plot full trajectory
    if traj_x and traj_y:
        ax5.plot(traj_x, traj_y, 'b-', linewidth=2, alpha=0.4, label='Full Trajectory', zorder=1)
    
    # Plot obstacle positions from mission file
    mission_obstacles = mission.get('obstacles', [])
    if mission_obstacles:
        print(f"\nPlotting {len(mission_obstacles)} obstacles from mission file:")
        for obs_idx, obs in enumerate(mission_obstacles):
            pos = obs.get('position', {})
            obs_x = pos.get('local_x', None)
            obs_y = pos.get('local_y', None)
            
            if obs_x is not None and obs_y is not None:
                print(f"  Obstacle {obs_idx+1}: ({obs_x}, {obs_y}), shape: {obs.get('shape', 'unknown')}")
                
                # Draw obstacle shape
                dims = obs.get('dimensions', {})
                obs_shape = obs.get('shape', 'cylinder')
                
                if obs_shape == 'cylinder':
                    radius = dims.get('radius', 5)
                    circle = plt.Circle((obs_x, obs_y), radius, color='gray', 
                                      alpha=0.3, zorder=2, linewidth=2, edgecolor='black')
                    ax5.add_patch(circle)
                    ax5.plot(obs_x, obs_y, 'kx', markersize=15, markeredgewidth=3, zorder=3)
                    # Add label
                    ax5.text(obs_x, obs_y + radius + 5, f'Obs {obs_idx+1}\n({obs_shape})', 
                            ha='center', fontsize=9, fontweight='bold',
                            bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.7))
                else:  # box
                    width = dims.get('width', 10)
                    length = dims.get('length', 10)
                    from matplotlib.patches import Rectangle
                    rect = Rectangle((obs_x - width/2, obs_y - length/2), width, length,
                                   color='gray', alpha=0.3, zorder=2, linewidth=2, edgecolor='black')
                    ax5.add_patch(rect)
                    ax5.plot(obs_x, obs_y, 'kx', markersize=15, markeredgewidth=3, zorder=3)
                    # Add label
                    ax5.text(obs_x, obs_y + length/2 + 5, f'Obs {obs_idx+1}\n({obs_shape})', 
                            ha='center', fontsize=9, fontweight='bold',
                            bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.7))
    
    # Plot FIRST encounter location for each obstacle (not every update)
    for idx, (aid, obs) in enumerate(obstacles.items()):
        # Only plot the FIRST detection point
        lat = obs['nav_lat'][0]
        lon = obs['nav_lon'][0]
        
        if not math.isnan(lat) and not math.isnan(lon):
            avoid_x, avoid_y = latlon_to_enu([lat], [lon], lat0, lon0)
            
            # Plot encounter location with large marker
            ax5.scatter(avoid_x, avoid_y, c=[colors[idx]], s=300, 
                       label=f'Obstacle {aid} First Detection', zorder=5, 
                       edgecolors='black', linewidths=2, alpha=0.9, marker='D')
            
            # Draw arrow pointing to detection
            ax5.annotate(f'#{aid}', xy=(avoid_x[0], avoid_y[0]), 
                       xytext=(20, 20), textcoords='offset points',
                       fontsize=14, fontweight='bold',
                       bbox=dict(boxstyle='round,pad=0.5', facecolor=colors[idx], alpha=0.8, edgecolor='black', linewidth=2),
                       arrowprops=dict(arrowstyle='->', color='black', lw=2))
    
    ax5.set_xlabel('East (m)', fontsize=11)
    ax5.set_ylabel('North (m)', fontsize=11)
    ax5.legend(loc='best', fontsize=9, ncol=2)
    ax5.grid(True, alpha=0.3)
    ax5.axis('equal')
    
    plt.tight_layout()
    
    # Print summary statistics
    print("\n" + "="*60)
    print("OBSTACLE AVOIDANCE SUMMARY")
    print("="*60)
    for aid, obs in obstacles.items():
        print(f"\nObstacle {aid}:")
        
        valid_dists = [d for d in obs['distances'] if not math.isnan(d) and d < 100.0]
        if valid_dists:
            print(f"  First Detection Range: {valid_dists[0]:.2f}m")
            print(f"  Min Distance: {min(valid_dists):.2f}m")
            print(f"  Avg Distance: {sum(valid_dists)/len(valid_dists):.2f}m")
        
        avoid_count = sum(1 for s in obs['states'] if s == 'AVOIDING')
        clear_count = sum(1 for s in obs['states'] if s == 'CLEARING')
        print(f"  Samples AVOIDING: {avoid_count}")
        print(f"  Samples CLEARING: {clear_count}")
        
        # Only show depth info if available
        if 'using_depth' in obs and obs['using_depth']:
            depth_used = sum(1 for u in obs['using_depth'] if u == 'Y')
            print(f"  Used Depth Maneuver: {'Yes' if depth_used > 0 else 'No'}")
        
        if obs['times']:
            duration = obs['times'][-1] - obs['times'][0]
            print(f"  Encounter Duration: {duration:.1f}s")
    
    print("="*60 + "\n")
    
    plt.show()

# ----------------------------
# Mission Replay (Animation) - COMPLETE IMPLEMENTATION
# ----------------------------

class MissionReplayPlotter:
    """Animated mission replay with obstacles"""
    def __init__(self, csv_file, mission_file, speed_multiplier=1.0):
        self.csv_file = csv_file
        self.mission_file = mission_file
        self.speed_multiplier = speed_multiplier
        
        # Load data
        print(f"Loading mission replay data...")
        self.data = load_csv(csv_file)
        
        try:
            with open(mission_file, 'r') as f:
                self.mission = json.load(f)
        except:
            self.mission = {}
        
        self.waypoints, _ = extract_mission_waypoints(self.mission)
        
        # Extract trajectory data
        self.times = self.data.get("mission_time", [])
        self.local_x = self.data.get("nav_state_local_x", [])
        self.local_y = self.data.get("nav_state_local_y", [])
        self.heading = self.data.get("nav_state_heading", [])
        self.depth = self.data.get("nav_state_depth", [])
        self.speed = self.data.get("nav_state_speed", [])
        self.current_task = self.data.get("current_task", [])
        self.battery = self.data.get("energy_state_capacity_remaining", [])
        
        # Filter valid data
        self.valid_indices = [i for i in range(len(self.times)) 
                             if not math.isnan(self.times[i]) and not math.isnan(self.local_x[i]) 
                             and not math.isnan(self.local_y[i])]
        
        if not self.valid_indices:
            print("No valid trajectory data for replay")
            return
        
        print(f"Loaded {len(self.valid_indices)} valid trajectory points")
        print(f"Mission duration: {self.times[self.valid_indices[-1]]:.1f}s")
        print(f"Playback speed: {speed_multiplier}x")
    
    def start(self):
        """Start the animated replay"""
        if not self.valid_indices:
            print("No data to replay")
            return
        
        # Setup figure
        fig = plt.figure(figsize=(16, 10))
        
        # Main trajectory plot
        ax_traj = fig.add_subplot(2, 2, (1, 3))
        ax_traj.set_title('Mission Replay - Vehicle Trajectory', fontsize=14, fontweight='bold')
        ax_traj.set_xlabel('East (m)', fontsize=11)
        ax_traj.set_ylabel('North (m)', fontsize=11)
        ax_traj.grid(True, alpha=0.3)
        ax_traj.axis('equal')
        
        # Plot mission obstacles
        mission_obstacles = self.mission.get('obstacles', [])
        if mission_obstacles:
            for obs_idx, obs in enumerate(mission_obstacles):
                pos = obs.get('position', {})
                obs_x = pos.get('local_x', None)
                obs_y = pos.get('local_y', None)
                
                if obs_x is not None and obs_y is not None:
                    dims = obs.get('dimensions', {})
                    obs_shape = obs.get('shape', 'cylinder')
                    
                    if obs_shape == 'cylinder':
                        radius = dims.get('radius', 5)
                        circle = plt.Circle((obs_x, obs_y), radius, color='red', 
                                          alpha=0.3, zorder=2, linewidth=2, edgecolor='darkred')
                        ax_traj.add_patch(circle)
                        ax_traj.plot(obs_x, obs_y, 'rx', markersize=15, markeredgewidth=3, zorder=3)
                    else:  # box
                        width = dims.get('width', 10)
                        length = dims.get('length', 10)
                        from matplotlib.patches import Rectangle
                        rect = Rectangle((obs_x - width/2, obs_y - length/2), width, length,
                                       color='red', alpha=0.3, zorder=2, linewidth=2, edgecolor='darkred')
                        ax_traj.add_patch(rect)
                        ax_traj.plot(obs_x, obs_y, 'rx', markersize=15, markeredgewidth=3, zorder=3)
        
        # Plot waypoints
        if self.waypoints:
            lat0 = self.waypoints[0]["lat"]
            lon0 = self.waypoints[0]["lon"]
            wp_lats = [wp['lat'] for wp in self.waypoints]
            wp_lons = [wp['lon'] for wp in self.waypoints]
            wp_x, wp_y = latlon_to_enu(wp_lats, wp_lons, lat0, lon0)
            ax_traj.scatter(wp_x, wp_y, c='green', s=200, marker='*', 
                           edgecolors='black', linewidths=2, zorder=10, label='Waypoints')
            
            # Connect waypoints
            for i in range(len(wp_x) - 1):
                ax_traj.plot([wp_x[i], wp_x[i+1]], [wp_y[i], wp_y[i+1]], 
                            'g--', linewidth=1.5, alpha=0.5)
        
        # Initialize animated elements
        trail_line, = ax_traj.plot([], [], 'b-', linewidth=2, alpha=0.7, label='Vehicle Path')
        vehicle_marker, = ax_traj.plot([], [], 'ro', markersize=12, markeredgecolor='black', 
                                       markeredgewidth=2, zorder=11, label='Vehicle')
        heading_arrow = ax_traj.arrow(0, 0, 0, 0, head_width=3, head_length=5, fc='red', ec='black', lw=2, zorder=12)
        
        ax_traj.legend(loc='best', fontsize=9)
        
        # Depth plot
        ax_depth = fig.add_subplot(2, 2, 2)
        ax_depth.set_title('Depth Profile', fontsize=12, fontweight='bold')
        ax_depth.set_xlabel('Mission Time (s)', fontsize=10)
        ax_depth.set_ylabel('Depth (m)', fontsize=10)
        ax_depth.grid(True, alpha=0.3)
        ax_depth.invert_yaxis()
        
        depth_line, = ax_depth.plot([], [], 'b-', linewidth=2)
        depth_marker, = ax_depth.plot([], [], 'ro', markersize=8, markeredgecolor='black')
        
        # Speed plot
        ax_speed = fig.add_subplot(2, 2, 4)
        ax_speed.set_title('Speed Profile', fontsize=12, fontweight='bold')
        ax_speed.set_xlabel('Mission Time (s)', fontsize=10)
        ax_speed.set_ylabel('Speed (m/s)', fontsize=10)
        ax_speed.grid(True, alpha=0.3)
        
        speed_line, = ax_speed.plot([], [], 'g-', linewidth=2)
        speed_marker, = ax_speed.plot([], [], 'ro', markersize=8, markeredgecolor='black')
        
        # Status text
        status_text = fig.text(0.02, 0.98, '', fontsize=11, verticalalignment='top', 
                              family='monospace', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.tight_layout()
        
        # Animation update function
        def update(frame):
            if frame >= len(self.valid_indices):
                return trail_line, vehicle_marker, depth_line, depth_marker, speed_line, speed_marker, status_text
            
            idx = self.valid_indices[frame]
            
            # Update trajectory
            x_trail = [self.local_x[self.valid_indices[i]] for i in range(frame + 1)]
            y_trail = [self.local_y[self.valid_indices[i]] for i in range(frame + 1)]
            trail_line.set_data(x_trail, y_trail)
            
            # Update vehicle position
            x_current = self.local_x[idx]
            y_current = self.local_y[idx]
            vehicle_marker.set_data([x_current], [y_current])
            
            # Update heading arrow
            heading_rad = math.radians(90 - self.heading[idx])  # Convert to math convention
            arrow_length = 10
            dx = arrow_length * math.cos(heading_rad)
            dy = arrow_length * math.sin(heading_rad)
            
            # Remove old arrow and add new one
            nonlocal heading_arrow
            heading_arrow.remove()
            heading_arrow = ax_traj.arrow(x_current, y_current, dx, dy, 
                                         head_width=3, head_length=5, fc='red', ec='black', lw=2, zorder=12)
            
            # Update depth plot
            t_trail = [self.times[self.valid_indices[i]] for i in range(frame + 1)]
            d_trail = [self.depth[self.valid_indices[i]] for i in range(frame + 1) 
                      if not math.isnan(self.depth[self.valid_indices[i]])]
            t_depth = [self.times[self.valid_indices[i]] for i in range(frame + 1) 
                      if not math.isnan(self.depth[self.valid_indices[i]])]
            depth_line.set_data(t_depth, d_trail)
            depth_marker.set_data([self.times[idx]], [self.depth[idx]])
            ax_depth.relim()
            ax_depth.autoscale_view()
            
            # Update speed plot
            s_trail = [self.speed[self.valid_indices[i]] for i in range(frame + 1) 
                      if not math.isnan(self.speed[self.valid_indices[i]])]
            t_speed = [self.times[self.valid_indices[i]] for i in range(frame + 1) 
                      if not math.isnan(self.speed[self.valid_indices[i]])]
            speed_line.set_data(t_speed, s_trail)
            speed_marker.set_data([self.times[idx]], [self.speed[idx]])
            ax_speed.relim()
            ax_speed.autoscale_view()
            
            # Update status text
            status = f"Time: {self.times[idx]:.1f}s\n"
            status += f"Position: ({x_current:.1f}, {y_current:.1f})m\n"
            status += f"Depth: {self.depth[idx]:.1f}m\n"
            status += f"Heading: {self.heading[idx]:.1f}°\n"
            status += f"Speed: {self.speed[idx]:.2f}m/s\n"
            status += f"Task: {self.current_task[idx]}\n"
            status += f"Battery: {self.battery[idx]:.1f}%"
            status_text.set_text(status)
            
            return trail_line, vehicle_marker, depth_line, depth_marker, speed_line, speed_marker, status_text
        
        # Calculate frame interval based on speed multiplier
        # Assuming data was logged at ~10Hz
        interval_ms = (100 / self.speed_multiplier)  # milliseconds per frame
        
        print(f"\nStarting animation...")
        print(f"Total frames: {len(self.valid_indices)}")
        print(f"Frame interval: {interval_ms:.1f}ms")
        
        anim = animation.FuncAnimation(fig, update, frames=len(self.valid_indices),
                                      interval=interval_ms, blit=False, repeat=False)
        
        plt.show()

# ----------------------------
# Main Menu
# ----------------------------

def main():
    print("=" * 70)
    print("SIMPLR-AUV Telemetry Analysis Tool - ALL ANALYSES IMPLEMENTED")
    print("=" * 70)
    
    csv_file = find_latest_csv_file()
    mission_file = find_latest_mission_file()
    
    if not csv_file:
        csv_files = find_all_csv_files()
        if not csv_files:
            print("No CSV files found. Exiting.")
            return
        csv_file = pick_file_from_list(csv_files, "Select telemetry CSV file:")
        if not csv_file:
            return
    
    if not mission_file:
        mission_files = find_all_mission_files()
        if not mission_files:
            print("No mission files found. Exiting.")
            return
        mission_file = pick_file_from_list(mission_files, "Select mission JSON file:")
        if not mission_file:
            return
    
    # Show what files were found and allow user to change
    print(f"\nAuto-selected files:")
    print(f"  Telemetry: {os.path.basename(csv_file)}")
    print(f"  Mission:   {os.path.basename(mission_file)}")
    
    # Show all available mission files for context
    all_missions = find_all_mission_files()
    if len(all_missions) > 1:
        print(f"\nAvailable mission files:")
        for i, mf in enumerate(all_missions):
            marker = " <-- SELECTED" if mf == mission_file else ""
            print(f"    {i+1}. {os.path.basename(mf)}{marker}")
        
        change = input("\nChange mission file? (y/n): ").strip().lower()
        if change == 'y':
            mission_file = pick_file_from_list(all_missions, "Select mission JSON file:")
            if mission_file:
                print(f"Changed to: {os.path.basename(mission_file)}")
    
    oa_file = find_latest_oa_file()
    if oa_file:
        print(f"  OA Log:    {os.path.basename(oa_file)}")
    else:
        print("  OA Log:    (none found)")
    
    data = load_csv(csv_file)
    if not data:
        print("Failed to load telemetry data. Exiting.")
        return
    
    oa_data = load_oa_csv(oa_file) if oa_file else {}
    
    while True:
        print("\n" + "=" * 50)
        print("ANALYSIS OPTIONS:")
        print("  1. Coordinate System Validation")
        print("  2. FOLLOW_TRACK Navigation Analysis (ENHANCED)")
        print("  3. Comprehensive Telemetry Overview")
        print("  4. Navigation Analysis")
        print("  5. Energy Analysis")
        print("  6. Obstacle Avoidance Analysis")
        print("  7. MISSION REPLAY (Animated)")
        print("  8. Change Files")
        print("  9. Exit")
        print("=" * 50)
        
        choice = input("Select option (1-9): ").strip()
        
        try:
            if choice == '1':
                plot_coordinate_system_validation(data, mission_file)
            elif choice == '2':
                plot_follow_track_navigation_analysis(data, mission_file)
            elif choice == '3':
                plot_telemetry_overview(data, mission_file)
            elif choice == '4':
                plot_navigation_analysis(data, mission_file)
            elif choice == '5':
                plot_energy_analysis(data)
            elif choice == '6':
                if not oa_data:
                    print("No OA data loaded.")
                else:
                    plot_obstacle_avoidance_analysis(data, oa_data, mission_file)
            elif choice == '7':
                print("\nSelect playback speed:")
                print("  1. 0.5x  2. 1x  3. 2x  4. 5x  5. 10x")
                speed_choice = input("Speed (1-5): ").strip()
                speed_map = {'1': 0.5, '2': 1.0, '3': 2.0, '4': 5.0, '5': 10.0}
                speed = speed_map.get(speed_choice, 1.0)
                plotter = MissionReplayPlotter(csv_file, mission_file, speed_multiplier=speed)
                plotter.start()
            elif choice == '8':
                csv_files = find_all_csv_files()
                mission_files = find_all_mission_files()
                new_csv = pick_file_from_list(csv_files, "Select new telemetry CSV:")
                if new_csv:
                    csv_file = new_csv
                    data = load_csv(csv_file)
                new_mission = pick_file_from_list(mission_files, "Select new mission JSON:")
                if new_mission:
                    mission_file = new_mission
                new_oa = find_latest_oa_file()
                if new_oa:
                    oa_file = new_oa
                    oa_data = load_oa_csv(oa_file)
            elif choice == '9':
                print("Exiting.")
                break
            else:
                print("Invalid choice.")
        except Exception as e:
            print(f"\nERROR: Plot generation failed!")
            print(f"Error message: {e}")
            import traceback
            print("\nFull traceback:")
            traceback.print_exc()
            print("\nPress Enter to continue...")
            input()

if __name__ == "__main__":
    main()
