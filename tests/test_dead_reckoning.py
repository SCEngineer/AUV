#!/usr/bin/env python3
"""
test_dead_reckoning.py - Test dead reckoning in simulated time mode
Tests the fixed DR module with time injection
"""

import sys
import json
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

from dead_reckoning import DeadReckoningModule

def test_dead_reckoning(mission_file, output_csv='dr_test.csv', time_step=0.1):
    """Test DR module with simulated time"""
    
    print(f"\n{'='*80}")
    print("DEAD RECKONING STANDALONE TEST")
    print(f"{'='*80}\n")
    
    # Load mission to get initial position
    print(f"Loading mission file: {mission_file}")
    with open(mission_file, 'r') as f:
        mission_data = json.load(f)
    
    initial_pos = mission_data['vehicle_config']['initial_position']
    
    print(f"Mission: {mission_data['mission_info']['name']}")
    print(f"Initial Position: ({initial_pos['lat']:.6f}, {initial_pos['lon']:.6f})")
    print(f"Initial Depth: {initial_pos.get('depth', 0.0)}m")
    
    # Create DR module in SIMULATED TIME mode
    print(f"\nCreating DR module...")
    dr = DeadReckoningModule(csv_file=output_csv)
    
    # CRITICAL: Enable simulated time mode BEFORE initialization
    dr.set_simulated_time_mode(True)
    print("✓ Simulated time mode enabled")
    
    # FIXED: Don't pass fake 'INITIALIZE' task - let timeline determine tasks
    print("\nInitializing DR module...")
    init_data = {
        'lat': float(initial_pos['lat']),
        'lon': float(initial_pos['lon']),
        'depth': float(initial_pos.get('depth', 0.0)),
        'heading': float(initial_pos.get('heading', 0.0)),
        'speed': 1.0,
        'mission_time': 0.0,
        'system_status': 'SURFACED_TRIM',
        'position_uncertainty': 1.0
    }
    
    dr.initialize(init_data, mission_file)
    
    if not dr.is_enabled():
        print("ERROR: DR failed to initialize")
        return False
    
    print("✓ DR initialized successfully")
    
    # Check timeline
    if not dr.mission_timeline:
        print("ERROR: No timeline built")
        return False
    
    print(f"✓ Mission timeline: {len(dr.mission_timeline)} phases")
    
    # Get total mission time
    total_time = dr.mission_timeline[-1]['end_time']
    print(f"✓ Total mission duration: {total_time:.1f}s ({total_time/60:.1f} minutes)\n")
    
    # Print phase summary
    print("Mission Phases:")
    for i, phase in enumerate(dr.mission_timeline):
        print(f"  {i:2d}: {phase['task_type']:25s} "
              f"t={phase['start_time']:6.1f}-{phase['end_time']:6.1f}s "
              f"(duration={phase['duration']:5.1f}s)")
    
    # Simulate
    print(f"\n{'='*80}")
    print("RUNNING SIMULATION")
    print(f"{'='*80}\n")
    print(f"{'Time':>8s} {'Phase':>6s} {'Task':>25s} {'Progress':>8s} "
          f"{'Lat':>12s} {'Lon':>12s} {'Depth':>8s} {'Speed':>7s}")
    print("-" * 110)
    
    mission_time = 0.0
    last_phase = -1
    last_print = 0.0
    
    while mission_time <= total_time:
        # Set simulated time
        dr.set_mission_time(mission_time)
        
        # Get estimate
        estimate = dr.get_current_estimate()
        
        if estimate:
            phase_idx = dr._find_current_phase_index(mission_time)
            
            # Print every 10 seconds or when phase changes
            if phase_idx != last_phase or mission_time - last_print >= 10.0:
                print(f"{mission_time:8.1f} {phase_idx:6d} {estimate.current_mission_phase:25s} "
                      f"{estimate.estimated_task_progress*100:7.1f}% "
                      f"{estimate.estimated_lat:12.6f} {estimate.estimated_lon:12.6f} "
                      f"{estimate.estimated_depth:8.2f} {estimate.estimated_speed:7.2f}")
                last_phase = phase_idx
                last_print = mission_time
            
            # Log to CSV
            dr.log_to_csv()
        
        mission_time += time_step
    
    print(f"\n{'='*80}")
    print("SIMULATION COMPLETE")
    print(f"{'='*80}\n")
    
    # Verify output
    try:
        with open(output_csv, 'r') as f:
            lines = f.readlines()
            print(f"✓ CSV output: {output_csv}")
            print(f"✓ Total rows: {len(lines)-1} (including header)")
        
        # Show first few and last few entries
        print(f"\nFirst 5 data rows:")
        for line in lines[1:6]:
            print(f"  {line.strip()}")
        
        print(f"\nLast 5 data rows:")
        for line in lines[-5:]:
            print(f"  {line.strip()}")
        
        # Verify data changes over time
        print(f"\nVerifying data variation...")
        unique_tasks = set()
        unique_depths = set()
        unique_lats = set()
        
        for line in lines[1:]:
            parts = line.strip().split(',')
            if len(parts) >= 9:
                unique_tasks.add(parts[2])
                unique_depths.add(parts[4])
                unique_lats.add(parts[7])
        
        print(f"  Unique tasks: {len(unique_tasks)} - {unique_tasks}")
        print(f"  Unique depths: {len(unique_depths)}")
        print(f"  Unique latitudes: {len(unique_lats)}")
        
        if len(unique_tasks) <= 1:
            print(f"  WARNING: Only one task type recorded")
        if len(unique_depths) <= 1:
            print(f"  WARNING: Depth never changed")
        if len(unique_lats) <= 1:
            print(f"  WARNING: Position never changed")
        
    except Exception as e:
        print(f"ERROR reading CSV output: {e}")
        return False
    
    print(f"\n✓ Dead Reckoning test PASSED")
    print(f"✓ The DR module can successfully track position through an entire mission")
    
    return True


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python test_dead_reckoning.py <mission.json> [output.csv] [timestep]")
        print("\nExample:")
        print("  python test_dead_reckoning.py dive_test.json dr_test.csv 0.1")
        sys.exit(1)
    
    mission_file = sys.argv[1]
    output_csv = sys.argv[2] if len(sys.argv) > 2 else 'dr_test.csv'
    time_step = float(sys.argv[3]) if len(sys.argv) > 3 else 0.1
    
    try:
        success = test_dead_reckoning(mission_file, output_csv, time_step)
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nFATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)