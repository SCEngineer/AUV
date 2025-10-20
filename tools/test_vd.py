#!/usr/bin/env python3
"""
test_vd.py - Test harness for SIMPLR-AUV vehicle_dynamics with EnhancedHAL

Runs the VehicleDynamics model with EnhancedHAL for ballast simulation, prints state evolution,
logs to a single CSV, and generates SVG plots without matplotlib.
"""

import time
import math
import sys
import csv

# Import the required modules
try:
    import vehicle_dynamics
    import vehicle_state
    import enhanced_hal
except ImportError as e:
    print("❌ Could not import module:", e)
    sys.exit(1)

def run_simulation(v_state, hal, dynamics, command, dt, sim_steps, csv_writer, case_name):
    """Run a single simulation case with HAL and return state history"""
    print(f"\nRunning simulation case: {case_name}")
    
    times = []
    positions_x = []
    positions_y = []
    speeds = []
    depths = []
    headings = []

    try:
        for step in range(sim_steps):
            # Advance simulation using HAL
            hal.update(dt)

            # Retrieve state from dynamics
            state = dynamics.get_state()

            # Log to CSV with test case identifier and correct buoyancy_state
            with v_state._state_lock:
                csv_writer.writerow([
                    case_name,
                    step * dt,
                    state['x'],
                    state['y'],
                    state['z'],
                    state['speed'],
                    state['depth'],
                    state['heading'],
                    v_state.buoyancy_state  # Use computed buoyancy_state from VehicleState
                ])

            # Store state
            times.append(step * dt)
            positions_x.append(state['x'])
            positions_y.append(state['y'])
            speeds.append(state['speed'])
            depths.append(state['depth'])
            headings.append(state['heading'])

            # Enhanced debug: Print current ballast state for verification
            status = v_state.get_status()
            ballast_state = status['ballast_state']
            print(
                f"[t={step*dt:5.1f}s] "
                f"pos=({state['x']:.2f}, {state['y']:.2f}, {state['z']:.2f}) "
                f"heading={state['heading']:.1f}° "
                f"speed={state['speed']:.2f} m/s "
                f"depth={state['depth']:.2f} m "
                f"buoyancy={v_state.buoyancy_state} "
                f"tank_volume={ballast_state['tank_volume']:.6f} m³ "
                f"tank_status={ballast_state['tank_status']} "
                f"target_volume={ballast_state['tank_capacity'] * 0.99:.6f} m³"
            )

            time.sleep(0.1)  # slow down printing

        print(f"✅ Simulation case '{case_name}' completed.")
        print(f"Final position: ({positions_x[-1]:.2f}, {positions_y[-1]:.2f}, {depths[-1]:.2f}) m")
        print(f"Final speed: {speeds[-1]:.2f} m/s")
        print(f"Final heading: {headings[-1]:.1f}°")

        return times, positions_x, positions_y, speeds, depths, headings

    except Exception as e:
        print(f"❌ Simulation case '{case_name}' failed: {e}")
        return None

def generate_svg_plot(results, filename="simulation_results.svg"):
    """Generate SVG plots for all test cases without matplotlib"""
    if not results:  # Safeguard for empty results
        print("⚠️ No simulation results to plot. Skipping SVG generation.")
        return

    width, height = 800, 600
    margin = 50
    plot_height = (height - 2 * margin) / 4  # Four plots: x-y, speed, depth, heading

    svg = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">',
        '<rect width="100%" height="100%" fill="white"/>'
    ]

    colors = ["blue", "red", "green", "purple"]
    max_time = max([max(res[1][0]) for res in results if res[1]])  # Max time across all cases

    # Normalize data for plotting
    def normalize(data, max_val, min_val, plot_range):
        if max_val == min_val:
            return [plot_range / 2] * len(data)
        return [(x - min_val) / (max_val - min_val) * plot_range for x in data]

    # Plot 1: X-Y trajectory
    svg.append(f'<text x="{margin}" y="{margin - 10}" font-size="14">X-Y Trajectory</text>')
    all_x = [x for res in results if res[1] for x in res[1][1]]
    all_y = [y for res in results if res[1] for y in res[1][2]]
    max_x, min_x = max(all_x, default=1), min(all_x, default=0)
    max_y, min_y = max(all_y, default=1), min(all_y, default=0)

    for i, (case_name, (times, pos_x, pos_y, _, _, _)) in enumerate(results):
        if not times:
            continue
        norm_x = normalize(pos_x, max_x, min_x, width - 2 * margin)
        norm_y = normalize(pos_y, max_y, min_y, plot_height - 20)
        points = [f"{margin + x},{margin + plot_height - y}" for x, y in zip(norm_x, norm_y)]
        svg.append(f'<polyline points="{" ".join(points)}" stroke="{colors[i % len(colors)]}" fill="none" stroke-width="2"/>')
        svg.append(f'<text x="{width - margin}" y="{margin + i * 15}" font-size="12" fill="{colors[i % len(colors)]}">{case_name}</text>')

    # Plot 2: Speed vs Time
    svg.append(f'<text x="{margin}" y="{margin + plot_height - 10}" font-size="14">Speed vs Time</text>')
    all_speeds = [s for res in results if res[1] for s in res[1][3]]
    max_speed = max(all_speeds, default=1)
    for i, (case_name, (times, _, _, speeds, _, _)) in enumerate(results):
        if not times:
            continue
        norm_speeds = normalize(speeds, max_speed, 0, plot_height - 20)
        points = [f"{margin + (t / max_time) * (width - 2 * margin)},{margin + plot_height + plot_height - norm_speeds[j]}" for j, t in enumerate(times)]
        svg.append(f'<polyline points="{" ".join(points)}" stroke="{colors[i % len(colors)]}" fill="none" stroke-width="2"/>')

    # Plot 3: Depth vs Time
    svg.append(f'<text x="{margin}" y="{margin + 2 * plot_height - 10}" font-size="14">Depth vs Time</text>')
    all_depths = [d for res in results if res[1] for d in res[1][4]]
    max_depth = max(all_depths, default=1)
    for i, (case_name, (times, _, _, _, depths, _)) in enumerate(results):
        if not times:
            continue
        norm_depths = normalize(depths, max_depth, 0, plot_height - 20)
        points = [f"{margin + (t / max_time) * (width - 2 * margin)},{margin + 2 * plot_height + plot_height - norm_depths[j]}" for j, t in enumerate(times)]
        svg.append(f'<polyline points="{" ".join(points)}" stroke="{colors[i % len(colors)]}" fill="none" stroke-width="2"/>')

    # Plot 4: Heading vs Time
    svg.append(f'<text x="{margin}" y="{margin + 3 * plot_height - 10}" font-size="14">Heading vs Time</text>')
    all_headings = [h for res in results if res[1] for h in res[1][5]]
    max_heading = max(all_headings, default=360)
    for i, (case_name, (times, _, _, _, _, headings)) in enumerate(results):
        if not times:
            continue
        norm_headings = normalize(headings, max_heading, 0, plot_height - 20)
        points = [f"{margin + (t / max_time) * (width - 2 * margin)},{margin + 3 * plot_height + plot_height - norm_headings[j]}" for j, t in enumerate(times)]
        svg.append(f'<polyline points="{" ".join(points)}" stroke="{colors[i % len(colors)]}" fill="none" stroke-width="2"/>')

    svg.append('</svg>')
    with open(filename, 'w') as f:
        f.write('\n'.join(svg))
    print(f"SVG plot saved to {filename}")

def main():
    print("Starting vehicle_dynamics simulation with EnhancedHAL...")

    # Simulation settings
    dt = 0.1        # Timestep for finer resolution [s]
    sim_steps = 500 # Steps to ensure tank fills (50 s total)
    log_file = "simulation_log.csv"  # Single CSV file
    config = {"mode": "simulation"}  # Config for HAL

    # Test cases
    test_cases = [
        {
            "name": "Neutral Buoyancy, Straight Path",
            "command": {
                "thruster_cmd": 0.6,
                "UL": 0.0,
                "UR": 0.0,
                "LL": 0.0,
                "LR": 0.0,
                "ballast_cmd": "FILL"  # Full tank → NEUTRAL buoyancy
            }
        },
        {
            "name": "Positive Buoyancy, Straight Path",
            "command": {
                "thruster_cmd": 0.6,
                "UL": 0.0,
                "UR": 0.0,
                "LL": 0.0,
                "LR": 0.0,
                "ballast_cmd": "EMPTY"  # Empty tank → POSITIVE buoyancy
            }
        },
        {
            "name": "Turning with Neutral Buoyancy",
            "command": {
                "thruster_cmd": 0.6,
                "UL": 50.0,
                "UR": -50.0,
                "LL": 50.0,
                "LR": -50.0,
                "ballast_cmd": "FILL"  # Full tank → NEUTRAL buoyancy
            }
        },
        {
            "name": "Diving with Neutral Buoyancy",
            "command": {
                "thruster_cmd": 0.6,
                "UL": 50.0,
                "UR": 50.0,
                "LL": -50.0,
                "LR": -50.0,
                "ballast_cmd": "FILL"  # Full tank → NEUTRAL buoyancy
            }
        }
    ]

    all_results = []

    try:
        # Initialize CSV log with headers
        with open(log_file, 'w', newline='') as f:
            csv_writer = csv.writer(f)
            csv_writer.writerow(["test_case", "time", "x", "y", "z", "speed", "depth", "heading", "buoyancy_state"])

        # Open CSV file in append mode for simulation
        with open(log_file, 'a', newline='') as f:
            csv_writer = csv.writer(f)

            for case in test_cases:
                # Initialize fresh state, HAL, and dynamics for each case
                v_state = vehicle_state.VehicleState()
                hal = enhanced_hal.EnhancedHAL(config, v_state)
                dynamics = vehicle_dynamics.VehicleDynamics(v_state)  # Note: dt is set in HAL init

                # Set initial command to trigger HAL updates
                v_state.actuator_commands.update(case["command"])
                
                # Run simulation
                result = run_simulation(
                    v_state, hal, dynamics, case["command"], dt, sim_steps, csv_writer, case["name"]
                )
                if result:
                    all_results.append((case["name"], result))

        print("\nSimulation completed. Results logged to simulation_log.csv.")
        generate_svg_plot(all_results)
        print("Use simulation_log.csv for data analysis and simulation_results.svg for visualization.")

    except Exception as e:
        print(f"❌ Simulation failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()