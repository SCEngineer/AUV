#!/usr/bin/env python3
"""
analyze_obstacle_avoidance_composite_final.py
--------------------------------------------
FULLY WORKING version with FILE PICKERS for CSV + JSON
"""

import os
import csv
import json
import sys
import traceback
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from collections import defaultdict
import numpy as np


# -------------------------------------------------
# 1. FILE PICKER (tkinter)
# -------------------------------------------------
def pick_file(title: str, filetypes: list, initialdir: str = None) -> str | None:
    try:
        import tkinter as tk
        from tkinter import filedialog
    except Exception as e:
        print(f"[WARN] No tkinter ({e}) – auto-detect only")
        return None

    root = tk.Tk()
    root.withdraw()
    path = filedialog.askopenfilename(
        title=title,
        filetypes=filetypes,
        initialdir=initialdir or os.getcwd()
    )
    root.destroy()
    return path or None


# -------------------------------------------------
# 2. AUTO-DETECT FALLBACKS
# -------------------------------------------------
def find_latest_csv_fallback() -> str:
    base_dir = os.path.dirname(os.path.abspath(__file__))
    logs_dir = os.path.join(base_dir, "logs")
    if not os.path.exists(logs_dir):
        logs_dir = base_dir
    files = [f for f in os.listdir(logs_dir)
             if f.startswith("obstacle_avoidance_log_") and f.endswith(".csv")]
    if not files:
        raise FileNotFoundError(f"No OA CSV in {logs_dir}")
    files.sort(key=lambda f: os.path.getmtime(os.path.join(logs_dir, f)), reverse=True)
    latest = os.path.join(logs_dir, files[0])
    print(f"[INFO] Auto-detected CSV: {latest}")
    return latest


def find_mission_file_fallback() -> str | None:
    mission_path = os.environ.get('SIMPLR_MISSION_FILE')
    if mission_path and os.path.exists(mission_path):
        print(f"[INFO] Mission from env: {mission_path}")
        return mission_path

    base_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(base_dir, "box_route_with_obstacles.json"),
        os.path.join(base_dir, "missions", "box_route_with_obstacles.json"),
        os.path.join(base_dir, "missions", "sonar_beam_alignment_test_v2.json"),
    ]
    missions_dir = os.path.join(base_dir, "missions")
    if os.path.exists(missions_dir):
        mission_files = [os.path.join(missions_dir, f)
                         for f in os.listdir(missions_dir) if f.endswith('.json')]
        if mission_files:
            mission_files.sort(key=os.path.getmtime, reverse=True)
            candidates.insert(0, mission_files[0])
    for path in candidates:
        if os.path.exists(path):
            print(f"[INFO] Auto-detected mission: {path}")
            return path
    print("[INFO] No mission file – overlay skipped")
    return None


# -------------------------------------------------
# 3. INPUT SELECTION
# -------------------------------------------------
def determine_inputs():
    print("\n" + "=" * 70)
    print("SELECT FILES (Cancel = Auto-Detect)")
    print("=" * 70)

    csv_path = pick_file("Select Obstacle Avoidance CSV", [("CSV", "*.csv")])
    if not csv_path:
        csv_path = find_latest_csv_fallback()

    mission_path = pick_file("Select Mission JSON (Optional)", [("JSON", "*.json")])
    if not mission_path:
        mission_path = find_mission_file_fallback()

    return csv_path, mission_path


# -------------------------------------------------
# 4. LOAD CSV
# -------------------------------------------------
def load_csv(file_path):
    events = []
    with open(file_path, newline='') as f:
        reader = csv.DictReader(f)
        headers = reader.fieldnames
        if headers:
            print(f"[INFO] Columns: {', '.join(headers)}")

        first_ts = None
        for i, row in enumerate(reader, 2):
            try:
                ts = float(row.get("timestamp", 0) or 0)
                if first_ts is None: first_ts = ts
                mission_time = ts - first_ts

                aid = int(float(row.get("avoidance_id", 0) or 0))
                state = row.get("state", "").strip()

                dist = None
                d = row.get("sonar_distance", "")
                if d and d not in ("", "None", "nan"):
                    dist = float(d)

                conf = None
                c = row.get("sonar_confidence", "")
                if c and c not in ("", "None", "nan"):
                    conf = float(c)

                vx = float(row.get("veh_x", 0) or 0)
                vy = float(row.get("veh_y", 0) or 0)

                cx = row.get("carrot_x", "")
                cy = row.get("carrot_y", "")
                carrot_x = float(cx) if cx and cx not in ("", "None") else None
                carrot_y = float(cy) if cy and cy not in ("", "None") else None

                dc = row.get("dist_to_carrot", "")
                dist_to_carrot = float(dc) if dc and dc not in ("", "None") else None

                events.append({
                    "mission_time": mission_time,
                    "avoidance_id": aid,
                    "state": state,
                    "distance": dist,
                    "confidence": conf,
                    "local_x": vx,
                    "local_y": vy,
                    "carrot_x": carrot_x,
                    "carrot_y": carrot_y,
                    "dist_to_carrot": dist_to_carrot
                })
            except Exception as e:
                if i <= 5:
                    print(f"[WARN] Row {i}: {e}")
    print(f"[INFO] Loaded {len(events)} events")
    return events


# -------------------------------------------------
# 5. GROUP & SUMMARIZE
# -------------------------------------------------
def group_by_avoidance(events):
    groups = defaultdict(list)
    for e in events:
        groups[e["avoidance_id"]].append(e)
    for g in groups.values():
        g.sort(key=lambda x: x["mission_time"])
    return groups


def summarize(groups, events):
    print("\n" + "=" * 70)
    print("OBSTACLE AVOIDANCE EVENT SUMMARY")
    print("=" * 70)

    state_counts = defaultdict(int)
    for e in events:
        state_counts[e["state"]] += 1
    total = len(events)
    print("\nState Distribution:")
    for s, c in sorted(state_counts.items()):
        print(f"  {s:15s}: {c:5d} ({c/total*100:5.1f}%)")

    aids = [a for a in groups if a > 0]
    print(f"\nAvoidance Events: {len(aids)}")
    if aids:
        print("\nDetailed Maneuvers:")
        for aid in sorted(aids):
            g = groups[aid]
            t = [e["mission_time"] for e in g]
            d = [e["distance"] for e in g if e["distance"] is not None]
            c = [e["confidence"] for e in g if e["confidence"] is not None]
            cd = [e["dist_to_carrot"] for e in g if e["dist_to_carrot"] is not None]
            states = list(dict.fromkeys(e["state"] for e in g))
            dur = max(t) - min(t)
            print(f"\n  ID {aid}: {dur:.1f}s | {' → '.join(states)}")
            if d:
                print(f"    Sonar: min {min(d):.1f}m, mean {sum(d)/len(d):.1f}m")
            if c:
                print(f"    Conf: mean {sum(c)/len(c):.1f}%")
            if cd:
                print(f"    Final carrot dist: {cd[-1]:.1f}m")

    dists = [e["distance"] for e in events if e["distance"] is not None]
    if dists:
        print(f"\nSonar Stats: min {min(dists):.1f}, max {max(dists):.1f}, "
              f"mean {np.mean(dists):.1f}, median {np.median(dists):.1f}")
    print("=" * 70 + "\n")


# -------------------------------------------------
# 6. PLOTS (FULL WORKING CODE)
# -------------------------------------------------
def plot_composite(groups, events):
    state_colors = {"NORMAL": "#90EE90", "AVOIDING": "#FFD700", "CLEARING": "#FFA500"}
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle("Obstacle Avoidance Mission Timeline", fontsize=14, fontweight="bold")

    # Sonar
    times = [e["mission_time"] for e in events if e["distance"] is not None]
    dists = [e["distance"] for e in events if e["distance"] is not None]
    if times:
        ax1.plot(times, dists, 'k-', lw=1.5, alpha=0.6, label="Sonar Distance")
        current, start = None, None
        for e in events:
            if e["state"] != current:
                if current and start is not None:
                    ax1.axvspan(start, e["mission_time"], color=state_colors.get(current, 'white'), alpha=0.3)
                current, start = e["state"], e["mission_time"]
        if current and start is not None:
            ax1.axvspan(start, events[-1]["mission_time"], color=state_colors.get(current, 'white'), alpha=0.3)
        seen = set()
        for e in events:
            if e["avoidance_id"] > 0 and e["avoidance_id"] not in seen:
                ax1.axvline(e["mission_time"], color='red', ls='--', alpha=0.5)
                ax1.text(e["mission_time"], max(dists)*0.95, f"A{e['avoidance_id']}", rotation=90, fontsize=8, color='red')
                seen.add(e["avoidance_id"])
        ax1.axhline(100, color='gray', ls=':', alpha=0.5, label='Max Range')
        ax1.axhline(65, color='orange', ls=':', alpha=0.5, label='Activation')
        ax1.set_ylabel("Sonar Distance (m)")
        ax1.legend(); ax1.grid(alpha=0.3)

    # Confidence
    ctimes = [e["mission_time"] for e in events if e["confidence"] is not None]
    cvals = [e["confidence"] for e in events if e["confidence"] is not None]
    if ctimes:
        ax2.plot(ctimes, cvals, 'b-', lw=1.5, alpha=0.6, label="Confidence")
        ax2.axhline(60, color='orange', ls='--', alpha=0.5, label='Threshold')
        ax2.set_ylabel("Confidence (%)"); ax2.legend(); ax2.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("oa_all_events.png", dpi=150); plt.close()
    print("[PLOT] oa_all_events.png")


def plot_carrot_chase(events):
    fig, ax = plt.subplots(figsize=(12,10))
    ax.set_title("Carrot Chase Trajectory", fontweight="bold")
    xs, ys = [e["local_x"] for e in events], [e["local_y"] for e in events]
    colors = {"NORMAL":"lightblue", "AVOIDING":"orange", "CLEARING":"green"}
    for i in range(len(events)-1):
        ax.plot([xs[i], xs[i+1]], [ys[i], ys[i+1]], color=colors.get(events[i]["state"], "gray"), lw=2)
    cx = [e["carrot_x"] for e in events if e["carrot_x"] is not None]
    cy = [e["carrot_y"] for e in events if e["carrot_y"] is not None]
    if cx:
        ax.scatter(cx, cy, c='red', s=50, marker='*', label='Carrot', zorder=5)
        for e in events:
            if e["state"] == "AVOIDING" and e["carrot_x"] is not None:
                ax.plot([e["local_x"], e["carrot_x"]], [e["local_y"], e["carrot_y"]], 'r--', alpha=0.3)
    ax.plot(xs[0], ys[0], 'go', ms=12, label='Start')
    ax.plot(xs[-1], ys[-1], 'rs', ms=12, label='End')
    ax.legend(); ax.grid(alpha=0.3); ax.axis('equal')
    plt.tight_layout(); plt.savefig("oa_carrot_chase.png", dpi=150); plt.close()
    print("[PLOT] oa_carrot_chase.png")


def overlay_obstacles(mission_file, events):
    if not mission_file or not os.path.exists(mission_file):
        return
    try:
        with open(mission_file) as f:
            mission = json.load(f)
    except:
        return
    obs = mission.get("obstacles", [])
    if not obs:
        return
    fig, ax = plt.subplots(figsize=(12,10))
    ax.set_title("Obstacle Map with Trajectory", fontweight="bold")
    for o in obs:
        x, y = o["position"].get("local_x", 0), o["position"].get("local_y", 0)
        shape = o.get("shape", "cylinder")
        name = o.get("name", "")
        if shape == "cylinder":
            r = o["dimensions"].get("radius", 5)
            ax.add_patch(plt.Circle((x,y), r, color='red', alpha=0.3, ec='darkred', lw=2))
        elif shape == "box":
            w, l = o["dimensions"].get("width", 10), o["dimensions"].get("length", 10)
            ax.add_patch(mpatches.Rectangle((x-w/2, y-l/2), w, l, color='orange', alpha=0.3, ec='darkorange', lw=2))
        ax.text(x, y, name, ha='center', va='center', fontweight='bold', color='darkred')
    xs = [e["local_x"] for e in events]
    ys = [e["local_y"] for e in events]
    colors = {"NORMAL":"lightblue", "AVOIDING":"orange", "CLEARING":"green"}
    for i in range(len(events)-1):
        ax.plot([xs[i], xs[i+1]], [ys[i], ys[i+1]], color=colors.get(events[i]["state"], "gray"), lw=2)
    ax.plot(xs[0], ys[0], 'go', ms=12, label='Start')
    ax.plot(xs[-1], ys[-1], 'rs', ms=12, label='End')
    cx = [e["carrot_x"] for e in events if e["carrot_x"] is not None]
    cy = [e["carrot_y"] for e in events if e["carrot_y"] is not None]
    if cx:
        ax.scatter(cx, cy, c='purple', s=50, marker='*', label='Carrot')
    ax.legend(loc="upper right"); ax.grid(alpha=0.3); ax.axis('equal')
    plt.tight_layout(); plt.savefig("oa_obstacle_overlay.png", dpi=150); plt.close()
    print("[PLOT] oa_obstacle_overlay.png")


def plot_state_summary(events):
    counts = defaultdict(int)
    colors = {"NORMAL": "#90EE90", "AVOIDING": "#FFD700", "CLEARING": "#FFA500"}
    for e in events:
        counts[e["state"]] += 1
    if not counts:
        return
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14,6))
    fig.suptitle("State Analysis", fontweight="bold")
    states, vals = list(counts.keys()), list(counts.values())
    cols = [colors.get(s, 'gray') for s in states]
    ax1.pie(vals, labels=states, colors=cols, autopct='%1.1f%%', textprops={'fontsize':10})
    ax1.set_title("Distribution")
    ax2.bar(states, vals, color=cols, ec='black')
    ax2.set_ylabel("Count"); ax2.set_title("Occurrences")
    ax2.tick_params(axis='x', rotation=45); ax2.grid(axis='y', alpha=0.3)
    plt.tight_layout(); plt.savefig("oa_state_summary.png", dpi=150); plt.close()
    print("[PLOT] oa_state_summary.png")


# -------------------------------------------------
# 7. MAIN
# -------------------------------------------------
def main():
    print("\n" + "=" * 70)
    print("SIMPLR-AUV OA ANALYZER (with file pickers)")
    print("=" * 70)

    try:
        csv_path, mission_path = determine_inputs()
        events = load_csv(csv_path)
        if not events:
            print("[ERROR] No data"); return 1
        groups = group_by_avoidance(events)
        summarize(groups, events)
        print("Generating plots...")
        plot_composite(groups, events)
        plot_state_summary(events)
        plot_carrot_chase(events)
        overlay_obstacles(mission_path, events)
        print("\nDONE! Check PNGs in script folder.")
        return 0
    except Exception as e:
        print(f"\n[ERROR] {e}")
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())