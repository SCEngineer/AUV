#!/usr/bin/env python3
"""
obstacle_avoidance.py - 3D CARROT CHASE WITH CROSS-TRACK ERROR MINIMIZATION
-----------------------------------------------------------------------------
- Detect obstacle via sonar
- Evaluate avoidance options: LEFT, RIGHT, DIVE, CLIMB
- Select maneuver that minimizes cross-track error from mission route
- Generate 3D carrot (x, y, z) with adaptive radius based on position uncertainty
- Navigate to carrot → return to NORMAL

MISSION-AGNOSTIC DESIGN:
- Algorithm uses ONLY real-time sensor data + mission route waypoints
- Does NOT use a priori obstacle positions from mission file
- Mission obstacles used only for post-mission visualization/analysis

ADAPTIVE WAYPOINT RADIUS:
- Uses position_uncertainty directly from vehicle_state.nav_state
- Scales waypoint acceptance radius based on navigation uncertainty
- Separate horizontal and vertical tolerances
"""

import time
import math
import csv
import os
import json
from typing import Dict, Any, Optional, Tuple, List
from pathlib import Path


class ObstacleAvoidance:
    def __init__(self, vehicle_state, config: Optional[dict] = None, config_file: Optional[str] = None, 
                 mission_file: Optional[str] = None):
        self.vehicle_state = vehicle_state

        # Load configuration
        cfg = {}
        if config_file and Path(config_file).exists():
            try:
                with open(config_file, "r") as f:
                    cfg = json.load(f)
                print(f"[OA] Loaded config: {config_file}")
            except Exception as e:
                print(f"[OA] Failed to load config {config_file}: {e}")
        if config:
            cfg = {**cfg, **config}
        self.config = cfg

        # Load mission file for waypoint/track information (NOT for obstacle positions)
        self.mission = {}
        self.waypoints = []
        if mission_file and Path(mission_file).exists():
            try:
                with open(mission_file, "r") as f:
                    self.mission = json.load(f)
                self._extract_waypoints()
                print(f"[OA] Loaded mission: {mission_file}")
                print(f"[OA] Found {len(self.waypoints)} waypoints for track calculation")
            except Exception as e:
                print(f"[OA] Failed to load mission {mission_file}: {e}")

        detection_cfg = cfg.get("detection", {})
        behavior_cfg  = cfg.get("behavior", {})
        depth_cfg = cfg.get("depth", {})
        speed_cfg     = cfg.get("speed", {})
        logging_cfg   = cfg.get("logging", {})

        # Detection thresholds
        self.max_range = float(detection_cfg.get("max_range", 100.0))
        
        if "activation_distance_factor" in detection_cfg:
            self.activation_distance = self.max_range * float(detection_cfg.get("activation_distance_factor"))
        else:
            self.activation_distance = float(detection_cfg.get("activation_distance", 65.0))
        
        self.activation_confidence = float(detection_cfg.get("activation_confidence", 0.60))

        # Horizontal carrot parameters
        self.lateral_offset = float(behavior_cfg.get("lateral_offset", 55.0))
        
        if "carrot_distance" in behavior_cfg:
            self.carrot_distance = float(behavior_cfg.get("carrot_distance", 40.0))
            self.use_fixed_carrot_distance = True
            self.range_multiplier = 1.2
        else:
            self.range_multiplier = float(behavior_cfg.get("range_multiplier", 1.2))
            self.use_fixed_carrot_distance = False
            self.carrot_distance = 40.0
        
        # Depth avoidance parameters
        self.enable_depth_avoidance = bool(depth_cfg.get("enable_depth_avoidance", True))
        self.vertical_clearance_margin = float(depth_cfg.get("vertical_clearance_margin", 5.0))
        self.horizontal_clearance_margin = float(depth_cfg.get("horizontal_clearance_margin", 10.0))
        self.min_operating_depth = float(depth_cfg.get("min_operating_depth", 2.0))
        self.max_operating_depth = float(depth_cfg.get("max_operating_depth", 50.0))
        self.depth_tolerance = float(depth_cfg.get("depth_tolerance", 2.0))
        
        # Cross-track error weighting for maneuver selection
        self.cross_track_weight = float(behavior_cfg.get("cross_track_weight", 1.0))
        
        # Adaptive waypoint radius parameters
        self.use_adaptive_radius = bool(behavior_cfg.get("use_adaptive_radius", True))
        self.base_waypoint_radius = float(behavior_cfg.get("base_waypoint_radius", 5.0))
        self.max_waypoint_radius = float(behavior_cfg.get("max_waypoint_radius", 25.0))
        self.uncertainty_scale_factor = float(behavior_cfg.get("uncertainty_scale_factor", 0.5))
        self.fixed_waypoint_radius = float(behavior_cfg.get("waypoint_radius", 10.0))

        # Speeds
        self.avoidance_speed = float(speed_cfg.get("avoidance_speed", 0.7))
        self.normal_speed = float(speed_cfg.get("normal_speed", 1.0))

        # State
        self.state = "NORMAL"
        self.carrot_waypoint: Optional[Tuple[float, float, float]] = None  # Now 3D: (x, y, z)
        self.avoidance_id = 1
        self.selected_maneuver = None  # Track which maneuver was selected
        self.nominal_depth = 0.0  # Depth to return to after avoidance
        
        # Logging
        self.csv_logging_enabled = bool(logging_cfg.get("enabled", True))
        self.csv_output_enabled = bool(logging_cfg.get("csv_output", True))
        self.log_directory = logging_cfg.get("log_directory", "logs")
        self.debug_output = bool(logging_cfg.get("debug_output", True))
        self.csv_file = None
        self.csv_writer = None
        self._init_csv_logging()

        print("=" * 70)
        print("OBSTACLE AVOIDANCE - 3D CARROT CHASE WITH CROSS-TRACK MINIMIZATION")
        print(f"  Activation: range < {self.activation_distance:.1f}m, conf > {self.activation_confidence}")
        print(f"  Depth Avoidance: {'ENABLED' if self.enable_depth_avoidance else 'DISABLED'}")
        if self.enable_depth_avoidance:
            print(f"    Vertical clearance: {self.vertical_clearance_margin}m")
            print(f"    Depth limits: {self.min_operating_depth}m - {self.max_operating_depth}m")
            print(f"    Depth tolerance: {self.depth_tolerance}m")
        print(f"  Horizontal clearance: {self.horizontal_clearance_margin}m")
        if self.use_adaptive_radius:
            print(f"  Adaptive waypoint radius:")
            print(f"    Base: {self.base_waypoint_radius}m")
            print(f"    Max:  {self.max_waypoint_radius}m")
            print(f"    Scale factor: {self.uncertainty_scale_factor}")
        print("=" * 70)

    def _extract_waypoints(self):
        """Extract waypoints from mission file for track calculation"""
        if 'vehicle_config' in self.mission and 'initial_position' in self.mission['vehicle_config']:
            initial = self.mission['vehicle_config']['initial_position']
            if 'lat' in initial and 'lon' in initial:
                try:
                    self.waypoints.append({
                        'lat': float(initial['lat']),
                        'lon': float(initial['lon']),
                        'local_x': 0.0,  # Will be set by first waypoint
                        'local_y': 0.0,
                        'type': 'start'
                    })
                except (ValueError, TypeError):
                    pass
        
        for task in self.mission.get('tasks', []):
            task_type = task.get('type', '')
            if task_type == 'SWIM_TO_WAYPOINT' and 'lat' in task and 'lon' in task:
                try:
                    self.waypoints.append({
                        'lat': float(task['lat']),
                        'lon': float(task['lon']),
                        'local_x': task.get('local_x', 0.0),
                        'local_y': task.get('local_y', 0.0),
                        'type': 'waypoint'
                    })
                except (ValueError, TypeError):
                    pass
            elif task_type == 'FOLLOW_TRACK' and 'waypoints' in task:
                for wp in task['waypoints']:
                    try:
                        self.waypoints.append({
                            'lat': float(wp['lat']),
                            'lon': float(wp['lon']),
                            'local_x': wp.get('local_x', 0.0),
                            'local_y': wp.get('local_y', 0.0),
                            'type': 'track_waypoint'
                        })
                    except (ValueError, TypeError, KeyError):
                        pass

    def _init_csv_logging(self):
        if not (self.csv_logging_enabled and self.csv_output_enabled):
            return
        try:
            os.makedirs(self.log_directory, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            path = os.path.join(self.log_directory, f"obstacle_avoidance_log_{ts}.csv")
            self.csv_file = open(path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file, quoting=csv.QUOTE_MINIMAL)
            self.csv_writer.writerow([
                "timestamp", "state", "sonar_distance", "sonar_confidence",
                "veh_x", "veh_y", "veh_z", "veh_heading", "veh_speed",
                "tgt_heading", "tgt_speed", "tgt_depth",
                "carrot_x", "carrot_y", "carrot_z",
                "dist_to_carrot", "cross_track_error", "maneuver",
                "position_uncertainty", "adaptive_radius", "avoidance_id"
            ])
            self.csv_file.flush()
            print(f"[OA] CSV logging → {path}")
        except Exception as e:
            print(f"[OA] CSV init failed: {e}")
            self.csv_logging_enabled = False

    def _log_to_csv(self, control: Dict[str, Any], distance: Optional[float], 
                    confidence: Optional[float], position_uncertainty: float, 
                    adaptive_radius: float, cross_track_error: float):
        if not self.csv_logging_enabled:
            return
        try:
            nav = self.vehicle_state.nav_state
            vx = nav.get("local_x", 0.0)
            vy = nav.get("local_y", 0.0)
            vz = nav.get("depth", 0.0)

            carrot_x = self.carrot_waypoint[0] if self.carrot_waypoint else ""
            carrot_y = self.carrot_waypoint[1] if self.carrot_waypoint else ""
            carrot_z = self.carrot_waypoint[2] if self.carrot_waypoint else ""
            
            dist_to_carrot = ""
            if self.carrot_waypoint:
                dx = self.carrot_waypoint[0] - vx
                dy = self.carrot_waypoint[1] - vy
                dz = self.carrot_waypoint[2] - vz
                dist_to_carrot = f"{math.sqrt(dx*dx + dy*dy + dz*dz):.1f}"

            self.csv_writer.writerow([
                f"{time.time():.3f}", self.state,
                ("" if distance is None else f"{distance:.2f}"),
                ("" if confidence is None else f"{confidence:.2f}"),
                f"{vx:.1f}", f"{vy:.1f}", f"{vz:.1f}",
                f"{nav.get('heading', 0.0):.1f}",
                f"{nav.get('speed', 0.0):.2f}",
                f"{control.get('target_heading', 0.0):.1f}",
                f"{control.get('target_speed', 0.0):.2f}",
                f"{control.get('target_depth', 0.0):.1f}",
                carrot_x, carrot_y, carrot_z, dist_to_carrot,
                f"{cross_track_error:.1f}",
                self.selected_maneuver if self.selected_maneuver else "",
                f"{position_uncertainty:.1f}",
                f"{adaptive_radius:.1f}",
                self.avoidance_id
            ])
            self.csv_file.flush()
        except Exception as e:
            print(f"[OA] CSV log error: {e}")

    def _calculate_adaptive_waypoint_radius(self) -> float:
        """Calculate adaptive waypoint radius based on position uncertainty"""
        if not self.use_adaptive_radius:
            return self.fixed_waypoint_radius
        
        try:
            nav = self.vehicle_state.nav_state
            position_uncertainty = nav.get('position_uncertainty', 10.0)
            
            uncertainty_component = position_uncertainty * self.uncertainty_scale_factor
            adaptive_radius = self.base_waypoint_radius + uncertainty_component
            adaptive_radius = max(self.base_waypoint_radius, min(adaptive_radius, self.max_waypoint_radius))
            
            return adaptive_radius
            
        except Exception as e:
            print(f"[OA] Warning: Error calculating adaptive radius: {e}")
            return self.base_waypoint_radius

    def _get_current_track_segment(self, vx: float, vy: float) -> Optional[Tuple[Dict, Dict, int]]:
        """
        Find the current track segment the vehicle is on or closest to.
        Returns: (waypoint1, waypoint2, segment_index) or None
        """
        if len(self.waypoints) < 2:
            return None
        
        min_dist = float('inf')
        closest_segment = None
        closest_idx = 0
        
        for i in range(len(self.waypoints) - 1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]
            
            # Calculate distance from vehicle to track segment
            x1, y1 = wp1['local_x'], wp1['local_y']
            x2, y2 = wp2['local_x'], wp2['local_y']
            
            # Point-to-line distance
            num = abs((y2 - y1) * vx - (x2 - x1) * vy + x2 * y1 - y2 * x1)
            den = math.sqrt((y2 - y1)**2 + (x2 - x1)**2)
            
            if den > 0:
                dist = num / den
                if dist < min_dist:
                    min_dist = dist
                    closest_segment = (wp1, wp2, i)
                    closest_idx = i
        
        return closest_segment

    def _calculate_cross_track_error(self, vx: float, vy: float) -> float:
        """
        Calculate cross-track error: perpendicular distance from vehicle to track.
        Returns signed distance (positive = right of track, negative = left of track)
        """
        segment = self._get_current_track_segment(vx, vy)
        if not segment:
            return 0.0
        
        wp1, wp2, _ = segment
        x1, y1 = wp1['local_x'], wp1['local_y']
        x2, y2 = wp2['local_x'], wp2['local_y']
        
        # Calculate signed cross-track error
        # Positive = right of track, negative = left
        track_dx = x2 - x1
        track_dy = y2 - y1
        
        to_vehicle_dx = vx - x1
        to_vehicle_dy = vy - y1
        
        # Cross product gives signed distance
        cross = track_dx * to_vehicle_dy - track_dy * to_vehicle_dx
        track_length = math.sqrt(track_dx**2 + track_dy**2)
        
        if track_length > 0:
            return cross / track_length
        return 0.0

    def _evaluate_maneuver_options(self, vx: float, vy: float, vz: float, heading: float, 
                                   detected_range: float) -> List[Dict[str, Any]]:
        """
        Evaluate all possible avoidance maneuvers and their resulting cross-track errors.
        Returns list of maneuver options with scores.
        
        MISSION-AGNOSTIC: Uses only current vehicle state, detected range, and mission track
        """
        options = []
        
        # Calculate forward distance for carrot
        if self.use_fixed_carrot_distance:
            forward_dist = self.carrot_distance
        else:
            forward_dist = detected_range * self.range_multiplier
        
        # Forward direction vector
        theta = math.radians(90.0 - heading)
        forward_x = math.cos(theta)
        forward_y = math.sin(theta)
        
        # Perpendicular vectors (left/right)
        left_x = math.sin(theta)
        left_y = -math.cos(theta)
        right_x = -left_x
        right_y = -left_y
        
        # Option 1: LEFT maneuver
        carrot_left_x = vx + forward_x * forward_dist + left_x * self.lateral_offset
        carrot_left_y = vy + forward_y * forward_dist + left_y * self.lateral_offset
        cte_left = abs(self._calculate_cross_track_error(carrot_left_x, carrot_left_y))
        
        options.append({
            'maneuver': 'LEFT',
            'carrot': (carrot_left_x, carrot_left_y, vz),
            'cross_track_error': cte_left,
            'feasible': True,
            'score': cte_left * self.cross_track_weight
        })
        
        # Option 2: RIGHT maneuver
        carrot_right_x = vx + forward_x * forward_dist + right_x * self.lateral_offset
        carrot_right_y = vy + forward_y * forward_dist + right_y * self.lateral_offset
        cte_right = abs(self._calculate_cross_track_error(carrot_right_x, carrot_right_y))
        
        options.append({
            'maneuver': 'RIGHT',
            'carrot': (carrot_right_x, carrot_right_y, vz),
            'cross_track_error': cte_right,
            'feasible': True,
            'score': cte_right * self.cross_track_weight
        })
        
        if self.enable_depth_avoidance:
            # Option 3: DIVE maneuver (stay on track horizontally, go deeper)
            carrot_dive_x = vx + forward_x * forward_dist
            carrot_dive_y = vy + forward_y * forward_dist
            target_dive_depth = vz + self.vertical_clearance_margin + detected_range * 0.3
            
            # Check if dive is feasible
            dive_feasible = target_dive_depth <= self.max_operating_depth
            cte_dive = abs(self._calculate_cross_track_error(carrot_dive_x, carrot_dive_y))
            
            options.append({
                'maneuver': 'DIVE',
                'carrot': (carrot_dive_x, carrot_dive_y, target_dive_depth),
                'cross_track_error': cte_dive,
                'feasible': dive_feasible,
                'score': cte_dive * self.cross_track_weight if dive_feasible else float('inf')
            })
            
            # Option 4: CLIMB maneuver (stay on track horizontally, go shallower)
            carrot_climb_x = vx + forward_x * forward_dist
            carrot_climb_y = vy + forward_y * forward_dist
            target_climb_depth = max(self.min_operating_depth, vz - self.vertical_clearance_margin)
            
            # Check if climb is feasible
            climb_feasible = target_climb_depth >= self.min_operating_depth
            cte_climb = abs(self._calculate_cross_track_error(carrot_climb_x, carrot_climb_y))
            
            options.append({
                'maneuver': 'CLIMB',
                'carrot': (carrot_climb_x, carrot_climb_y, target_climb_depth),
                'cross_track_error': cte_climb,
                'feasible': climb_feasible,
                'score': cte_climb * self.cross_track_weight if climb_feasible else float('inf')
            })
        
        return options

    def _select_best_maneuver(self, options: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Select the maneuver with the lowest cross-track error that is feasible"""
        feasible_options = [opt for opt in options if opt['feasible']]
        
        if not feasible_options:
            # If no feasible options, default to RIGHT
            print("[OA] WARNING: No feasible maneuvers, defaulting to RIGHT")
            return next(opt for opt in options if opt['maneuver'] == 'RIGHT')
        
        # Select option with minimum score (lowest cross-track error)
        best_option = min(feasible_options, key=lambda x: x['score'])
        return best_option

    def update_sonar(self, sonar_pkt: Optional[dict] = None):
        """Record latest sonar reading."""
        pass

    def _calculate_heading_to(self, vx: float, vy: float, wx: float, wy: float) -> float:
        """Calculate heading to waypoint."""
        dx = wx - vx
        dy = wy - vy
        math_bearing = math.degrees(math.atan2(dy, dx)) % 360.0
        nautical_bearing = (90.0 - math_bearing) % 360.0
        return nautical_bearing

    def update(self, dt: float, sonar_distance: Optional[float] = None, 
              sonar_confidence: Optional[float] = None) -> Dict[str, Any]:
        """Main update - detect, evaluate, select maneuver, navigate to 3D carrot"""
        nav = self.vehicle_state.nav_state
        vx = nav.get("local_x", 0.0)
        vy = nav.get("local_y", 0.0)
        vz = nav.get("depth", 0.0)
        heading = nav.get("heading", 0.0)
        position_uncertainty = nav.get("position_uncertainty", 10.0)

        # Calculate cross-track error
        cross_track_error = self._calculate_cross_track_error(vx, vy)

        control = {
            "active": False,
            "state": self.state,
            "target_heading": heading,
            "target_speed": self.normal_speed,
            "target_depth": vz,
            "target_lat": None,
            "target_lon": None,
            "avoidance_id": self.avoidance_id,
        }

        # Calculate current adaptive waypoint radius
        current_waypoint_radius = self._calculate_adaptive_waypoint_radius()

        # NORMAL: check for detection
        if self.state == "NORMAL":
            if (sonar_distance is not None and sonar_confidence is not None and
                sonar_distance < self.activation_distance and 
                sonar_confidence > self.activation_confidence):
                
                # Store nominal depth to return to
                self.nominal_depth = vz
                
                # DETECT → Evaluate all maneuver options
                options = self._evaluate_maneuver_options(vx, vy, vz, heading, sonar_distance)
                
                # Select best maneuver (minimum cross-track error)
                best_option = self._select_best_maneuver(options)
                
                self.carrot_waypoint = best_option['carrot']
                self.selected_maneuver = best_option['maneuver']
                self.state = "AVOIDING"
                
                if self.debug_output:
                    cx, cy, cz = self.carrot_waypoint
                    dist_2d = math.hypot(cx - vx, cy - vy)
                    dist_3d = math.sqrt((cx - vx)**2 + (cy - vy)**2 + (cz - vz)**2)
                    hdg_to_carrot = self._calculate_heading_to(vx, vy, cx, cy)
                    
                    print(f"\n{'='*70}")
                    print(f"[OA] DETECTION @ {sonar_distance:.1f}m → AVOIDING (ID {self.avoidance_id})")
                    print(f"  Vehicle Position: ({vx:.1f}, {vy:.1f}, {vz:.1f}m)")
                    print(f"  Vehicle Heading:  {heading:.1f}°")
                    print(f"  Cross-Track Error: {cross_track_error:+.1f}m")
                    print(f"\n  Evaluated {len(options)} maneuver options:")
                    for opt in sorted(options, key=lambda x: x['score']):
                        status = "SELECTED" if opt['maneuver'] == self.selected_maneuver else ""
                        feasible = "✓" if opt['feasible'] else "✗"
                        print(f"    {opt['maneuver']:8s} {feasible} CTE: {opt['cross_track_error']:6.1f}m  Score: {opt['score']:6.1f}  {status}")
                    
                    print(f"\n  SELECTED: {self.selected_maneuver}")
                    print(f"  Carrot Position:   ({cx:.1f}, {cy:.1f}, {cz:.1f}m)")
                    print(f"  Heading to Carrot: {hdg_to_carrot:.1f}°")
                    print(f"  Distance (2D):     {dist_2d:.1f}m")
                    print(f"  Distance (3D):     {dist_3d:.1f}m")
                    print(f"  Waypoint Radius:   {current_waypoint_radius:.1f}m (adaptive)")
                    print(f"{'='*70}\n")

        # AVOIDING: navigate to 3D carrot
        elif self.state == "AVOIDING":
            control["active"] = True
            control["target_speed"] = self.avoidance_speed
            
            if self.carrot_waypoint:
                cx, cy, cz = self.carrot_waypoint
                
                # Steer toward carrot (horizontal)
                hdg_to_carrot = self._calculate_heading_to(vx, vy, cx, cy)
                control["target_heading"] = hdg_to_carrot
                
                # Command depth
                control["target_depth"] = cz
                
                # Check if reached (3D distance)
                dx = cx - vx
                dy = cy - vy
                dz = cz - vz
                dist_horizontal = math.hypot(dx, dy)
                dist_3d = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # Debug output every 2 seconds
                if self.debug_output and int(time.time() * 10) % 20 == 0:
                    print(f"[OA] AVOIDING ({self.selected_maneuver}):")
                    print(f"  Vehicle: ({vx:.1f}, {vy:.1f}, {vz:.1f}m) @ {heading:.1f}°")
                    print(f"  Carrot:  ({cx:.1f}, {cy:.1f}, {cz:.1f}m)")
                    print(f"  Distance (horiz): {dist_horizontal:.1f}m | Depth error: {abs(dz):.1f}m")
                    print(f"  Cross-track error: {cross_track_error:+.1f}m")
                    print(f"  Adaptive radius: {current_waypoint_radius:.1f}m")
                
                # Check if carrot reached (horizontal within radius AND depth within tolerance)
                if dist_horizontal < current_waypoint_radius and abs(dz) < self.depth_tolerance:
                    if self.debug_output:
                        print(f"\n{'='*70}")
                        print(f"[OA] CARROT REACHED → NORMAL (ID {self.avoidance_id} complete)")
                        print(f"  Maneuver: {self.selected_maneuver}")
                        print(f"  Final Position: ({vx:.1f}, {vy:.1f}, {vz:.1f}m)")
                        print(f"  Distance from carrot: {dist_3d:.1f}m")
                        print(f"  Final cross-track error: {cross_track_error:+.1f}m")
                        print(f"{'='*70}\n")
                    
                    self.state = "NORMAL"
                    self.carrot_waypoint = None
                    self.selected_maneuver = None
                    self.avoidance_id += 1

        control["state"] = self.state
        self._log_to_csv(control, sonar_distance, sonar_confidence, 
                        position_uncertainty, current_waypoint_radius, cross_track_error)
        return control

    def shutdown(self):
        if self.csv_file:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:
                pass
        print("[OA] Shutdown complete")
