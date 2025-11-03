#!/usr/bin/env python3
"""
simulated_sonar.py - PHYSICALLY REALISTIC BEAM MODEL
Realistic forward-looking sonar with single detection per ping and proper beam physics
No "see-through" detection - only nearest obstacle within beam volume is detected
"""

import math
import json
from typing import Dict, Any, List, Optional, Tuple


class SimulatedSonar:
    """Simulates a forward-looking single-beam sonar with realistic beam physics."""
    
    def __init__(self, config: Optional[Dict[str, Any]] = None, mission_file: str = None):
        config = config or {}
        
        # Sonar parameters
        self.max_range = float(config.get('max_range', 100.0))
        self.min_range = float(config.get('min_range', 0.3))
        self.beam_width = float(config.get('beam_width', 25.0))  # 3dB beamwidth in degrees
        self.vertical_beam_width = float(config.get('vertical_beam_width', 25.0))
        self.mount_angle = float(config.get('mount_angle', 0.0))
        self.debug = bool(config.get('debug', False))
        
        # Internal state
        self.obstacles: List[Dict[str, Any]] = []
        self.last_distance = self.max_range
        self.last_confidence = 0.0
        self.last_detection: Optional[Dict] = None

        # Load mission obstacles if provided
        if mission_file:
            self._load_mission_obstacles(mission_file)
        
        self._print_init_summary()
    
    def _load_mission_obstacles(self, mission_file: str):
        """Load obstacles from mission JSON with local_x/local_y"""
        try:
            with open(mission_file, 'r') as f:
                mission = json.load(f)
            raw_obstacles = mission.get("obstacles", [])
            
            self.obstacles = []
            for obs in raw_obstacles:
                pos = obs.get("position", {})
                if 'local_x' in pos and 'local_y' in pos:
                    self.obstacles.append(obs)
                else:
                    name = obs.get("name", "Unknown")
                    print(f"[Sonar] WARNING: Skipping obstacle '{name}' - missing local_x/local_y")
            
            print(f"[Sonar] Loaded {len(self.obstacles)} valid obstacle(s) from mission")
        except Exception as e:
            print(f"[Sonar] ERROR: Failed to load mission file '{mission_file}': {e}")
            self.obstacles = []

    def _print_init_summary(self):
        print("=" * 70)
        print("REALISTIC SONAR BEAM MODEL INITIALIZED")
        print("=" * 70)
        print(f"  Max Range:          {self.max_range:.1f} m")
        print(f"  Min Range:          {self.min_range:.1f} m")
        print(f"  Horizontal Beam:    ±{self.beam_width/2:.1f}°")
        print(f"  Vertical Beam:      ±{self.vertical_beam_width/2:.1f}°")
        print(f"  Mount Angle:        {self.mount_angle:.1f}°")
        print(f"  Obstacles Loaded:   {len(self.obstacles)}")
        print(f"  Debug Mode:         {'ON' if self.debug else 'OFF'}")
        print(f"  Beam Physics:       SINGLE DETECTION PER PING")
        print(f"  Occlusion:          ENABLED (no see-through)")
        print(f"  Confidence Model:   BEAM PATTERN TAPER")
        print("=" * 70)

    def load_obstacles(self, obstacles: List[Dict[str, Any]]):
        """Manually load obstacles (alternative to mission file)"""
        self.obstacles = []
        for obs in obstacles:
            pos = obs.get("position", {})
            if 'local_x' in pos and 'local_y' in pos:
                self.obstacles.append(obs)
        print(f"[Sonar] Manually loaded {len(self.obstacles)} obstacle(s)")

    def wrap_deg(self, angle_deg: float) -> float:
        """Wrap angle to [-180, 180) range"""
        return ((angle_deg + 180.0) % 360.0) - 180.0

    def unit_vec_from_heading_deg(self, heading_deg: float) -> Tuple[float, float]:
        """Get unit vector from nautical heading (0°=North, 90°=East)"""
        # Convert nautical to mathematical for trig functions
        math_heading = (90 - heading_deg) % 360
        th = math.radians(math_heading)
        return math.cos(th), math.sin(th)

    def ray_circle_intersection(self, px: float, py: float, vx: float, vy: float, 
                               cx: float, cy: float, r: float) -> Optional[float]:
        """
        Ray from P in direction V (unit vector) vs circle center C with radius r.
        Returns the smallest positive s (range along ray) or None if no hit.
        """
        dx, dy = cx - px, cy - py
        t = dx * vx + dy * vy  # projection along the ray
        
        if t < 0:
            return None  # circle is behind the ray origin
        
        # perpendicular distance from ray to circle center
        d2 = dx*dx + dy*dy - t*t
        if d2 > r*r:
            return None  # ray misses the circle
        
        # entry point along the ray
        dt = math.sqrt(max(r*r - d2, 0.0))
        s = t - dt
        return s if s >= 0 else (t + dt)

    def ray_box_intersection(self, px: float, py: float, vx: float, vy: float,
                            cx: float, cy: float, width: float, length: float) -> Optional[float]:
        """
        Ray from P in direction V vs axis-aligned box centered at C.
        Returns intersection distance or None.
        """
        # Box bounds in ENU coordinates
        min_x, max_x = cx - width/2, cx + width/2
        min_y, max_y = cy - length/2, cy + length/2

        t_near = float('-inf')
        t_far = float('inf')

        # Check X-axis slabs
        if abs(vx) < 1e-6:
            if px < min_x or px > max_x:
                return None
        else:
            t1 = (min_x - px) / vx
            t2 = (max_x - px) / vx
            t_near = max(t_near, min(t1, t2))
            t_far = min(t_far, max(t1, t2))

        # Check Y-axis slabs  
        if abs(vy) < 1e-6:
            if py < min_y or py > max_y:
                return None
        else:
            t1 = (min_y - py) / vy
            t2 = (max_y - py) / vy
            t_near = max(t_near, min(t1, t2))
            t_far = min(t_far, max(t1, t2))

        if t_far < 0 or t_near > t_far:
            return None
        
        t = t_near if t_near >= 0 else t_far
        return t if t >= 0 else None

    def get_distance_with_info(self, vehicle_position: Dict[str, float], 
                              vehicle_heading: float) -> Tuple[float, Dict]:
        """
        Realistic sonar ping - single detection per ping with beam physics.
        
        Args:
            vehicle_position: dict with 'x'/'local_x', 'y'/'local_y', 'depth'
            vehicle_heading: degrees in NAUTICAL convention (0 = North, 90 = East)
        
        Returns:
            (distance, {"confidence": 0-100, "obstacle_id": str, "bearing": float})
        """
        if not self.obstacles:
            return self.max_range, {"confidence": 0.0}

        # Extract vehicle position
        veh_x = vehicle_position.get('x', vehicle_position.get('local_x', 0.0))
        veh_y = vehicle_position.get('y', vehicle_position.get('local_y', 0.0))
        veh_depth = vehicle_position.get('depth', 0.0)
        heading = vehicle_heading % 360.0

        # Get beam direction vector
        beam_vx, beam_vy = self.unit_vec_from_heading_deg(heading)
        half_bw = self.beam_width / 2.0

        nearest_detection = None  # (range, obstacle, relative_bearing)

        for obs in self.obstacles:
            pos = obs.get('position', {})
            obs_x, obs_y = pos.get('local_x', 0.0), pos.get('local_y', 0.0)
            
            # Depth check
            dims = obs.get('dimensions', {})
            obs_depth_top = dims.get('depth_top', 0.0)
            obs_depth_bottom = dims.get('depth_bottom', 100.0)
            
            # Calculate vertical beam coverage at rough distance estimate
            rough_dist = math.hypot(obs_x - veh_x, obs_y - veh_y)
            vertical_half_angle_rad = math.radians(self.vertical_beam_width / 2)
            beam_vertical_extent = rough_dist * math.tan(vertical_half_angle_rad)
            
            beam_depth_top = veh_depth - beam_vertical_extent
            beam_depth_bottom = veh_depth + beam_vertical_extent
            
            if beam_depth_bottom < obs_depth_top or beam_depth_top > obs_depth_bottom:
                continue  # No depth overlap

            # Relative bearing check
            dx, dy = obs_x - veh_x, obs_y - veh_y
            bearing_to_obs = math.degrees(math.atan2(dy, dx)) % 360.0  # Mathematical bearing
            bearing_nautical = (90 - bearing_to_obs) % 360.0  # Convert to nautical
            rel_bearing = self.wrap_deg(bearing_nautical - heading)
            
            if abs(rel_bearing) > half_bw:
                continue  # Outside horizontal beam

            # Ray intersection based on obstacle shape
            shape = obs.get('shape', 'box').lower()
            if shape == 'cylinder':
                radius = float(dims.get('radius', 5.0))
                range_m = self.ray_circle_intersection(veh_x, veh_y, beam_vx, beam_vy, 
                                                      obs_x, obs_y, radius)
            else:  # box
                width = float(dims.get('width', 10.0))
                length = float(dims.get('length', 10.0))
                range_m = self.ray_box_intersection(veh_x, veh_y, beam_vx, beam_vy,
                                                   obs_x, obs_y, width, length)

            if range_m is None or range_m < self.min_range or range_m > self.max_range:
                continue

            # Keep nearest detection only (real beam physics - no see-through)
            if nearest_detection is None or range_m < nearest_detection[0]:
                nearest_detection = (range_m, obs, rel_bearing)

        # No detection
        if nearest_detection is None:
            self.last_distance = self.max_range
            self.last_confidence = 0.0
            self.last_detection = None
            return self.max_range, 0.0

        range_m, detected_obs, rel_bearing = nearest_detection

        # Beam pattern confidence (Gaussian-like taper)
        angle_ratio = abs(rel_bearing) / half_bw
        confidence = max(0.0, 1.0 - (angle_ratio ** 2))  # 100% at center, 0% at edges
        
        # Convert to percentage and apply range-based attenuation
        confidence_pct = confidence * 100.0
        range_att_factor = max(0.5, 1.0 - (range_m / self.max_range) * 0.3)  # Minor range effect
        confidence_pct *= range_att_factor

        self.last_distance = range_m
        self.last_confidence = confidence_pct
        self.last_detection = detected_obs

        if self.debug and confidence_pct > 30:
            name = detected_obs.get('name', 'Obstacle')
            print(f"[Sonar] DETECTED {name} @ {range_m:.1f}m | "
                  f"conf={confidence_pct:.1f}% | rel_bearing={rel_bearing:.1f}°")
            print(f"[Sonar] Vehicle: ({veh_x:.1f}E, {veh_y:.1f}N, {veh_depth:.1f}m) "
                  f"| Heading: {heading:.1f}°N")

        return range_m, confidence_pct

    def get_status(self) -> Dict[str, Any]:
        """Return sonar status for debugging"""
        return {
            'max_range': self.max_range,
            'last_distance': self.last_distance,
            'last_confidence': self.last_confidence,
            'obstacles_loaded': len(self.obstacles),
            'last_detection': self.last_detection.get('name', 'None') if self.last_detection else 'None',
            'debug_mode': self.debug,
            'beam_physics': 'realistic_single_detection',
            'occlusion_model': 'enabled'
        }

    def set_debug(self, enabled: bool):
        """Enable/disable debug prints"""
        self.debug = enabled
        print(f"[Sonar] Debug mode: {'ENABLED' if enabled else 'DISABLED'}")


# ----------------------------------------------------------------------
# TEST - Verify realistic beam behavior
# ----------------------------------------------------------------------
if __name__ == "__main__":
    # Create test mission
    test_mission = {
        "obstacles": [
            {
                "name": "Close Obstacle",
                "position": {"local_x": 0.0, "local_y": 30.0},  # 30m North
                "dimensions": {"radius": 5.0, "depth_top": 5.0, "depth_bottom": 15.0},
                "shape": "cylinder"
            },
            {
                "name": "Far Obstacle", 
                "position": {"local_x": 0.0, "local_y": 60.0},  # 60m North (behind close one)
                "dimensions": {"radius": 5.0, "depth_top": 5.0, "depth_bottom": 15.0},
                "shape": "cylinder"
            }
        ]
    }
    with open("test_mission.json", "w") as f:
        json.dump(test_mission, f, indent=2)

    # Initialize sonar
    sonar = SimulatedSonar(config={"max_range": 150.0, "debug": True}, mission_file="test_mission.json")

    print("\nTesting REALISTIC BEAM PHYSICS...")
    
    # Test: Should only detect CLOSE obstacle, not far one (beam occlusion)
    print("\nTEST: Heading North (0°) - should detect ONLY close obstacle")
    pos = {"local_x": 0.0, "local_y": 0.0, "depth": 10.0}
    dist, info = sonar.get_distance_with_info(pos, 0.0)
    
    expected_close = 25.0  # 30m - 5m radius
    if abs(dist - expected_close) < 2.0:
        print(f"✓ PASS: Detected close obstacle @ {dist:.1f}m (expected ~{expected_close}m)")
        print(f"  Confidence: {info['confidence']:.1f}%")
        if info.get('obstacle_id') == 'Close Obstacle':
            print("✓ PASS: Correctly identified close obstacle (beam occlusion working)")
        else:
            print("✗ FAIL: Wrong obstacle detected")
    else:
        print(f"✗ FAIL: Unexpected range {dist:.1f}m")
    
    print("\nBeam physics test complete.")