#!/usr/bin/env python3
"""
simulated_sonar.py - Enhanced Simulated Ping Sonar for SIMPLR AUV

FIXED: Gracefully handles missions without obstacles
"""

import math
from typing import Dict, Any, List, Optional, Tuple


class SimulatedSonar:
    """Simulates a forward-looking single-beam sonar for obstacle detection."""
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        config = config or {}
        
        # Sonar characteristics
        self.max_range = config.get('max_range', 100.0)
        self.min_range = config.get('min_range', 0.3)
        self.beam_width = config.get('beam_width', 25.0)
        self.frequency = config.get('frequency', 115)
        self.mount_angle = config.get('mount_angle', 0.0)
        self.vertical_beam_width = config.get('vertical_beam_width', 25.0)
        
        # Obstacles - FIXED: Always initialize as empty list
        self.obstacles = []
        
        # Current sonar reading
        self.last_distance = self.max_range
        self.last_confidence = 0.0
        self.last_detection = None
        
        # Debug mode
        self.debug = config.get('debug', False)
        
        print("=" * 60)
        print("ENHANCED SIMULATED SONAR INITIALIZED")
        print("=" * 60)
        print(f"  Max range: {self.max_range}m")
        print(f"  Min range: {self.min_range}m")
        print(f"  Horizontal beam width: {self.beam_width}°")
        print(f"  Vertical beam width: {self.vertical_beam_width}°")
        print(f"  Mount angle: {self.mount_angle}° (0=horizontal, -=down, +=up)")
        print(f"  Frequency: {self.frequency} kHz")
        print(f"  Debug mode: {'ON' if self.debug else 'OFF'}")
        print("=" * 60)
    
    def load_obstacles(self, obstacles: Optional[List[Dict[str, Any]]]):
        """
        Load obstacles from mission file.
        
        FIXED: Handles None, empty list, or missing obstacles gracefully
        
        Args:
            obstacles: List of obstacle definitions from mission JSON (can be None)
        """
        # FIXED: Handle None or empty obstacles
        if obstacles is None or not obstacles:
            self.obstacles = []
            print(f"\nSimulated Sonar: No obstacles in mission (obstacle avoidance disabled)")
            return
        
        # Validate that obstacles is actually a list
        if not isinstance(obstacles, list):
            print(f"\nSimulated Sonar: WARNING - obstacles is not a list, ignoring")
            self.obstacles = []
            return
        
        # Load valid obstacles
        self.obstacles = obstacles
        print(f"\nSimulated Sonar: Loaded {len(obstacles)} obstacle(s)")
        for i, obs in enumerate(obstacles):
            name = obs.get('name', f'Obstacle {i+1}')
            shape = obs.get('shape', 'box')
            pos = obs.get('position', {})
            dims = obs.get('dimensions', {})
            print(f"  [{i+1}] {name} ({shape})")
            print(f"      Position: ({pos.get('local_x', 0):.1f}, {pos.get('local_y', 0):.1f})")
            if shape == 'box':
                print(f"      Dimensions: {dims.get('width', 0):.1f}m x {dims.get('length', 0):.1f}m")
            else:
                print(f"      Radius: {dims.get('radius', 0):.1f}m")
            print(f"      Depth: {dims.get('depth_top', 0):.1f}m - {dims.get('depth_bottom', 10):.1f}m")
    
    def get_distance(self, vehicle_position: Dict[str, float], 
                     vehicle_heading: float) -> float:
        """Calculate sonar distance reading (finite, with max_range if no detection)."""
        distance, _ = self.get_distance_with_info(vehicle_position, vehicle_heading)
        return distance
    
    def get_distance_with_info(self, vehicle_position: Dict[str, float], 
                              vehicle_heading: float) -> Tuple[float, Optional[Dict[str, Any]]]:
        """
        Calculate sonar distance reading WITH obstacle information and confidence.
        
        FIXED: Returns max_range with 0% confidence when no obstacles loaded
        """
        # FIXED: Handle case when no obstacles are loaded
        if not self.obstacles:
            self.last_distance = self.max_range
            self.last_confidence = 0.0
            self.last_detection = None
            return self.last_distance, {"confidence": 0.0}
        
        # Get vehicle state
        veh_x = vehicle_position.get('x', 0.0)
        veh_y = vehicle_position.get('y', 0.0)
        veh_depth = vehicle_position.get('depth', 0.0)
        
        # Calculate effective sonar beam direction
        beam_heading = vehicle_heading
        beam_vertical_angle = self.mount_angle
        
        # Find closest obstacle intersection
        min_distance = None
        closest_obstacle = None
        
        for obstacle in self.obstacles:
            distance = self._ray_cast_to_obstacle(
                veh_x, veh_y, veh_depth,
                beam_heading, beam_vertical_angle,
                obstacle
            )
            
            if distance is not None:
                if min_distance is None or distance < min_distance:
                    min_distance = distance
                    closest_obstacle = obstacle
        
        # Apply range limits and compute confidence
        if min_distance is not None:
            if min_distance < self.min_range:
                if self.debug:
                    print(f"  Sonar: Detection {min_distance:.2f}m in dead zone (<{self.min_range}m)")
                min_distance = self.max_range
                confidence = 0.0
                closest_obstacle = None
            elif min_distance > self.max_range:
                if self.debug:
                    print(f"  Sonar: Detection {min_distance:.2f}m beyond max range (>{self.max_range}m)")
                min_distance = self.max_range
                confidence = 0.0
                closest_obstacle = None
            else:
                # Linear confidence: 100% at 0m, 0% at max_range
                confidence = max(0.0, 100.0 * (1.0 - min_distance / self.max_range))
        else:
            min_distance = self.max_range
            confidence = 0.0
            closest_obstacle = None
        
        # Store reading
        self.last_distance = min_distance
        self.last_confidence = confidence
        self.last_detection = closest_obstacle
        
        # Debug output
        if self.debug and confidence > 0.0:
            obs_name = closest_obstacle.get('name', 'Unknown') if closest_obstacle else 'Unknown'
            print(f"  Sonar: Detected {obs_name} at {min_distance:.2f}m (conf: {confidence:.1f}%) "
                  f"(vehicle: {veh_x:.1f}, {veh_y:.1f}, {veh_depth:.1f}m, hdg: {vehicle_heading:.1f}°)")
        
        return min_distance, {"confidence": confidence}
    
    def _ray_cast_to_obstacle(self, veh_x: float, veh_y: float, veh_depth: float,
                             heading: float, vertical_angle: float,
                             obstacle: Dict[str, Any]) -> Optional[float]:
        """Ray-cast from vehicle to obstacle with 3D depth checking."""
        # Get obstacle depth range
        dims = obstacle.get('dimensions', {})
        obs_depth_top = dims.get('depth_top', 0.0)
        obs_depth_bottom = dims.get('depth_bottom', 10.0)
        
        # Check if vehicle depth is within obstacle's vertical range
        half_vertical_beam = self.vertical_beam_width / 2.0
        beam_depth_min = veh_depth - half_vertical_beam * 0.1
        beam_depth_max = veh_depth + half_vertical_beam * 0.1
        
        # Check if beam intersects with obstacle's depth range
        if beam_depth_max < obs_depth_top or beam_depth_min > obs_depth_bottom:
            if self.debug:
                obs_name = obstacle.get('name', 'Unknown')
                print(f"    {obs_name}: Beam miss (beam depth {beam_depth_min:.1f}-{beam_depth_max:.1f}m, "
                      f"obstacle {obs_depth_top:.1f}-{obs_depth_bottom:.1f}m)")
            return None
        
        shape = obstacle.get('shape', 'box')
        
        if shape == 'box':
            return self._ray_cast_box(veh_x, veh_y, veh_depth, heading, vertical_angle, obstacle)
        elif shape == 'cylinder':
            return self._ray_cast_cylinder(veh_x, veh_y, veh_depth, heading, vertical_angle, obstacle)
        else:
            print(f"Warning: Unknown obstacle shape '{shape}'")
            return None
    
    def _ray_cast_box(self, veh_x: float, veh_y: float, veh_depth: float,
                      heading: float, vertical_angle: float,
                      obstacle: Dict[str, Any]) -> Optional[float]:
        """Ray-cast to box obstacle."""
        pos = obstacle.get('position', {})
        obs_x = pos.get('local_x', 0.0)
        obs_y = pos.get('local_y', 0.0)
        
        dims = obstacle.get('dimensions', {})
        width = dims.get('width', 1.0)
        length = dims.get('length', 1.0)
        
        heading_rad = math.radians(heading)
        ray_dx = math.sin(heading_rad)
        ray_dy = math.cos(heading_rad)
        
        box_x_min = obs_x - width / 2.0
        box_x_max = obs_x + width / 2.0
        box_y_min = obs_y - length / 2.0
        box_y_max = obs_y + length / 2.0
        
        distance = self._ray_aabb_intersection(
            veh_x, veh_y, ray_dx, ray_dy,
            box_x_min, box_x_max, box_y_min, box_y_max
        )
        
        if distance is not None and self.debug:
            obs_name = obstacle.get('name', 'Unknown')
            print(f"    {obs_name}: Box hit at {distance:.2f}m")
        
        return distance
    
    def _ray_cast_cylinder(self, veh_x: float, veh_y: float, veh_depth: float,
                          heading: float, vertical_angle: float,
                          obstacle: Dict[str, Any]) -> Optional[float]:
        """Ray-cast to cylindrical obstacle."""
        pos = obstacle.get('position', {})
        obs_x = pos.get('local_x', 0.0)
        obs_y = pos.get('local_y', 0.0)
        
        dims = obstacle.get('dimensions', {})
        radius = dims.get('radius', 0.5)
        
        heading_rad = math.radians(heading)
        ray_dx = math.sin(heading_rad)
        ray_dy = math.cos(heading_rad)
        
        distance = self._ray_circle_intersection(
            veh_x, veh_y, ray_dx, ray_dy,
            obs_x, obs_y, radius
        )
        
        if distance is not None and self.debug:
            obs_name = obstacle.get('name', 'Unknown')
            print(f"    {obs_name}: Cylinder hit at {distance:.2f}m")
        
        return distance
    
    def _ray_aabb_intersection(self, ray_x: float, ray_y: float,
                               ray_dx: float, ray_dy: float,
                               box_x_min: float, box_x_max: float,
                               box_y_min: float, box_y_max: float) -> Optional[float]:
        """Ray-AABB intersection."""
        epsilon = 1e-10
        
        if abs(ray_dx) < epsilon:
            ray_dx = epsilon
        if abs(ray_dy) < epsilon:
            ray_dy = epsilon
        
        t_x_min = (box_x_min - ray_x) / ray_dx
        t_x_max = (box_x_max - ray_x) / ray_dx
        t_y_min = (box_y_min - ray_y) / ray_dy
        t_y_max = (box_y_max - ray_y) / ray_dy
        
        if t_x_min > t_x_max:
            t_x_min, t_x_max = t_x_max, t_x_min
        if t_y_min > t_y_max:
            t_y_min, t_y_max = t_y_max, t_y_min
        
        t_min = max(t_x_min, t_y_min)
        t_max = min(t_x_max, t_y_max)
        
        if t_max < 0 or t_min > t_max:
            return None
        
        distance = t_min if t_min > 0 else 0.0
        return distance
    
    def _ray_circle_intersection(self, ray_x: float, ray_y: float,
                                 ray_dx: float, ray_dy: float,
                                 circle_x: float, circle_y: float,
                                 radius: float) -> Optional[float]:
        """Ray-circle intersection in 2D."""
        to_center_x = circle_x - ray_x
        to_center_y = circle_y - ray_y
        
        proj_length = to_center_x * ray_dx + to_center_y * ray_dy
        
        closest_x = ray_x + proj_length * ray_dx
        closest_y = ray_y + proj_length * ray_dy
        
        dist_to_center = math.sqrt((closest_x - circle_x)**2 + (closest_y - circle_y)**2)
        
        if dist_to_center > radius:
            return None
        
        chord_half = math.sqrt(radius**2 - dist_to_center**2)
        distance = proj_length - chord_half
        
        if distance < 0:
            return None
        
        return distance
    
    def get_status(self) -> Dict[str, Any]:
        """Get sonar status for telemetry/debugging."""
        status = {
            'max_range': self.max_range,
            'min_range': self.min_range,
            'beam_width': self.beam_width,
            'vertical_beam_width': self.vertical_beam_width,
            'mount_angle': self.mount_angle,
            'obstacles_loaded': len(self.obstacles),
            'last_distance': self.last_distance,
            'last_confidence': self.last_confidence,
            'last_detection': (
                self.last_detection.get('name', 'Unknown') 
                if self.last_detection else None
            ),
            'debug_mode': self.debug
        }
        return status
    
    def set_mount_angle(self, angle_degrees: float):
        """Set sonar mount angle."""
        self.mount_angle = angle_degrees
        print(f"Simulated Sonar: Mount angle set to {angle_degrees}°")
    
    def set_debug(self, enabled: bool):
        """Enable or disable debug output"""
        self.debug = enabled
        print(f"Simulated Sonar: Debug mode {'ENABLED' if enabled else 'DISABLED'}")


if __name__ == '__main__':
    print("Enhanced SimulatedSonar module loaded successfully")
    print("\nFIXED: Handles missions without obstacles gracefully")