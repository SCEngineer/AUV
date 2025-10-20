#!/usr/bin/env python3
"""
climb.py - IMPROVED: Added approach control to prevent overshoot
FIXED: Respects obstacle avoidance state
"""

import time
from plugin_registry import GuidanceTaskPlugin

class ClimbPlugin(GuidanceTaskPlugin):
    TASK_NAME = "CLIMB"
    
    def __init__(self, vehicle_state):
        super().__init__(vehicle_state)
        self.name = self.TASK_NAME
        self.target_depth = 0.0
        self.target_speed = 1.0
        self.timeout = 120.0  # Increased for controlled approach
        self.depth_tolerance = 0.5  # Tighter tolerance
        self.stability_threshold = 5  # More stability checks
        self.stable_count = 0
        self.start_time = 0.0
        self.initialized = False
        self.status_message = "Not initialized"
        
        # Approach control parameters
        self.approach_zone = 2.0  # Start slowing down when within 2m of target
        self.min_speed = 0.3      # Minimum speed during approach
        self.overshoot_limit = 1.0 # Maximum overshoot allowed
    
    def initialize(self, task_params):
        try:
            self.target_depth = task_params.get('depth', 0.0)
            self.target_speed = task_params.get('speed', 1.0) 
            self.timeout = task_params.get('timeout', 120.0)
            self.start_time = self.vehicle_state.mission_time  # Use mission time
            
            # Set initial target state
            self.vehicle_state.update_target_state(
                target_depth=self.target_depth,
                target_speed=self.target_speed
            )
            
            self.stable_count = 0
            self.initialized = True
            self.status_message = f"Climbing to {self.target_depth:.1f}m at {self.target_speed:.1f}m/s"
            
            print(f"CLIMB initialized: target_depth={self.target_depth:.1f}m, speed={self.target_speed:.1f}m/s, timeout={self.timeout:.1f}s")
            
            # Log initial state
            current_depth = self.vehicle_state.nav_state.get('depth', 0.0)
            buoyancy = self.vehicle_state.buoyancy_state
            print(f"  Initial: depth={current_depth:.1f}m, buoyancy={buoyancy}")
            
            return True
            
        except Exception as e:
            self.status_message = f"Climb init failed: {e}"
            print(f"CLIMB initialization failed: {e}")
            return False
    
    def execute(self, time_step):
        if not self.initialized:
            return
        
        # OBSTACLE AVOIDANCE CHECK - Pause guidance when actively avoiding
        if hasattr(self.vehicle_state, 'obstacle_avoidance'):
            oa = self.vehicle_state.obstacle_avoidance
            if oa and oa.state == "AVOIDING":
                self.status_message = "Paused - avoiding obstacle"
                return
            
        current_depth = self.vehicle_state.nav_state.get('depth', 0.0)
        depth_error = self.target_depth - current_depth  # Negative for climbing up
        vertical_velocity = self.vehicle_state.true_state.get('true_vertical_velocity', 0.0)
        
        # Calculate approach speed based on distance to target
        commanded_speed = self._calculate_approach_speed(depth_error, vertical_velocity)
        
        # Update target state with approach control
        self.vehicle_state.update_target_state(
            target_depth=self.target_depth,
            target_speed=commanded_speed
        )
        
        # Check if we're stable at target
        if abs(depth_error) <= self.depth_tolerance and abs(vertical_velocity) < 0.2:
            self.stable_count += 1
        else:
            self.stable_count = 0
        
        # Enhanced status message
        if abs(depth_error) > self.depth_tolerance:
            approach_mode = "APPROACH" if abs(depth_error) < self.approach_zone else "CLIMB"
            self.status_message = f"{approach_mode}: {self.target_depth:.1f}m, current {current_depth:.1f}m, vz={vertical_velocity:+.2f}m/s, spd={commanded_speed:.1f}m/s"
        else:
            self.status_message = f"At target depth {self.target_depth:.1f}m, stable: {self.stable_count}/{self.stability_threshold}"
        
        # Check timeout
        elapsed = self.vehicle_state.mission_time - self.start_time
        if elapsed > self.timeout:
            self.status_message = f"Climb TIMEOUT after {elapsed:.1f}s at {current_depth:.1f}m (target {self.target_depth:.1f}m)"
            print(f"CLIMB timeout:")
            print(f"  Elapsed: {elapsed:.1f}s / {self.timeout:.1f}s") 
            print(f"  Depth: {current_depth:.1f}m / {self.target_depth:.1f}m (error: {depth_error:.1f}m)")
            print(f"  Vertical velocity: {vertical_velocity:+.3f}m/s")
            print(f"  Stable count: {self.stable_count}/{self.stability_threshold}")
            raise TimeoutError(f"Climb timeout - reached {current_depth:.1f}m, needed {self.target_depth:.1f}m")
    
    def _calculate_approach_speed(self, depth_error, vertical_velocity):
        """Calculate approach speed to prevent overshoot during climb"""
        
        # If we're overshooting upward (above target with upward velocity), emergency slow
        if depth_error > self.overshoot_limit and vertical_velocity > 0.1:
            return self.min_speed
            
        # If we're within approach zone, reduce speed based on distance
        if abs(depth_error) < self.approach_zone:
            # Linear speed reduction: full speed at approach_zone, min_speed at target
            speed_factor = abs(depth_error) / self.approach_zone
            calculated_speed = self.min_speed + (self.target_speed - self.min_speed) * speed_factor
            
            # Further reduce speed if we have excessive upward velocity
            if vertical_velocity > 1.0:  # If rising too fast
                velocity_factor = max(0.3, 1.0 - vertical_velocity / 2.0)  # Slow down more for faster rising
                calculated_speed *= velocity_factor
            
            return max(self.min_speed, min(self.target_speed, calculated_speed))
        
        # Full speed if far from target
        return self.target_speed
    
    def check_completion(self):
        """Check if climb completed with stability"""
        if not self.initialized:
            return False
            
        # Must be stable for required number of timesteps
        completed = self.stable_count >= self.stability_threshold
        
        if completed:
            current_depth = self.vehicle_state.nav_state.get('depth', 0.0)
            vertical_velocity = self.vehicle_state.true_state.get('true_vertical_velocity', 0.0)
            self.status_message = f"Climb completed at {current_depth:.1f}m (target {self.target_depth:.1f}m, vz={vertical_velocity:+.3f}m/s)"
            print(f"CLIMB completed: reached {current_depth:.1f}m (target {self.target_depth:.1f}m)")
        
        return completed
