#!/usr/bin/env python3
"""
dive.py - FIXED: Respects obstacle avoidance state
"""

from plugin_registry import GuidanceTaskPlugin

class DivePlugin(GuidanceTaskPlugin):
    TASK_NAME = "DIVE"
    
    def __init__(self, vehicle_state):
        super().__init__(vehicle_state)
        self.name = self.TASK_NAME
        self.target_depth = 0.0
        self.target_speed = 1.0
        self.timeout = 60.0
        self.start_time = 0.0
        self.depth_tolerance = 1.5
        self.initialized = False
        self.status_message = "Not initialized"
        
        # Approach control parameters
        self.approach_zone = 1.0
        self.min_speed = 0.1
        self.overshoot_limit = 1.0
    
    def initialize(self, task_params):
        try:
            self.target_depth = float(task_params.get('depth', 20.0))
            if self.target_depth <= 0.0:
                self.status_message = "ERROR: Dive requires positive depth"
                self.vehicle_state.add_fault(self.status_message)
                return False
            self.target_speed = float(task_params.get('speed', 2.0))
            self.timeout = float(task_params.get('timeout', 120.0))
            self.start_time = self.vehicle_state.mission_time
            
            # Set initial target depth and speed
            self.vehicle_state.update_target_state(
                target_depth=self.target_depth,
                target_speed=self.target_speed
            )
            
            self.initialized = True
            self.status_message = f"Diving to {self.target_depth:.1f}m at {self.target_speed:.1f}m/s"
            print(f"DIVE initialized: target_depth={self.target_depth:.1f}m, speed={self.target_speed:.1f}m/s, timeout={self.timeout:.1f}s")
            
            # Log initial state
            current_depth = self.vehicle_state.nav_state.get('depth', 0.0)
            buoyancy = self.vehicle_state.buoyancy_state
            print(f"  Initial: depth={current_depth:.1f}m, buoyancy={buoyancy}")
            
            return True
            
        except Exception as e:
            self.status_message = f"Dive init failed: {e}"
            print(f"DIVE initialization failed: {e}")
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
        depth_error = self.target_depth - current_depth
        vertical_velocity = self.vehicle_state.true_state.get('true_vertical_velocity', 0.0)
        
        # Calculate approach speed based on distance to target
        commanded_speed = self._calculate_approach_speed(depth_error, vertical_velocity)
        
        # Stabilize by holding current depth if within tolerance
        if abs(depth_error) < self.depth_tolerance:
            self.vehicle_state.update_target_state(
                target_depth=current_depth,
                target_speed=commanded_speed
            )
        else:
            self.vehicle_state.update_target_state(
                target_depth=self.target_depth,
                target_speed=commanded_speed
            )
        
        # Enhanced status message
        if depth_error > self.depth_tolerance:
            approach_mode = "APPROACH" if abs(depth_error) < self.approach_zone else "DIVE"
            self.status_message = f"{approach_mode}: {self.target_depth:.1f}m, current {current_depth:.1f}m, vz={vertical_velocity:+.2f}m/s, spd={commanded_speed:.1f}m/s"
        else:
            self.status_message = f"At target depth {self.target_depth:.1f}m, current {current_depth:.1f}m, vz={vertical_velocity:+.2f}m/s"
        
        # Check timeout
        elapsed = self.vehicle_state.mission_time - self.start_time
        if elapsed > self.timeout:
            self.status_message = f"Dive TIMEOUT after {elapsed:.1f}s at {current_depth:.1f}m (target {self.target_depth:.1f}m)"
            print(f"DIVE timeout:")
            print(f"  Elapsed: {elapsed:.1f}s / {self.timeout:.1f}s")
            print(f"  Depth: {current_depth:.1f}m / {self.target_depth:.1f}m (error: {depth_error:.1f}m)")
            print(f"  Vertical velocity: {vertical_velocity:+.3f}m/s")
            print(f"  Buoyancy: {self.vehicle_state.buoyancy_state}")
            raise TimeoutError(f"Dive timeout - reached {current_depth:.1f}m, needed {self.target_depth:.1f}m")
    
    def _calculate_approach_speed(self, depth_error, vertical_velocity):
        """Calculate approach speed to prevent overshoot without reverse"""
        # Emergency slow if overshooting
        if depth_error < -self.overshoot_limit and vertical_velocity < -0.1:
            return self.min_speed
        
        # Aggressive slowing within approach zone
        if abs(depth_error) < self.approach_zone:
            speed_factor = abs(depth_error) / self.approach_zone
            calculated_speed = self.min_speed + (self.target_speed - self.min_speed) * speed_factor
            # Stronger reduction if sinking fast
            if vertical_velocity < -0.5:
                velocity_factor = max(0.2, 1.0 + vertical_velocity / 3.0)
                calculated_speed *= velocity_factor
            return max(self.min_speed, min(self.target_speed, calculated_speed))
        
        # Full speed if far from target
        return self.target_speed
    
    def check_completion(self):
        """Check if target depth reached with relaxed criteria"""
        if not self.initialized:
            return False
            
        current_depth = self.vehicle_state.nav_state.get('depth', 0.0)
        depth_error = abs(self.target_depth - current_depth)
        vertical_velocity = self.vehicle_state.true_state.get('true_vertical_velocity', 0.0)
        
        # Relaxed completion criteria
        depth_ok = depth_error < (self.depth_tolerance * 2)
        velocity_ok = abs(vertical_velocity) < 0.5
        completed = depth_ok and velocity_ok
        
        if completed:
            self.status_message = f"Dive completed at {current_depth:.1f}m (target {self.target_depth:.1f}m, vz={vertical_velocity:+.3f}m/s)"
            print(f"DIVE completed: reached {current_depth:.1f}m (target {self.target_depth:.1f}m)")
        
        return completed
