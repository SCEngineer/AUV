#!/usr/bin/env python3
"""
go_to_submerged_trim.py - IMPROVED: Better monitoring and diagnostics
"""

from plugin_registry import GuidanceTaskPlugin

class GoToSubmergedTrimPlugin(GuidanceTaskPlugin):
    TASK_NAME = "GO_TO_SUBMERGED_TRIM"
    
    def __init__(self, vehicle_state):
        super().__init__(vehicle_state)
        self.name = self.TASK_NAME
        self.timeout = 90.0
        self.target_depth = 5.0
        self.start_time = 0.0
        self.initialized = False
        self.status_message = "Not initialized"
    
    def initialize(self, task_params):
        try:
            self.timeout = task_params.get('timeout', 90.0)
            self.target_depth = task_params.get('depth', 5.0)
            self.start_time = self.vehicle_state.mission_time
            
            # Set submerged target and ballast command for NEUTRAL buoyancy
            self.vehicle_state.update_target_state(
                target_depth=self.target_depth,
                ballast_cmd='FILL'
            )
            
            self.initialized = True
            self.status_message = f"Filling ballast for submerged trim at {self.target_depth}m"
            print(f"GO_TO_SUBMERGED_TRIM: Starting ballast fill, timeout={self.timeout}s")
            
            # Log initial ballast state
            ballast_state = self.vehicle_state.ballast_state
            print(f"  Initial ballast: volume={ballast_state.get('tank_volume', 0.0):.6f}m³, capacity={ballast_state.get('tank_capacity', 0.001982):.6f}m³")
            print(f"  Initial buoyancy: {self.vehicle_state.buoyancy_state}")
            
            return True
            
        except Exception as e:
            self.status_message = f"Submerged trim init failed: {e}"
            print(f"GO_TO_SUBMERGED_TRIM init error: {e}")
            return False
    
    def execute(self, time_step):
        if not self.initialized:
            return
            
        # Get current ballast state for monitoring
        ballast_state = self.vehicle_state.ballast_state
        tank_volume = ballast_state.get('tank_volume', 0.0)
        tank_capacity = ballast_state.get('tank_capacity', 0.001982)
        tank_status = ballast_state.get('tank_status', 'UNKNOWN')
        pump_running = ballast_state.get('pump_running', False)
        fill_percentage = (tank_volume / tank_capacity) * 100.0 if tank_capacity > 0 else 0.0
        
        # Continue setting ballast target until NEUTRAL buoyancy achieved
        current_buoyancy = self.vehicle_state.buoyancy_state
        
        if current_buoyancy != 'NEUTRAL':
            # Still need to fill
            self.vehicle_state.update_target_state(ballast_cmd='FILL')
            self.status_message = f"Filling ballast: {fill_percentage:.1f}% full, {tank_status}, pump {'ON' if pump_running else 'OFF'}, buoyancy {current_buoyancy}"
        else:
            # Achieved neutral buoyancy
            self.vehicle_state.update_target_state(ballast_cmd='OFF')
            self.status_message = f"Submerged trim achieved: ballast {fill_percentage:.1f}% full, buoyancy {current_buoyancy}"
            
        # Check timeout
        elapsed = self.vehicle_state.mission_time - self.start_time
        if elapsed > self.timeout:
            self.status_message = f"Submerged trim TIMEOUT after {elapsed:.1f}s - ballast {fill_percentage:.1f}% full, buoyancy {current_buoyancy}"
            print(f"GO_TO_SUBMERGED_TRIM timeout:")
            print(f"  Elapsed: {elapsed:.1f}s / {self.timeout:.1f}s")
            print(f"  Ballast: {tank_volume:.6f}m³ / {tank_capacity:.6f}m³ ({fill_percentage:.1f}%)")
            print(f"  Tank status: {tank_status}, pump: {'ON' if pump_running else 'OFF'}")
            print(f"  Buoyancy: {current_buoyancy} (needed NEUTRAL)")
            raise TimeoutError(f"Submerged trim timeout - only {fill_percentage:.1f}% filled")
    
    def check_completion(self):
        """Check if neutral buoyancy achieved"""
        completed = self.vehicle_state.buoyancy_state == 'NEUTRAL'
        if completed:
            ballast_state = self.vehicle_state.ballast_state
            fill_percentage = (ballast_state.get('tank_volume', 0.0) / ballast_state.get('tank_capacity', 0.001982)) * 100.0
            print(f"GO_TO_SUBMERGED_TRIM completed successfully - ballast {fill_percentage:.1f}% full, buoyancy NEUTRAL")
        return completed