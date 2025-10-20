#!/usr/bin/env python3
"""
mission_complete.py - Rewritten for clean task management
"""

import time
from plugin_registry import GuidanceTaskPlugin

class MissionCompletePlugin(GuidanceTaskPlugin):
    TASK_NAME = "MISSION_COMPLETE"
    
    def __init__(self, vehicle_state):
        super().__init__(vehicle_state)
        self.name = self.TASK_NAME
        
        # Task parameters
        self.timeout = 10.0
        
        # State
        self.start_time = 0.0
        self.shutdown_initiated = False
        self.completed = False
    
    def initialize(self, task_params):
        """Initialize mission completion sequence"""
        try:
            self.timeout = float(task_params.get('timeout', 10.0))
            
            # Reset state
            self.start_time = time.time()
            self.shutdown_initiated = False
            self.completed = False
            self.initialized = True
            
            self.status_message = "Mission complete - initiating safe shutdown"
            print(f"MISSION_COMPLETE: {self.status_message}")
            return True
            
        except Exception as e:
            self.status_message = f"Mission complete init failed: {e}"
            return False
    
    def execute(self, time_step):
        """Execute mission completion sequence"""
        if not self.initialized or self.completed:
            return
            
        if not self.shutdown_initiated:
            # Set safe end-of-mission targets
            self.vehicle_state.update_target_state(
                target_depth=0.0,      # Surface for recovery
                target_speed=0.0,      # Stop all motion
                target_heading=self.vehicle_state.nav_state.get('heading', 0.0)  # Hold current heading
            )
            
            # Set ballast for positive buoyancy
            if hasattr(self.vehicle_state, 'actuator_commands'):
                self.vehicle_state.actuator_commands['ballast_cmd'] = 'EMPTY'
            
            self.shutdown_initiated = True
            self.status_message = "Safe shutdown sequence initiated"
        
        # Mission complete after brief delay to ensure commands are processed
        elapsed = time.time() - self.start_time
        if elapsed > 2.0:  # 2 second delay
            self.completed = True
            self.status_message = "Mission completed successfully"
            print(f"MISSION_COMPLETE: {self.status_message}")
        else:
            self.status_message = f"Completing mission shutdown ({elapsed:.1f}s)"
        
        # Check timeout
        if elapsed > self.timeout:
            self.completed = True
            self.status_message = "Mission complete (timeout)"
            print(f"MISSION_COMPLETE: Timeout after {elapsed:.1f}s")
    
    def check_completion(self):
        """Check if mission completion sequence is done"""
        return self.completed