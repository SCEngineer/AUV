#!/usr/bin/env python3
"""
emergency_surface.py - UPDATED: Auto-triggering leak detection monitoring
Continuously monitors three watertight cylinders and auto-triggers emergency surface
"""

import time
from plugin_registry import GuidanceTaskPlugin

class EmergencySurfacePlugin(GuidanceTaskPlugin):
    TASK_NAME = "EMERGENCY_SURFACE"
    
    def __init__(self, vehicle_state):
        super().__init__(vehicle_state)
        self.name = self.TASK_NAME
        self.timeout = 60.0
        self.start_time = 0.0
        self.leak_detected = False
        self.leak_source = ""
        self.auto_triggered = False  # Track if we auto-triggered
    
    def should_activate(self):
        """
        This method is called by the task manager to check if this plugin should take over
        Returns True if leak detected and we haven't already been activated
        """
        leak_detected, _ = self._check_leak_detection()
        return leak_detected and not self.initialized
    
    def initialize(self, task_params):
        try:
            self.timeout = task_params.get('timeout', 60.0)
            self.start_time = time.time()
            
            # Check for leak detection that triggered this emergency surface
            self.leak_detected, self.leak_source = self._check_leak_detection()
            
            # Determine if this was auto-triggered by leak detection
            self.auto_triggered = self.leak_detected
            
            # Emergency surface - empty ballast and stop propulsion
            self.vehicle_state.update_target_state(
                target_depth=0.0,
                target_speed=0.0
            )
            self.vehicle_state.actuator_commands['ballast_cmd'] = 'EMPTY'
            
            self.initialized = True
            
            if self.leak_detected:
                self.status_message = f"AUTO-TRIGGERED: LEAK in {self.leak_source} - Emergency surface initiated"
                # Trigger failsafe state due to leak
                self.vehicle_state.trigger_failsafe(f"Water leak detected in {self.leak_source}")
                print(f"EMERGENCY SURFACE AUTO-TRIGGERED: Leak detected in {self.leak_source}")
            else:
                self.status_message = "Emergency surface initiated (manual)"
                
            return True
            
        except Exception as e:
            self.status_message = f"Emergency surface init failed: {e}"
            return False
    
    def execute(self, time_step):
        # Continuously monitor for leaks during emergency surface
        current_leak_detected, current_leak_source = self._check_leak_detection()
        
        # Update leak status if new leaks detected
        if current_leak_detected and not self.leak_detected:
            self.leak_detected = True
            self.leak_source = current_leak_source
            self.auto_triggered = True
            self.vehicle_state.trigger_failsafe(f"Water leak detected in {current_leak_source}")
        elif current_leak_detected and current_leak_source != self.leak_source:
            # Additional leak detected
            self.leak_source = current_leak_source
            self.vehicle_state.trigger_failsafe(f"Multiple water leaks detected: {current_leak_source}")
        
        # Force emergency surface - empty ballast and stop propulsion
        self.vehicle_state.update_target_state(
            target_depth=0.0,
            target_speed=0.0
        )
        self.vehicle_state.actuator_commands['ballast_cmd'] = 'EMPTY'
        
        current_depth = self.vehicle_state.nav_state.get('depth', 0.0)
        
        if self.leak_detected:
            self.status_message = f"EMERGENCY: {self.leak_source} leak - Surfacing from {current_depth:.1f}m"
        else:
            self.status_message = f"Emergency surface: depth {current_depth:.1f}m"
        
        # Check timeout
        elapsed = time.time() - self.start_time
        if elapsed > self.timeout:
            self.status_message = f"Emergency surface timed out after {elapsed:.1f}s"
            raise TimeoutError("Emergency surface timeout")
    
    def check_completion(self):
        """
        Task is complete when vehicle reaches surface
        If this was auto-triggered by leak, executive should set FAILSAFE state
        """
        current_depth = self.vehicle_state.nav_state.get('depth', 0.0)
        surfaced = current_depth < 0.5
        
        if surfaced and self.auto_triggered:
            # Signal to executive that this was emergency due to leak
            self.status_message = f"EMERGENCY SURFACE COMPLETE: Leak response finished - Vehicle at surface"
            print("EMERGENCY SURFACE COMPLETE: Vehicle surfaced due to leak detection")
            print("Executive should now set FAILSAFE state and initiate shutdown")
        elif surfaced:
            self.status_message = "Emergency surface complete"
        
        return surfaced
    
    def was_auto_triggered(self):
        """
        Returns True if this emergency surface was automatically triggered by leak detection
        Executive can use this to determine if FAILSAFE state should be set
        """
        return self.auto_triggered
    
    def _check_leak_detection(self) -> tuple:
        """
        Check all three watertight cylinders for leaks
        Returns: (leak_detected: bool, leak_source: str)
        """
        try:
            leak_status = self.vehicle_state.leak_detection
            leaked_cylinders = []
            
            # Check each watertight cylinder
            if leak_status.get('FWD_WTC', 'DRY') == 'LEAK':
                leaked_cylinders.append('FWD_WTC')
            if leak_status.get('MID_WTC', 'DRY') == 'LEAK':
                leaked_cylinders.append('MID_WTC')
            if leak_status.get('AFT_WTC', 'DRY') == 'LEAK':
                leaked_cylinders.append('AFT_WTC')
            
            if leaked_cylinders:
                leak_source = '+'.join(leaked_cylinders)  # e.g., "FWD_WTC+MID_WTC"
                return True, leak_source
            else:
                return False, ""
                
        except Exception as e:
            # If we can't read leak status, assume no leak but log error
            self.vehicle_state.add_fault(f"Leak detection read error: {e}")
            return False, ""