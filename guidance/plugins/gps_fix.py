#!/usr/bin/env python3
"""
gps_fix.py - ENHANCED: Immediate GPS correction after surfacing
"""

import time
from plugin_registry import GuidanceTaskPlugin

class GPSFixPlugin(GuidanceTaskPlugin):
    TASK_NAME = "GPS_FIX"
    
    def __init__(self, vehicle_state):
        super().__init__(vehicle_state)
        self.name = self.TASK_NAME
        self.timeout = 30.0
        self.start_time = 0.0
        self.fix_acquired = False
        self.max_expected_drift = 500.0
    
    def initialize(self, task_params):
        try:
            self.timeout = task_params.get('timeout', 30.0)
            self.max_expected_drift = task_params.get('max_expected_drift', 500.0)
            self.start_time = time.time()
            self.fix_acquired = False
            
            # Enable resurfacing correction in navigation system
            try:
                # Find navigation object - try common locations
                nav = getattr(self.vehicle_state, 'navigation', None) or getattr(self.vehicle_state, 'nav_system', None)
                
                if nav and hasattr(nav, 'prepare_for_resurfacing_correction'):
                    nav.prepare_for_resurfacing_correction(
                        max_expected_drift=self.max_expected_drift,
                        timeout=self.timeout
                    )
                    print(f"GPS_FIX: Resurfacing correction enabled (max drift: {self.max_expected_drift:.1f}m)")
            except Exception as e:
                print(f"GPS_FIX: Could not enable resurfacing correction: {e}")
            
            self.initialized = True
            self.status_message = "Acquiring GPS fix"
            return True
            
        except Exception as e:
            self.status_message = f"GPS fix init failed: {e}"
            return False
    
    def execute(self, time_step):
        elapsed = time.time() - self.start_time
        depth = self.vehicle_state.nav_state.get('depth', 0.0)
        
        # Wait for surface
        if depth >= 0.5:
            self.status_message = f"Waiting for surface: depth {depth:.1f}m"
            return
        
        # At surface - acquire fix immediately
        if not self.fix_acquired:
            self.fix_acquired = True
            self.status_message = "GPS fix acquired"
            print(f"GPS_FIX: Fix acquired immediately at surface (elapsed: {elapsed:.1f}s)")
        
        # Check timeout
        if elapsed > self.timeout:
            self.status_message = f"GPS fix timed out after {elapsed:.1f}s"
            raise TimeoutError("GPS fix timeout")
    
    def check_completion(self):
        return self.fix_acquired