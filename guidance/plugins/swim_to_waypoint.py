#!/usr/bin/env python3
"""
swim_to_waypoint.py - FIXED: Respects obstacle avoidance state
"""

import math
from vehicle_state import VehicleState
from plugin_registry import GuidanceTaskPlugin

EARTH_R = 6371000.0  # Earth radius in meters

def haversine_distance(lat1, lon1, lat2, lon2):
    """Return great-circle distance (meters) between two lat/lon points."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_R * c

def initial_bearing(lat1, lon1, lat2, lon2):
    """Return initial bearing (deg) from point1 to point2 (0..360)"""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    brng = math.degrees(math.atan2(y, x)) % 360.0
    return brng

class SwimToWaypointPlugin(GuidanceTaskPlugin):
    TASK_NAME = "SWIM_TO_WAYPOINT"

    def __init__(self, vehicle_state: VehicleState):
        super().__init__(vehicle_state)
        self.name = self.TASK_NAME
        self._reset_state()

    def _reset_state(self):
        """Reset all state variables to initial values"""
        self.target_lat = None
        self.target_lon = None
        self.target_speed = 2.0
        self.target_depth = 0.0
        self.timeout = 300.0
        self.waypoint_tolerance = 15.0
        self.min_dwell_time = 1.0
        self.start_time = 0.0
        self.entered_circle_time = None
        self.completed = False
        self.initialized = False
        self.status_message = "Not initialized"

    def initialize(self, task_params):
        """Initialize waypoint navigation with full state reset"""
        self._reset_state()
        
        try:
            # Required parameters
            self.target_lat = float(task_params['lat'])
            self.target_lon = float(task_params['lon'])
            
            # Optional parameters
            self.target_speed = float(task_params.get('speed', 2.0))
            self.target_depth = float(task_params.get('depth', 0.0))
            self.timeout = float(task_params.get('timeout', 300.0))
            
            # Validate coordinates
            if not (-90.0 <= self.target_lat <= 90.0) or not (-180.0 <= self.target_lon <= 180.0):
                self.status_message = f"ERROR: Invalid coordinates ({self.target_lat}, {self.target_lon})"
                self.vehicle_state.add_fault(self.status_message)
                return False
                
            # Set start time
            self.start_time = self.vehicle_state.mission_time
            self.initialized = True
            self.status_message = f"Navigating to waypoint ({self.target_lat:.6f}, {self.target_lon:.6f})"
            print(f"SWIM_TO_WAYPOINT initialized: lat={self.target_lat}, lon={self.target_lon}, speed={self.target_speed}, depth={self.target_depth}")
            return True
            
        except Exception as e:
            self.status_message = f"Initialization failed: {e}"
            self.vehicle_state.add_fault(self.status_message)
            print(f"SWIM_TO_WAYPOINT init error: {e}")
            return False

    def execute(self, time_step):
        """Execute simple waypoint navigation - just point at the waypoint"""
        if not self.initialized or self.completed:
            return
        
        # OBSTACLE AVOIDANCE CHECK - Pause guidance when actively avoiding
        if hasattr(self.vehicle_state, 'obstacle_avoidance'):
            oa = self.vehicle_state.obstacle_avoidance
            if oa and oa.state == "AVOIDING":
                self.status_message = "Paused - avoiding obstacle"
                return
            
        # Get current position
        nav = self.vehicle_state.nav_state
        cur_lat = nav.get('lat', 0.0)
        cur_lon = nav.get('lon', 0.0)
        
        # Calculate distance and bearing to waypoint
        dist_to_waypoint = haversine_distance(cur_lat, cur_lon, self.target_lat, self.target_lon)
        bearing_to_waypoint = initial_bearing(cur_lat, cur_lon, self.target_lat, self.target_lon)
        
        # Check arrival
        if dist_to_waypoint <= self.waypoint_tolerance:
            if self.entered_circle_time is None:
                self.entered_circle_time = self.vehicle_state.mission_time
                self.status_message = f"Entering arrival circle, distance {dist_to_waypoint:.1f}m"
            elif (self.vehicle_state.mission_time - self.entered_circle_time) >= self.min_dwell_time:
                self.completed = True
                self.vehicle_state.set_task_complete(True)
                self.status_message = f"Waypoint reached (within {self.waypoint_tolerance:.1f}m)"
                return
        else:
            self.entered_circle_time = None
            
        # Simple navigation: just point directly at the waypoint
        target_heading = bearing_to_waypoint
        target_speed = self.target_speed
        
        # Update target state
        self.vehicle_state.update_target_state(
            target_heading=target_heading,
            target_speed=target_speed,
            target_depth=self.target_depth,
            distance_to_waypoint=dist_to_waypoint
        )
        
        # Update status
        self.status_message = f"Distance {dist_to_waypoint:.1f}m, bearing {bearing_to_waypoint:.0f} deg, speed {target_speed:.1f}m/s"
        
        # Check timeout
        elapsed = self.vehicle_state.mission_time - self.start_time
        if elapsed > self.timeout:
            self.status_message = f"Waypoint navigation timeout after {elapsed:.1f}s"
            self.vehicle_state.add_fault(self.status_message)
            raise TimeoutError("Waypoint navigation timeout")

    def check_completion(self):
        """Check if waypoint has been reached"""
        return self.completed
