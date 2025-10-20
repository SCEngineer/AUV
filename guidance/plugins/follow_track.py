#!/usr/bin/env python3
"""
follow_track.py - FIXED: Respects obstacle avoidance state
"""

import math
from typing import List, Tuple
from vehicle_state import VehicleState
from plugin_registry import GuidanceTaskPlugin

EARTH_R = 6371000.0

def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1 = math.radians(lat1); phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1); dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2.0)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2.0)**2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))
    return EARTH_R * c

def initial_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1 = math.radians(lat1); phi2 = math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

def cross_track_distance(lat_cur: float, lon_cur: float,
                         lat_start: float, lon_start: float,
                         lat_end: float, lon_end: float) -> float:
    d13 = haversine_distance(lat_start, lon_start, lat_cur, lon_cur)
    if d13 == 0.0:
        return 0.0
    theta13 = math.radians(initial_bearing(lat_start, lon_start, lat_cur, lon_cur))
    theta12 = math.radians(initial_bearing(lat_start, lon_start, lat_end, lon_end))
    val = math.sin(d13 / EARTH_R) * math.sin(theta13 - theta12)
    val = max(-1.0, min(1.0, val))
    return math.asin(val) * EARTH_R

def along_track_distance(lat_cur: float, lon_cur: float,
                         lat_start: float, lon_start: float,
                         lat_end: float, lon_end: float) -> float:
    d13 = haversine_distance(lat_start, lon_start, lat_cur, lon_cur)
    if d13 == 0.0:
        return 0.0
    theta13 = math.radians(initial_bearing(lat_start, lon_start, lat_cur, lon_cur))
    theta12 = math.radians(initial_bearing(lat_start, lon_start, lat_end, lon_end))
    return d13 * math.cos(theta13 - theta12)

class FollowTrackPlugin(GuidanceTaskPlugin):
    TASK_NAME = "FOLLOW_TRACK"

    def __init__(self, vehicle_state: VehicleState):
        super().__init__(vehicle_state)
        self.name = self.TASK_NAME
        self._reset_state()

    def _reset_state(self):
        self.waypoints: List[Tuple[float, float, float, float]] = []
        self.current_segment: int = 0
        self.segment_lengths: List[float] = []
        self.total_track_distance: float = 0.0
        self.lookahead_distance: float = 20.0
        self.cross_track_gain: float = 1.0
        self.max_cross_track_correction: float = 30.0
        self.waypoint_tolerance: float = 15.0
        self.arrival_confirm_count: int = 2
        self._arrival_counter: int = 0
        self.start_time: float = 0.0
        self.timeout: float = 600.0
        self.completed: bool = False
        self.initialized: bool = False
        self.status_message: str = "Not initialized"

    def initialize(self, task_params: dict) -> bool:
        self._reset_state()
        try:
            wps = task_params.get('waypoints', []) if task_params else []
            if not wps:
                self.status_message = "ERROR: No waypoints provided"
                return False

            for i, wp in enumerate(wps):
                lat = float(wp['lat']); lon = float(wp['lon'])
                spd = float(wp.get('speed', 2.0)); depth = float(wp.get('depth', 0.0))
                if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                    self.status_message = f"ERROR: invalid waypoint {i}"
                    return False
                self.waypoints.append((lat, lon, spd, depth))

            if len(self.waypoints) < 2:
                self.status_message = "ERROR: need at least 2 waypoints"
                return False

            self.lookahead_distance = float(task_params.get('lookahead_distance', self.lookahead_distance))
            self.cross_track_gain = float(task_params.get('cross_track_gain', self.cross_track_gain))
            self.max_cross_track_correction = float(task_params.get('max_cross_track_correction', self.max_cross_track_correction))
            self.waypoint_tolerance = float(task_params.get('waypoint_tolerance', self.waypoint_tolerance))
            self.arrival_confirm_count = int(task_params.get('arrival_confirm_count', self.arrival_confirm_count))
            self.timeout = float(task_params.get('timeout', self.timeout))

            self._precompute_segments()
            self.start_time = getattr(self.vehicle_state, 'mission_time', 0.0)
            self.current_segment = 0
            self.initialized = True
            self.status_message = f"Following {len(self.waypoints)} wps, {self.total_track_distance:.0f} m total"
            print(f"FOLLOW_TRACK: init ok: {len(self.waypoints)} wps, total {self.total_track_distance:.1f} m, lookahead {self.lookahead_distance} m")
            return True
        except Exception as e:
            self.status_message = f"ERROR: init failed: {e}"
            print("FOLLOW_TRACK init error:", e)
            return False

    def _precompute_segments(self):
        self.segment_lengths = []
        self.total_track_distance = 0.0
        for i in range(len(self.waypoints) - 1):
            s_lat, s_lon, _, _ = self.waypoints[i]
            e_lat, e_lon, _, _ = self.waypoints[i+1]
            L = haversine_distance(s_lat, s_lon, e_lat, e_lon)
            self.segment_lengths.append(L)
            self.total_track_distance += L
        print("FOLLOW_TRACK: segment lengths:", [f"{l:.1f}m" for l in self.segment_lengths])

    def execute(self, time_step: float):
        if not self.initialized or self.completed:
            return

        # OBSTACLE AVOIDANCE CHECK - Pause guidance when actively avoiding
        if hasattr(self.vehicle_state, 'obstacle_avoidance'):
            oa = self.vehicle_state.obstacle_avoidance
            if oa and oa.state == "AVOIDING":
                self.status_message = "Paused - avoiding obstacle"
                return

        try:
            nav = getattr(self.vehicle_state, 'nav_state', None)
            if not self._has_valid_position(nav):
                self.status_message = "NO VALID POSITION"
                return

            lat = float(nav.get('lat')); lon = float(nav.get('lon'))
            seg_idx = min(self.current_segment, max(0, len(self.segment_lengths)-1))
            s_lat, s_lon, _, _ = self.waypoints[seg_idx]
            e_lat, e_lon, e_speed, e_depth = self.waypoints[seg_idx+1]

            signed_xte = cross_track_distance(lat, lon, s_lat, s_lon, e_lat, e_lon)
            along = along_track_distance(lat, lon, s_lat, s_lon, e_lat, e_lon)
            seg_len = self.segment_lengths[seg_idx] if seg_idx < len(self.segment_lengths) else 0.0

            next_wp_idx = seg_idx + 1
            nx_lat, nx_lon, _, _ = self.waypoints[next_wp_idx]
            dist_to_next_wp = haversine_distance(lat, lon, nx_lat, nx_lon)

            if dist_to_next_wp <= self.waypoint_tolerance or (seg_len > 0 and (seg_len - along) <= self.waypoint_tolerance):
                self._arrival_counter += 1
                if self._arrival_counter >= max(1, self.arrival_confirm_count):
                    max_seg_idx = max(0, len(self.segment_lengths) - 1)
                    if self.current_segment < max_seg_idx:
                        old = self.current_segment
                        self.current_segment += 1
                        print(f"FOLLOW_TRACK: advanced {old} -> {self.current_segment} (confirmed arrival)")
                    self._arrival_counter = 0
            else:
                self._arrival_counter = 0

            la_lat, la_lon, tgt_speed, tgt_depth = self._find_lookahead_point_sequential(lat, lon, self.current_segment, self.lookahead_distance)
            base_bearing = initial_bearing(lat, lon, la_lat, la_lon)
            la = max(1.0, self.lookahead_distance)
            angle_rad = math.atan2(self.cross_track_gain * signed_xte, la)
            correction_deg = -math.degrees(angle_rad)
            correction_deg = max(-self.max_cross_track_correction, min(self.max_cross_track_correction, correction_deg))
            target_heading = (base_bearing + correction_deg) % 360.0
            remaining = self._calculate_remaining_distance(lat, lon, self.current_segment)

            self.vehicle_state.update_target_state(
                target_heading=target_heading,
                target_speed=tgt_speed,
                target_depth=tgt_depth,
                distance_to_waypoint=remaining
            )

            total_segments = max(1, len(self.waypoints) - 1)
            self.status_message = f"Seg {self.current_segment+1}/{total_segments}, XTE:{signed_xte:+.1f}m, Rem:{remaining:.0f}m"
            
            elapsed = getattr(self.vehicle_state, 'mission_time', 0.0) - self.start_time
            if elapsed > self.timeout:
                self.status_message = f"Timeout after {elapsed:.1f}s"
                self.vehicle_state.add_fault(self.status_message)
                self.completed = True

            self._check_completion(lat, lon)

        except Exception as e:
            self.status_message = f"ERROR: Execution failed: {e}"
            print("FOLLOW_TRACK exec error:", e)

    def _find_lookahead_point_sequential(self, lat: float, lon: float, start_segment: int, lookahead: float) -> Tuple[float, float, float, float]:
        idx = min(start_segment, max(0, len(self.segment_lengths)-1))
        remaining = lookahead
        if idx < len(self.waypoints) - 1:
            s_lat, s_lon, _, _ = self.waypoints[idx]
            e_lat, e_lon, _, _ = self.waypoints[idx+1]
            cur_along = along_track_distance(lat, lon, s_lat, s_lon, e_lat, e_lon)
        else:
            cur_along = 0.0

        while idx < len(self.segment_lengths) and remaining > 0.0:
            seg_len = self.segment_lengths[idx]
            rem_in_seg = max(0.0, seg_len - cur_along)
            if remaining <= rem_in_seg:
                s_lat, s_lon, _, _ = self.waypoints[idx]
                e_lat, e_lon, e_speed, e_depth = self.waypoints[idx+1]
                frac = (cur_along + remaining) / seg_len if seg_len > 0 else 1.0
                frac = max(0.0, min(1.0, frac))
                la_lat = s_lat + frac * (e_lat - s_lat)
                la_lon = s_lon + frac * (e_lon - s_lon)
                return (la_lat, la_lon, e_speed, e_depth)
            remaining -= rem_in_seg
            idx += 1
            cur_along = 0.0

        fl_lat, fl_lon, fl_spd, fl_depth = self.waypoints[-1]
        return (fl_lat, fl_lon, fl_spd, fl_depth)

    def _calculate_remaining_distance(self, lat: float, lon: float, seg_idx: int) -> float:
        rem = 0.0
        if seg_idx < len(self.segment_lengths):
            s_lat, s_lon, _, _ = self.waypoints[seg_idx]
            e_lat, e_lon, _, _ = self.waypoints[seg_idx+1]
            cur_along = along_track_distance(lat, lon, s_lat, s_lon, e_lat, e_lon)
            rem += max(0.0, self.segment_lengths[seg_idx] - cur_along)
        for i in range(seg_idx + 1, len(self.segment_lengths)):
            rem += self.segment_lengths[i]
        return rem

    def _check_completion(self, lat: float, lon: float) -> bool:
        try:
            final_lat, final_lon, _, _ = self.waypoints[-1]
            dist = haversine_distance(lat, lon, final_lat, final_lon)
            is_on_final_segment = (self.current_segment >= len(self.waypoints) - 2)
            if is_on_final_segment and dist <= self.waypoint_tolerance:
                self.completed = True
                self.vehicle_state.set_task_complete(True)
                self.status_message = f"Track completed - reached final waypoint ({dist:.1f} m)"
                print("FOLLOW_TRACK:", self.status_message)
                return True
            return False
        except Exception as e:
            print("FOLLOW_TRACK completion error:", e)
            return False

    def _has_valid_position(self, nav) -> bool:
        try:
            if not isinstance(nav, dict):
                return False
            lat = nav.get('lat'); lon = nav.get('lon')
            if lat is None or lon is None:
                return False
            lat = float(lat); lon = float(lon)
            if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                return False
            if lat == 0.0 and lon == 0.0:
                gps_fix = nav.get('gps_fix', nav.get('fix', None))
                if gps_fix is False or gps_fix is None:
                    return False
            return True
        except Exception:
            return False

    def check_completion(self) -> bool:
        return self.completed