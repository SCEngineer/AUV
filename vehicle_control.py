#!/usr/bin/env python3
"""
vehicle_control.py - Cascaded controller for a dynamic-diver AUV (depth -> pitch -> fins)

Purpose:
- Controls an AUV using a cascaded depth-to-pitch-to-fins architecture.
- Accepts guidance targets: target_depth (m), target_speed (m/s), target_heading (deg), optional target_depth_rate (m/s) or target_pitch_cmd (deg).
- Features outer loop (depth PID to pitch command), feed-forward pitch, inner loop (pitch PID to fin command), heading PID, speed PID, and X-tail fin mixing.
- Includes integrator anti-windup, rate-limited depth commands, and integrator resets for stability.
- Uses gain scheduling based on forward speed and integrates obstacle avoidance and sonar data for reactive control.
- Optimized for 10 Hz operation with scaled PID gains, improved numerical stability, and depth integrator reset on large command changes.

API expectations:
- vehicle_state.target_state: dict with guidance targets (depth/speed/heading)
- vehicle_state.nav_state: dict with current 'depth', 'speed', 'heading', 'pitch'
- vehicle_state.sensor_data: dict with 'sonar' containing {range, confidence, timestamp}
- vehicle_state.update_actuator_commands(thruster_cmd, UL, UR, LL, LR, ballast_cmd)
- vehicle_state.update_control_errors(heading_error, depth_error, speed_error, position_error)
- vehicle_state.obstacle_avoidance: Optional ObstacleAvoidance instance

Coordinate System:
- Nautical convention: 0 deg = North, 90 deg = East, 180 deg = South, 270 deg = West (clockwise)
- ENU coordinates: East (x), North (y), Up (z)
"""

import math
from typing import Dict, Any
from vehicle_state import VehicleState


class VehicleControl:
    def __init__(self, vehicle_state: VehicleState):
        """Initialize cascaded controller with vehicle state object"""
        self.vehicle_state = vehicle_state

        # Get control gains from vehicle state
        loaded_gains = vehicle_state.control_gains.get('gains', {})
        # Nominal 10 Hz tuned gains for cascaded architecture
        tuned_10hz_defaults = {
            'depth':   {'kp': 0.80, 'ki': 0.05,  'kd': 0.20},   # deg per meter (outer)
            'pitch':   {'kp': 2.00, 'ki': 0.02,  'kd': 0.25},   # fin authority mapping (inner)
            'heading': {'kp': 0.26, 'ki': 0.026, 'kd': 0.06},   # Scaled for 10 Hz
            'speed':   {'kp': 30.0, 'ki': 0.25,  'kd': 0.30},   # Aligned with verify_gains_loaded
        }

        # Obstacle avoidance integration settings
        self.obstacle_avoidance_enabled = vehicle_state.control_gains.get('obstacle_avoidance_enabled', True)
        # Patched defaults: realistic slow-down band for your logs (70–100m typical -> act by ~50m)
        self.sonar_speed_reduction_threshold = vehicle_state.control_gains.get('sonar_speed_reduction_threshold', 50.0)  # m
        self.sonar_speed_reduction_factor = vehicle_state.control_gains.get('sonar_speed_reduction_factor', 0.6)

        # Merge loaded gains with 10 Hz defaults
        self.gains = {}
        for axis in ['depth', 'pitch', 'heading', 'speed']:
            if axis in loaded_gains:
                if isinstance(loaded_gains[axis], list) and len(loaded_gains[axis]) == 3:
                    kp, ki, kd = loaded_gains[axis]
                    self.gains[axis] = {'kp': kp, 'ki': ki, 'kd': kd}
                else:
                    self.gains[axis] = dict(tuned_10hz_defaults[axis], **loaded_gains[axis])
            else:
                self.gains[axis] = tuned_10hz_defaults[axis]

        print("=" * 50)
        print("CASCADED VEHICLE CONTROL GAINS LOADED (10 Hz SCALED):")
        print("=" * 50)
        for axis, gains in self.gains.items():
            kp = gains.get('kp', 0.0)
            ki = gains.get('ki', 0.0)
            kd = gains.get('kd', 0.0)
            print(f"  {axis.upper()}: Kp={kp:4.2f}, Ki={ki:4.3f}, Kd={kd:4.2f}")
        print(f"  Obstacle Avoidance: {'ENABLED' if self.obstacle_avoidance_enabled else 'DISABLED'}")
        print(f"  Sonar Speed Reduction: threshold={self.sonar_speed_reduction_threshold}m, factor={self.sonar_speed_reduction_factor}")
        print("=" * 50)

        # Integrators, previous errors, and derivative smoothing
        self.integrators = {'depth': 0.0, 'pitch': 0.0, 'heading': 0.0, 'speed': 0.0}
        self.prev_errors = {'depth': 0.0, 'pitch': 0.0, 'heading': 0.0, 'speed': 0.0}
        self.smooth_derivatives = {'depth': 0.0, 'pitch': 0.0, 'heading': 0.0, 'speed': 0.0}

        # Integrator limits (anti-windup)
        self.integrator_limits = {
            'depth': 20.0,
            'pitch': 30.0,
            'heading': 5.0,
            'speed': 10.0
        }

        # Depth command rate limiting
        self.prev_depth_cmd = 0.0
        self.max_depth_rate = 2.0  # m/s

        # Integrator reset threshold for depth commands
        self.depth_reset_threshold = 5.0  # m

        # Pitch and fin actuators limits
        self.pitch_cmd_limit = 40.0   # degrees
        self.fin_limit_deg = 30.0     # degrees

        # Gain scheduling / feedforward params
        self.speed_nominal = 0.6          # m/s
        self.min_speed_for_feedforward = 0.1

        # Track last waypoint and obstacle avoidance state
        self.last_waypoint_global = None   # (lat, lon, depth)
        self.last_waypoint_local = None    # (local_x, local_y, depth)
        self.last_oa_state = "NORMAL"

        print("VehicleControl (cascaded depth->pitch->fins) initialized")
        print(f"  Pitch command limit: +/-{self.pitch_cmd_limit} deg")
        print(f"  Fin deflection limit: +/-{self.fin_limit_deg} deg")
        print(f"  Depth rate limiting: {self.max_depth_rate} m/s")
        print(f"  Integrator reset threshold: {self.depth_reset_threshold}m")

    def _normalize_heading_error(self, error: float) -> float:
        """Normalize heading error to [-180, 180] degrees"""
        while error > 180.0:
            error -= 360.0
        while error < -180.0:
            error += 360.0
        return error

    def _check_for_waypoint_change(self):
        """Check if waypoint changed and reset integrators if needed (global AND local)"""
        t = self.vehicle_state.target_state

        current_wp_global = (
            t.get('target_lat'),
            t.get('target_lon'),
            t.get('target_depth')
        )
        current_wp_local = (
            t.get('target_local_x'),
            t.get('target_local_y'),
            t.get('target_depth')
        )

        global_changed = (
            current_wp_global != self.last_waypoint_global and
            current_wp_global[0] is not None and current_wp_global[1] is not None
        )
        local_changed = (
            current_wp_local != self.last_waypoint_local and
            current_wp_local[0] is not None and current_wp_local[1] is not None
        )

        if global_changed or local_changed:
            self.reset_integrators()
            print("VehicleControl: New waypoint detected -> integrators reset")
            if global_changed:
                lat, lon, d = current_wp_global
                print(f"  New waypoint (global): ({lat:.6f}, {lon:.6f}, {d:.1f}m)")
            if local_changed:
                lx, ly, d = current_wp_local
                print(f"  New waypoint (local):  (x={lx:.1f}m, y={ly:.1f}m, {d:.1f}m)")

        self.last_waypoint_global = current_wp_global
        self.last_waypoint_local = current_wp_local

    def _check_for_oa_state_change(self):
        """Check for obstacle avoidance state changes and reset integrators"""
        if hasattr(self.vehicle_state, 'obstacle_avoidance') and self.vehicle_state.obstacle_avoidance:
            current_oa_state = self.vehicle_state.obstacle_avoidance.state
            if current_oa_state != self.last_oa_state:
                self.reset_integrators()
                print(f"VehicleControl: Obstacle avoidance state changed: {self.last_oa_state} -> {current_oa_state}")
                self.last_oa_state = current_oa_state

    def _rate_limit_depth_command(self, new_cmd: float, dt: float) -> float:
        """Rate limit depth commands to prevent aggressive changes at 10 Hz"""
        if dt <= 0.0:
            return new_cmd
        max_change = self.max_depth_rate * dt
        cmd_change = new_cmd - self.prev_depth_cmd
        if abs(cmd_change) > max_change:
            limited_cmd = self.prev_depth_cmd + math.copysign(max_change, cmd_change)
        else:
            limited_cmd = new_cmd
        self.prev_depth_cmd = limited_cmd
        return limited_cmd

    def _check_depth_integrator_reset(self, cmd_depth: float, prev_cmd_before_rate: float):
        """Reset depth integrator on large command changes (compare to pre-rate-limit value)"""
        if abs(cmd_depth - prev_cmd_before_rate) > self.depth_reset_threshold:
            old_int = self.integrators['depth']
            self.integrators['depth'] = 0.0
            self.prev_errors['depth'] = 0.0
            self.smooth_derivatives['depth'] = 0.0
            print(f"VehicleControl: depth integrator reset (Δcmd {cmd_depth - prev_cmd_before_rate:.2f} m)")
            print(f"  old integrator: {old_int:.3f} -> 0.0")

    def _pid_generic(self, axis: str, error: float, dt: float) -> float:
        """Generic PID helper for heading/pitch/speed inner loops"""
        if dt <= 0.0:
            return 0.0

        g = self.gains.get(axis, {'kp': 0.0, 'ki': 0.0, 'kd': 0.0})

        self.integrators[axis] += error * dt
        ilim = self.integrator_limits.get(axis, None)
        if ilim is not None:
            self.integrators[axis] = max(-ilim, min(ilim, self.integrators[axis]))

        raw_deriv = (error - self.prev_errors[axis]) / dt
        alpha = 0.8
        self.smooth_derivatives[axis] = alpha * self.smooth_derivatives[axis] + (1 - alpha) * raw_deriv
        deriv = self.smooth_derivatives[axis]

        self.prev_errors[axis] = error

        output = g['kp'] * error + g['ki'] * self.integrators[axis] + g['kd'] * deriv
        return output

    def _is_oa_active(self) -> bool:
        """
        Return True when OA should have priority over guidance.
        Per your requirement: OA is active during SCAN, APPROACHING, AVOID_LEFT, AVOID_RIGHT.
        OA influence stops in CLEARING.
        """
        if not (self.obstacle_avoidance_enabled and
                hasattr(self.vehicle_state, 'obstacle_avoidance') and
                self.vehicle_state.obstacle_avoidance):
            return False
        oa_state = self.vehicle_state.obstacle_avoidance.state
        if oa_state is None:
            return False
        if oa_state.startswith("AVOID_"):
            return True
        if oa_state in {"SCAN", "APPROACHING"}:
            return True
        # Explicitly stop OA influence in CLEARING (your choice)
        return False

    def update(self, dt: float = 0.1) -> Dict[str, Any]:
        """
        Main control update for cascaded architecture. Call at 10 Hz (dt=0.1).
        Prioritizes obstacle avoidance targets and uses sonar data for speed adjustment.
        """
        if dt <= 0.0:
            dt = 0.1

        # Check for waypoint and obstacle avoidance state changes
        self._check_for_waypoint_change()
        self._check_for_oa_state_change()

        # Read targets and current state
        target = self.vehicle_state.target_state
        current = self.vehicle_state.nav_state

        cmd_heading = target.get('target_heading', 0.0)
        raw_cmd_depth = target.get('target_depth', 0.0)
        cmd_speed = target.get('target_speed', 0.0)

        # Prioritize obstacle avoidance targets if OA active
        oa_active = self._is_oa_active()
        if oa_active:
            # Prefer OA's targets (OA writes into target_state)
            cmd_heading = target.get('target_heading', cmd_heading)
            raw_cmd_depth = target.get('target_depth', raw_cmd_depth)
            cmd_speed   = target.get('target_speed', cmd_speed)
            # Optionally soften heading integral to avoid fighting OA-induced turns
            self.integrators['heading'] *= 0.3

        # Adjust speed based on sonar proximity (confidence is 0-100%)
        sonar_pkt = self.vehicle_state.sensor_data.get("sonar", {})
        sonar_range = sonar_pkt.get("range", float('inf'))
        sonar_confidence = sonar_pkt.get("confidence", 0.0)
        if (self.obstacle_avoidance_enabled and
                sonar_confidence >= 30.0 and
                sonar_range < self.sonar_speed_reduction_threshold):
            # Scale reduction with proximity; clamp min scale to 0.2
            proximity = max(0.0, min(1.0, 1.0 - sonar_range / self.sonar_speed_reduction_threshold))
            speed_reduction = self.sonar_speed_reduction_factor * proximity
            new_speed = cmd_speed * max(0.2, 1.0 - speed_reduction)
            if new_speed < cmd_speed:
                print(f"VehicleControl: Sonar proximity ({sonar_range:.1f}m, conf={sonar_confidence:.1f}%) -> "
                      f"speed reduced {cmd_speed:.2f} -> {new_speed:.2f} m/s")
                cmd_speed = new_speed

        # Rate-limit depth command and protect integrator (use pre-rate-limit snapshot)
        prev_before_rate = self.prev_depth_cmd
        cmd_depth = self._rate_limit_depth_command(raw_cmd_depth, dt)
        self._check_depth_integrator_reset(cmd_depth, prev_before_rate)

        current_heading = current.get('heading', 0.0)
        current_depth = current.get('depth', 0.0)
        current_speed = max(0.0, current.get('speed', 0.0))
        current_pitch = current.get('pitch', 0.0)

        # OUTER LOOP: depth -> pitch_cmd (with speed-based gain scheduling)
        depth_error = cmd_depth - current_depth
        gain_factor = current_speed / self.speed_nominal if self.speed_nominal > 0.0 else 1.0
        gain_factor = max(0.5, min(2.0, gain_factor))

        depth_kp = self.gains['depth']['kp'] * gain_factor
        depth_ki = self.gains['depth']['ki'] * gain_factor
        depth_kd = self.gains['depth']['kd'] * gain_factor

        self.integrators['depth'] += depth_error * dt
        self.integrators['depth'] = max(-self.integrator_limits['depth'],
                                        min(self.integrator_limits['depth'], self.integrators['depth']))
        raw_depth_deriv = (depth_error - self.prev_errors['depth']) / dt
        alpha = 0.8
        self.smooth_derivatives['depth'] = alpha * self.smooth_derivatives['depth'] + (1 - alpha) * raw_depth_deriv
        depth_deriv = self.smooth_derivatives['depth']
        self.prev_errors['depth'] = depth_error

        pid_pitch_cmd = depth_kp * depth_error + depth_ki * self.integrators['depth'] + depth_kd * depth_deriv

        # Feed-forward pitch
        depth_rate_cmd = target.get('target_depth_rate', (raw_cmd_depth - current_depth) * 0.1)
        speed_for_ff = max(self.min_speed_for_feedforward, current_speed)
        pitch_ff_rad = math.atan2(depth_rate_cmd, speed_for_ff)
        pitch_ff_deg = math.degrees(pitch_ff_rad)

        pitch_cmd_deg = pid_pitch_cmd + pitch_ff_deg
        # Limit pitch command to reasonable authority
        if abs(pitch_cmd_deg) > self.pitch_cmd_limit:
            pitch_cmd_deg = math.copysign(self.pitch_cmd_limit, pitch_cmd_deg)

        # INNER LOOP: pitch -> fin commands
        pitch_error = pitch_cmd_deg - current_pitch
        pitch_output = self._pid_generic('pitch', pitch_error, dt)
        fin_effect = pitch_output

        # HEADING and SPEED controllers
        heading_error = self._normalize_heading_error(cmd_heading - current_heading)
        yaw_output = self._pid_generic('heading', heading_error, dt)
        # Clamp yaw_output to fin authority to avoid mixing saturation bursts
        yaw_output = max(-self.fin_limit_deg, min(self.fin_limit_deg, yaw_output))

        speed_error = cmd_speed - current_speed
        thrust_output = self._pid_generic('speed', speed_error, dt)
        thruster_cmd = max(0.0, min(100.0, thrust_output))

        # X-tail mixing (UL/UR upper; LL/LR lower)
        yaw_cmd = yaw_output
        pitch_cmd = fin_effect

        fin_ul = pitch_cmd + yaw_cmd
        fin_ur = pitch_cmd - yaw_cmd
        fin_ll = -pitch_cmd - yaw_cmd
        fin_lr = -pitch_cmd + yaw_cmd

        def clamp(x, lo=-self.fin_limit_deg, hi=self.fin_limit_deg):
            return max(lo, min(hi, x))

        fin_ul = clamp(fin_ul)
        fin_ur = clamp(fin_ur)
        fin_ll = clamp(fin_ll)
        fin_lr = clamp(fin_lr)

        # Ballast passthrough
        ballast_cmd = target.get('ballast_cmd', 'OFF')

        # Update actuators
        self.vehicle_state.update_actuator_commands(
            thruster_cmd=thruster_cmd,
            UL=fin_ul,
            UR=fin_ur,
            LL=fin_ll,
            LR=fin_lr,
            ballast_cmd=ballast_cmd
        )

        # Update control errors
        self.vehicle_state.update_control_errors(
            heading_error=heading_error,
            depth_error=depth_error,
            speed_error=speed_error,
            position_error=math.sqrt(depth_error ** 2 + (heading_error * 0.1) ** 2)
        )

        # Diagnostics
        return {
            "thruster_cmd": thruster_cmd,
            "fins": {"UL": fin_ul, "UR": fin_ur, "LL": fin_ll, "LR": fin_lr},
            "heading_error": heading_error,
            "depth_error": depth_error,
            "speed_error": speed_error,
            "rate_limited_depth_cmd": cmd_depth,
            "raw_depth_cmd": raw_cmd_depth,
            "pitch_cmd_deg": pitch_cmd_deg,
            "current_pitch_deg": current_pitch,
            "pitch_error": pitch_error,
            "gains_used": {
                "depth": {'kp': depth_kp, 'ki': depth_ki, 'kd': depth_kd},
                "pitch": self.gains['pitch'],
                "heading": self.gains['heading'],
                "speed": self.gains['speed'],
            },
            "gain_scheduling_factor": gain_factor,
            "feedforward_pitch_deg": pitch_ff_deg,
            "obstacle_avoidance_active": oa_active,
            "sonar_range": sonar_range,
            "sonar_confidence": sonar_confidence
        }

    def reset_integrators(self):
        """Reset all PID integrators"""
        for k in self.integrators:
            self.integrators[k] = 0.0
        for k in self.prev_errors:
            self.prev_errors[k] = 0.0
        for k in self.smooth_derivatives:
            self.smooth_derivatives[k] = 0.0
        print("VehicleControl: All integrators manually reset")

    def set_gains(self, gains_dict: Dict[str, Dict[str, float]]):
        """Update control gains"""
        for axis, vals in gains_dict.items():
            if axis in self.gains:
                self.gains[axis].update(vals)
        print("VehicleControl gains updated (10 Hz scaled):")
        for axis, gains in self.gains.items():
            kp = gains.get('kp', 0.0)
            ki = gains.get('ki', 0.0)
            kd = gains.get('kd', 0.0)
            print(f"  {axis}: kp={kp}, ki={ki}, kd={kd}")

    def get_controller_status(self) -> Dict[str, Any]:
        """Get current controller state for debugging"""
        status = {
            'gains': self.gains,
            'integrators': self.integrators.copy(),
            'prev_errors': self.prev_errors.copy(),
            'integrator_limits': self.integrator_limits.copy(),
            'last_waypoint_global': self.last_waypoint_global,
            'last_waypoint_local': self.last_waypoint_local,
            'loaded_from_file': bool(self.vehicle_state.control_gains.get('control_gains')),
            'update_rate': '10 Hz',
            'controller_type': 'cascaded_depth_pitch_fins',
            'gains_scaled_for': '10 Hz operation',
            'rate_limiting': {
                'depth_max_rate': self.max_depth_rate,
                'prev_depth_cmd': self.prev_depth_cmd
            },
            'integrator_reset': {
                'depth_reset_threshold': self.depth_reset_threshold,
                'enabled': True
            },
            'limits': {
                'pitch_cmd_limit': self.pitch_cmd_limit,
                'fin_limit_deg': self.fin_limit_deg
            },
            'gain_scheduling': {
                'speed_nominal': self.speed_nominal,
                'min_speed_for_feedforward': self.min_speed_for_feedforward
            },
            'obstacle_avoidance': {
                'enabled': self.obstacle_avoidance_enabled,
                'state': (self.vehicle_state.obstacle_avoidance.state
                          if hasattr(self.vehicle_state, 'obstacle_avoidance') and
                             self.vehicle_state.obstacle_avoidance else "N/A"),
                'sonar_speed_reduction': {
                    'threshold': self.sonar_speed_reduction_threshold,
                    'factor': self.sonar_speed_reduction_factor
                }
            }
        }
        sonar_pkt = self.vehicle_state.sensor_data.get("sonar", {})
        status['sonar'] = {
            'range': sonar_pkt.get("range", float('inf')),
            'confidence': sonar_pkt.get("confidence", 0.0),
            'timestamp': sonar_pkt.get("timestamp", 0.0)
        }
        return status

    def verify_gains_loaded(self):
        """Debug method to verify 10 Hz cascaded gains"""
        print("\n" + "="*60)
        print("CASCADED CONTROL GAIN VERIFICATION (10 Hz NOMINAL)")
        print("="*60)
        print("Expected cascaded gains:")
        print("  DEPTH:   kp=0.80,  ki=0.05, kd=0.20 (outer: m->deg)")
        print("  PITCH:   kp=2.00,  ki=0.02, kd=0.25 (inner: deg->deg)")
        print("  HEADING: kp=0.26,  ki=0.026, kd=0.06")
        print("  SPEED:   kp=30.0,  ki=0.25, kd=0.30")
        print("\nActually loaded gains:")
        for axis, g in self.gains.items():
            kp = g.get('kp', 0.0)
            ki = g.get('ki', 0.0)
            kd = g.get('kd', 0.0)
            print(f"  {axis.upper():8}: kp={kp:.4f}, ki={ki:.4f}, kd={kd:.4f}")

        expected_cascaded = {
            'depth': {'kp': 0.80, 'ki': 0.05, 'kd': 0.20},
            'pitch': {'kp': 2.00, 'ki': 0.02, 'kd': 0.25},
            'heading': {'kp': 0.26, 'ki': 0.026, 'kd': 0.06},
            'speed': {'kp': 30.0, 'ki': 0.25, 'kd': 0.30}
        }

        gains_correct = True
        for axis, exp_gains in expected_cascaded.items():
            actual_gains = self.gains[axis]
            for param, exp_val in exp_gains.items():
                actual_val = actual_gains.get(param, 0.0)
                if abs(exp_val - actual_val) > 0.001:
                    gains_correct = False
                    print(f"  WARNING: {axis} {param} mismatch - expected {exp_val}, got {actual_val}")

        if gains_correct:
            print("[OK] All cascaded 10 Hz gains loaded correctly!")
        else:
            print("[ERROR] Some gains don't match expected cascaded values!")
            print("Check gain configuration for cascaded depth->pitch->fins architecture")

        print(f"Controller type: cascaded depth->pitch->fins")
        print(f"Obstacle avoidance integration: {'ENABLED' if self.obstacle_avoidance_enabled else 'DISABLED'}")
        print(f"Pitch command limits: +/-{self.pitch_cmd_limit} deg")
        print(f"Fin deflection limits: +/-{self.fin_limit_deg} deg")
        print(f"Rate limiting: depth commands limited to +/-{self.max_depth_rate} m/s")
        print(f"Integrator reset threshold: {self.depth_reset_threshold}m")
        print(f"Gain scheduling: nominal speed {self.speed_nominal} m/s")
        print("="*60)
