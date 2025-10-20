#!/usr/bin/env python3
"""
obstacle_avoidance.py - SAFE Multi-Obstacle Avoidance for SIMPLR AUV

REALISTIC SONAR ASSUMPTIONS:
- Forward sonar only provides: distance and confidence
- NO information about obstacle shape, size, or vertical extent
- NO altitude sensor (don't know distance to bottom)

SAFE AVOIDANCE STRATEGY:
1. PRIMARY: Horizontal turning (always safe)
2. SECONDARY: Depth changes ONLY if:
   - Not in SURFACED_TRIM state (can't dive from surface)
   - Not at minimum depth (can't go up from surface)
   - Have known safe depth limits from mission
   - Horizontal turning is failing

FIXES APPLIED:
- Earlier detection (50m threshold)
- No flip-flopping (3s minimum between reversals)
- Committed turn execution (5s minimum)
- Smart strategy switching
- Respects vehicle state constraints
- Conservative depth changes only as last resort
"""

import time
import csv
from typing import Dict, Any, Optional, List
from vehicle_state import VehicleState, SystemState


class ObstacleEvent:
    """Tracks individual obstacle encounters"""
    def __init__(self, distance: float, confidence: float, timestamp: float):
        self.distance = distance
        self.confidence = confidence
        self.timestamp = timestamp
        self.avoidance_start_time = None
        self.avoidance_duration = 0.0
        self.cleared = False


class ObstacleAvoidance:
    """Safe multi-obstacle avoidance using single forward sonar"""

    def __init__(self, vehicle_state: VehicleState, config: Optional[Dict[str, Any]] = None):
        self.vehicle_state = vehicle_state
        cfg = config or {}

        # Core detection parameters
        self.enabled = cfg.get("enabled", True)
        self.detection_threshold = cfg.get("detection_threshold", 50.0)
        self.clear_threshold = cfg.get("clear_threshold", 60.0)
        self.confidence_threshold = cfg.get("confidence_threshold", 60.0)
        self.turn_rate = cfg.get("turn_rate", 15.0)
        self.speed_reduction_factor = cfg.get("speed_reduction_factor", 0.5)
        
        # Sonar parameters
        self.max_range = cfg.get("max_range", 100.0)
        self.out_of_beam_confidence = cfg.get("out_of_beam_confidence", 10.0)

        # SAFE depth avoidance parameters
        self.avoidance_mode = cfg.get("avoidance_mode", "horizontal")  # Default: horizontal only
        self.depth_avoidance_offset = cfg.get("depth_avoidance_offset", 3.0)
        self.min_avoidance_depth = cfg.get("min_avoidance_depth", 0.5)
        self.max_avoidance_depth = cfg.get("max_avoidance_depth", 15.0)
        self.use_depth_as_last_resort = cfg.get("use_depth_as_last_resort", False)

        # Turn commitment parameters
        self.min_turn_commitment_time = cfg.get("min_turn_commitment_time", 5.0)
        self.min_reversal_interval = cfg.get("min_reversal_interval", 3.0)
        self.last_reversal_time = 0.0
        self.current_turn_start_time = 0.0
        self.turn_committed = False
        self.min_heading_change_for_commit = cfg.get("min_heading_change", 45.0)
        self.heading_at_turn_start = None

        # Maneuver failure detection
        self.maneuver_failing_threshold = cfg.get("maneuver_failing_threshold", 5.0)  # 5s
        self.time_approaching_obstacle = 0.0
        self.last_distance_check = None
        self.maneuver_failure_count = 0
        self.max_maneuver_failures = 3
        self.tried_depth_avoidance = False

        # Multi-obstacle tracking
        self.obstacle_cooldown = cfg.get("obstacle_cooldown", 10.0)
        self.min_obstacle_separation = cfg.get("min_obstacle_separation", 5.0)
        
        # Mission context
        self.original_mission_context = None

        # CSV logging
        self.csv_logging_enabled = cfg.get("csv_logging", True)
        self.csv_file = None
        self.csv_writer = None
        self.log_counter = 0

        # Internal state
        self.state = "NORMAL"
        self.current_obstacle = None
        self.obstacle_history: List[ObstacleEvent] = []
        
        # Current sonar reading
        self.sonar_distance: Optional[float] = None
        self.sonar_confidence: Optional[float] = None
        self.prev_sonar_distance: Optional[float] = None
        self.prev_sonar_confidence: Optional[float] = None
        self.sonar_timestamp: float = 0.0

        # Avoidance parameters
        self.turn_direction: Optional[str] = None
        self.depth_direction: Optional[str] = None
        self.original_heading: Optional[float] = None
        self.original_speed: Optional[float] = None
        self.original_depth: Optional[float] = None
        self.avoidance_start_time: Optional[float] = None
        self.using_depth_avoidance = False

        # Statistics
        self.total_avoidances = 0
        self.total_avoidance_time = 0.0
        self.direction_reversals = 0
        self.current_avoidance_id = 0

        if self.csv_logging_enabled:
            self._init_csv_logging()

        print("=" * 70)
        print("SAFE MULTI-OBSTACLE AVOIDANCE SYSTEM INITIALIZED")
        print("=" * 70)
        print(f"  Status: {'ENABLED' if self.enabled else 'DISABLED'}")
        print(f"  Primary strategy: HORIZONTAL TURNING")
        print(f"  Secondary strategy: {'DEPTH (last resort)' if self.use_depth_as_last_resort else 'DISABLED'}")
        print(f"  Detection: <{self.detection_threshold}m, Clear: >{self.clear_threshold}m")
        print(f"  Confidence: ≥{self.confidence_threshold}%")
        print(f"  Turn rate: {self.turn_rate}°/s, Speed reduction: {self.speed_reduction_factor}x")
        print(f"  Turn commitment: {self.min_turn_commitment_time}s minimum")
        print(f"  Safe depth range: {self.min_avoidance_depth}m - {self.max_avoidance_depth}m")
        print("=" * 70)

    def update_sonar(self, sonar_pkt: Any):
        """Accept sonar packet"""
        try:
            self.prev_sonar_distance = self.sonar_distance
            self.prev_sonar_confidence = self.sonar_confidence

            if isinstance(sonar_pkt, dict) and "range" in sonar_pkt:
                rng = sonar_pkt.get("range", None)
                conf = sonar_pkt.get("confidence", 100.0)
                self.sonar_distance = float(rng) if rng is not None else None
                self.sonar_confidence = float(conf) if conf is not None else None
                self.sonar_timestamp = float(sonar_pkt.get("timestamp", time.time()))
            else:
                self.sonar_distance = float(sonar_pkt) if sonar_pkt is not None else None
                self.sonar_confidence = 100.0
                self.sonar_timestamp = time.time()
                
        except Exception as e:
            print(f"ObstacleAvoidance: Sonar packet error: {e}")
            self.sonar_distance = None
            self.sonar_confidence = None

    def _is_new_obstacle(self, distance: float, confidence: float) -> bool:
        """Determine if this is a new obstacle"""
        if self.state == "AVOIDING":
            return False
            
        if distance is None:
            return False
            
        current_time = time.time()
        for obstacle in self.obstacle_history:
            if (not obstacle.cleared and 
                current_time - obstacle.timestamp < self.obstacle_cooldown and
                abs(obstacle.distance - distance) < self.min_obstacle_separation):
                return False
                
        return True

    def update(self, dt: float = 0.1):
        if not self.enabled:
            self._log_to_csv()
            return
        
        self._clean_obstacle_history()

        if self.state == "NORMAL":
            self._handle_normal_state()
        elif self.state == "AVOIDING":
            self._handle_avoiding_state(dt)
        elif self.state == "CLEARING":
            self._handle_clearing_state()

        self._log_to_csv()

    def _clean_obstacle_history(self):
        """Remove old obstacles from history"""
        current_time = time.time()
        self.obstacle_history = [
            obs for obs in self.obstacle_history 
            if current_time - obs.timestamp < self.obstacle_cooldown * 2
        ]

    def _handle_normal_state(self):
        """Check for new obstacles"""
        if (self.sonar_distance is not None and 
            self.sonar_distance < self.detection_threshold and
            self.sonar_confidence is not None and
            self.sonar_confidence >= self.confidence_threshold and
            self._is_new_obstacle(self.sonar_distance, self.sonar_confidence)):
            
            self._enter_avoidance_mode()

    def _can_use_depth_avoidance(self) -> tuple[bool, str]:
        """
        Check if depth avoidance is safe and allowed.
        Returns (can_use, reason)
        """
        # Check if depth avoidance is even enabled
        if not self.use_depth_as_last_resort:
            return False, "Depth avoidance disabled (horizontal-only mode)"
        
        # Check vehicle system state
        try:
            current_state = self.vehicle_state.current_state
            if current_state == SystemState.SURFACED_TRIM:
                return False, "Vehicle in SURFACED_TRIM - cannot dive"
        except:
            pass
        
        # Check current depth
        nav = self.vehicle_state.nav_state
        current_depth = nav.get('depth', 0.0)
        
        # Too shallow to go up
        if current_depth <= self.min_avoidance_depth:
            return False, f"At minimum depth ({current_depth:.1f}m) - cannot go shallower"
        
        # Too deep to go down safely
        if current_depth >= self.max_avoidance_depth:
            return False, f"At maximum safe depth ({current_depth:.1f}m) - cannot go deeper"
        
        # Check if we have safe depth margins
        if self.depth_direction == "UP":
            target_depth = current_depth - self.depth_avoidance_offset
            if target_depth < self.min_avoidance_depth:
                return False, f"Going up would breach minimum depth"
        elif self.depth_direction == "DOWN":
            target_depth = current_depth + self.depth_avoidance_offset
            if target_depth > self.max_avoidance_depth:
                return False, f"Going down would breach maximum safe depth (no altitude sensor!)"
        
        return True, "Depth avoidance safe"

    def _enter_avoidance_mode(self):
        """Start avoiding a new obstacle"""
        self.state = "AVOIDING"
        self.avoidance_start_time = time.time()
        self.total_avoidances += 1
        self.current_avoidance_id += 1

        # Create new obstacle event
        self.current_obstacle = ObstacleEvent(
            distance=self.sonar_distance,
            confidence=self.sonar_confidence,
            timestamp=self.sonar_timestamp
        )
        self.current_obstacle.avoidance_start_time = self.avoidance_start_time
        self.obstacle_history.append(self.current_obstacle)

        # Save complete mission context
        self._save_mission_context()
        
        target = self.vehicle_state.target_state
        nav = self.vehicle_state.nav_state

        self.original_heading = target.get("target_heading", nav.get("heading", 0.0))
        self.original_speed = target.get("target_speed", nav.get("speed", 0.0))
        self.original_depth = target.get("target_depth", nav.get("depth", 0.0))

        # Initialize commitment tracking
        self.heading_at_turn_start = self.original_heading
        self.current_turn_start_time = time.time()
        self.turn_committed = False
        self.time_approaching_obstacle = 0.0
        self.last_distance_check = self.sonar_distance
        self.maneuver_failure_count = 0
        self.tried_depth_avoidance = False
        self.using_depth_avoidance = False

        # PRIMARY STRATEGY: Horizontal turning
        self.turn_direction = "LEFT" if (self.total_avoidances % 2 == 1) else "RIGHT"

        # SECONDARY STRATEGY: Depth (only if allowed and as last resort)
        self.depth_direction = None
        if self.use_depth_as_last_resort:
            can_use_depth, reason = self._can_use_depth_avoidance()
            if can_use_depth:
                # Prefer going up (safer than diving blind)
                if nav.get("depth", 0.0) > self.min_avoidance_depth + 2.0:
                    self.depth_direction = "UP"
                else:
                    self.depth_direction = "DOWN"
            else:
                print(f"  Note: {reason}")

        self.direction_reversals = 0

        print("\n" + "=" * 70)
        print(f"[OA #{self.current_avoidance_id}] OBSTACLE DETECTED at {self.sonar_distance:.1f}m")
        print(f"    Confidence: {self.sonar_confidence:.1f}%")
        print(f"    PRIMARY strategy: TURN {self.turn_direction}")
        if self.depth_direction:
            print(f"    SECONDARY strategy: DEPTH {self.depth_direction} (last resort)")
        else:
            print(f"    SECONDARY strategy: NONE (horizontal turning only)")
        print("=" * 70 + "\n")

    def _save_mission_context(self):
        """Save the complete mission context for proper restoration"""
        target = self.vehicle_state.target_state
        self.original_mission_context = {
            'target_heading': target.get("target_heading", 0.0),
            'target_speed': target.get("target_speed", 0.0),
            'target_depth': target.get("target_depth", 0.0),
            'distance_to_waypoint': target.get("distance_to_waypoint", 0.0),
            'mission_time': self.vehicle_state.mission_time
        }

    def _restore_mission_context(self):
        """Restore the complete mission context after avoidance"""
        if self.original_mission_context:
            self.vehicle_state.update_target_state(
                target_heading=self.original_mission_context['target_heading'],
                target_speed=self.original_mission_context['target_speed'],
                target_depth=self.original_mission_context['target_depth'],
                distance_to_waypoint=self.original_mission_context['distance_to_waypoint']
            )

    def _handle_avoiding_state(self, dt: float):
        """Execute avoidance maneuvers - SAFE strategy"""
        current_time = time.time()
        
        # Check if maneuver is failing (still approaching obstacle)
        if self.sonar_distance is not None and self.last_distance_check is not None:
            if self.sonar_distance < self.last_distance_check:
                self.time_approaching_obstacle += dt
                
                if self.time_approaching_obstacle >= self.maneuver_failing_threshold:
                    print(f"  ⚠ WARNING: Still approaching after {self.time_approaching_obstacle:.1f}s!")
                    print(f"    Distance: {self.last_distance_check:.1f}m → {self.sonar_distance:.1f}m")
                    
                    if self.maneuver_failure_count < self.max_maneuver_failures:
                        self._switch_avoidance_strategy()
                        self.time_approaching_obstacle = 0.0
                        self.maneuver_failure_count += 1
            else:
                # Distance is increasing, maneuver is working
                self.time_approaching_obstacle = 0.0
            
            self.last_distance_check = self.sonar_distance

        # Check if we've committed to this turn direction
        if not self.turn_committed:
            time_turning = current_time - self.current_turn_start_time
            nav = self.vehicle_state.nav_state
            current_heading = nav.get("heading", self.heading_at_turn_start)
            heading_change = abs(self._normalize_angle(current_heading - self.heading_at_turn_start))
            
            if (time_turning >= self.min_turn_commitment_time or 
                heading_change >= self.min_heading_change_for_commit):
                self.turn_committed = True
                print(f"  ✓ Turn committed after {time_turning:.1f}s (Δheading: {heading_change:.1f}°)")

        # Anti-flip-flop protection for turn reversals
        time_since_reversal = current_time - self.last_reversal_time
        time_in_current_direction = current_time - self.current_turn_start_time
        
        can_reverse = (
            time_since_reversal >= self.min_reversal_interval and
            time_in_current_direction >= self.min_turn_commitment_time and
            self.turn_committed
        )
        
        if (can_reverse and
            self.prev_sonar_distance is not None and 
            self.sonar_distance is not None and
            self.prev_sonar_confidence is not None and 
            self.sonar_confidence is not None and
            self.prev_sonar_confidence >= self.confidence_threshold):
            
            range_change = self.sonar_distance - self.prev_sonar_distance
            confidence_change = self.sonar_confidence - self.prev_sonar_confidence
            
            # Conservative reversal criteria
            if range_change < -0.5 and confidence_change > 2.0:
                self._reverse_turn_direction()
                print(f"  ⟲ Turn reversal (Range Δ={range_change:.2f}m, Conf Δ={confidence_change:.1f}%)")

        # EXECUTE MANEUVERS
        target = self.vehicle_state.target_state
        current_heading = target.get("target_heading", self.original_heading or 0.0)

        # PRIMARY: Horizontal turn (always executed)
        turn_increment = self.turn_rate * dt
        if self.turn_direction == "LEFT":
            new_heading = (current_heading + turn_increment) % 360.0
        else:
            new_heading = (current_heading - turn_increment) % 360.0
        target["target_heading"] = new_heading

        # SECONDARY: Depth change (only if safe and using it)
        if self.using_depth_avoidance and self.depth_direction:
            can_use, reason = self._can_use_depth_avoidance()
            if can_use:
                if self.depth_direction == "UP":
                    new_depth = max(
                        self.min_avoidance_depth,
                        (self.original_depth or 0.0) - self.depth_avoidance_offset,
                    )
                else:
                    new_depth = min(
                        self.max_avoidance_depth,
                        (self.original_depth or 0.0) + self.depth_avoidance_offset,
                    )
                target["target_depth"] = new_depth
            else:
                # Can no longer use depth, disable it
                print(f"  Note: Disabling depth avoidance - {reason}")
                self.using_depth_avoidance = False

        # Speed reduction
        if self.original_speed is not None:
            target["target_speed"] = self.original_speed * self.speed_reduction_factor

        # Check for clearance
        obstacle_cleared = False
        
        if self.sonar_distance is None:
            obstacle_cleared = True
        elif self.sonar_distance >= self.max_range * 0.95:
            obstacle_cleared = True
        elif (self.sonar_confidence is not None and 
              self.sonar_confidence < self.out_of_beam_confidence):
            obstacle_cleared = True
        elif (self.sonar_distance > self.clear_threshold and 
              self.sonar_confidence is not None and
              self.sonar_confidence >= self.confidence_threshold):
            obstacle_cleared = True
        
        if obstacle_cleared:
            self._enter_clearing_state()

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to -180 to +180 range"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def _switch_avoidance_strategy(self):
        """Switch avoidance strategy when current maneuver is failing"""
        print(f"  🔄 Switching strategy (failure #{self.maneuver_failure_count + 1})")
        
        # First try: Reverse turn direction
        if self.maneuver_failure_count == 0:
            self._reverse_turn_direction()
            print(f"    Strategy: Reverse turn to {self.turn_direction}")
        
        # Second try: Add depth avoidance (if safe)
        elif self.maneuver_failure_count == 1 and not self.tried_depth_avoidance:
            can_use, reason = self._can_use_depth_avoidance()
            if can_use:
                self.using_depth_avoidance = True
                self.tried_depth_avoidance = True
                print(f"    Strategy: Add DEPTH {self.depth_direction} to horizontal turn")
            else:
                print(f"    Strategy: Cannot use depth - {reason}")
                # Just increase turn rate instead
                self.turn_rate = min(self.turn_rate * 1.5, 30.0)
                print(f"    Strategy: Increase turn rate to {self.turn_rate:.1f}°/s")
        
        # Third try: More aggressive turning
        elif self.maneuver_failure_count == 2:
            self.turn_rate = min(self.turn_rate * 1.5, 30.0)
            print(f"    Strategy: Increase turn rate to {self.turn_rate:.1f}°/s")

    def _reverse_turn_direction(self):
        """Reverse turn direction with tracking"""
        old = self.turn_direction
        self.turn_direction = "RIGHT" if self.turn_direction == "LEFT" else "LEFT"
        self.direction_reversals += 1
        self.last_reversal_time = time.time()
        self.current_turn_start_time = time.time()
        self.turn_committed = False
        
        nav = self.vehicle_state.nav_state
        self.heading_at_turn_start = nav.get("heading", self.heading_at_turn_start)

    def _enter_clearing_state(self):
        """Transition to clearing state"""
        self.state = "CLEARING"
        duration = time.time() - (self.avoidance_start_time or time.time())
        self.total_avoidance_time += duration
        
        if self.current_obstacle:
            self.current_obstacle.cleared = True
            self.current_obstacle.avoidance_duration = duration

        clearance_reason = "no sonar" if self.sonar_distance is None else f"{self.sonar_distance:.1f}m"
        
        print("\n" + "=" * 70)
        print(f"[OA #{self.current_avoidance_id}] OBSTACLE CLEARED at {clearance_reason}")
        print(f"    Duration: {duration:.1f}s")
        print(f"    Reversals: {self.direction_reversals}")
        print(f"    Used depth: {'YES' if self.using_depth_avoidance else 'NO'}")
        print(f"    Failures: {self.maneuver_failure_count}")
        print("=" * 70 + "\n")

    def _handle_clearing_state(self):
        """Restore mission parameters and reset"""
        self._restore_mission_context()

        # Full reset
        self.state = "NORMAL"
        self.turn_direction = None
        self.depth_direction = None
        self.original_heading = None
        self.original_speed = None
        self.original_depth = None
        self.avoidance_start_time = None
        self.current_obstacle = None
        self.original_mission_context = None
        self.direction_reversals = 0
        self.turn_committed = False
        self.heading_at_turn_start = None
        self.current_turn_start_time = 0.0
        self.time_approaching_obstacle = 0.0
        self.last_distance_check = None
        self.maneuver_failure_count = 0
        self.tried_depth_avoidance = False
        self.using_depth_avoidance = False

        print(f"[OA] System reset - ready for next obstacle")

    def _init_csv_logging(self):
        try:
            ts = time.strftime("%Y%m%d_%H%M%S")
            fname = f"safe_obstacle_avoidance_log_{ts}.csv"
            self.csv_file = open(fname, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "timestamp", "avoidance_id", "state", "obstacle_count",
                "sonar_distance", "sonar_confidence", "vehicle_depth", 
                "vehicle_heading", "vehicle_speed", "target_heading", 
                "target_depth", "target_speed", "turn_direction", 
                "depth_direction", "using_depth", "direction_reversals", 
                "total_avoidances", "turn_committed", "time_approaching", 
                "maneuver_failures"
            ])
            print(f"  Safe CSV logging: {fname}")
        except Exception as e:
            print(f"  WARNING: CSV logging failed: {e}")
            self.csv_logging_enabled = False

    def _log_to_csv(self):
        if not self.csv_logging_enabled or not self.csv_writer:
            return
        try:
            nav = self.vehicle_state.nav_state
            tgt = self.vehicle_state.target_state
            self.csv_writer.writerow([
                time.time(),
                self.current_avoidance_id,
                self.state,
                len([o for o in self.obstacle_history if not o.cleared]),
                ("" if self.sonar_distance is None else f"{self.sonar_distance:.2f}"),
                ("" if self.sonar_confidence is None else f"{self.sonar_confidence:.2f}"),
                f"{nav.get('depth', 0.0):.2f}",
                f"{nav.get('heading', 0.0):.1f}",
                f"{nav.get('speed', 0.0):.2f}",
                f"{tgt.get('target_heading', 0.0):.1f}",
                f"{tgt.get('target_depth', 0.0):.2f}",
                f"{tgt.get('target_speed', 0.0):.2f}",
                (self.turn_direction or ""),
                (self.depth_direction or ""),
                ("Y" if self.using_depth_avoidance else "N"),
                self.direction_reversals,
                self.total_avoidances,
                ("Y" if self.turn_committed else "N"),
                f"{self.time_approaching_obstacle:.2f}",
                self.maneuver_failure_count
            ])
            self.log_counter += 1
            if self.log_counter % 10 == 0:
                self.csv_file.flush()
        except Exception as e:
            if self.log_counter % 100 == 0:
                print(f"CSV logging error: {e}")

    def get_status(self) -> Dict[str, Any]:
        """Get current avoidance system status"""
        return {
            'state': self.state,
            'enabled': self.enabled,
            'current_obstacle_distance': self.sonar_distance,
            'current_obstacle_confidence': self.sonar_confidence,
            'total_avoidances': self.total_avoidances,
            'active_obstacles': len([o for o in self.obstacle_history if not o.cleared]),
            'turn_direction': self.turn_direction,
            'using_depth': self.using_depth_avoidance,
            'avoidance_id': self.current_avoidance_id
        }

    def reset_system(self):
        """Force reset the entire avoidance system"""
        print("[OA] Manual system reset")
        self.state = "NORMAL"
        self.current_obstacle = None
        self.obstacle_history.clear()
        self.turn_direction = None
        self.depth_direction = None
        self.original_heading = None
        self.original_speed = None
        self.original_depth = None
        self.avoidance_start_time = None
        self.original_mission_context = None
        self.direction_reversals = 0
        self.turn_committed = False
        self.heading_at_turn_start = None
        self.current_turn_start_time = 0.0
        self.time_approaching_obstacle = 0.0
        self.last_distance_check = None
        self.maneuver_failure_count = 0
        self.tried_depth_avoidance = False
        self.using_depth_avoidance = False
        self._restore_mission_context()

    def enable(self):
        if not self.enabled:
            self.enabled = True
            print("Obstacle avoidance ENABLED")
    
    def disable(self):
        if self.enabled:
            self.enabled = False
            if self.state != 'NORMAL':
                self._handle_clearing_state()
            print("Obstacle avoidance DISABLED")
    
    def close(self):
        if self.csv_file is not None:
            try:
                self.csv_file.close()
                print("Obstacle avoidance CSV log closed")
            except Exception as e:
                print(f"Error closing CSV log: {e}")
    
    def __del__(self):
        self.close()


if __name__ == '__main__':
    print("SAFE ObstacleAvoidance module")
    print("\nSafety features:")
    print("  1. ✓ Respects SURFACED_TRIM state (no diving from surface)")
    print("  2. ✓ Checks depth limits before vertical maneuvers")
    print("  3. ✓ No blind diving (no altitude sensor = dangerous)")
    print("  4. ✓ Horizontal turning is PRIMARY strategy")
    print("  5. ✓ Depth changes only as LAST RESORT")
    print("  6. ✓ Earlier detection (50m)")
    print("  7. ✓ No flip-flopping (3s minimum between reversals)")
    print("  8. ✓ Committed turn execution (5s minimum)")