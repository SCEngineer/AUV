#!/usr/bin/env python3
"""
servo_controller.py - Servo conditioning and control for SIMPLR AUV
Handles slew rate limiting, deadband, and safe servo operation
"""

import time
from typing import Dict, Optional


class ServoController:
    """
    Manages servo conditioning for safe, stable operation
    
    Features:
    - Slew rate limiting (prevents rapid position changes)
    - Deadband/hysteresis (prevents jitter from small commands)
    - Position limits (enforces safe servo range)
    - Per-servo configuration
    """
    
    def __init__(self, config: Optional[Dict] = None):
        """
        Initialize servo controller
        
        Args:
            config: Optional configuration dict with keys:
                - max_slew_rate: degrees per second (default: 180)
                - deadband: degrees (default: 0.5)
                - min_angle: minimum servo angle (default: -45)
                - max_angle: maximum servo angle (default: 45)
                - update_rate: Hz (default: 50)
        """
        config = config or {}
        
        # Slew rate limiting (degrees per second)
        # 180 deg/s = 0.25 seconds for full sweep, safe for most servos
        self.max_slew_rate = float(config.get('max_slew_rate', 180.0))
        
        # Deadband (degrees) - ignore commands smaller than this
        # Prevents servo jitter from noise or small PID oscillations
        self.deadband = float(config.get('deadband', 0.5))
        
        # Position limits (degrees)
        self.min_angle = float(config.get('min_angle', -45.0))
        self.max_angle = float(config.get('max_angle', 45.0))
        
        # Update rate (Hz) - how often servo positions are updated
        self.update_rate = float(config.get('update_rate', 50.0))
        self.dt = 1.0 / self.update_rate
        
        # Current servo positions (actual hardware position)
        self.current_positions: Dict[str, float] = {}
        
        # Target positions (where control wants servo to be)
        self.target_positions: Dict[str, float] = {}
        
        # Last update time
        self.last_update_time = time.time()
        
        # Statistics
        self.stats = {
            'slew_limited_count': 0,
            'deadband_filtered_count': 0,
            'limit_clipped_count': 0
        }
    
    def initialize_servo(self, name: str, initial_position: float = 0.0):
        """
        Initialize a servo to a starting position
        
        Args:
            name: Servo name (e.g., 'UL', 'UR', 'LR', 'LL')
            initial_position: Starting angle in degrees
        """
        initial_position = self._clamp_angle(initial_position)
        self.current_positions[name] = initial_position
        self.target_positions[name] = initial_position
    
    def set_target(self, name: str, target_angle: float):
        """
        Set target position for a servo (will be conditioned)
        
        Args:
            name: Servo name
            target_angle: Desired angle in degrees
        """
        # Clamp to safe limits
        clamped = self._clamp_angle(target_angle)
        if clamped != target_angle:
            self.stats['limit_clipped_count'] += 1
        
        self.target_positions[name] = clamped
    
    def update(self, dt: Optional[float] = None) -> Dict[str, float]:
        """
        Update all servo positions with slew rate limiting and deadband
        
        Args:
            dt: Time step in seconds (if None, uses actual elapsed time)
        
        Returns:
            Dict of servo names to conditioned positions (degrees)
        """
        # Calculate time step
        if dt is None:
            current_time = time.time()
            dt = current_time - self.last_update_time
            self.last_update_time = current_time
        
        # Update each servo
        for name in self.current_positions.keys():
            target = self.target_positions.get(name, self.current_positions[name])
            current = self.current_positions[name]
            
            # Calculate error
            error = target - current
            
            # Apply deadband - ignore small movements
            if abs(error) < self.deadband:
                self.stats['deadband_filtered_count'] += 1
                continue  # Don't move servo
            
            # Calculate maximum movement allowed this time step
            max_movement = self.max_slew_rate * dt
            
            # Apply slew rate limiting
            if abs(error) > max_movement:
                # Limit movement to max slew rate
                movement = max_movement if error > 0 else -max_movement
                self.current_positions[name] = current + movement
                self.stats['slew_limited_count'] += 1
            else:
                # Can reach target this step
                self.current_positions[name] = target
        
        return self.current_positions.copy()
    
    def get_position(self, name: str) -> float:
        """Get current conditioned position of a servo"""
        return self.current_positions.get(name, 0.0)
    
    def get_all_positions(self) -> Dict[str, float]:
        """Get all current conditioned positions"""
        return self.current_positions.copy()
    
    def is_at_target(self, name: str, tolerance: float = 0.1) -> bool:
        """
        Check if servo has reached its target position
        
        Args:
            name: Servo name
            tolerance: Acceptable error in degrees
        
        Returns:
            True if servo is within tolerance of target
        """
        current = self.current_positions.get(name, 0.0)
        target = self.target_positions.get(name, 0.0)
        return abs(current - target) <= tolerance
    
    def all_at_target(self, tolerance: float = 0.1) -> bool:
        """Check if all servos have reached their targets"""
        return all(self.is_at_target(name, tolerance) 
                  for name in self.current_positions.keys())
    
    def emergency_stop(self):
        """Stop all servos immediately (no slew limiting)"""
        for name in self.current_positions.keys():
            self.target_positions[name] = self.current_positions[name]
    
    def center_all(self):
        """Command all servos to center position (0 degrees)"""
        for name in self.current_positions.keys():
            self.target_positions[name] = 0.0
    
    def get_stats(self) -> Dict[str, int]:
        """Get statistics about servo conditioning"""
        return self.stats.copy()
    
    def reset_stats(self):
        """Reset statistics counters"""
        self.stats = {
            'slew_limited_count': 0,
            'deadband_filtered_count': 0,
            'limit_clipped_count': 0
        }
    
    def _clamp_angle(self, angle: float) -> float:
        """Clamp angle to safe servo limits"""
        return max(self.min_angle, min(self.max_angle, angle))


# Example usage and testing
if __name__ == "__main__":
    print("Servo Controller Test")
    print("=" * 60)
    
    # Create servo controller with custom config
    config = {
        'max_slew_rate': 90.0,  # 90 deg/s = slower, more conservative
        'deadband': 1.0,         # 1 degree deadband
        'min_angle': -45.0,
        'max_angle': 45.0,
        'update_rate': 50.0      # 50 Hz update rate
    }
    
    controller = ServoController(config)
    
    # Initialize X-tail servos
    for servo in ['UL', 'UR', 'LR', 'LL']:
        controller.initialize_servo(servo, 0.0)
    
    print(f"\nInitialized 4 servos at center (0°)")
    print(f"Max slew rate: {config['max_slew_rate']} deg/s")
    print(f"Deadband: {config['deadband']}°")
    
    # Test 1: Large command change
    print("\n" + "-" * 60)
    print("Test 1: Large command (0° → 45°) with slew limiting")
    print("-" * 60)
    
    controller.set_target('UL', 45.0)
    
    print(f"\nTarget set to 45°, updating at 50 Hz (20ms steps)...")
    for i in range(30):
        positions = controller.update(dt=0.02)  # 20ms = 50Hz
        if i % 5 == 0:  # Print every 5th update (100ms)
            print(f"  t={i*20:4d}ms: UL={positions['UL']:6.2f}° "
                  f"(error={45.0-positions['UL']:5.2f}°)")
    
    print(f"\n  Final position: {positions['UL']:.2f}°")
    print(f"  At target: {controller.is_at_target('UL')}")
    
    # Test 2: Small command change (deadband filtering)
    print("\n" + "-" * 60)
    print("Test 2: Small command (45° → 45.3°) - deadband filtering")
    print("-" * 60)
    
    controller.set_target('UL', 45.3)
    positions_before = controller.get_position('UL')
    positions = controller.update(dt=0.02)
    positions_after = controller.get_position('UL')
    
    print(f"\n  Position before: {positions_before:.2f}°")
    print(f"  Target: 45.3°")
    print(f"  Position after: {positions_after:.2f}°")
    print(f"  Moved: {abs(positions_after - positions_before) > 0.01}")
    print(f"  → Command filtered by deadband (< {config['deadband']}°)")
    
    # Test 3: Out of range command
    print("\n" + "-" * 60)
    print("Test 3: Out of range command (100°) - limit clamping")
    print("-" * 60)
    
    controller.set_target('UR', 100.0)
    print(f"\n  Commanded: 100°")
    print(f"  Target set to: {controller.target_positions['UR']:.2f}°")
    print(f"  → Clamped to max_angle ({config['max_angle']}°)")
    
    # Show statistics
    print("\n" + "-" * 60)
    print("Statistics")
    print("-" * 60)
    stats = controller.get_stats()
    print(f"  Slew limited: {stats['slew_limited_count']} times")
    print(f"  Deadband filtered: {stats['deadband_filtered_count']} times")
    print(f"  Limit clipped: {stats['limit_clipped_count']} times")
    
    print("\n" + "=" * 60)
    print("Servo controller test complete!")
    print("=" * 60)
