# failsafe.py
# Monitors critical safety parameters and triggers emergency procedures

import time
from typing import Dict, List, Any
from vehicle_state import VehicleState

class Failsafe:
    """
    Failsafe continuously monitors critical safety parameters during the mission.
    Checks for predefined fault conditions such as excessive depth, low battery, 
    or leak detection. If any condition is met, it triggers an emergency transition 
    to FAILSAFE mode and initiates recovery behavior.
    """
    
    def __init__(self, vehicle_state: VehicleState, hal=None):
        self.vehicle_state = vehicle_state
        self.hal = hal  # Hardware Abstraction Layer
        
        # Safety thresholds
        self.max_depth = 40.0        # Maximum safe depth in meters
        self.min_voltage = 10.5      # Minimum battery voltage
        self.max_nav_error = 100.0   # Maximum navigation error in meters
        self.max_mission_time = 3600.0  # Maximum mission time in seconds
        
        # Monitoring state
        self.fault_history: List[Dict] = []
        self.last_check_time = time.time()
        
        print("Failsafe module initialized")
        print(f"Safety limits - Max depth: {self.max_depth}m, "
              f"Min voltage: {self.min_voltage}V, Max nav error: {self.max_nav_error}m")
    
    def update(self, time_step: float):
        """Monitor safety parameters and check for fault conditions"""
        current_time = time.time()
        
        # Perform safety checks
        self._check_depth_limits()
        self._check_battery_voltage()
#       self._check_navigation_error() # Leaving this out for now
        self._check_mission_time()
        self._check_leak_detection()
        self._check_system_faults()
        
        self.last_check_time = current_time
    
    def _check_depth_limits(self):
        """Monitor depth for overdepth condition"""
        current_depth = self.vehicle_state.nav_state['depth']
        
        if current_depth > self.max_depth:
            fault_msg = f"DEPTH_EXCEEDED: {current_depth:.1f}m > {self.max_depth}m"
            self._trigger_failsafe(fault_msg)
            self.vehicle_state.add_fault("DEPTH_EXCEEDED")
    
    def _check_battery_voltage(self):
        """Monitor battery voltage for low voltage condition"""
        voltage = self.vehicle_state.energy_state['voltage']
        
        if voltage < self.min_voltage:
            fault_msg = f"LOW_VOLTAGE: {voltage:.1f}V < {self.min_voltage}V"
            self._trigger_failsafe(fault_msg)
            self.vehicle_state.add_fault("LOW_VOLTAGE")
    
    def _check_mission_time(self):
        """Monitor total mission time"""
        mission_time = self.vehicle_state.mission_time
        
        if mission_time > self.max_mission_time:
            fault_msg = f"MISSION_TIMEOUT: {mission_time:.0f}s > {self.max_mission_time:.0f}s"
            self._trigger_failsafe(fault_msg)
            self.vehicle_state.add_fault("MISSION_TIMEOUT")
    
    def _check_leak_detection(self):
        """Monitor for leak detection"""
        # In simulation, we won't trigger leak detection
        # In real system, this would read from leak sensors via HAL
        if self.hal:
            try:
                leak_detected = self.hal.read_leak_sensor()
                if leak_detected:
                    fault_msg = "LEAK_DETECTED: Water ingress detected"
                    self._trigger_failsafe(fault_msg)
                    self.vehicle_state.add_fault("LEAK_DETECTED")
            except AttributeError:
                # HAL doesn't have leak sensor method
                pass
    
    def _check_system_faults(self):
        """Check for system-level faults"""
        # Check if any critical faults exist in vehicle state
        critical_faults = [
            "HARDWARE_COMMUNICATION_ERROR",
            "SENSOR_FAILURE",
            "ACTUATOR_FAILURE",
            "TASK_TIMEOUT_EMERGENCY_SURFACE"
        ]
        
        for fault in critical_faults:
            if self.vehicle_state.has_fault(fault):
                fault_msg = f"SYSTEM_FAULT: {fault}"
                self._trigger_failsafe(fault_msg)
                break
    
    def _trigger_failsafe(self, reason: str):
        """Trigger failsafe mode"""
        if not self.vehicle_state.failsafe_triggered:
            print(f"FAILSAFE: TRIGGERED - {reason}")
            
            self.vehicle_state.failsafe_triggered = True
            self.vehicle_state.failsafe_reason = reason
            
            # Log fault
            fault_record = {
                'time': time.time(),
                'reason': reason,
                'position': self.vehicle_state.get_position_dict(),
                'system_state': self.vehicle_state.current_state.value
            }
            self.fault_history.append(fault_record)
            
            # Force emergency surface procedure
            self._initiate_emergency_surface()
    
    def _initiate_emergency_surface(self):
        """Initiate emergency surface procedure"""
        print("FAILSAFE: Initiating emergency surface procedure")
        
        # Set emergency commands directly
        self.vehicle_state.actuator_commands.update({
            'thruster_cmd': 0.0,  # Stop propulsion
            'rudder_cmd': 0.0,    # Center control surfaces
            'elevator_cmd': 0.0,  # Center control surfaces
            'ballast_cmd': 'EMPTY'  # Emergency ballast blow
        })
        
        # Override target commands
        self.vehicle_state.target_state.update({
            'target_depth': 0.0,
            'target_speed': 0.0
        })
    
    def reset_failsafe(self, authorization_code: str = "RESET_AUTH"):
        """Reset failsafe condition (for testing or recovery)"""
        if authorization_code == "RESET_AUTH":
            print("FAILSAFE: Resetting failsafe condition")
            self.vehicle_state.failsafe_triggered = False
            self.vehicle_state.failsafe_reason = ""
            
            # Clear non-critical faults
            non_critical_faults = ["TASK_TIMEOUT", "MINOR_SENSOR_ERROR"]
            for fault in non_critical_faults:
                self.vehicle_state.clear_fault(fault)
        else:
            print("FAILSAFE: Invalid authorization code for reset")
    
    def set_safety_limits(self, max_depth: float = None, min_voltage: float = None,
                         max_nav_error: float = None, max_mission_time: float = None):
        """Update safety limits"""
        if max_depth is not None:
            self.max_depth = max_depth
        if min_voltage is not None:
            self.min_voltage = min_voltage
        if max_nav_error is not None:
            self.max_nav_error = max_nav_error
        if max_mission_time is not None:
            self.max_mission_time = max_mission_time
            
        print(f"FAILSAFE: Updated safety limits - depth: {self.max_depth}m, "
              f"voltage: {self.min_voltage}V, nav_error: {self.max_nav_error}m, "
              f"mission_time: {self.max_mission_time}s")
    
    def get_status(self) -> Dict[str, Any]:
        """Get failsafe status for telemetry"""
        return {
            'failsafe_triggered': self.vehicle_state.failsafe_triggered,
            'failsafe_reason': self.vehicle_state.failsafe_reason,
            'active_faults': self.vehicle_state.faults.copy(),
            'safety_limits': {
                'max_depth': self.max_depth,
                'min_voltage': self.min_voltage,
                'max_nav_error': self.max_nav_error,
                'max_mission_time': self.max_mission_time
            },
            'current_values': {
                'depth': self.vehicle_state.nav_state['depth'],
                'voltage': self.vehicle_state.energy_state['voltage'],
                'nav_uncertainty': self.vehicle_state.nav_state.get('position_uncertainty', 0),
                'mission_time': self.vehicle_state.mission_time
            },
            'fault_count': len(self.fault_history)
        }
    
    def get_fault_history(self) -> List[Dict]:
        """Get complete fault history"""
        return self.fault_history.copy()
    
    def test_failsafe_systems(self):
        """Test failsafe detection (for system verification)"""
        print("FAILSAFE: Testing failsafe detection systems")
        
        # Save original limits
        orig_limits = {
            'max_depth': self.max_depth,
            'min_voltage': self.min_voltage,
            'max_nav_error': self.max_nav_error
        }
        
        # Test depth detection
        print("Testing depth limit detection...")
        original_depth = self.vehicle_state.nav_state['depth']
        self.vehicle_state.update_nav_state(depth=25.0)  # Exceed limit
        self._check_depth_limits()
        self.vehicle_state.update_nav_state(depth=original_depth)  # Restore
        
        # Reset for next test
        self.reset_failsafe("RESET_AUTH")
        
        # Test voltage detection
        print("Testing voltage limit detection...")
        original_voltage = self.vehicle_state.energy_state['voltage']
        self.vehicle_state.energy_state['voltage'] = 10.0  # Below limit
        self._check_battery_voltage()
        self.vehicle_state.energy_state['voltage'] = original_voltage  # Restore
        
        # Reset failsafe
        self.reset_failsafe("RESET_AUTH")
        
        print("FAILSAFE: Test sequence complete")