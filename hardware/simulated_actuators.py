#!/usr/bin/env python3
"""
hardware/simulated_actuators.py - Simulated actuator interface for HAL
"""

import time
from typing import Dict, Any

class SimulatedActuatorInterface:
    """Simulated actuator interface for testing and simulation"""
    
    def __init__(self, vehicle_state, config: Dict[str, Any] = None):
        self.vehicle_state = vehicle_state
        self.config = config or {}
        self.available = True
        
        # Ballast simulation
        self.ballast_fill_rate = 0.002  # m³/s
        self.ballast_capacity = 0.001982  # m³
        
        print("Simulated actuators initialized")
    
    def set_fin_positions(self, fin_commands: Dict[str, float]):
        """Set simulated fin positions"""
        if not self.available:
            return
        
        # Update vehicle state with fin commands
        self.vehicle_state.update_actuator_commands(**fin_commands)
    
    def set_thruster(self, thrust_percent: float):
        """Set simulated thruster"""
        if not self.available:
            return
        
        self.vehicle_state.update_actuator_commands(thruster_cmd=thrust_percent)
    
    def set_ballast_pump(self, command: str):
        """Control simulated ballast pump"""
        if not self.available:
            return
        
        self.vehicle_state.update_actuator_commands(ballast_cmd=command)
    
    def update_ballast_simulation(self, time_step: float):
        """Update ballast tank simulation"""
        if time_step <= 0:
            return
        
        ballast_state = self.vehicle_state.ballast_state
        current_volume = ballast_state.get('tank_volume', 0.0)
        command = self.vehicle_state.actuator_commands.get('ballast_cmd', 'OFF')
        
        if command == 'FILL' and current_volume < self.ballast_capacity:
            volume_change = self.ballast_fill_rate * time_step
            new_volume = min(current_volume + volume_change, self.ballast_capacity)
            ballast_state['tank_volume'] = new_volume
            
            if new_volume >= self.ballast_capacity * 0.99:
                self.vehicle_state.buoyancy_state = "NEUTRAL"
        
        elif command == 'EMPTY' and current_volume > 0.0:
            volume_change = self.ballast_fill_rate * time_step
            new_volume = max(current_volume - volume_change, 0.0)
            ballast_state['tank_volume'] = new_volume
            
            if new_volume <= 0.0:
                self.vehicle_state.buoyancy_state = "POSITIVE"
    
    def emergency_stop(self):
        """Execute emergency stop"""
        print("Simulated actuators: Emergency stop")
        self.vehicle_state.update_actuator_commands(
            thruster_cmd=0.0,
            ballast_cmd='EMPTY',
            fin_upper_left=0.0,
            fin_upper_right=0.0,
            fin_lower_right=0.0,
            fin_lower_left=0.0
        )
    
    def run_self_test(self) -> bool:
        """Run actuator self-test"""
        return self.available
    
    def is_available(self) -> bool:
        return self.available
    
    def close(self):
        """Cleanup simulated actuators"""
        pass
