# energy.py
# Monitors battery usage and energy status

import time
from typing import Dict, Any
from vehicle_state import VehicleState

class Energy:
    """
    Energy monitors the AUV's battery usage and energy status.
    Estimates available power, tracks energy consumption over time, 
    and provides alerts or flags to the failsafe system if power 
    levels fall below critical thresholds.
    """
    
    def __init__(self, vehicle_state: VehicleState, hal=None):
        self.vehicle_state = vehicle_state
        self.hal = hal  # Hardware Abstraction Layer
        
        # Battery configuration
        self.battery_capacity = 12.0  # Amp-hours (12V 12Ah battery)
        self.nominal_voltage = 12.0   # Nominal voltage
        self.initial_capacity = 12.0  # Starting capacity
        
        # Power consumption models (watts)
        self.base_power = 5.0         # Base system power consumption
        self.thruster_power_coeff = 0.5  # Watts per PWM unit
        self.ballast_pump_power = 20.0   # Watts when running
        self.servo_power = 2.0        # Watts per servo when moving
        
        # State tracking
        self.last_update_time = time.time()
        self.cumulative_energy = 0.0  # Watt-hours consumed
        self.capacity_remaining = self.initial_capacity
        
        # Initialize vehicle state
        self.vehicle_state.energy_state.update({
            'voltage': self.nominal_voltage,
            'capacity_remaining': 100.0,  # Percentage
            'power_draw': self.base_power,
            'current_draw': self.base_power / self.nominal_voltage
        })
        
        print(f"Energy module initialized - {self.battery_capacity}Ah battery")
    
    def update(self, time_step: float):
        """Update energy consumption and battery state"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        if dt <= 0:
            return
        
        # Calculate current power consumption
        power_draw = self._calculate_power_consumption()
        
        # Update energy consumption
        energy_consumed = power_draw * (dt / 3600.0)  # Convert to watt-hours
        self.cumulative_energy += energy_consumed
        
        # Update remaining capacity
        amp_hours_consumed = energy_consumed / self.nominal_voltage
        self.capacity_remaining = max(0.0, self.capacity_remaining - amp_hours_consumed)
        
        # Calculate voltage based on discharge curve
        voltage = self._calculate_voltage()
        
        # Update vehicle state
        self.vehicle_state.energy_state.update({
            'voltage': voltage,
            'capacity_remaining': (self.capacity_remaining / self.initial_capacity) * 100.0,
            'power_draw': power_draw,
            'current_draw': power_draw / voltage if voltage > 0 else 0.0
        })
    
    def _calculate_power_consumption(self) -> float:
        """Calculate instantaneous power consumption"""
        total_power = self.base_power
        
        # Thruster power consumption
        thruster_cmd = self.vehicle_state.actuator_commands.get('thruster_cmd', 0.0)
        thruster_power = (thruster_cmd / 100.0) * self.thruster_power_coeff * 100.0
        total_power += thruster_power
        
        # Ballast pump power
        ballast_state = self.vehicle_state.ballast_state
        if ballast_state.get('pump_running', False):
            total_power += self.ballast_pump_power
        
        # Servo power (simplified - assume power when rudders or elevators are deflected)
        rudder_cmd = self.vehicle_state.actuator_commands.get('rudder_cmd', 0.0)
        elevator_cmd = self.vehicle_state.actuator_commands.get('elevator_cmd', 0.0)
        active_servos = sum(1 for angle in [rudder_cmd, elevator_cmd] if abs(angle) > 1.0)
        total_power += active_servos * self.servo_power
        
        # Additional power for sensors and navigation systems
        if self.vehicle_state.current_state.value in ['SURFACED_TRIM', 'GPS_FIX']:
            total_power += 3.0  # GPS and communication systems
        
        return total_power
    
    def _calculate_voltage(self) -> float:
        """Calculate battery voltage based on discharge curve"""
        # Simple linear discharge model (real batteries have non-linear curves)
        capacity_percent = self.capacity_remaining / self.initial_capacity
        
        if capacity_percent > 0.8:
            # High charge: voltage stays near nominal
            voltage = self.nominal_voltage - (1.0 - capacity_percent) * 2.0
        elif capacity_percent > 0.2:
            # Mid charge: linear decrease
            voltage = self.nominal_voltage - (1.0 - capacity_percent) * 1.5
        else:
            # Low charge: rapid voltage drop
            voltage = self.nominal_voltage - (1.0 - capacity_percent) * 3.0
        
        # Apply voltage sag under load
        power_draw = self.vehicle_state.energy_state.get('power_draw', 0.0)
        load_factor = min(1.0, power_draw / 50.0)  # Normalized load
        voltage_sag = load_factor * 0.5  # Up to 0.5V sag under full load
        
        return max(8.0, voltage - voltage_sag)  # Minimum 8V
    
    def get_estimated_runtime(self) -> float:
        """Estimate remaining runtime in hours"""
        current_power = self.vehicle_state.energy_state.get('power_draw', self.base_power)
        
        if current_power <= 0:
            return float('inf')
        
        # Remaining energy in watt-hours
        remaining_energy = self.capacity_remaining * self.nominal_voltage
        
        # Runtime at current power consumption
        runtime_hours = remaining_energy / current_power
        
        return max(0.0, runtime_hours)
    
    def get_range_estimate(self, cruise_speed: float = 1.0) -> float:
        """Estimate remaining range in meters at given speed"""
        runtime_hours = self.get_estimated_runtime()
        
        # Convert speed from m/s to m/h and calculate range
        speed_mh = cruise_speed * 3600.0  # meters per hour
        range_meters = runtime_hours * speed_mh
        
        return range_meters
    
    def reset_battery(self, capacity_percent: float = 100.0):
        """Reset battery to specified charge level"""
        self.capacity_remaining = (capacity_percent / 100.0) * self.initial_capacity
        self.cumulative_energy = 0.0
        
        self.vehicle_state.energy_state.update({
            'voltage': self.nominal_voltage,
            'capacity_remaining': capacity_percent,
            'power_draw': self.base_power,
            'current_draw': self.base_power / self.nominal_voltage
        })
        
        print(f"ENERGY: Battery reset to {capacity_percent}% capacity")
    
    def set_battery_config(self, capacity: float = None, nominal_voltage: float = None):
        """Update battery configuration"""
        if capacity is not None:
            self.battery_capacity = capacity
            self.initial_capacity = capacity
            
        if nominal_voltage is not None:
            self.nominal_voltage = nominal_voltage
            
        print(f"ENERGY: Battery config updated - {self.battery_capacity}Ah, {self.nominal_voltage}V")
    
    def get_power_breakdown(self) -> Dict[str, float]:
        """Get detailed power consumption breakdown"""
        breakdown = {
            'base_systems': self.base_power,
            'thruster': 0.0,
            'ballast_pump': 0.0,
            'servos': 0.0,
            'navigation': 0.0
        }
        
        # Thruster power
        thruster_cmd = self.vehicle_state.actuator_commands.get('thruster_cmd', 0.0)
        breakdown['thruster'] = (thruster_cmd / 100.0) * self.thruster_power_coeff * 100.0
        
        # Ballast pump
        if self.vehicle_state.ballast_state.get('pump_running', False):
            breakdown['ballast_pump'] = self.ballast_pump_power
        
        # Servos
        rudder_cmd = self.vehicle_state.actuator_commands.get('rudder_cmd', 0.0)
        elevator_cmd = self.vehicle_state.actuator_commands.get('elevator_cmd', 0.0)
        active_servos = sum(1 for angle in [rudder_cmd, elevator_cmd] if abs(angle) > 1.0)
        breakdown['servos'] = active_servos * self.servo_power
        
        # Navigation systems
        if self.vehicle_state.current_state.value in ['SURFACED_TRIM', 'GPS_FIX']:
            breakdown['navigation'] = 3.0
        
        return breakdown
    
    def get_status(self) -> Dict[str, Any]:
        """Get energy status for telemetry"""
        power_breakdown = self.get_power_breakdown()
        
        return {
            'voltage': self.vehicle_state.energy_state['voltage'],
            'capacity_remaining_percent': self.vehicle_state.energy_state['capacity_remaining'],
            'capacity_remaining_ah': self.capacity_remaining,
            'power_draw': self.vehicle_state.energy_state['power_draw'],
            'current_draw': self.vehicle_state.energy_state['current_draw'],
            'cumulative_energy_wh': self.cumulative_energy,
            'estimated_runtime_hours': self.get_estimated_runtime(),
            'estimated_range_meters': self.get_range_estimate(),
            'power_breakdown': power_breakdown,
            'battery_config': {
                'capacity_ah': self.battery_capacity,
                'nominal_voltage': self.nominal_voltage
            }
        }
    
    def simulate_battery_aging(self, cycles: int = 100):
        """Simulate battery aging effects (for long-term testing)"""
        # Typical battery degradation: ~20% capacity loss after 500 cycles
        degradation_factor = min(0.2, cycles / 500.0 * 0.2)
        new_capacity = self.initial_capacity * (1.0 - degradation_factor)
        
        # Also reduce remaining capacity proportionally
        capacity_ratio = self.capacity_remaining / self.battery_capacity
        self.battery_capacity = new_capacity
        self.capacity_remaining = new_capacity * capacity_ratio
        
        print(f"ENERGY: Simulated {cycles} battery cycles, "
              f"capacity reduced to {new_capacity:.1f}Ah ({degradation_factor*100:.1f}% degradation)")