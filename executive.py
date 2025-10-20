#!/usr/bin/env python3
"""
executive.py - UPDATED: Use task_params for clean separation of concerns
Updated to handle FOLLOW_TRACK waypoints through task_params instead of target_state
"""

import json
import os
import time
from typing import List, Dict, Any
from vehicle_state import VehicleState, SystemState, TaskType

class Executive:
    def __init__(self, vehicle_state: VehicleState):
        self.vehicle_state = vehicle_state
        self.mission_tasks: List[Dict[str, Any]] = []
        self.current_task_index = -1
        self.mission_loaded = False
        self.mission_start_time = 0.0
        
    def load_mission(self, mission_file: str) -> bool:
        """Load mission file and set initial position"""
        if not os.path.exists(mission_file):
            self.vehicle_state.add_fault(f"Mission file {mission_file} not found")
            return False
            
        try:
            with open(mission_file, 'r') as f:
                mission_data = json.load(f)
            
            # Set initial position
            vehicle_config = mission_data.get('vehicle_config', {})
            initial_pos = vehicle_config.get('initial_position', {})
            
            if initial_pos:
                lat = float(initial_pos['lat'])
                lon = float(initial_pos['lon']) 
                depth = float(initial_pos['depth'])
                heading = float(initial_pos['heading']) % 360.0
                
                self.vehicle_state.set_initial_position(lat, lon, depth, heading)
                self.vehicle_state.update_target_state(target_heading=heading, target_depth=depth)
            
            # Load task list
            self.mission_tasks = mission_data.get('tasks', [])
            
            # Load obstacles if present (for obstacle avoidance)
            if 'obstacles' in mission_data:
                obstacles = mission_data.get('obstacles', [])
                self.vehicle_state.mission_obstacles = obstacles
                print(f"Loaded {len(obstacles)} obstacles from mission file")
            else:
                self.vehicle_state.mission_obstacles = []
            
            self.mission_loaded = True
            print(f"Mission loaded with {len(self.mission_tasks)} tasks")
            return True
            
        except Exception as e:
            self.vehicle_state.add_fault(f"Failed to load mission: {e}")
            return False
    
    def update(self, dt: float):
        """Update mission execution with proper state validation"""
        if not self.mission_loaded:
            return
            
        # Start mission timing
        if self.mission_start_time == 0.0:
            self.mission_start_time = time.time()
        
        self.vehicle_state.mission_time = time.time() - self.mission_start_time
        
        # Handle failsafe
        if self.vehicle_state.failsafe_triggered:
            self.vehicle_state.current_task = TaskType.EMERGENCY_SURFACE.value
            self.vehicle_state.set_system_state(SystemState.EMERGENCY)
            return
        
        # Check if mission complete
        if self.current_task_index >= len(self.mission_tasks):
            if self.vehicle_state.current_state != SystemState.MISSION_COMPLETE:
                self.vehicle_state.set_system_state(SystemState.MISSION_COMPLETE)
            return
        
        # Handle task completion with state validation
        if self.vehicle_state.task_complete and self.current_task_index >= 0:
            completed_task = self.mission_tasks[self.current_task_index]
            completed_task_type = completed_task.get('type', 'UNKNOWN')
            
            print(f"EXECUTIVE DEBUG: Task {completed_task_type} marked complete, validating")
            
            # Validate that vehicle state actually matches task objective
            task_actually_successful = self._validate_task_completion(completed_task_type)
            
            if task_actually_successful:
                # Task genuinely completed - update system state
                completion_state_map = {
                    TaskType.GO_TO_SURFACED_TRIM.value: SystemState.SURFACED_TRIM,
                    TaskType.GO_TO_SUBMERGED_TRIM.value: SystemState.SUBMERGED_TRIM
                }
                new_state = completion_state_map.get(completed_task_type, self.vehicle_state.current_state)
                if new_state != self.vehicle_state.current_state:
                    self.vehicle_state.set_system_state(new_state)
                    print(f"Task {completed_task_type} completed successfully, state -> {new_state.value}")
            else:
                # Task "completed" but vehicle state is wrong - this is a fault
                self.vehicle_state.add_fault(f"Task {completed_task_type} completed but vehicle state validation failed")
                self.vehicle_state.set_system_state(SystemState.FAULT)
                print(f"Task {completed_task_type} failed state validation - setting FAULT state")
                return
        
        # Start next task if needed
        if self.current_task_index == -1 or self.vehicle_state.task_complete:
            self._advance_to_next_task()
    
    def _validate_task_completion(self, task_type: str) -> bool:
        """Validate that vehicle state actually matches what the completed task should achieve"""
        buoyancy = self.vehicle_state.buoyancy_state
        
        print(f"EXECUTIVE DEBUG: Validating completion of task: {task_type}")
        print(f"EXECUTIVE DEBUG: Current buoyancy_state: {buoyancy}")
        
        if task_type == TaskType.GO_TO_SURFACED_TRIM.value:
            # Should have positive buoyancy (empty ballast tank)
            valid = buoyancy == 'POSITIVE'
            print(f"EXECUTIVE DEBUG: GO_TO_SURFACED_TRIM validation: expected POSITIVE, got {buoyancy}, valid={valid}")
            if not valid:
                print(f"GO_TO_SURFACED_TRIM validation failed: buoyancy is {buoyancy}, expected POSITIVE")
            return valid
            
        elif task_type == TaskType.GO_TO_SUBMERGED_TRIM.value:
            # Should have neutral buoyancy (full ballast tank)
            valid = buoyancy == 'NEUTRAL'
            print(f"EXECUTIVE DEBUG: GO_TO_SUBMERGED_TRIM validation: expected NEUTRAL, got {buoyancy}, valid={valid}")
            if not valid:
                print(f"GO_TO_SUBMERGED_TRIM validation failed: buoyancy is {buoyancy}, expected NEUTRAL")
            return valid
            
        else:
            # For other tasks, just trust the completion status
            return True
    
    def _advance_to_next_task(self):
        """Advance to the next task in sequence"""
        print(f"EXECUTIVE DEBUG: _advance_to_next_task called, current_task_index={self.current_task_index}")
        print(f"EXECUTIVE DEBUG: task_complete={self.vehicle_state.task_complete}")
        print(f"EXECUTIVE DEBUG: Total mission tasks: {len(self.mission_tasks)}")
        
        self.vehicle_state.set_task_complete(False)
        
        # Reset target_state with defaults
        self.vehicle_state.target_state = {
            'target_lat': None,
            'target_lon': None,
            'target_depth': 0.0,  # Ensure default depth
            'target_heading': self.vehicle_state.nav_state.get('heading', 0.0),
            'target_speed': 0.0,
            'distance_to_waypoint': None
        }
        
        # Clear previous task parameters
        self.vehicle_state.clear_task_params()
        print(f"EXECUTIVE DEBUG: Cleared task_params, advancing to task index {self.current_task_index + 1}")
        
        # Advance task index
        self.current_task_index += 1
        
        # Check if mission complete
        if self.current_task_index >= len(self.mission_tasks):
            self.vehicle_state.set_system_state(SystemState.MISSION_COMPLETE)
            print("Mission complete")
            return
        
        # Get current task
        task = self.mission_tasks[self.current_task_index]
        task_type = task.get('type', 'UNKNOWN')
        
        print(f"EXECUTIVE DEBUG: Starting task {self.current_task_index}: {task_type}")
        print(f"EXECUTIVE DEBUG: Task data: {task}")
        
        # Check state requirements
        state_requirements = {
            TaskType.DIVE.value: SystemState.SUBMERGED_TRIM,
            TaskType.CLIMB.value: SystemState.SUBMERGED_TRIM,
            TaskType.TELEMETRY.value: SystemState.SURFACED_TRIM,
            TaskType.GPS_FIX.value: SystemState.SURFACED_TRIM
        }
        required_state = state_requirements.get(task_type)
        if required_state and self.vehicle_state.current_state != required_state:
            self.vehicle_state.add_fault(f"Invalid state for task {task_type}: requires {required_state.value}, current {self.vehicle_state.current_state.value}")
            self.vehicle_state.set_system_state(SystemState.FAULT)
            return
        
        # Set current task
        self.vehicle_state.current_task = task_type
        self.vehicle_state.task_status = f"Starting {task_type}"
        
        # Handle task parameter mapping based on task type
        if task_type == "FOLLOW_TRACK":
            print("=== EXECUTIVE: USING NEW FOLLOW_TRACK CODE ===")
            
            # FOLLOW_TRACK: Put configuration into task_params
            waypoints = task.get('waypoints', [])
            print(f"EXECUTIVE DEBUG: Found {len(waypoints)} waypoints in mission task")
            print(f"EXECUTIVE DEBUG: Waypoints data: {waypoints}")
            
            if not waypoints:
                self.vehicle_state.add_fault("FOLLOW_TRACK task missing waypoints")
                self.vehicle_state.set_system_state(SystemState.FAULT)
                print("EXECUTIVE DEBUG: No waypoints found - setting FAULT state")
                return
            
            # Store FOLLOW_TRACK configuration in task_params
            task_config = {
                'waypoints': waypoints,
                'lookahead_distance': task.get('lookahead_distance', 30.0),
                'cross_track_tolerance': task.get('cross_track_tolerance', 10.0),
                'timeout': task.get('timeout', 600.0)
            }
            
            print(f"EXECUTIVE DEBUG: Setting task_params for FOLLOW_TRACK:")
            print(f"  waypoints count: {len(waypoints)}")
            print(f"  task_config: {task_config}")
            
            self.vehicle_state.update_task_params(**task_config)
            
            # Verify it was set
            current_params = self.vehicle_state.get_task_params()
            print(f"EXECUTIVE DEBUG: Verified task_params after update: {current_params}")
            
            print(f"FOLLOW_TRACK: Loaded {len(waypoints)} waypoints into task_params")
            
            # No target_state updates needed for FOLLOW_TRACK - plugin handles waypoint navigation
            
        else:
            # Normal tasks: Map parameters to target_state and task_params appropriately
            task_params = {k: v for k, v in task.items() if k != 'type'}
            
            # Separate navigation targets from configuration parameters
            target_updates = {}
            config_params = {}
            
            # Map coordinate parameters to target_state
            if 'lat' in task_params:
                target_updates['target_lat'] = float(task_params['lat'])
            if 'lon' in task_params:
                target_updates['target_lon'] = float(task_params['lon'])
            if 'depth' in task_params:
                target_updates['target_depth'] = float(task_params['depth'])
            else:
                target_updates['target_depth'] = 0.0  # Enforce default
            if 'heading' in task_params:
                target_updates['target_heading'] = float(task_params['heading'])
            if 'speed' in task_params:
                target_updates['target_speed'] = float(task_params['speed'])
            
            # Put non-navigation parameters in task_params
            navigation_keys = {'lat', 'lon', 'depth', 'heading', 'speed'}
            for key, value in task_params.items():
                if key not in navigation_keys:
                    config_params[key] = value
            
            # Update both target_state and task_params as needed
            if target_updates:
                self.vehicle_state.update_target_state(**target_updates)
            if config_params:
                self.vehicle_state.update_task_params(**config_params)
            
            # Validate target_state
            if target_updates and self.vehicle_state.target_state.get('target_depth') is None:
                self.vehicle_state.add_fault("Target depth is None after update")
                self.vehicle_state.target_state['target_depth'] = 0.0
            
            print(f"Task {task_type} configured:")
            if target_updates:
                print(f"  Target state: {target_updates}")
            if config_params:
                print(f"  Task params: {config_params}")
    
    def get_mission_status(self) -> Dict[str, Any]:
        """Get mission status"""
        if not self.mission_loaded:
            return {'error': 'No mission loaded'}
        
        progress = 0.0
        if len(self.mission_tasks) > 0:
            completed = max(0, self.current_task_index)
            if self.vehicle_state.task_complete and self.current_task_index >= 0:
                completed += 1
            progress = (completed / len(self.mission_tasks)) * 100.0
        
        return {
            'current_task_index': self.current_task_index,
            'total_tasks': len(self.mission_tasks),
            'current_task': self.vehicle_state.current_task,
            'mission_progress': progress,
            'mission_complete': self.current_task_index >= len(self.mission_tasks),
            'mission_time': self.vehicle_state.mission_time,
            'system_state': self.vehicle_state.current_state.value
        }