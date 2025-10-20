#!/usr/bin/env python3
"""
task_manager.py - UPDATED: Use task_params instead of target_state for configuration
Updated to extract FOLLOW_TRACK waypoints from task_params and target_state appropriately
"""

from typing import Dict, Any, Optional
from plugin_registry import PluginRegistry

class TaskManager:
    def __init__(self, vehicle_state):
        self.vehicle_state = vehicle_state
        self.plugin_registry = PluginRegistry.get_instance()
        self.current_plugin = None
        self.last_task_name = None
        self.task_start_time = 0.0
        
        # Load plugins
        if not self.plugin_registry.load_plugins():
            raise RuntimeError("Failed to load plugins")
        
        print(f"TaskManager: Loaded {len(self.plugin_registry.get_available_plugins())} plugins")
    
    def update(self, dt: float):
        """Update task execution"""
        current_task = self.vehicle_state.current_task
        
        print(f"TASKMANAGER DEBUG: update() called, current_task={current_task}, last_task={self.last_task_name}")
        print(f"TASKMANAGER DEBUG: current_plugin exists: {self.current_plugin is not None}")
        print(f"TASKMANAGER DEBUG: task_complete: {self.vehicle_state.task_complete}")
        
        # Start new task if:
        # 1. Task name changed, OR 
        # 2. We don't have a current plugin (handles consecutive same-type tasks), OR
        # 3. Current plugin is completed (redundant safety check)
        should_restart = (
            (current_task != self.last_task_name) or  # Different task name
            (self.current_plugin is None and current_task and current_task != "NONE") or  # No active plugin
            (self.current_plugin and self.current_plugin.check_completion())  # Current plugin completed
        )
        
        print(f"TASKMANAGER DEBUG: should_restart={should_restart}")
        
        if should_restart:
            print(f"TASKMANAGER DEBUG: Starting new task: {current_task}")
            self._start_new_task(current_task)
            self.last_task_name = current_task
            self.task_start_time = self.vehicle_state.mission_time
        
        # Run current plugin
        if self.current_plugin:
            try:
                print(f"TASKMANAGER DEBUG: Executing plugin for {current_task}")
                self.current_plugin.execute(dt)
                
                # Update status
                if hasattr(self.current_plugin, 'status_message'):
                    self.vehicle_state.task_status = self.current_plugin.status_message
                
                # Check completion
                completion_result = self.current_plugin.check_completion()
                print(f"TASKMANAGER DEBUG: Plugin check_completion() returned: {completion_result}")
                
                if completion_result:
                    print(f"TASKMANAGER DEBUG: Setting task_complete=True for {current_task}")
                    self.vehicle_state.set_task_complete(True)
                    self.current_plugin = None
                    
            except Exception as e:
                print(f"TASKMANAGER DEBUG: Exception in plugin execution: {e}")
                self.vehicle_state.task_status = f"ERROR: {e}"
                self.vehicle_state.set_task_complete(True)
                self.current_plugin = None
        else:
            print(f"TASKMANAGER DEBUG: No current plugin to execute")
    
    def _start_new_task(self, task_name: str):
        """Start a new task with fresh plugin instance"""
        print(f"TASKMANAGER DEBUG: _start_new_task called for {task_name}")
        
        # Stop current plugin
        self.current_plugin = None
        
        # Skip empty/invalid tasks
        if not task_name or task_name == "NONE":
            print(f"TASKMANAGER DEBUG: Skipping empty/invalid task: {task_name}")
            return
        
        # Check if plugin exists
        if not self.plugin_registry.has_plugin(task_name):
            print(f"TASKMANAGER DEBUG: No plugin found for {task_name}")
            self.vehicle_state.task_status = f"ERROR: No plugin for {task_name}"
            self.vehicle_state.set_task_complete(True)
            return
        
        # Create fresh plugin instance
        try:
            print(f"TASKMANAGER DEBUG: Creating plugin instance for {task_name}")
            self.current_plugin = self.plugin_registry.create_plugin(task_name, self.vehicle_state)
            
            # Extract task parameters from both task_params and target_state
            task_params = self._extract_task_params()
            
            print(f"TASKMANAGER DEBUG: Initializing {task_name} plugin with params: {task_params}")
            
            # Initialize plugin
            if self.current_plugin.initialize(task_params):
                print(f"TASKMANAGER DEBUG: Plugin {task_name} initialized successfully")
            else:
                print(f"TASKMANAGER DEBUG: Plugin {task_name} initialization failed")
                self.vehicle_state.task_status = f"ERROR: Failed to initialize {task_name}"
                self.vehicle_state.set_task_complete(True)
                self.current_plugin = None
                
        except Exception as e:
            print(f"TASKMANAGER DEBUG: Exception creating/initializing plugin {task_name}: {e}")
            self.vehicle_state.task_status = f"ERROR: {e}"
            self.vehicle_state.set_task_complete(True)
            self.current_plugin = None
    
    def _extract_task_params(self) -> Dict[str, Any]:
        """Extract task parameters from task_params and target_state"""
        # Start with task_params (configuration data)
        task_params = self.vehicle_state.get_task_params()
        
        print(f"TASKMANAGER DEBUG: Raw task_params from vehicle_state: {task_params}")
        print(f"TASKMANAGER DEBUG: task_params type: {type(task_params)}")
        
        # Add navigation targets from target_state if not already in task_params
        target_state = self.vehicle_state.target_state
        print(f"TASKMANAGER DEBUG: target_state: {target_state}")
        
        # Extract navigation parameters from target_state if they're not in task_params
        navigation_mapping = {
            'lat': ['target_lat'],
            'lon': ['target_lon'], 
            'speed': ['target_speed'],
            'depth': ['target_depth'],
            'heading': ['target_heading']
        }
        
        for param_name, possible_keys in navigation_mapping.items():
            # Only add from target_state if not already in task_params
            if param_name not in task_params:
                for key in possible_keys:
                    if key in target_state and target_state[key] is not None:
                        # Skip zero values for coordinates (they're usually defaults)
                        if param_name in ['lat', 'lon'] and target_state[key] == 0.0:
                            continue
                        task_params[param_name] = target_state[key]
                        print(f"TASKMANAGER DEBUG: Added {param_name}={target_state[key]} from target_state")
                        break
        
        print(f"TASKMANAGER DEBUG: Final task_params to send to plugin: {task_params}")
        
        # Log what we extracted
        if task_params:
            if 'waypoints' in task_params:
                print(f"TaskManager: Extracted {len(task_params['waypoints'])} waypoints from task_params")
            else:
                param_summary = {k: v for k, v in task_params.items() if k != 'waypoints'}
                if param_summary:
                    print(f"TaskManager: Extracted parameters: {param_summary}")
        else:
            print("TASKMANAGER DEBUG: No task_params found!")
        
        return task_params