#!/usr/bin/env python3
"""
plugin_registry.py - Centralized registry for guidance plugins
Provides plugin discovery and validation for the entire system
"""

import importlib
import inspect
from pathlib import Path
from typing import Dict, List, Type, Set
from abc import ABC, abstractmethod

class GuidanceTaskPlugin(ABC):
    """Base class for all guidance task plugins"""
    
    def __init__(self, vehicle_state):
        self.vehicle_state = vehicle_state
        self.name = "BASE_TASK"
        self.initialized = False
        self.status_message = ""
    
    @abstractmethod
    def initialize(self, task_params: Dict[str, any]) -> bool:
        """Initialize the task with given parameters"""
        pass
    
    @abstractmethod
    def execute(self, time_step: float) -> None:
        """Execute task logic for one time step"""
        pass
    
    @abstractmethod
    def check_completion(self) -> bool:
        """Check if task is complete"""
        pass

class PluginRegistry:
    """Centralized registry for all guidance plugins"""
    
    _instance = None
    _plugins = {}  # Dict[str, Type[GuidanceTaskPlugin]]
    _loaded = False
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(PluginRegistry, cls).__new__(cls)
        return cls._instance
    
    @classmethod
    def get_instance(cls):
        """Get singleton instance of plugin registry"""
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance
    
    def load_plugins(self, plugins_dir: Path = None) -> bool:
        """Load all plugins from the plugins directory"""
        if self._loaded:
            return True
        
        if plugins_dir is None:
            # Default to guidance/plugins directory
            plugins_dir = Path(__file__).parent / 'guidance' / 'plugins'
        
        if not plugins_dir.exists():
            print(f"ERROR: Plugins directory {plugins_dir} does not exist")
            return False
        
        self._plugins = {}
        load_count = 0
        
        for plugin_file in plugins_dir.glob('*.py'):
            if plugin_file.stem == '__init__':
                continue
            
            try:
                # Import the module
                module_name = f"guidance.plugins.{plugin_file.stem}"
                module = importlib.import_module(module_name)
                
                # Find all classes that inherit from GuidanceTaskPlugin
                for name, obj in inspect.getmembers(module, inspect.isclass):
                    if (issubclass(obj, GuidanceTaskPlugin) and 
                        obj != GuidanceTaskPlugin and 
                        hasattr(obj, 'TASK_NAME')):
                        
                        task_name = obj.TASK_NAME
                        if task_name in self._plugins:
                            print(f"WARNING: Duplicate plugin for task {task_name}")
                        
                        self._plugins[task_name] = obj
                        load_count += 1
                        print(f"Registered plugin: {task_name}")
                        
            except Exception as e:
                print(f"ERROR: Failed to load plugin from {plugin_file}: {e}")
                return False
        
        self._loaded = True
        print(f"Plugin registry loaded {load_count} plugins")
        return True
    
    def get_available_plugins(self) -> Set[str]:
        """Get set of all available plugin names"""
        if not self._loaded:
            self.load_plugins()
        return set(self._plugins.keys())
    
    def has_plugin(self, task_name: str) -> bool:
        """Check if a plugin exists for the given task"""
        if not self._loaded:
            self.load_plugins()
        return task_name in self._plugins
    
    def get_plugin_class(self, task_name: str) -> Type[GuidanceTaskPlugin]:
        """Get the plugin class for a task"""
        if not self._loaded:
            self.load_plugins()
        return self._plugins.get(task_name)
    
    def validate_tasks(self, task_list: List[str]) -> Dict[str, any]:
        """Validate that all tasks in the list have corresponding plugins
        
        Returns:
            Dict with 'valid': bool, 'missing': List[str], 'available': Set[str]
        """
        if not self._loaded:
            self.load_plugins()
        
        available_plugins = set(self._plugins.keys())
        required_tasks = set(task_list)
        missing_plugins = required_tasks - available_plugins
        
        return {
            'valid': len(missing_plugins) == 0,
            'missing': list(missing_plugins),
            'available': available_plugins,
            'required': required_tasks
        }
    
    def create_plugin(self, task_name: str, vehicle_state):
        """Create an instance of the specified plugin"""
        if not self._loaded:
            self.load_plugins()
        
        plugin_class = self._plugins.get(task_name)
        if plugin_class is None:
            raise ValueError(f"No plugin found for task: {task_name}")
        
        return plugin_class(vehicle_state)

# Global registry instance
_registry = PluginRegistry.get_instance()