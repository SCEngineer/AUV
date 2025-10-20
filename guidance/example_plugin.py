#!/usr/bin/env python3
"""
guidance/plugins/example_plugin.py - Example custom guidance plugin
"""

from guidance.plugin_manager import GuidanceTaskPlugin
from typing import Dict, Any

class ExampleTaskPlugin(GuidanceTaskPlugin):
    """Example custom task plugin"""
    TASK_NAME = "EXAMPLE_TASK"
    
    def __init__(self, vehicle_state):
        super().__init__(vehicle_state)
        self.name = "EXAMPLE_TASK"
        self.task_duration = 10.0
        self.elapsed_time = 0.0
    
    def initialize(self, task_params: Dict[str, Any]) -> bool:
        """Initialize example task"""
        self.task_duration = task_params.get('duration', 10.0)
        self.elapsed_time = 0.0
        self.status_message = f"Example task running for {self.task_duration}s"
        return True
    
    def execute(self, time_step: float) -> None:
        """Execute example task logic"""
        self.elapsed_time += time_step
        remaining = max(0.0, self.task_duration - self.elapsed_time)
        self.status_message = f"Example task: {remaining:.1f}s remaining"
    
    def check_completion(self) -> bool:
        """Check if example task is complete"""
        return self.elapsed_time >= self.task_duration
