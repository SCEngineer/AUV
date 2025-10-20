#!/usr/bin/env python3
"""
tests/test_basic.py - Basic system tests
"""

import pytest
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

def test_imports():
    """Test that core modules can be imported"""
    try:
        from vehicle_state import VehicleState
        from enhanced_hal import EnhancedHAL
        assert True
    except ImportError:
        assert False, "Core modules failed to import"

def test_vehicle_state():
    """Test VehicleState functionality"""
    from vehicle_state import VehicleState
    
    vs = VehicleState()
    vs.set_initial_position(33.6094, -117.7070, 0.0, 180.0)
    assert vs.is_initial_position_set()

def test_enhanced_hal():
    """Test Enhanced HAL"""
    from vehicle_state import VehicleState
    from enhanced_hal import EnhancedHAL
    
    vs = VehicleState()
    config = {'mode': 'simulation'}
    hal = EnhancedHAL(config, vs)
    
    assert hal.get_mode() == 'simulation'
    
    # Test update
    hal.update(0.1)

def test_plugin_manager():
    """Test plugin manager"""
    try:
        from vehicle_state import VehicleState
        from guidance.plugin_manager import GuidancePluginManager
        
        vs = VehicleState()
        guidance = GuidancePluginManager(vs)
        
        tasks = guidance.get_available_tasks()
        assert len(tasks) > 0
        assert 'DIVE' in tasks
        
    except ImportError:
        pytest.skip("Guidance system not available")

if __name__ == "__main__":
    pytest.main([__file__])
