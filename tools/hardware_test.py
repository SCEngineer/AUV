#!/usr/bin/env python3
"""
tools/hardware_test.py - Hardware testing utility
"""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

def main():
    """Run hardware tests"""
    print("🔧 SIMPLR-AUV Hardware Test Tool")
    print("=" * 40)
    
    try:
        from config.hardware_config import HARDWARE_CONFIG
        from enhanced_hal import EnhancedHAL
        from vehicle_state import VehicleState
        
        # Initialize systems
        vs = VehicleState()
        hal = EnhancedHAL(HARDWARE_CONFIG, vs)
        
        if hal.get_mode() != 'hardware':
            print("❌ Hardware not available, running in simulation mode")
            return 1
        
        print("✓ Hardware mode detected")
        
        # Run hardware test
        print("\nRunning hardware tests...")
        test_results = hal.run_hardware_test()
        
        for component, result in test_results.items():
            status = "✓" if result else "❌"
            print(f"  {status} {component}")
        
        if test_results.get('overall', False):
            print("\n✅ Hardware test passed!")
            return 0
        else:
            print("\n❌ Hardware test failed!")
            return 1
            
    except Exception as e:
        print(f"❌ Test error: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())
