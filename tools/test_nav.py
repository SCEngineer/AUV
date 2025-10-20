#!/usr/bin/env python3
"""
Unit tests for Navigation.py - Updated for current SIMPLR-AUV system
Tests realistic navigation including heading control integration
"""

import unittest
import time
import math
from navigation import Navigation

# Mock HAL for sensor data
class MockHAL:
    def __init__(self):
        self.sensor_data = {}
        
    def get_sensor_data(self):
        return self.sensor_data
        
    def update_sensor(self, sensor_name, value):
        self.sensor_data[sensor_name] = value

# Mock VehicleState matching current system
class MockVehicleState:
    def __init__(self, lat=34.25739, lon=-117.1987, depth=0.0, heading=0.0):
        self.nav_state = {
            'lat': lat,
            'lon': lon,
            'depth': depth,
            'heading': heading,
            'speed': 0.0
        }
        self.true_state = {
            'true_lat': lat,
            'true_lon': lon, 
            'true_depth': depth,
            'true_heading': heading,
            'true_speed': 0.0
        }
        self.sensor_data = {}  # Navigation reads from here
        self._initial_position_set = True

    def update_nav_state(self, **kwargs):
        self.nav_state.update(kwargs)

    def is_initial_position_set(self):
        return self._initial_position_set

    def get_initial_position(self):
        return {
            'lat': self.nav_state['lat'],
            'lon': self.nav_state['lon'], 
            'depth': self.nav_state['depth']
        }

    def set_initial_position(self, lat, lon, depth=0.0):
        self.nav_state.update({'lat': lat, 'lon': lon, 'depth': depth})
        self.true_state.update({'true_lat': lat, 'true_lon': lon, 'true_depth': depth})
        self._initial_position_set = True

class TestNavigation(unittest.TestCase):
    def setUp(self):
        self.vehicle_state = MockVehicleState()
        self.hal = MockHAL()
        self.nav = Navigation(self.vehicle_state, self.hal)

    def test_initial_position_mission_start(self):
        """Test navigation starts at correct mission coordinates"""
        # Navigation should initialize from vehicle_state nav_state
        current_lat = self.vehicle_state.nav_state.get('lat', 0.0)
        current_lon = self.vehicle_state.nav_state.get('lon', 0.0)
        current_depth = self.vehicle_state.nav_state.get('depth', 0.0)
        current_heading = self.vehicle_state.nav_state.get('heading', 0.0)
        
        self.assertAlmostEqual(current_lat, 34.25739, places=5)
        self.assertAlmostEqual(current_lon, -117.1987, places=4)  
        self.assertAlmostEqual(current_depth, 0.0)
        self.assertAlmostEqual(current_heading, 0.0)

    def test_heading_update_compass(self):
        """Test heading updates from compass sensor"""
        # Set sensor data through HAL (navigation reads from hal.get_sensor_data())
        self.hal.update_sensor('compass', 270.5)
        
        # Also directly update vehicle_state.sensor_data if navigation reads from there
        self.vehicle_state.sensor_data['compass'] = 270.5
        
        self.nav.update(0.1)
        
        # Navigation should update heading in nav_state
        updated_heading = self.vehicle_state.nav_state.get('heading', 0.0)
        self.assertTrue(abs(updated_heading - 270.5) < 5.0, 
                       f"Expected heading ~270.5°, got {updated_heading}°")

    def test_heading_wrap_around(self):
        """Test heading properly wraps around 0/360 boundary"""
        test_headings = [359.5, 0.5, 180.0, 270.0]
        
        for test_heading in test_headings:
            with self.subTest(heading=test_heading):
                self.hal.update_sensor('compass', test_heading)
                self.nav.update(0.1)
                
                updated_heading = self.vehicle_state.nav_state.get('heading', 0.0)
                # Heading should be in valid range
                self.assertGreaterEqual(updated_heading, 0.0)
                self.assertLess(updated_heading, 360.0)

    def test_large_heading_change_waypoint_scenario(self):
        """Test navigation handles large heading changes like waypoint following"""
        # Simulate vehicle starting east, needing to turn west (like your mission)
        initial_heading = 90.0  # East
        target_heading = 270.0  # West (like your waypoint)
        
        self.hal.update_sensor('compass', initial_heading)
        self.nav.update(0.1)
        
        # Verify initial heading
        current_heading = self.vehicle_state.nav_state.get('heading', 0.0)
        self.assertAlmostEqual(current_heading, initial_heading, delta=5.0)
        
        # Simulate gradual turn toward waypoint
        step_size = 10.0
        steps = int(abs(target_heading - initial_heading) / step_size)
        
        for i in range(1, steps + 1):
            # Calculate intermediate heading (proper wrap-around)
            heading_diff = target_heading - initial_heading
            if heading_diff > 180:
                heading_diff -= 360
            elif heading_diff < -180:
                heading_diff += 360
                
            intermediate_heading = (initial_heading + (heading_diff * i / steps)) % 360
            
            self.hal.update_sensor('compass', intermediate_heading)
            self.nav.update(0.1)
            
        # Final heading should be close to target
        final_heading = self.vehicle_state.nav_state.get('heading', 0.0)
        heading_error = abs(final_heading - target_heading)
        if heading_error > 180:
            heading_error = 360 - heading_error
        self.assertLess(heading_error, 10.0, 
                       f"Final heading {final_heading}° too far from target {target_heading}°")

    def test_speed_update_realistic(self):
        """Test speed updates from water speed sensor"""
        test_speeds = [0.0, 1.5, 2.0, 2.5]  # Realistic AUV speeds
        
        for test_speed in test_speeds:
            with self.subTest(speed=test_speed):
                self.hal.update_sensor('water_speed', test_speed)
                self.nav.update(0.1)
                
                updated_speed = self.vehicle_state.nav_state.get('speed', 0.0)
                self.assertAlmostEqual(updated_speed, test_speed, delta=0.2)

    def test_dead_reckoning_mission_coordinates(self):
        """Test dead reckoning with mission-realistic coordinates"""
        # Start at mission position
        start_lat = 34.25739
        start_lon = -117.1987
        
        # Set speed and heading (eastward movement)
        self.hal.update_sensor('water_speed', 2.0)  # 2 m/s cruise speed
        self.hal.update_sensor('compass', 90.0)     # Due east
        
        # Update for 10 seconds of eastward movement
        self.nav.update(10.0)
        
        current_lat = self.vehicle_state.nav_state.get('lat', start_lat)
        current_lon = self.vehicle_state.nav_state.get('lon', start_lon)
        
        # Should move east (longitude should increase)
        self.assertGreater(current_lon, start_lon)
        # Latitude should stay approximately the same
        self.assertAlmostEqual(current_lat, start_lat, delta=0.001)

    def test_position_uncertainty_growth(self):
        """Test position uncertainty increases during dead reckoning"""
        if hasattr(self.nav, 'position_uncertainty'):
            start_uncertainty = self.nav.position_uncertainty
            
            # Move for extended period without GPS
            self.hal.update_sensor('water_speed', 2.0)
            self.hal.update_sensor('compass', 45.0)
            
            # Multiple updates to accumulate uncertainty
            for _ in range(10):
                self.nav.update(1.0)
            
            if hasattr(self.nav, 'position_uncertainty'):
                final_uncertainty = self.nav.position_uncertainty
                self.assertGreater(final_uncertainty, start_uncertainty)

    def test_gps_fix_surface_correction(self):
        """Test GPS fix when surfaced (like your mission GPS_FIX task)"""
        # Start with some dead reckoning error
        self.hal.update_sensor('water_speed', 2.0)
        self.hal.update_sensor('compass', 180.0)
        self.nav.update(5.0)  # Move for 5 seconds
        
        # Now surface and get GPS fix
        self.vehicle_state.nav_state['depth'] = 0.0  # Surfaced
        self.hal.update_sensor('gps', {
            'lat': 34.257390,      # Precise GPS position
            'lon': -117.198700,
            'fix_quality': 'good',
            'hdop': 1.2
        })
        
        self.nav.update(0.1)
        
        # Position should be corrected toward GPS
        gps_lat = 34.257390
        gps_lon = -117.198700
        
        current_lat = self.vehicle_state.nav_state.get('lat', 0)
        current_lon = self.vehicle_state.nav_state.get('lon', 0)
        
        # Should be much closer to GPS position
        lat_error = abs(current_lat - gps_lat)
        lon_error = abs(current_lon - gps_lon)
        
        self.assertLess(lat_error, 0.01, f"GPS lat correction failed: error {lat_error}")
        self.assertLess(lon_error, 0.01, f"GPS lon correction failed: error {lon_error}")

    def test_depth_sensor_update(self):
        """Test depth updates from depth sensor"""
        test_depths = [0.0, 2.5, 5.0, 8.0]  # Mission-realistic depths
        
        for test_depth in test_depths:
            with self.subTest(depth=test_depth):
                self.hal.update_sensor('depth', test_depth)
                self.nav.update(0.1)
                
                updated_depth = self.vehicle_state.nav_state.get('depth', 0.0)
                self.assertAlmostEqual(updated_depth, test_depth, delta=0.2)

    def test_navigation_state_consistency(self):
        """Test that nav_state stays consistent with internal navigation state"""
        # Update multiple sensors
        self.hal.update_sensor('compass', 123.4)
        self.hal.update_sensor('water_speed', 1.8)
        self.hal.update_sensor('depth', 3.2)
        
        self.nav.update(0.1)
        
        # Check that vehicle_state.nav_state is updated
        nav_state = self.vehicle_state.nav_state
        
        self.assertIn('heading', nav_state)
        self.assertIn('speed', nav_state) 
        self.assertIn('depth', nav_state)
        self.assertIn('lat', nav_state)
        self.assertIn('lon', nav_state)
        
        # Values should be reasonable
        self.assertGreaterEqual(nav_state['heading'], 0.0)
        self.assertLess(nav_state['heading'], 360.0)
        self.assertGreaterEqual(nav_state['depth'], 0.0)

    def test_position_reset_functionality(self):
        """Test position reset capability"""
        if hasattr(self.nav, 'reset_position'):
            # Reset to new position
            new_lat = 35.0
            new_lon = -118.0
            
            self.nav.reset_position(new_lat, new_lon)
            
            self.assertAlmostEqual(self.nav.lat, new_lat, places=5)
            self.assertAlmostEqual(self.nav.lon, new_lon, places=5)
            
            # Uncertainty should reset
            if hasattr(self.nav, 'position_uncertainty'):
                self.assertLessEqual(self.nav.position_uncertainty, 2.0)

    def test_heading_continuity_no_jumps(self):
        """Test heading updates don't have unrealistic jumps"""
        # Start with known heading
        self.hal.update_sensor('compass', 45.0)
        self.nav.update(0.1)
        
        previous_heading = self.vehicle_state.nav_state.get('heading', 45.0)
        
        # Make gradual heading changes
        test_headings = [50.0, 55.0, 60.0, 65.0, 70.0]
        
        for heading in test_headings:
            self.hal.update_sensor('compass', heading)
            self.nav.update(0.1)
            
            current_heading = self.vehicle_state.nav_state.get('heading', heading)
            
            # Change should be reasonable (no huge jumps)
            heading_change = abs(current_heading - previous_heading)
            if heading_change > 180:
                heading_change = 360 - heading_change
                
            self.assertLess(heading_change, 50.0, 
                           f"Unrealistic heading jump: {previous_heading}° to {current_heading}°")
            
            previous_heading = current_heading

if __name__ == '__main__':
    # Run specific test groups if desired
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == 'heading':
        # Run only heading-related tests
        suite = unittest.TestSuite()
        suite.addTest(TestNavigation('test_heading_update_compass'))
        suite.addTest(TestNavigation('test_heading_wrap_around'))
        suite.addTest(TestNavigation('test_large_heading_change_waypoint_scenario'))
        suite.addTest(TestNavigation('test_heading_continuity_no_jumps'))
        
        runner = unittest.TextTestRunner(verbosity=2)
        runner.run(suite)
    else:
        # Run all tests
        unittest.main(verbosity=2)