#!/usr/bin/env python3
"""
Interface Alignment Checker for SIMPLR-AUV modules
Identifies integration issues between modules
"""

import ast
import os
from typing import Dict, List, Set, Tuple
from pathlib import Path

class InterfaceAnalyzer:
    def __init__(self):
        self.modules = {}
        self.interfaces = {}
        self.issues = []
        
    def analyze_file(self, filepath: str):
        """Analyze a Python file for interface definitions"""
        try:
            with open(filepath, 'r') as f:
                content = f.read()
            
            tree = ast.parse(content)
            module_name = Path(filepath).stem
            
            self.modules[module_name] = {
                'classes': {},
                'functions': {},
                'imports': [],
                'vehicle_state_usage': {'reads': set(), 'writes': set()}
            }
            
            for node in ast.walk(tree):
                if isinstance(node, ast.ClassDef):
                    self._analyze_class(node, module_name)
                elif isinstance(node, ast.FunctionDef):
                    self._analyze_function(node, module_name)
                elif isinstance(node, ast.Import) or isinstance(node, ast.ImportFrom):
                    self._analyze_import(node, module_name)
                elif isinstance(node, ast.Attribute):
                    self._analyze_vehicle_state_access(node, module_name)
                    
        except Exception as e:
            print(f"Error analyzing {filepath}: {e}")
    
    def _analyze_class(self, node: ast.ClassDef, module_name: str):
        """Analyze class definitions"""
        class_info = {
            'methods': {},
            'init_params': [],
            'vehicle_state_usage': {'reads': set(), 'writes': set()}
        }
        
        for item in node.body:
            if isinstance(item, ast.FunctionDef):
                if item.name == '__init__':
                    class_info['init_params'] = [arg.arg for arg in item.args.args[1:]]  # Skip 'self'
                class_info['methods'][item.name] = {
                    'params': [arg.arg for arg in item.args.args[1:]],  # Skip 'self'
                    'returns': self._has_return(item)
                }
        
        self.modules[module_name]['classes'][node.name] = class_info
    
    def _analyze_function(self, node: ast.FunctionDef, module_name: str):
        """Analyze function definitions"""
        self.modules[module_name]['functions'][node.name] = {
            'params': [arg.arg for arg in node.args.args],
            'returns': self._has_return(node)
        }
    
    def _analyze_import(self, node, module_name: str):
        """Analyze import statements"""
        if isinstance(node, ast.Import):
            for alias in node.names:
                self.modules[module_name]['imports'].append(alias.name)
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                for alias in node.names:
                    self.modules[module_name]['imports'].append(f"{node.module}.{alias.name}")
    
    def _analyze_vehicle_state_access(self, node: ast.Attribute, module_name: str):
        """Analyze vehicle_state attribute access patterns"""
        if hasattr(node, 'value') and isinstance(node.value, ast.Attribute):
            if (hasattr(node.value, 'attr') and node.value.attr == 'vehicle_state' or
                hasattr(node.value, 'id') and node.value.id == 'vehicle_state'):
                
                # Determine if it's a read or write
                parent = getattr(node, 'parent', None)
                if isinstance(parent, ast.Assign):
                    self.modules[module_name]['vehicle_state_usage']['writes'].add(node.attr)
                else:
                    self.modules[module_name]['vehicle_state_usage']['reads'].add(node.attr)
    
    def _has_return(self, node: ast.FunctionDef) -> bool:
        """Check if function has return statement"""
        for item in ast.walk(node):
            if isinstance(item, ast.Return) and item.value is not None:
                return True
        return False
    
    def check_integration_issues(self):
        """Check for integration issues between modules"""
        print("=" * 80)
        print("SIMPLR-AUV INTERFACE ALIGNMENT ANALYSIS")
        print("=" * 80)
        
        # Issue 1: Multiple modules updating navigation state
        self._check_nav_state_conflicts()
        
        # Issue 2: Sensor data flow inconsistencies
        self._check_sensor_data_flow()
        
        # Issue 3: Control loop integration
        self._check_control_loop_integration()
        
        # Issue 4: Update method signatures
        self._check_update_method_consistency()
        
        # Issue 5: Vehicle state interface usage
        self._check_vehicle_state_usage()
        
        self._print_summary()
    
    def _check_nav_state_conflicts(self):
        """Check for conflicts in navigation state updates"""
        print("\n1. NAVIGATION STATE UPDATE CONFLICTS:")
        print("-" * 50)
        
        nav_updaters = []
        for module, info in self.modules.items():
            if 'nav_state' in info['vehicle_state_usage']['writes'] or \
               'update_nav_state' in str(info):
                nav_updaters.append(module)
        
        if len(nav_updaters) > 1:
            print(f"❌ CONFLICT: Multiple modules updating nav_state: {nav_updaters}")
            print("   RECOMMENDATION: Only sensor_models (via HAL) should update nav_state")
            self.issues.append("Multiple nav_state updaters")
        else:
            print(f"✅ OK: Navigation state updated by: {nav_updaters}")
    
    def _check_sensor_data_flow(self):
        """Check sensor data flow consistency"""
        print("\n2. SENSOR DATA FLOW:")
        print("-" * 50)
        
        # Check if sensor_models is integrated properly
        has_sensor_models = 'sensor_models' in self.modules
        has_navigation = 'navigation' in self.modules
        
        if has_sensor_models and has_navigation:
            print("❌ CONFLICT: Both sensor_models and navigation modules exist")
            print("   ISSUE: navigation.py has its own sensor processing")
            print("   RECOMMENDATION: Remove navigation.py, use sensor_models via HAL")
            self.issues.append("Duplicate sensor processing")
        elif has_sensor_models:
            print("✅ OK: Using sensor_models for sensor simulation")
        elif has_navigation:
            print("⚠️  WARNING: Using legacy navigation module")
        else:
            print("❌ ERROR: No sensor processing module found")
    
    def _check_control_loop_integration(self):
        """Check control loop integration"""
        print("\n3. CONTROL LOOP INTEGRATION:")
        print("-" * 50)
        
        has_vehicle_control = 'vehicle_control' in self.modules
        has_vehicle_dynamics = 'vehicle_dynamics' in self.modules
        
        if has_vehicle_control and has_vehicle_dynamics:
            # Check if they're properly connected
            vc_info = self.modules.get('vehicle_control', {})
            vd_info = self.modules.get('vehicle_dynamics', {})
            
            print("✅ OK: Both vehicle_control and vehicle_dynamics present")
            
            # Check for pitch update conflicts
            print("   Checking pitch/attitude updates...")
            if 'nav_pitch' in vc_info.get('vehicle_state_usage', {}).get('reads', set()):
                print("   ✅ vehicle_control reads nav_pitch (correct)")
            else:
                print("   ❌ vehicle_control may not be reading nav_pitch")
                self.issues.append("vehicle_control not reading nav_pitch")
                
        else:
            print("❌ ERROR: Missing control loop components")
            self.issues.append("Incomplete control loop")
    
    def _check_update_method_consistency(self):
        """Check update method parameter consistency"""
        print("\n4. UPDATE METHOD SIGNATURES:")
        print("-" * 50)
        
        update_signatures = {}
        for module, info in self.modules.items():
            for class_name, class_info in info['classes'].items():
                if 'update' in class_info['methods']:
                    params = class_info['methods']['update']['params']
                    update_signatures[f"{module}.{class_name}"] = params
        
        # Check for inconsistencies
        param_sets = set(tuple(params) for params in update_signatures.values())
        
        if len(param_sets) > 1:
            print("❌ INCONSISTENT update() method signatures:")
            for class_name, params in update_signatures.items():
                print(f"   {class_name}: {params}")
            print("   RECOMMENDATION: Standardize on update(self, dt: float)")
            self.issues.append("Inconsistent update signatures")
        else:
            print("✅ OK: Consistent update() method signatures")
            for class_name, params in update_signatures.items():
                print(f"   {class_name}: {params}")
    
    def _check_vehicle_state_usage(self):
        """Check vehicle_state interface usage patterns"""
        print("\n5. VEHICLE_STATE INTERFACE USAGE:")
        print("-" * 50)
        
        critical_fields = {
            'true_state': 'Should only be written by vehicle_dynamics',
            'nav_state': 'Should only be written by sensor processing (HAL)',
            'target_state': 'Should be written by guidance tasks',
            'actuator_commands': 'Should be written by vehicle_control'
        }
        
        for field, recommendation in critical_fields.items():
            writers = []
            readers = []
            
            for module, info in self.modules.items():
                if field in info['vehicle_state_usage']['writes']:
                    writers.append(module)
                if field in info['vehicle_state_usage']['reads']:
                    readers.append(module)
            
            print(f"\n   {field}:")
            print(f"     Writers: {writers}")
            print(f"     Readers: {readers}")
            print(f"     Rule: {recommendation}")
            
            # Check for violations
            if field == 'true_state' and writers and 'vehicle_dynamics' not in writers:
                print(f"     ❌ VIOLATION: {field} written by non-dynamics modules")
                self.issues.append(f"Incorrect {field} writers")
            elif field == 'nav_state' and len(writers) > 1:
                print(f"     ❌ VIOLATION: Multiple writers for {field}")
                self.issues.append(f"Multiple {field} writers")
    
    def _print_summary(self):
        """Print analysis summary"""
        print("\n" + "=" * 80)
        print("ANALYSIS SUMMARY")
        print("=" * 80)
        
        if not self.issues:
            print("✅ NO CRITICAL INTEGRATION ISSUES FOUND")
        else:
            print(f"❌ FOUND {len(self.issues)} INTEGRATION ISSUES:")
            for i, issue in enumerate(self.issues, 1):
                print(f"   {i}. {issue}")
        
        print(f"\n📊 MODULES ANALYZED: {len(self.modules)}")
        for module in sorted(self.modules.keys()):
            classes = len(self.modules[module]['classes'])
            functions = len(self.modules[module]['functions'])
            print(f"   {module}: {classes} classes, {functions} functions")
        
        # Key recommendations
        print("\n🔧 KEY RECOMMENDATIONS:")
        print("   1. Remove navigation.py - use sensor_models via enhanced_hal")
        print("   2. Ensure only HAL updates nav_state from sensors")
        print("   3. Fix nav_pitch not being updated from IMU data")
        print("   4. Standardize update() method signatures to update(dt: float)")
        print("   5. Verify pitch data flows: IMU → sensors → nav_state → control")


def main():
    """Main analysis function"""
    analyzer = InterfaceAnalyzer()
    
    # List of files to analyze
    files_to_analyze = [
        'vehicle_state.py',
        'vehicle_control.py', 
        'vehicle_dynamics.py',
        'sensor_models.py',
        'navigation.py',
        'enhanced_hal.py',
        'auv_main.py'
    ]
    
    print("Analyzing SIMPLR-AUV module interfaces...")
    
    for filename in files_to_analyze:
        if os.path.exists(filename):
            print(f"  Analyzing {filename}...")
            analyzer.analyze_file(filename)
        else:
            print(f"  ⚠️  File not found: {filename}")
    
    analyzer.check_integration_issues()
    
    print(f"\n💡 TO FIX THE CLIMB TASK PITCH ISSUE:")
    print("   The problem is likely that nav_pitch stays at 0° while true_pitch is 30°")
    print("   This means IMU pitch data isn't flowing to nav_state properly")
    print("   Check: sensor_models._simulate_imu() → enhanced_hal._update_nav_from_sensors()")

if __name__ == "__main__":
    main()