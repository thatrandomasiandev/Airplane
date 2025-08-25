#!/usr/bin/env python3
"""
Test Script for Advanced Aerodynamic Factors
Tests airfoil profiles, aircraft geometry, and materials in efficiency planning
"""

import math
from efficiency_route_planner import (AirfoilProfile, AircraftGeometry, AircraftMaterial, 
                                    AerodynamicState, EfficiencyRoutePlanner)

def test_airfoil_profiles():
    """Test different airfoil profiles"""
    print("=== TESTING AIRFOIL PROFILES ===")
    
    # Test different airfoil types
    airfoils = [
        AirfoilProfile("NACA 0012", 0.12, 0.0, 0.25, 0.0, 6.28, 1.2, 0.006, 500000),      # Symmetric
        AirfoilProfile("NACA 2412", 0.12, 0.02, 0.25, 0.4, 6.5, 1.4, 0.008, 500000),      # Cambered
        AirfoilProfile("NACA 0006", 0.06, 0.0, 0.25, 0.0, 6.0, 0.8, 0.004, 300000),       # Thin
        AirfoilProfile("NACA 0018", 0.18, 0.0, 0.25, 0.0, 6.5, 1.6, 0.012, 800000),       # Thick
    ]
    
    # Test at different angles of attack
    angles = [-5, 0, 5, 10, 15]
    reynolds = 500000
    
    for airfoil in airfoils:
        print(f"\n{airfoil.name}:")
        print(f"  Thickness: {airfoil.thickness_ratio:.2f}, Camber: {airfoil.camber:.3f}")
        
        for angle in angles:
            cl = airfoil.get_lift_coefficient(angle, reynolds)
            cd = airfoil.get_drag_coefficient(angle, reynolds)
            l_d_ratio = cl / cd if cd > 0 else 0
            
            print(f"    AOA {angle:2d}°: CL={cl:5.3f}, CD={cd:5.3f}, L/D={l_d_ratio:5.2f}")
    
    print("✅ Airfoil profile testing completed")

def test_aircraft_geometry():
    """Test different aircraft geometries"""
    print("\n=== TESTING AIRCRAFT GEOMETRY ===")
    
    # Test different aircraft configurations
    geometries = [
        AircraftGeometry(1.0, 0.4, 2.5, 0.35, 0.6, 0.08, 0.04, 0.4, 0.15),    # High aspect ratio
        AircraftGeometry(1.2, 0.5, 2.88, 0.4, 0.8, 0.1, 0.05, 0.5, 0.2),      # Medium aspect ratio
        AircraftGeometry(1.5, 0.6, 3.75, 0.45, 1.0, 0.12, 0.06, 0.6, 0.25),   # Low aspect ratio
    ]
    
    for i, geom in enumerate(geometries):
        print(f"\nGeometry {i+1}:")
        print(f"  Wingspan: {geom.wingspan}m, Area: {geom.wing_area}m²")
        print(f"  Aspect ratio: {geom.aspect_ratio:.2f}")
        print(f"  Induced drag factor: {geom.get_induced_drag_factor():.3f}")
        print(f"  Parasitic drag factor: {geom.get_parasitic_drag_factor():.3f}")
    
    print("✅ Aircraft geometry testing completed")

def test_aircraft_materials():
    """Test different aircraft materials"""
    print("\n=== TESTING AIRCRAFT MATERIALS ===")
    
    # Test different material types
    materials = [
        AircraftMaterial("Aluminum 6061-T6", 2700, 276, 70, 23e-6, 0.9, 0.95),
        AircraftMaterial("Carbon Fiber", 1600, 500, 135, 2e-6, 0.95, 0.98),
        AircraftMaterial("Steel 4130", 7850, 560, 200, 12e-6, 0.7, 0.85),
        AircraftMaterial("Balsa Wood", 150, 30, 4, 3e-6, 0.6, 0.7),
    ]
    
    for material in materials:
        print(f"\n{material.name}:")
        print(f"  Density: {material.density} kg/m³")
        print(f"  Tensile strength: {material.tensile_strength} MPa")
        print(f"  Weight factor: {material.get_weight_factor():.3f}")
        print(f"  Structural efficiency: {material.get_structural_efficiency():.3f}")
    
    print("✅ Aircraft material testing completed")

def test_aerodynamic_state():
    """Test complete aerodynamic state"""
    print("\n=== TESTING AERODYNAMIC STATE ===")
    
    # Create sample components
    airfoil = AirfoilProfile("NACA 0012", 0.12, 0.0, 0.25, 0.0, 6.28, 1.2, 0.006, 500000)
    geometry = AircraftGeometry(1.2, 0.5, 2.88, 0.4, 0.8, 0.1, 0.05, 0.5, 0.2)
    material = AircraftMaterial("Aluminum 6061-T6", 2700, 276, 70, 23e-6, 0.9, 0.95)
    
    # Test different flight conditions
    conditions = [
        {"airspeed": 15, "altitude": 50, "pitch": 0, "roll": 0, "aoa": 3},
        {"airspeed": 20, "altitude": 100, "pitch": 2, "roll": 1, "aoa": 5},
        {"airspeed": 25, "altitude": 150, "pitch": 5, "roll": 2, "aoa": 8},
    ]
    
    for i, condition in enumerate(conditions):
        print(f"\nFlight condition {i+1}:")
        print(f"  Airspeed: {condition['airspeed']} m/s, Altitude: {condition['altitude']}m")
        print(f"  Pitch: {condition['pitch']}°, Roll: {condition['roll']}°, AOA: {condition['aoa']}°")
        
        # Create aerodynamic state
        aero_state = AerodynamicState(
            airspeed=condition['airspeed'],
            altitude=condition['altitude'],
            pitch=condition['pitch'],
            roll=condition['roll'],
            angle_of_attack=condition['aoa'],
            sideslip=0.0,
            lift_coefficient=0.8,
            drag_coefficient=0.06,
            reynolds_number=150000,
            airfoil_profile=airfoil,
            aircraft_geometry=geometry,
            aircraft_material=material
        )
        
        # Calculate efficiency
        efficiency = aero_state.get_efficiency_score()
        l_d_ratio = aero_state.get_lift_drag_ratio()
        optimal_speed = aero_state.get_optimal_airspeed(2.0, 1.15)  # 2kg aircraft, 1.15 kg/m³ air
        
        print(f"  Efficiency score: {efficiency:.3f}")
        print(f"  L/D ratio: {l_d_ratio:.2f}")
        print(f"  Optimal speed: {optimal_speed:.1f} m/s")
        
        # Calculate aerodynamic forces
        forces = aero_state.calculate_aerodynamic_forces(1.15)
        print(f"  Lift: {forces['lift']:.1f} N, Drag: {forces['drag']:.1f} N")
    
    print("✅ Aerodynamic state testing completed")

def test_efficiency_planning_with_aerodynamics():
    """Test efficiency planning with new aerodynamic factors"""
    print("\n=== TESTING EFFICIENCY PLANNING WITH AERODYNAMICS ===")
    
    planner = EfficiencyRoutePlanner()
    
    # Test different airfoil configurations
    airfoils = [
        AirfoilProfile("NACA 0012", 0.12, 0.0, 0.25, 0.0, 6.28, 1.2, 0.006, 500000),      # Standard
        AirfoilProfile("NACA 2412", 0.12, 0.02, 0.25, 0.4, 6.5, 1.4, 0.008, 500000),      # Cambered
        AirfoilProfile("NACA 0006", 0.06, 0.0, 0.25, 0.0, 6.0, 0.8, 0.004, 300000),       # High performance
    ]
    
    # Fixed other parameters
    battery_state = BatteryState(12.3, 3.2, 2200, 22.0, 15, 95.0)
    thrust_profile = ThrustProfile(0.85, 0.75, 25.0, 15.0, 8500, 180.0)
    environmental = EnvironmentalConditions(1.15, 18.0, 101325, 65.0, 3.0, 45.0, 0.2, 5000)
    payload = PayloadCharacteristics(0.3, (0.0, 0.0, 0.05), 0.1, 0.02, "balanced")
    geometry = AircraftGeometry(1.2, 0.5, 2.88, 0.4, 0.8, 0.1, 0.05, 0.5, 0.2)
    material = AircraftMaterial("Aluminum 6061-T6", 2700, 276, 70, 23e-6, 0.9, 0.95)
    
    start_pos = (37.7849, -122.4094, 0.0)
    target_pos = (37.7749, -122.4194, 0.0)
    
    for i, airfoil in enumerate(airfoils):
        print(f"\nTesting airfoil {i+1}: {airfoil.name}")
        print(f"  Thickness: {airfoil.thickness_ratio:.2f}, Camber: {airfoil.camber:.3f}")
        print(f"  CL_alpha: {airfoil.cl_alpha:.2f}, CD_min: {airfoil.cd_min:.3f}")
        
        # Create aerodynamic state with this airfoil
        aerodynamic = AerodynamicState(
            airspeed=22.0,
            altitude=100.0,
            pitch=2.0,
            roll=0.5,
            angle_of_attack=4.5,
            sideslip=0.0,
            lift_coefficient=0.8,
            drag_coefficient=0.06,
            reynolds_number=150000,
            airfoil_profile=airfoil,
            aircraft_geometry=geometry,
            aircraft_material=material
        )
        
        try:
            flight_plan = planner.plan_efficient_route(
                start_pos, target_pos,
                battery_state, thrust_profile, environmental, payload, aerodynamic
            )
            
            print(f"  Efficiency score: {flight_plan['efficiency_score']:.3f}")
            print(f"  Route type: {flight_plan['route_type']}")
            print(f"  Optimal airspeed: {flight_plan['optimal_airspeed']:.1f} m/s")
            
        except Exception as e:
            print(f"  Error: {e}")
    
    print("✅ Efficiency planning with aerodynamics testing completed")

def run_comprehensive_aerodynamic_test():
    """Run comprehensive aerodynamic factors test"""
    print("🚀 COMPREHENSIVE AERODYNAMIC FACTORS TEST")
    print("=" * 60)
    
    test_results = {}
    
    try:
        # Test 1: Airfoil profiles
        test_results['airfoil_profiles'] = True
        test_airfoil_profiles()
        
        # Test 2: Aircraft geometry
        test_results['aircraft_geometry'] = True
        test_aircraft_geometry()
        
        # Test 3: Aircraft materials
        test_results['aircraft_materials'] = True
        test_aircraft_materials()
        
        # Test 4: Aerodynamic state
        test_results['aerodynamic_state'] = True
        test_aerodynamic_state()
        
        # Test 5: Efficiency planning with aerodynamics
        test_results['efficiency_planning'] = True
        test_efficiency_planning_with_aerodynamics()
        
    except Exception as e:
        print(f"❌ Test error: {e}")
        return False
    
    # Print test results
    print("\n" + "=" * 60)
    print("📊 AERODYNAMIC FACTORS TEST RESULTS SUMMARY")
    print("=" * 60)
    
    for test_name, result in test_results.items():
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{test_name:.<35} {status}")
    
    # Calculate pass rate
    passed = sum(test_results.values())
    total = len(test_results)
    pass_rate = (passed / total) * 100
    
    print(f"\nOverall: {passed}/{total} tests passed ({pass_rate:.1f}%)")
    
    if pass_rate >= 90:
        print("🎉 Excellent! Aerodynamic factors system is working perfectly")
    elif pass_rate >= 70:
        print("⚠️  Good! Some issues detected, review failed tests")
    else:
        print("🚨 Poor! Multiple issues detected, system needs attention")
    
    return pass_rate >= 70

def main():
    """Main test function"""
    print("🛩️  Advanced Aerodynamic Factors Test Suite")
    print("Testing airfoil profiles, aircraft geometry, and materials")
    print("=" * 60)
    
    try:
        success = run_comprehensive_aerodynamic_test()
        
        if success:
            print("\n✅ Aerodynamic factors test completed successfully")
        else:
            print("\n❌ Aerodynamic factors test failed")
            
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n\nTest error: {e}")
    
    print("\nTest suite completed")

if __name__ == "__main__":
    main()
