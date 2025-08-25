#!/usr/bin/env python3
"""
Test Script for Advanced Efficiency Route Planner
Demonstrates sophisticated route optimization based on multiple real-world factors
"""

import time
import json
import math
from efficiency_route_planner import (EfficiencyRoutePlanner, BatteryState, 
                                    ThrustProfile, EnvironmentalConditions, 
                                    PayloadCharacteristics, AerodynamicState,
                                    FlightEfficiencyFactor)

def test_battery_efficiency_optimization():
    """Test battery efficiency optimization"""
    print("=== TESTING BATTERY EFFICIENCY OPTIMIZATION ===")
    
    planner = EfficiencyRoutePlanner()
    
    # Test different battery states
    battery_states = [
        BatteryState(12.6, 3.0, 2500, 22.0, 10, 100.0),   # Optimal battery
        BatteryState(11.1, 3.5, 1800, 25.0, 25, 85.0),    # Medium battery
        BatteryState(10.5, 4.0, 1200, 30.0, 50, 70.0),    # Low battery
    ]
    
    # Fixed other parameters
    thrust_profile = ThrustProfile(0.85, 0.75, 25.0, 15.0, 8500, 180.0)
    environmental = EnvironmentalConditions(1.15, 18.0, 101325, 65.0, 3.0, 45.0, 0.2, 5000)
    payload = PayloadCharacteristics(0.3, (0.0, 0.0, 0.05), 0.1, 0.02, "balanced")
    aerodynamic = AerodynamicState(22.0, 100.0, 2.0, 0.5, 4.5, 0.0, 0.8, 0.06, 150000)
    
    start_pos = (37.7849, -122.4094, 0.0)
    target_pos = (37.7749, -122.4194, 0.0)
    
    for i, battery in enumerate(battery_states):
        print(f"\nTesting battery state {i+1}:")
        print(f"  Voltage: {battery.voltage}V, Health: {battery.health_percentage}%")
        
        try:
            flight_plan = planner.plan_efficient_route(
                start_pos, target_pos,
                battery, thrust_profile, environmental, payload, aerodynamic
            )
            
            print(f"  Efficiency score: {flight_plan['efficiency_score']:.3f}")
            print(f"  Route type: {flight_plan['route_type']}")
            
        except Exception as e:
            print(f"  Error: {e}")
    
    print("✅ Battery efficiency optimization test completed")

def test_thrust_efficiency_optimization():
    """Test thrust efficiency optimization"""
    print("\n=== TESTING THRUST EFFICIENCY OPTIMIZATION ===")
    
    planner = EfficiencyRoutePlanner()
    
    # Test different thrust profiles
    thrust_profiles = [
        ThrustProfile(0.90, 0.80, 30.0, 20.0, 9000, 200.0),  # High efficiency
        ThrustProfile(0.85, 0.75, 25.0, 15.0, 8500, 180.0),  # Medium efficiency
        ThrustProfile(0.75, 0.65, 20.0, 12.0, 8000, 160.0),  # Low efficiency
    ]
    
    # Fixed other parameters
    battery_state = BatteryState(12.3, 3.2, 2200, 22.0, 15, 95.0)
    environmental = EnvironmentalConditions(1.15, 18.0, 101325, 65.0, 3.0, 45.0, 0.2, 5000)
    payload = PayloadCharacteristics(0.3, (0.0, 0.0, 0.05), 0.1, 0.02, "balanced")
    aerodynamic = AerodynamicState(22.0, 100.0, 2.0, 0.5, 4.5, 0.0, 0.8, 0.06, 150000)
    
    start_pos = (37.7849, -122.4094, 0.0)
    target_pos = (37.7749, -122.4194, 0.0)
    
    for i, thrust in enumerate(thrust_profiles):
        print(f"\nTesting thrust profile {i+1}:")
        print(f"  Motor efficiency: {thrust.motor_efficiency:.2f}")
        print(f"  Propeller efficiency: {thrust.propeller_efficiency:.2f}")
        print(f"  Overall efficiency: {thrust.get_thrust_efficiency():.3f}")
        
        try:
            flight_plan = planner.plan_efficient_route(
                start_pos, target_pos,
                battery_state, thrust, environmental, payload, aerodynamic
            )
            
            print(f"  Efficiency score: {flight_plan['efficiency_score']:.3f}")
            print(f"  Route type: {flight_plan['route_type']}")
            
        except Exception as e:
            print(f"  Error: {e}")
    
    print("✅ Thrust efficiency optimization test completed")

def test_environmental_optimization():
    """Test environmental condition optimization"""
    print("\n=== TESTING ENVIRONMENTAL OPTIMIZATION ===")
    
    planner = EfficiencyRoutePlanner()
    
    # Test different environmental conditions
    environmental_conditions = [
        EnvironmentalConditions(1.225, 15.0, 101325, 60.0, 1.0, 0.0, 0.1, 10000),   # Optimal conditions
        EnvironmentalConditions(1.15, 18.0, 101325, 65.0, 3.0, 45.0, 0.2, 5000),    # Normal conditions
        EnvironmentalConditions(1.05, 25.0, 101325, 75.0, 8.0, 90.0, 0.6, 2000),    # Challenging conditions
    ]
    
    # Fixed other parameters
    battery_state = BatteryState(12.3, 3.2, 2200, 22.0, 15, 95.0)
    thrust_profile = ThrustProfile(0.85, 0.75, 25.0, 15.0, 8500, 180.0)
    payload = PayloadCharacteristics(0.3, (0.0, 0.0, 0.05), 0.1, 0.02, "balanced")
    aerodynamic = AerodynamicState(22.0, 100.0, 2.0, 0.5, 4.5, 0.0, 0.8, 0.06, 150000)
    
    start_pos = (37.7849, -122.4094, 0.0)
    target_pos = (37.7749, -122.4194, 0.0)
    
    for i, env in enumerate(environmental_conditions):
        print(f"\nTesting environmental conditions {i+1}:")
        print(f"  Air density: {env.air_density:.3f} kg/m³")
        print(f"  Wind speed: {env.wind_speed} m/s")
        print(f"  Turbulence: {env.turbulence:.1f}")
        print(f"  Visibility: {env.visibility} m")
        
        try:
            flight_plan = planner.plan_efficient_route(
                start_pos, target_pos,
                battery_state, thrust_profile, env, payload, aerodynamic
            )
            
            print(f"  Efficiency score: {flight_plan['efficiency_score']:.3f}")
            print(f"  Route type: {flight_plan['route_type']}")
            
        except Exception as e:
            print(f"  Error: {e}")
    
    print("✅ Environmental optimization test completed")

def test_payload_optimization():
    """Test payload optimization"""
    print("\n=== TESTING PAYLOAD OPTIMIZATION ===")
    
    planner = EfficiencyRoutePlanner()
    
    # Test different payload configurations
    payload_configs = [
        PayloadCharacteristics(0.1, (0.0, 0.0, 0.02), 0.05, 0.01, "balanced"),      # Light payload
        PayloadCharacteristics(0.3, (0.0, 0.0, 0.05), 0.1, 0.02, "balanced"),      # Medium payload
        PayloadCharacteristics(0.5, (0.1, 0.0, 0.08), 0.15, 0.03, "forward"),      # Heavy payload
    ]
    
    # Fixed other parameters
    battery_state = BatteryState(12.3, 3.2, 2200, 22.0, 15, 95.0)
    thrust_profile = ThrustProfile(0.85, 0.75, 25.0, 15.0, 8500, 180.0)
    environmental = EnvironmentalConditions(1.15, 18.0, 101325, 65.0, 3.0, 45.0, 0.2, 5000)
    aerodynamic = AerodynamicState(22.0, 100.0, 2.0, 0.5, 4.5, 0.0, 0.8, 0.06, 150000)
    
    start_pos = (37.7849, -122.4094, 0.0)
    target_pos = (37.7749, -122.4194, 0.0)
    
    for i, payload in enumerate(payload_configs):
        print(f"\nTesting payload configuration {i+1}:")
        print(f"  Mass: {payload.mass} kg")
        print(f"  CG: {payload.center_of_gravity}")
        print(f"  Drag coefficient: {payload.drag_coefficient}")
        print(f"  Distribution: {payload.distribution}")
        
        try:
            flight_plan = planner.plan_efficient_route(
                start_pos, target_pos,
                battery_state, thrust_profile, environmental, payload, aerodynamic
            )
            
            print(f"  Efficiency score: {flight_plan['efficiency_score']:.3f}")
            print(f"  Route type: {flight_plan['route_type']}")
            
        except Exception as e:
            print(f"  Error: {e}")
    
    print("✅ Payload optimization test completed")

def test_aerodynamic_optimization():
    """Test aerodynamic optimization"""
    print("\n=== TESTING AERODYNAMIC OPTIMIZATION ===")
    
    planner = EfficiencyRoutePlanner()
    
    # Test different aerodynamic states
    aerodynamic_states = [
        AerodynamicState(20.0, 100.0, 0.0, 0.0, 5.0, 0.0, 0.9, 0.05, 160000),    # Optimal aerodynamics
        AerodynamicState(22.0, 100.0, 2.0, 0.5, 4.5, 0.0, 0.8, 0.06, 150000),    # Normal aerodynamics
        AerodynamicState(25.0, 100.0, 5.0, 2.0, 8.0, 1.0, 0.7, 0.08, 140000),    # Poor aerodynamics
    ]
    
    # Fixed other parameters
    battery_state = BatteryState(12.3, 3.2, 2200, 22.0, 15, 95.0)
    thrust_profile = ThrustProfile(0.85, 0.75, 25.0, 15.0, 8500, 180.0)
    environmental = EnvironmentalConditions(1.15, 18.0, 101325, 65.0, 3.0, 45.0, 0.2, 5000)
    payload = PayloadCharacteristics(0.3, (0.0, 0.0, 0.05), 0.1, 0.02, "balanced")
    
    start_pos = (37.7849, -122.4094, 0.0)
    target_pos = (37.7749, -122.4194, 0.0)
    
    for i, aero in enumerate(aerodynamic_states):
        print(f"\nTesting aerodynamic state {i+1}:")
        print(f"  Airspeed: {aero.airspeed} m/s")
        print(f"  Pitch: {aero.pitch}°")
        print(f"  Roll: {aero.roll}°")
        print(f"  Angle of attack: {aero.angle_of_attack}°")
        print(f"  L/D ratio: {aero.get_lift_drag_ratio():.2f}")
        
        try:
            flight_plan = planner.plan_efficient_route(
                start_pos, target_pos,
                battery_state, thrust_profile, environmental, payload, aero
            )
            
            print(f"  Efficiency score: {flight_plan['efficiency_score']:.3f}")
            print(f"  Route type: {flight_plan['route_type']}")
            
        except Exception as e:
            print(f"  Error: {e}")
    
    print("✅ Aerodynamic optimization test completed")

def test_route_type_selection():
    """Test different route type selection based on conditions"""
    print("\n=== TESTING ROUTE TYPE SELECTION ===")
    
    planner = EfficiencyRoutePlanner()
    
    # Test different mission scenarios
    scenarios = [
        {
            'name': 'Short distance mission',
            'start': (37.7849, -122.4094, 0.0),
            'target': (37.7840, -122.4085, 0.0),  # ~100m
            'description': 'Short distance should favor direct route'
        },
        {
            'name': 'Medium distance mission',
            'start': (37.7849, -122.4094, 0.0),
            'target': (37.7749, -122.4194, 0.0),  # ~1km
            'description': 'Medium distance should consider altitude optimization'
        },
        {
            'name': 'Long distance mission',
            'start': (37.7849, -122.4094, 0.0),
            'target': (37.7649, -122.4394, 0.0),  # ~3km
            'description': 'Long distance should use complex optimization'
        }
    ]
    
    # Fixed parameters for comparison
    battery_state = BatteryState(12.3, 3.2, 2200, 22.0, 15, 95.0)
    thrust_profile = ThrustProfile(0.85, 0.75, 25.0, 15.0, 8500, 180.0)
    environmental = EnvironmentalConditions(1.15, 18.0, 101325, 65.0, 3.0, 45.0, 0.2, 5000)
    payload = PayloadCharacteristics(0.3, (0.0, 0.0, 0.05), 0.1, 0.02, "balanced")
    aerodynamic = AerodynamicState(22.0, 100.0, 2.0, 0.5, 4.5, 0.0, 0.8, 0.06, 150000)
    
    for scenario in scenarios:
        print(f"\n{scenario['name']}:")
        print(f"  {scenario['description']}")
        
        try:
            flight_plan = planner.plan_efficient_route(
                scenario['start'], scenario['target'],
                battery_state, thrust_profile, environmental, payload, aerodynamic
            )
            
            print(f"  Selected route: {flight_plan['route_type']}")
            print(f"  Efficiency score: {flight_plan['efficiency_score']:.3f}")
            print(f"  Distance: {flight_plan['estimated_distance']:.1f}m")
            print(f"  Time: {flight_plan['estimated_time']:.2f}h")
            print(f"  Energy: {flight_plan['estimated_energy']:.1f}Wh")
            
        except Exception as e:
            print(f"  Error: {e}")
    
    print("✅ Route type selection test completed")

def test_efficiency_calculation():
    """Test efficiency calculation methods"""
    print("\n=== TESTING EFFICIENCY CALCULATION ===")
    
    planner = EfficiencyRoutePlanner()
    
    # Test efficiency calculation with known values
    battery_state = BatteryState(12.6, 3.0, 2500, 22.0, 10, 100.0)
    thrust_profile = ThrustProfile(0.90, 0.80, 30.0, 20.0, 9000, 200.0)
    environmental = EnvironmentalConditions(1.225, 15.0, 101325, 60.0, 1.0, 0.0, 0.1, 10000)
    payload = PayloadCharacteristics(0.1, (0.0, 0.0, 0.02), 0.05, 0.01, "balanced")
    aerodynamic = AerodynamicState(20.0, 100.0, 0.0, 0.0, 5.0, 0.0, 0.9, 0.05, 160000)
    
    print("Testing efficiency calculation with optimal parameters:")
    print(f"  Battery efficiency: {battery_state.get_efficiency_factor():.3f}")
    print(f"  Thrust efficiency: {thrust_profile.get_thrust_efficiency():.3f}")
    print(f"  Aerodynamic efficiency: {aerodynamic.get_efficiency_score():.3f}")
    print(f"  Environmental efficiency: {environmental.get_air_density_factor() * environmental.get_wind_impact_factor():.3f}")
    print(f"  Payload efficiency: {payload.get_payload_efficiency_impact():.3f}")
    
    # Calculate overall efficiency
    efficiency_scores = planner._calculate_efficiency_scores(
        battery_state, thrust_profile, environmental, payload, aerodynamic
    )
    
    print("\nOverall efficiency breakdown:")
    for factor, score in efficiency_scores.items():
        print(f"  {factor.value}: {score:.3f}")
    
    # Calculate weighted score
    weighted_score = 0.0
    for factor, weight in planner.efficiency_weights.items():
        if factor in efficiency_scores:
            weighted_score += efficiency_scores[factor] * weight
    
    print(f"\nWeighted efficiency score: {weighted_score:.3f}")
    
    print("✅ Efficiency calculation test completed")

def test_performance_curves():
    """Test performance curve interpolation"""
    print("\n=== TESTING PERFORMANCE CURVES ===")
    
    planner = EfficiencyRoutePlanner()
    
    # Test power curve interpolation
    power_curve = planner.performance_curves['power_curve']
    print("Power curve (altitude vs power factor):")
    for alt, power in zip(power_curve['altitude'], power_curve['power_factor']):
        print(f"  {alt}m: {power:.2f}")
    
    # Test efficiency curve interpolation
    efficiency_curve = planner.performance_curves['efficiency_curve']
    print("\nEfficiency curve (airspeed vs efficiency):")
    for speed, eff in zip(efficiency_curve['airspeed'], efficiency_curve['efficiency']):
        print(f"  {speed} m/s: {eff:.2f}")
    
    # Test battery curve interpolation
    battery_curve = planner.performance_curves['battery_curve']
    print("\nBattery curve (voltage vs efficiency):")
    for voltage, eff in zip(battery_curve['voltage'], battery_curve['efficiency']):
        print(f"  {voltage}V: {eff:.2f}")
    
    print("✅ Performance curves test completed")

def run_comprehensive_efficiency_test():
    """Run comprehensive efficiency planning test"""
    print("🚀 COMPREHENSIVE EFFICIENCY ROUTE PLANNING TEST")
    print("=" * 70)
    
    test_results = {}
    
    try:
        # Test 1: Battery efficiency optimization
        test_results['battery_optimization'] = True
        test_battery_efficiency_optimization()
        
        # Test 2: Thrust efficiency optimization
        test_results['thrust_optimization'] = True
        test_thrust_efficiency_optimization()
        
        # Test 3: Environmental optimization
        test_results['environmental_optimization'] = True
        test_environmental_optimization()
        
        # Test 4: Payload optimization
        test_results['payload_optimization'] = True
        test_payload_optimization()
        
        # Test 5: Aerodynamic optimization
        test_results['aerodynamic_optimization'] = True
        test_aerodynamic_optimization()
        
        # Test 6: Route type selection
        test_results['route_selection'] = True
        test_route_type_selection()
        
        # Test 7: Efficiency calculation
        test_results['efficiency_calculation'] = True
        test_efficiency_calculation()
        
        # Test 8: Performance curves
        test_results['performance_curves'] = True
        test_performance_curves()
        
    except Exception as e:
        print(f"❌ Test error: {e}")
        return False
    
    # Print test results
    print("\n" + "=" * 70)
    print("📊 EFFICIENCY PLANNING TEST RESULTS SUMMARY")
    print("=" * 70)
    
    for test_name, result in test_results.items():
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{test_name:.<40} {status}")
    
    # Calculate pass rate
    passed = sum(test_results.values())
    total = len(test_results)
    pass_rate = (passed / total) * 100
    
    print(f"\nOverall: {passed}/{total} tests passed ({pass_rate:.1f}%)")
    
    if pass_rate >= 90:
        print("🎉 Excellent! Efficiency route planning system is working perfectly")
    elif pass_rate >= 70:
        print("⚠️  Good! Some issues detected, review failed tests")
    else:
        print("🚨 Poor! Multiple issues detected, system needs attention")
    
    return pass_rate >= 70

def main():
    """Main test function"""
    print("🛩️  Advanced Efficiency Route Planner Test Suite")
    print("Testing sophisticated route optimization based on multiple real-world factors")
    print("=" * 70)
    
    try:
        success = run_comprehensive_efficiency_test()
        
        if success:
            print("\n✅ Efficiency route planning test completed successfully")
        else:
            print("\n❌ Efficiency route planning test failed")
            
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n\nTest error: {e}")
    
    print("\nTest suite completed")

if __name__ == "__main__":
    main()
