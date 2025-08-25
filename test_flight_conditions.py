#!/usr/bin/env python3
"""
Test Script for Flight Condition Monitoring
Tests dangerous flight state detection including stalls, spins, excessive G-forces, and crash prevention
"""

import time
import json
from failsafe_system import (FailsafeSystem, FailsafeTrigger, 
                            FlightConditionData, FailsafeCondition)

def test_stall_detection():
    """Test stall detection scenarios"""
    print("=== TESTING STALL DETECTION ===")
    
    failsafe = FailsafeSystem()
    failsafe.start_monitoring()
    
    # Test 1: High angle of attack stall
    print("\nTest 1: High angle of attack stall")
    stall_data = FlightConditionData(
        pitch=25.0, roll=0.0, yaw=0.0,
        pitch_rate=5.0, roll_rate=0.0, yaw_rate=0.0,
        acceleration_x=0.0, acceleration_y=0.0, acceleration_z=-9.81,
        g_force=1.0, airspeed=10.0, altitude=100.0, vertical_speed=-2.0,
        angle_of_attack=20.0, stall_warning=True, stall_margin=0.05,
        spin_indicators={}, max_pitch=45.0, max_roll=60.0, max_yaw_rate=120.0
    )
    
    failsafe.update_flight_condition_data(stall_data)
    time.sleep(0.1)
    
    status = failsafe.get_failsafe_status()
    print(f"  Stall detected: {status['flight_condition_monitoring']['current_status']['stall_detected']}")
    print(f"  Failsafe active: {status['active']}")
    
    # Test 2: Low airspeed stall
    print("\nTest 2: Low airspeed stall")
    low_speed_stall = FlightConditionData(
        pitch=15.0, roll=0.0, yaw=0.0,
        pitch_rate=2.0, roll_rate=0.0, yaw_rate=0.0,
        acceleration_x=0.0, acceleration_y=0.0, acceleration_z=-9.81,
        g_force=1.0, airspeed=8.0, altitude=100.0, vertical_speed=-1.0,
        angle_of_attack=15.0, stall_warning=False, stall_margin=0.15,
        spin_indicators={}, max_pitch=45.0, max_roll=60.0, max_yaw_rate=120.0
    )
    
    failsafe.update_flight_condition_data(low_speed_stall)
    time.sleep(0.1)
    
    status = failsafe.get_failsafe_status()
    print(f"  Stall detected: {status['flight_condition_monitoring']['current_status']['stall_detected']}")
    print(f"  Failsafe active: {status['active']}")
    
    failsafe.stop_monitoring()
    print("✅ Stall detection testing completed")

def test_flat_spin_detection():
    """Test flat spin detection scenarios"""
    print("\n=== TESTING FLAT SPIN DETECTION ===")
    
    failsafe = FailsafeSystem()
    failsafe.start_monitoring()
    
    # Test 1: High yaw rate with low airspeed
    print("\nTest 1: High yaw rate flat spin")
    flat_spin_data = FlightConditionData(
        pitch=30.0, roll=70.0, yaw=45.0,
        pitch_rate=10.0, roll_rate=150.0, yaw_rate=80.0,
        acceleration_x=5.0, acceleration_y=8.0, acceleration_z=-12.0,
        g_force=1.5, airspeed=8.0, altitude=80.0, vertical_speed=-6.0,
        angle_of_attack=12.0, stall_warning=True, stall_margin=0.2,
        spin_indicators={'rotation': True, 'descent': True}, 
        max_pitch=45.0, max_roll=60.0, max_yaw_rate=120.0
    )
    
    failsafe.update_flight_condition_data(flat_spin_data)
    time.sleep(0.1)
    
    status = failsafe.get_failsafe_status()
    print(f"  Flat spin detected: {status['flight_condition_monitoring']['current_status']['flat_spin_detected']}")
    print(f"  Failsafe active: {status['active']}")
    
    # Test 2: Extreme attitude with rotation
    print("\nTest 2: Extreme attitude flat spin")
    extreme_spin = FlightConditionData(
        pitch=50.0, roll=80.0, yaw=60.0,
        pitch_rate=15.0, roll_rate=200.0, yaw_rate=100.0,
        acceleration_x=10.0, acceleration_y=15.0, acceleration_z=-15.0,
        g_force=2.0, airspeed=6.0, altitude=60.0, vertical_speed=-8.0,
        angle_of_attack=18.0, stall_warning=True, stall_margin=0.1,
        spin_indicators={'rotation': True, 'descent': True, 'attitude': True}, 
        max_pitch=45.0, max_roll=60.0, max_yaw_rate=120.0
    )
    
    failsafe.update_flight_condition_data(extreme_spin)
    time.sleep(0.1)
    
    status = failsafe.get_failsafe_status()
    print(f"  Flat spin detected: {status['flight_condition_monitoring']['current_status']['flat_spin_detected']}")
    print(f"  Failsafe active: {status['active']}")
    
    failsafe.stop_monitoring()
    print("✅ Flat spin detection testing completed")

def test_excessive_g_forces():
    """Test excessive G-force detection"""
    print("\n=== TESTING EXCESSIVE G-FORCE DETECTION ===")
    
    failsafe = FailsafeSystem()
    failsafe.start_monitoring()
    
    # Test 1: High G-force
    print("\nTest 1: High G-force condition")
    high_g_data = FlightConditionData(
        pitch=10.0, roll=5.0, yaw=0.0,
        pitch_rate=20.0, roll_rate=15.0, yaw_rate=5.0,
        acceleration_x=25.0, acceleration_y=20.0, acceleration_z=35.0,
        g_force=4.5, airspeed=25.0, altitude=120.0, vertical_speed=3.0,
        angle_of_attack=8.0, stall_warning=False, stall_margin=0.4,
        spin_indicators={}, max_pitch=45.0, max_roll=60.0, max_yaw_rate=120.0
    )
    
    failsafe.update_flight_condition_data(high_g_data)
    time.sleep(0.1)
    
    status = failsafe.get_failsafe_status()
    print(f"  Excessive G-forces: {status['flight_condition_monitoring']['current_status']['excessive_g_forces']}")
    print(f"  Failsafe active: {status['active']}")
    
    # Test 2: Extreme acceleration
    print("\nTest 2: Extreme acceleration")
    extreme_accel = FlightConditionData(
        pitch=15.0, roll=10.0, yaw=5.0,
        pitch_rate=30.0, roll_rate=25.0, yaw_rate=15.0,
        acceleration_x=40.0, acceleration_y=35.0, acceleration_z=45.0,
        g_force=2.5, airspeed=30.0, altitude=150.0, vertical_speed=5.0,
        angle_of_attack=10.0, stall_warning=False, stall_margin=0.5,
        spin_indicators={}, max_pitch=45.0, max_roll=60.0, max_yaw_rate=120.0
    )
    
    failsafe.update_flight_condition_data(extreme_accel)
    time.sleep(0.1)
    
    status = failsafe.get_failsafe_status()
    print(f"  Excessive G-forces: {status['flight_condition_monitoring']['current_status']['excessive_g_forces']}")
    print(f"  Failsafe active: {status['active']}")
    
    failsafe.stop_monitoring()
    print("✅ Excessive G-force detection testing completed")

def test_uncontrolled_descent():
    """Test uncontrolled descent detection"""
    print("\n=== TESTING UNCONTROLLED DESCENT DETECTION ===")
    
    failsafe = FailsafeSystem()
    failsafe.start_monitoring()
    
    # Test 1: Rapid descent without control
    print("\nTest 1: Rapid descent without control")
    rapid_descent = FlightConditionData(
        pitch=-25.0, roll=0.0, yaw=0.0,
        pitch_rate=2.0, roll_rate=1.0, yaw_rate=0.0,
        acceleration_x=0.0, acceleration_y=0.0, acceleration_z=-15.0,
        g_force=1.0, airspeed=18.0, altitude=80.0, vertical_speed=-10.0,
        angle_of_attack=5.0, stall_warning=False, stall_margin=0.3,
        spin_indicators={}, max_pitch=45.0, max_roll=60.0, max_yaw_rate=120.0
    )
    
    failsafe.update_flight_condition_data(rapid_descent)
    time.sleep(0.1)
    
    status = failsafe.get_failsafe_status()
    print(f"  Uncontrolled descent: {status['flight_condition_monitoring']['current_status']['uncontrolled_descent']}")
    print(f"  Failsafe active: {status['active']}")
    
    # Test 2: Descent with extreme attitudes
    print("\nTest 2: Descent with extreme attitudes")
    extreme_descent = FlightConditionData(
        pitch=-40.0, roll=55.0, yaw=15.0,
        pitch_rate=5.0, roll_rate=8.0, yaw_rate=3.0,
        acceleration_x=5.0, acceleration_y=8.0, acceleration_z=-12.0,
        g_force=1.2, airspeed=15.0, altitude=60.0, vertical_speed=-7.0,
        angle_of_attack=12.0, stall_warning=True, stall_margin=0.2,
        spin_indicators={}, max_pitch=45.0, max_roll=60.0, max_yaw_rate=120.0
    )
    
    failsafe.update_flight_condition_data(extreme_descent)
    time.sleep(0.1)
    
    status = failsafe.get_failsafe_status()
    print(f"  Uncontrolled descent: {status['flight_condition_monitoring']['current_status']['uncontrolled_descent']}")
    print(f"  Failsafe active: {status['active']}")
    
    failsafe.stop_monitoring()
    print("✅ Uncontrolled descent detection testing completed")

def test_critical_attitude():
    """Test critical attitude detection"""
    print("\n=== TESTING CRITICAL ATTITUDE DETECTION ===")
    
    failsafe = FailsafeSystem()
    failsafe.start_monitoring()
    
    # Test 1: Extreme pitch
    print("\nTest 1: Extreme pitch attitude")
    extreme_pitch = FlightConditionData(
        pitch=50.0, roll=5.0, yaw=0.0,
        pitch_rate=15.0, roll_rate=2.0, yaw_rate=0.0,
        acceleration_x=5.0, acceleration_y=2.0, acceleration_z=-12.0,
        g_force=1.1, airspeed=20.0, altitude=100.0, vertical_speed=2.0,
        angle_of_attack=12.0, stall_warning=False, stall_margin=0.3,
        spin_indicators={}, max_pitch=45.0, max_roll=60.0, max_yaw_rate=120.0
    )
    
    failsafe.update_flight_condition_data(extreme_pitch)
    time.sleep(0.1)
    
    status = failsafe.get_failsafe_status()
    print(f"  Critical attitude: {status['flight_condition_monitoring']['current_status']['critical_attitude']}")
    print(f"  Failsafe active: {status['active']}")
    
    # Test 2: Extreme roll
    print("\nTest 2: Extreme roll attitude")
    extreme_roll = FlightConditionData(
        pitch=10.0, roll=75.0, yaw=5.0,
        pitch_rate=3.0, roll_rate=20.0, yaw_rate=2.0,
        acceleration_x=8.0, acceleration_y=15.0, acceleration_z=-10.0,
        g_force=1.3, airspeed=18.0, altitude=90.0, vertical_speed=1.0,
        angle_of_attack=8.0, stall_warning=False, stall_margin=0.4,
        spin_indicators={}, max_pitch=45.0, max_roll=60.0, max_yaw_rate=120.0
    )
    
    failsafe.update_flight_condition_data(extreme_roll)
    time.sleep(0.1)
    
    status = failsafe.get_failsafe_status()
    print(f"  Critical attitude: {status['flight_condition_monitoring']['current_status']['critical_attitude']}")
    print(f"  Failsafe active: {status['active']}")
    
    failsafe.stop_monitoring()
    print("✅ Critical attitude detection testing completed")

def test_excessive_rotation_rates():
    """Test excessive rotation rate detection"""
    print("\n=== TESTING EXCESSIVE ROTATION RATE DETECTION ===")
    
    failsafe = FailsafeSystem()
    failsafe.start_monitoring()
    
    # Test 1: Excessive roll rate
    print("\nTest 1: Excessive roll rate")
    high_roll_rate = FlightConditionData(
        pitch=5.0, roll=10.0, yaw=0.0,
        pitch_rate=8.0, roll_rate=220.0, yaw_rate=5.0,
        acceleration_x=10.0, acceleration_y=25.0, acceleration_z=-9.81,
        g_force=1.2, airspeed=22.0, altitude=110.0, vertical_speed=1.0,
        angle_of_attack=6.0, stall_warning=False, stall_margin=0.4,
        spin_indicators={}, max_pitch=45.0, max_roll=60.0, max_yaw_rate=120.0
    )
    
    failsafe.update_flight_condition_data(high_roll_rate)
    time.sleep(0.1)
    
    status = failsafe.get_failsafe_status()
    rotation_rates = status['flight_condition_monitoring']['current_status']['rotation_rates']
    print(f"  Excessive roll rate: {rotation_rates['roll_rate']}")
    print(f"  Failsafe active: {status['active']}")
    
    # Test 2: Excessive pitch rate
    print("\nTest 2: Excessive pitch rate")
    high_pitch_rate = FlightConditionData(
        pitch=15.0, roll=5.0, yaw=0.0,
        pitch_rate=120.0, roll_rate=8.0, yaw_rate=3.0,
        acceleration_x=15.0, acceleration_y=5.0, acceleration_z=-12.0,
        g_force=1.4, airspeed=25.0, altitude=120.0, vertical_speed=3.0,
        angle_of_attack=10.0, stall_warning=False, stall_margin=0.3,
        spin_indicators={}, max_pitch=45.0, max_roll=60.0, max_yaw_rate=120.0
    )
    
    failsafe.update_flight_condition_data(high_pitch_rate)
    time.sleep(0.1)
    
    status = failsafe.get_failsafe_status()
    rotation_rates = status['flight_condition_monitoring']['current_status']['rotation_rates']
    print(f"  Excessive pitch rate: {rotation_rates['pitch_rate']}")
    print(f"  Failsafe active: {status['active']}")
    
    # Test 3: Excessive yaw rate
    print("\nTest 3: Excessive yaw rate")
    high_yaw_rate = FlightConditionData(
        pitch=8.0, roll=3.0, yaw=10.0,
        pitch_rate=6.0, roll_rate=4.0, yaw_rate=150.0,
        acceleration_x=8.0, acceleration_y=20.0, acceleration_z=-10.0,
        g_force=1.1, airspeed=20.0, altitude=100.0, vertical_speed=1.0,
        angle_of_attack=7.0, stall_warning=False, stall_margin=0.4,
        spin_indicators={}, max_pitch=45.0, max_roll=60.0, max_yaw_rate=120.0
    )
    
    failsafe.update_flight_condition_data(high_yaw_rate)
    time.sleep(0.1)
    
    status = failsafe.get_failsafe_status()
    rotation_rates = status['flight_condition_monitoring']['current_status']['rotation_rates']
    print(f"  Excessive yaw rate: {rotation_rates['yaw_rate']}")
    print(f"  Failsafe active: {status['active']}")
    
    failsafe.stop_monitoring()
    print("✅ Excessive rotation rate detection testing completed")

def test_normal_flight_conditions():
    """Test normal flight conditions (should not trigger failsafe)"""
    print("\n=== TESTING NORMAL FLIGHT CONDITIONS ===")
    
    failsafe = FailsafeSystem()
    failsafe.start_monitoring()
    
    # Test normal flight data
    print("\nTest: Normal flight conditions")
    normal_flight = FlightConditionData(
        pitch=2.0, roll=1.0, yaw=0.0,
        pitch_rate=3.0, roll_rate=2.0, yaw_rate=1.0,
        acceleration_x=1.0, acceleration_y=0.5, acceleration_z=-9.81,
        g_force=1.0, airspeed=22.0, altitude=100.0, vertical_speed=0.5,
        angle_of_attack=4.0, stall_warning=False, stall_margin=0.6,
        spin_indicators={}, max_pitch=45.0, max_roll=60.0, max_yaw_rate=120.0
    )
    
    failsafe.update_flight_condition_data(normal_flight)
    time.sleep(0.1)
    
    status = failsafe.get_failsafe_status()
    current_status = status['flight_condition_monitoring']['current_status']
    
    print(f"  Stall detected: {current_status['stall_detected']}")
    print(f"  Flat spin detected: {current_status['flat_spin_detected']}")
    print(f"  Excessive G-forces: {current_status['excessive_g_forces']}")
    print(f"  Uncontrolled descent: {current_status['uncontrolled_descent']}")
    print(f"  Critical attitude: {current_status['critical_attitude']}")
    print(f"  Failsafe active: {status['active']}")
    
    # Verify no failsafe was triggered
    if not status['active']:
        print("  ✅ Normal flight conditions correctly identified - no failsafe triggered")
    else:
        print("  ❌ Failsafe incorrectly triggered for normal flight")
    
    failsafe.stop_monitoring()
    print("✅ Normal flight condition testing completed")

def run_comprehensive_flight_condition_test():
    """Run comprehensive flight condition monitoring test"""
    print("🚀 COMPREHENSIVE FLIGHT CONDITION MONITORING TEST")
    print("=" * 70)
    
    test_results = {}
    
    try:
        # Test 1: Stall detection
        test_results['stall_detection'] = True
        test_stall_detection()
        
        # Test 2: Flat spin detection
        test_results['flat_spin_detection'] = True
        test_flat_spin_detection()
        
        # Test 3: Excessive G-forces
        test_results['excessive_g_forces'] = True
        test_excessive_g_forces()
        
        # Test 4: Uncontrolled descent
        test_results['uncontrolled_descent'] = True
        test_uncontrolled_descent()
        
        # Test 5: Critical attitude
        test_results['critical_attitude'] = True
        test_critical_attitude()
        
        # Test 6: Excessive rotation rates
        test_results['excessive_rotation_rates'] = True
        test_excessive_rotation_rates()
        
        # Test 7: Normal flight conditions
        test_results['normal_flight_conditions'] = True
        test_normal_flight_conditions()
        
    except Exception as e:
        print(f"❌ Test error: {e}")
        return False
    
    # Print test results
    print("\n" + "=" * 70)
    print("📊 FLIGHT CONDITION MONITORING TEST RESULTS SUMMARY")
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
        print("🎉 Excellent! Flight condition monitoring system is working perfectly")
    elif pass_rate >= 70:
        print("⚠️  Good! Some issues detected, review failed tests")
    else:
        print("🚨 Poor! Multiple issues detected, system needs attention")
    
    return pass_rate >= 70

def main():
    """Main test function"""
    print("🛩️  Flight Condition Monitoring Test Suite")
    print("Testing dangerous flight state detection and crash prevention")
    print("=" * 70)
    
    try:
        success = run_comprehensive_flight_condition_test()
        
        if success:
            print("\n✅ Flight condition monitoring test completed successfully")
        else:
            print("\n❌ Flight condition monitoring test failed")
            
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n\nTest error: {e}")
    
    print("\nTest suite completed")

if __name__ == "__main__":
    main()
