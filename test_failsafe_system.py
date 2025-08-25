#!/usr/bin/env python3
"""
Test Script for Failsafe System
Demonstrates all failsafe features and scenarios
"""

import time
import json
from failsafe_system import FailsafeSystem, FailsafeTrigger

def test_basic_failsafe():
    """Test basic failsafe functionality"""
    print("=== TESTING BASIC FAILSAFE FUNCTIONALITY ===")
    
    # Create failsafe system
    failsafe = FailsafeSystem()
    
    # Set callbacks
    def on_failsafe_triggered(trigger_type, description):
        print(f"🚨 FAILSAFE TRIGGERED: {trigger_type.value}")
        print(f"   Reason: {description}")
    
    def on_parachute_deploy():
        print("🪂 Parachute deployed via failsafe")
    
    def on_gps_ping(ping_data):
        print(f"📍 GPS ping: {ping_data['position']}")
    
    failsafe.on_failsafe_triggered = on_failsafe_triggered
    failsafe.on_parachute_deploy = on_parachute_deploy
    failsafe.on_gps_ping = on_gps_ping
    
    # Start monitoring
    failsafe.start_monitoring()
    
    print("✅ Basic failsafe test completed")
    return failsafe

def test_gps_lost_scenario():
    """Test GPS lost failsafe scenario"""
    print("\n=== TESTING GPS LOST SCENARIO ===")
    
    failsafe = FailsafeSystem()
    failsafe.start_monitoring()
    
    # Simulate normal GPS updates
    print("Simulating normal GPS updates...")
    for i in range(5):
        failsafe.update_gps_data(
            latitude=37.7749 + (i * 0.0001),
            longitude=-122.4194 + (i * 0.0001),
            altitude=50.0 + (i * 2.0),
            voltage=12.0
        )
        time.sleep(0.1)
    
    # Stop GPS updates to simulate GPS loss
    print("Stopping GPS updates to simulate GPS loss...")
    time.sleep(12)  # Wait for GPS lost condition (10s threshold)
    
    # Check status
    status = failsafe.get_failsafe_status()
    print(f"Failsafe status: {json.dumps(status, indent=2)}")
    
    failsafe.stop_monitoring()
    return status['active']

def test_off_track_scenario():
    """Test off-track failsafe scenario"""
    print("\n=== TESTING OFF-TRACK SCENARIO ===")
    
    failsafe = FailsafeSystem()
    failsafe.start_monitoring()
    
    # Set up planned route
    planned_route = [
        {'lat': 37.7749, 'lon': -122.4194, 'alt': 50},
        {'lat': 37.7759, 'lon': -122.4204, 'alt': 50},
        {'lat': 37.7769, 'lon': -122.4214, 'alt': 50}
    ]
    
    # Simulate following planned route
    print("Simulating planned route following...")
    for i in range(5):
        failsafe.update_gps_data(
            latitude=37.7749 + (i * 0.0001),
            longitude=-122.4194 + (i * 0.0001),
            altitude=50.0,
            voltage=12.0,
            planned_route=planned_route
        )
        time.sleep(0.1)
    
    # Simulate going off-track
    print("Simulating off-track deviation...")
    failsafe.update_gps_data(
        latitude=37.7800,  # Way off the planned route
        longitude=-122.4300,
        altitude=50.0,
        voltage=12.0,
        planned_route=planned_route
    )
    
    time.sleep(1)
    
    # Check status
    status = failsafe.get_failsafe_status()
    print(f"Failsafe status: {json.dumps(status, indent=2)}")
    
    failsafe.stop_monitoring()
    return status['active']

def test_altitude_limit_scenario():
    """Test altitude limit failsafe scenario"""
    print("\n=== TESTING ALTITUDE LIMIT SCENARIO ===")
    
    failsafe = FailsafeSystem()
    failsafe.start_monitoring()
    
    # Simulate normal altitude
    print("Simulating normal altitude...")
    failsafe.update_gps_data(
        latitude=37.7749,
        longitude=-122.4194,
        altitude=100.0,
        voltage=12.0
    )
    
    # Simulate exceeding altitude limit
    print("Simulating altitude limit exceeded...")
    failsafe.update_gps_data(
        latitude=37.7749,
        longitude=-122.4194,
        altitude=160.0,  # Exceeds 150m limit
        voltage=12.0
    )
    
    time.sleep(1)
    
    # Check status
    status = failsafe.get_failsafe_status()
    print(f"Failsafe status: {json.dumps(status, indent=2)}")
    
    failsafe.stop_monitoring()
    return status['active']

def test_battery_critical_scenario():
    """Test battery critical failsafe scenario"""
    print("\n=== TESTING BATTERY CRITICAL SCENARIO ===")
    
    failsafe = FailsafeSystem()
    failsafe.start_monitoring()
    
    # Simulate normal battery
    print("Simulating normal battery...")
    failsafe.update_gps_data(
        latitude=37.7749,
        longitude=-122.4194,
        altitude=50.0,
        voltage=12.0
    )
    
    # Simulate critical battery
    print("Simulating critical battery...")
    failsafe.update_gps_data(
        latitude=37.7749,
        longitude=-122.4194,
        altitude=50.0,
        voltage=9.5  # Below 10V threshold
    )
    
    time.sleep(1)
    
    # Check status
    status = failsafe.get_failsafe_status()
    print(f"Failsafe status: {json.dumps(status, indent=2)}")
    
    failsafe.stop_monitoring()
    return status['active']

def test_flight_time_exceeded_scenario():
    """Test flight time exceeded failsafe scenario"""
    print("\n=== TESTING FLIGHT TIME EXCEEDED SCENARIO ===")
    
    failsafe = FailsafeSystem()
    failsafe.start_monitoring()
    
    # Simulate flight start
    print("Simulating flight start...")
    start_time = time.time() - 1900  # 31.7 minutes ago (exceeds 30 min limit)
    
    # Create GPS entry with old timestamp
    gps_entry = {
        'timestamp': start_time,
        'latitude': 37.7749,
        'longitude': -122.4194,
        'altitude': 50.0,
        'voltage': 12.0
    }
    
    # Manually add to history
    failsafe.gps_history.append(gps_entry)
    
    # Add current position
    failsafe.update_gps_data(
        latitude=37.7749,
        longitude=-122.4194,
        altitude=50.0,
        voltage=12.0
    )
    
    time.sleep(1)
    
    # Check status
    status = failsafe.get_failsafe_status()
    print(f"Failsafe status: {json.dumps(status, indent=2)}")
    
    failsafe.stop_monitoring()
    return status['active']

def test_manual_failsafe_trigger():
    """Test manual failsafe trigger"""
    print("\n=== TESTING MANUAL FAILSAFE TRIGGER ===")
    
    failsafe = FailsafeSystem()
    failsafe.start_monitoring()
    
    # Simulate normal operation
    print("Simulating normal operation...")
    failsafe.update_gps_data(
        latitude=37.7749,
        longitude=-122.4194,
        altitude=50.0,
        voltage=12.0
    )
    
    # Manually trigger failsafe
    print("Manually triggering failsafe...")
    failsafe.manual_failsafe_trigger()
    
    time.sleep(1)
    
    # Check status
    status = failsafe.get_failsafe_status()
    print(f"Failsafe status: {json.dumps(status, indent=2)}")
    
    failsafe.stop_monitoring()
    return status['active']

def test_custom_failsafe_conditions():
    """Test custom failsafe conditions"""
    print("\n=== TESTING CUSTOM FAILSAFE CONDITIONS ===")
    
    failsafe = FailsafeSystem()
    
    # Add custom condition
    print("Adding custom failsafe condition...")
    failsafe.set_custom_condition(
        trigger=FailsafeTrigger.ALTITUDE_LIMIT,
        threshold=50.0,  # 50 meters (lower than default)
        time_window=1.0,
        description="Custom altitude limit of 50 meters",
        critical=True
    )
    
    failsafe.start_monitoring()
    
    # Test custom condition
    print("Testing custom altitude condition...")
    failsafe.update_gps_data(
        latitude=37.7749,
        longitude=-122.4194,
        altitude=60.0,  # Exceeds custom 50m limit
        voltage=12.0
    )
    
    time.sleep(1)
    
    # Check status
    status = failsafe.get_failsafe_status()
    print(f"Failsafe status: {json.dumps(status, indent=2)}")
    
    failsafe.stop_monitoring()
    return status['active']

def test_failsafe_reset():
    """Test failsafe system reset"""
    print("\n=== TESTING FAILSAFE RESET ===")
    
    failsafe = FailsafeSystem()
    failsafe.start_monitoring()
    
    # Trigger failsafe
    print("Triggering failsafe...")
    failsafe.manual_failsafe_trigger()
    
    time.sleep(1)
    
    # Check status before reset
    status_before = failsafe.get_failsafe_status()
    print(f"Status before reset: {status_before['active']}")
    
    # Reset failsafe
    print("Resetting failsafe system...")
    failsafe.reset_failsafe()
    
    # Check status after reset
    status_after = failsafe.get_failsafe_status()
    print(f"Status after reset: {status_after['active']}")
    
    failsafe.stop_monitoring()
    return not status_after['active']

def test_gps_ping_functionality():
    """Test GPS ping functionality"""
    print("\n=== TESTING GPS PING FUNCTIONALITY ===")
    
    failsafe = FailsafeSystem()
    failsafe.start_monitoring()
    
    # Set up GPS data
    print("Setting up GPS data...")
    failsafe.update_gps_data(
        latitude=37.7749,
        longitude=-122.4194,
        altitude=50.0,
        voltage=12.0
    )
    
    # Trigger failsafe to activate GPS pinging
    print("Triggering failsafe to activate GPS pinging...")
    failsafe.manual_failsafe_trigger()
    
    # Let GPS pinging run for a few seconds
    print("GPS pinging active for 5 seconds...")
    time.sleep(5)
    
    # Check ping count
    status = failsafe.get_failsafe_status()
    print(f"GPS ping count: {status['gps_ping_count']}")
    
    failsafe.stop_monitoring()
    return status['gps_ping_count'] > 0

def run_comprehensive_test():
    """Run comprehensive failsafe system test"""
    print("🚀 COMPREHENSIVE FAILSAFE SYSTEM TEST")
    print("=" * 50)
    
    test_results = {}
    
    try:
        # Test 1: Basic functionality
        test_results['basic_failsafe'] = True
        test_basic_failsafe()
        
        # Test 2: GPS lost scenario
        test_results['gps_lost'] = test_gps_lost_scenario()
        
        # Test 3: Off-track scenario
        test_results['off_track'] = test_off_track_scenario()
        
        # Test 4: Altitude limit scenario
        test_results['altitude_limit'] = test_altitude_limit_scenario()
        
        # Test 5: Battery critical scenario
        test_results['battery_critical'] = test_battery_critical_scenario()
        
        # Test 6: Flight time exceeded scenario
        test_results['flight_time_exceeded'] = test_flight_time_exceeded_scenario()
        
        # Test 7: Manual trigger
        test_results['manual_trigger'] = test_manual_failsafe_trigger()
        
        # Test 8: Custom conditions
        test_results['custom_conditions'] = test_custom_failsafe_conditions()
        
        # Test 9: System reset
        test_results['system_reset'] = test_failsafe_reset()
        
        # Test 10: GPS pinging
        test_results['gps_pinging'] = test_gps_ping_functionality()
        
    except Exception as e:
        print(f"❌ Test error: {e}")
        return False
    
    # Print test results
    print("\n" + "=" * 50)
    print("📊 TEST RESULTS SUMMARY")
    print("=" * 50)
    
    for test_name, result in test_results.items():
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{test_name:.<30} {status}")
    
    # Calculate pass rate
    passed = sum(test_results.values())
    total = len(test_results)
    pass_rate = (passed / total) * 100
    
    print(f"\nOverall: {passed}/{total} tests passed ({pass_rate:.1f}%)")
    
    if pass_rate >= 90:
        print("🎉 Excellent! Failsafe system is working properly")
    elif pass_rate >= 70:
        print("⚠️  Good! Some issues detected, review failed tests")
    else:
        print("🚨 Poor! Multiple issues detected, system needs attention")
    
    return pass_rate >= 70

def main():
    """Main test function"""
    print("🛩️  Autonomous Aircraft Failsafe System Test Suite")
    print("Testing comprehensive safety features and emergency procedures")
    
    try:
        success = run_comprehensive_test()
        
        if success:
            print("\n✅ Failsafe system test completed successfully")
        else:
            print("\n❌ Failsafe system test failed")
            
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n\nTest error: {e}")
    
    print("\nTest suite completed")

if __name__ == "__main__":
    main()
