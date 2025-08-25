#!/usr/bin/env python3
"""
Failsafe System for Autonomous Aircraft
Monitors aircraft status and automatically activates safety measures
"""

import time
import math
import threading
from typing import Optional, Dict, List
from dataclasses import dataclass
from enum import Enum
import json

class FailsafeTrigger(Enum):
    GPS_LOST = "gps_lost"
    OFF_TRACK = "off_track"
    ALTITUDE_LIMIT = "altitude_limit"
    BATTERY_CRITICAL = "battery_critical"
    FLIGHT_TIME_EXCEEDED = "flight_time_exceeded"
    CONTROL_LOSS = "control_loss"
    SIGNAL_LOSS = "signal_loss"
    EMERGENCY_MANUAL = "emergency_manual"
    # New flight condition triggers
    STALL_DETECTED = "stall_detected"
    FLAT_SPIN_DETECTED = "flat_spin_detected"
    EXCESSIVE_G_FORCES = "excessive_g_forces"
    UNCONTROLLED_DESCENT = "uncontrolled_descent"
    CRITICAL_ATTITUDE = "critical_attitude"
    EXCESSIVE_ROLL_RATE = "excessive_roll_rate"
    EXCESSIVE_PITCH_RATE = "excessive_pitch_rate"
    EXCESSIVE_YAW_RATE = "excessive_yaw_rate"

@dataclass
class FailsafeCondition:
    trigger: FailsafeTrigger
    threshold: float
    time_window: float  # seconds
    description: str
    critical: bool = True

@dataclass
class FailsafeStatus:
    active: bool = False
    triggered_at: Optional[float] = None
    trigger_type: Optional[FailsafeTrigger] = None
    last_gps_ping: Optional[float] = None
    gps_ping_count: int = 0
    parachute_deployed: bool = False
    emergency_beacon_active: bool = False

@dataclass
class FlightConditionData:
    # IMU data
    pitch: float  # degrees
    roll: float   # degrees
    yaw: float    # degrees
    pitch_rate: float  # degrees/second
    roll_rate: float   # degrees/second
    yaw_rate: float    # degrees/second
    
    # Accelerometer data
    acceleration_x: float  # m/s²
    acceleration_y: float  # m/s²
    acceleration_z: float  # m/s²
    g_force: float        # G's (1G = 9.81 m/s²)
    
    # Flight dynamics
    airspeed: float       # m/s
    altitude: float       # meters
    vertical_speed: float # m/s (positive = climbing, negative = descending)
    angle_of_attack: float # degrees
    
    # Stall detection
    stall_warning: bool   # True if stall conditions detected
    stall_margin: float   # Percentage of stall margin remaining
    
    # Spin detection
    spin_indicators: Dict[str, bool]  # Various spin indicators
    
    # Attitude limits
    max_pitch: float      # Maximum safe pitch angle
    max_roll: float       # Maximum safe roll angle
    max_yaw_rate: float   # Maximum safe yaw rate
    
    def is_stall_condition(self) -> bool:
        """Detect stall conditions"""
        # Stall detection based on multiple factors
        stall_indicators = []
        
        # High angle of attack (typically >15-20 degrees)
        if self.angle_of_attack > 18.0:
            stall_indicators.append(True)
        
        # Low airspeed relative to stall speed
        if self.airspeed < 12.0:  # Assuming 12 m/s stall speed
            stall_indicators.append(True)
        
        # High pitch with low airspeed
        if self.pitch > 20.0 and self.airspeed < 15.0:
            stall_indicators.append(True)
        
        # Stall warning from sensors
        if self.stall_warning:
            stall_indicators.append(True)
        
        # Low stall margin
        if self.stall_margin < 0.1:  # Less than 10% margin
            stall_indicators.append(True)
        
        return len(stall_indicators) >= 2  # At least 2 indicators
    
    def is_flat_spin_condition(self) -> bool:
        """Detect flat spin conditions"""
        # Flat spin detection based on multiple factors
        spin_indicators = []
        
        # High yaw rate with low airspeed
        if abs(self.yaw_rate) > 60.0 and self.airspeed < 10.0:
            spin_indicators.append(True)
        
        # High roll rate with low airspeed
        if abs(self.roll_rate) > 120.0 and self.airspeed < 10.0:
            spin_indicators.append(True)
        
        # Extreme attitude angles
        if abs(self.roll) > 60.0 or abs(self.pitch) > 45.0:
            spin_indicators.append(True)
        
        # Rapid descent with rotation
        if self.vertical_speed < -5.0 and (abs(self.yaw_rate) > 30.0 or abs(self.roll_rate) > 60.0):
            spin_indicators.append(True)
        
        # Spin indicators from sensors
        for indicator, value in self.spin_indicators.items():
            if value:
                spin_indicators.append(True)
        
        return len(spin_indicators) >= 3  # At least 3 indicators
    
    def is_excessive_g_forces(self) -> bool:
        """Detect excessive G-forces"""
        # Check for excessive acceleration
        if abs(self.g_force) > 3.0:  # More than 3G
            return True
        
        # Check individual axis accelerations
        if abs(self.acceleration_x) > 30.0 or abs(self.acceleration_y) > 30.0 or abs(self.acceleration_z) > 30.0:
            return True
        
        return False
    
    def is_uncontrolled_descent(self) -> bool:
        """Detect uncontrolled descent"""
        # Check for rapid descent without control
        if self.vertical_speed < -8.0:  # Descending faster than 8 m/s
            # Check if we have control authority
            if abs(self.pitch_rate) < 5.0 and abs(self.roll_rate) < 10.0:
                # Low control rates during rapid descent = uncontrolled
                return True
        
        # Check for descent with extreme attitudes
        if self.vertical_speed < -5.0 and (abs(self.pitch) > 30.0 or abs(self.roll) > 45.0):
            return True
        
        return False
    
    def is_critical_attitude(self) -> bool:
        """Detect critical attitude conditions"""
        # Check for extreme pitch angles
        if abs(self.pitch) > 45.0:
            return True
        
        # Check for extreme roll angles
        if abs(self.roll) > 60.0:
            return True
        
        # Check for extreme yaw angles
        if abs(self.yaw) > 90.0:
            return True
        
        return False
    
    def is_excessive_rotation_rate(self) -> Dict[str, bool]:
        """Detect excessive rotation rates"""
        return {
            'roll_rate': abs(self.roll_rate) > 180.0,    # 180°/s
            'pitch_rate': abs(self.pitch_rate) > 90.0,   # 90°/s
            'yaw_rate': abs(self.yaw_rate) > 120.0       # 120°/s
        }

class FailsafeSystem:
    def __init__(self):
        self.status = FailsafeStatus()
        self.conditions: List[FailsafeCondition] = []
        self.gps_history: List[Dict] = []
        self.max_gps_history = 100
        
        # Flight condition monitoring
        self.flight_condition_history: List[FlightConditionData] = []
        self.max_flight_condition_history = 50
        self.current_flight_condition: Optional[FlightConditionData] = None
        
        # Failsafe thresholds
        self.setup_default_conditions()
        
        # GPS tracking parameters
        self.gps_ping_interval = 1.0  # seconds
        self.gps_ping_duration = 300  # seconds (5 minutes)
        self.last_gps_position = None
        self.gps_timeout = 10.0  # seconds
        
        # Off-track detection
        self.max_deviation_distance = 100.0  # meters
        self.max_deviation_angle = 30.0  # degrees
        self.track_tolerance = 20.0  # meters
        
        # Threading
        self.running = False
        self.monitor_thread = None
        self.gps_ping_thread = None
        
        # Callbacks
        self.on_failsafe_triggered = None
        self.on_parachute_deploy = None
        self.on_gps_ping = None
        
    def setup_default_conditions(self):
        """Setup default failsafe conditions"""
        self.conditions = [
            FailsafeCondition(
                trigger=FailsafeTrigger.GPS_LOST,
                threshold=10.0,  # 10 seconds without GPS
                time_window=5.0,
                description="GPS signal lost for more than 10 seconds",
                critical=True
            ),
            FailsafeCondition(
                trigger=FailsafeTrigger.OFF_TRACK,
                threshold=self.max_deviation_distance,
                time_window=5.0,
                description="Aircraft deviated more than 100m from planned route",
                critical=True
            ),
            FailsafeCondition(
                trigger=FailsafeTrigger.ALTITUDE_LIMIT,
                threshold=150.0,  # 150 meters
                time_window=2.0,
                description="Altitude exceeded 150 meters",
                critical=True
            ),
            FailsafeCondition(
                trigger=FailsafeTrigger.BATTERY_CRITICAL,
                threshold=10.0,  # 10 volts
                time_window=5.0,
                description="Battery voltage below 10 volts",
                critical=True
            ),
            FailsafeCondition(
                trigger=FailsafeTrigger.FLIGHT_TIME_EXCEEDED,
                threshold=1800.0,  # 30 minutes
                time_window=60.0,
                description="Flight time exceeded 30 minutes",
                critical=True
            ),
            FailsafeCondition(
                trigger=FailsafeTrigger.CONTROL_LOSS,
                threshold=5.0,  # 5 seconds
                time_window=2.0,
                description="Control signal lost for more than 5 seconds",
                critical=True
            ),
            FailsafeCondition(
                trigger=FailsafeTrigger.SIGNAL_LOSS,
                threshold=15.0,  # 15 seconds
                time_window=5.0,
                description="Communication signal lost for more than 15 seconds",
                critical=True
            ),
            # New flight condition failsafes
            FailsafeCondition(
                trigger=FailsafeTrigger.STALL_DETECTED,
                threshold=0.5,  # 0.5 seconds in stall condition
                time_window=0.2,
                description="Aircraft stall detected - immediate recovery required",
                critical=True
            ),
            FailsafeCondition(
                trigger=FailsafeTrigger.FLAT_SPIN_DETECTED,
                threshold=0.3,  # 0.3 seconds in flat spin
                time_window=0.1,
                description="Flat spin detected - critical flight condition",
                critical=True
            ),
            FailsafeCondition(
                trigger=FailsafeTrigger.EXCESSIVE_G_FORCES,
                threshold=3.0,  # 3G acceleration
                time_window=0.1,
                description="Excessive G-forces detected - structural stress",
                critical=True
            ),
            FailsafeCondition(
                trigger=FailsafeTrigger.UNCONTROLLED_DESCENT,
                threshold=2.0,  # 2 seconds of uncontrolled descent
                time_window=0.5,
                description="Uncontrolled descent detected - loss of control",
                critical=True
            ),
            FailsafeCondition(
                trigger=FailsafeTrigger.CRITICAL_ATTITUDE,
                threshold=0.5,  # 0.5 seconds in critical attitude
                time_window=0.2,
                description="Critical attitude detected - extreme pitch/roll",
                critical=True
            ),
            FailsafeCondition(
                trigger=FailsafeTrigger.EXCESSIVE_ROLL_RATE,
                threshold=180.0,  # 180 degrees/second roll rate
                time_window=0.1,
                description="Excessive roll rate detected - loss of control",
                critical=True
            ),
            FailsafeCondition(
                trigger=FailsafeTrigger.EXCESSIVE_PITCH_RATE,
                threshold=90.0,  # 90 degrees/second pitch rate
                time_window=0.1,
                description="Excessive pitch rate detected - loss of control",
                critical=True
            ),
            FailsafeCondition(
                trigger=FailsafeTrigger.EXCESSIVE_YAW_RATE,
                threshold=120.0,  # 120 degrees/second yaw rate
                time_window=0.1,
                description="Excessive yaw rate detected - loss of control",
                critical=True
            )
        ]
    
    def start_monitoring(self):
        """Start the failsafe monitoring system"""
        if self.running:
            return
        
        self.running = True
        self.status.active = True
        
        # Start monitoring thread
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        
        # Start GPS ping thread
        self.gps_ping_thread = threading.Thread(target=self._gps_ping_loop, daemon=True)
        self.gps_ping_thread.start()
        
        print("✅ Failsafe system monitoring started")
    
    def stop_monitoring(self):
        """Stop the failsafe monitoring system"""
        self.running = False
        self.status.active = False
        
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
        if self.gps_ping_thread:
            self.gps_ping_thread.join(timeout=2.0)
        
        print("✅ Failsafe system monitoring stopped")
    
    def _monitor_loop(self):
        """Main monitoring loop"""
        while self.running:
            try:
                # Check all failsafe conditions
                self._check_failsafe_conditions()
                
                # Sleep to maintain monitoring rate
                time.sleep(0.1)  # 10 Hz monitoring
                
            except Exception as e:
                print(f"Failsafe monitoring error: {e}")
                time.sleep(1.0)
    
    def _check_failsafe_conditions(self):
        """Check all failsafe conditions"""
        current_time = time.time()
        
        for condition in self.conditions:
            if self._evaluate_condition(condition, current_time):
                self._trigger_failsafe(condition.trigger, condition.description)
                break
    
    def _evaluate_condition(self, condition: FailsafeCondition, current_time: float) -> bool:
        """Evaluate if a specific failsafe condition is met"""
        if condition.trigger == FailsafeTrigger.GPS_LOST:
            return self._check_gps_lost(condition)
        elif condition.trigger == FailsafeTrigger.OFF_TRACK:
            return self._check_off_track(condition)
        elif condition.trigger == FailsafeTrigger.ALTITUDE_LIMIT:
            return self._check_altitude_limit(condition)
        elif condition.trigger == FailsafeTrigger.BATTERY_CRITICAL:
            return self._check_battery_critical(condition)
        elif condition.trigger == FailsafeTrigger.FLIGHT_TIME_EXCEEDED:
            return self._check_flight_time_exceeded(condition)
        elif condition.trigger == FailsafeTrigger.CONTROL_LOSS:
            return self._check_control_loss(condition)
        elif condition.trigger == FailsafeTrigger.SIGNAL_LOSS:
            return self._check_signal_loss(condition)
        
        return False
    
    def _check_gps_lost(self, condition: FailsafeCondition) -> bool:
        """Check if GPS signal is lost"""
        if not self.gps_history:
            return False
        
        # Check if we have recent GPS data
        current_time = time.time()
        recent_gps = [entry for entry in self.gps_history 
                     if current_time - entry['timestamp'] < condition.threshold]
        
        return len(recent_gps) == 0
    
    def _check_off_track(self, condition: FailsafeCondition) -> bool:
        """Check if aircraft is off track"""
        if len(self.gps_history) < 2:
            return False
        
        # Get current position and planned route
        current_pos = self.gps_history[-1]
        if not current_pos.get('planned_route'):
            return False
        
        # Calculate deviation from planned route
        deviation = self._calculate_route_deviation(current_pos)
        return deviation > condition.threshold
    
    def _check_altitude_limit(self, condition: FailsafeCondition) -> bool:
        """Check if altitude limit is exceeded"""
        if not self.gps_history:
            return False
        
        current_alt = self.gps_history[-1].get('altitude', 0)
        return current_alt > condition.threshold
    
    def _check_battery_critical(self, condition: FailsafeCondition) -> bool:
        """Check if battery is critically low"""
        if not self.gps_history:
            return False
        
        current_voltage = self.gps_history[-1].get('voltage', 12.0)
        return current_voltage < condition.threshold
    
    def _check_flight_time_exceeded(self, condition: FailsafeCondition) -> bool:
        """Check if maximum flight time is exceeded"""
        if not self.gps_history:
            return False
        
        flight_start = self.gps_history[0].get('timestamp', time.time())
        current_time = time.time()
        flight_duration = current_time - flight_start
        
        return flight_duration > condition.threshold
    
    def _check_control_loss(self, condition: FailsafeCondition) -> bool:
        """Check if control signal is lost"""
        # This would check for loss of control communication
        # For now, simulate based on time since last control command
        return False  # Placeholder
    
    def _check_signal_loss(self, condition: FailsafeCondition) -> bool:
        """Check if communication signal is lost"""
        # This would check for loss of ground communication
        # For now, simulate based on time since last communication
        return False  # Placeholder
    
    def _calculate_route_deviation(self, current_pos: Dict) -> float:
        """Calculate deviation from planned route"""
        if not current_pos.get('planned_route'):
            return 0.0
        
        # Get nearest point on planned route
        planned_route = current_pos['planned_route']
        current_lat = current_pos['latitude']
        current_lon = current_pos['longitude']
        
        min_distance = float('inf')
        for route_point in planned_route:
            distance = self._calculate_distance(
                current_lat, current_lon,
                route_point['lat'], route_point['lon']
            )
            min_distance = min(min_distance, distance)
        
        return min_distance
    
    def _calculate_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance between two GPS coordinates in meters"""
        R = 6371000  # Earth's radius in meters
        
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def _trigger_failsafe(self, trigger_type: FailsafeTrigger, description: str):
        """Trigger the failsafe system"""
        if self.status.active:
            return  # Already triggered
        
        current_time = time.time()
        
        print(f"🚨 FAILSAFE TRIGGERED: {trigger_type.value}")
        print(f"   Reason: {description}")
        print(f"   Time: {time.strftime('%H:%M:%S', time.localtime(current_time))}")
        
        # Update status
        self.status.active = True
        self.status.triggered_at = current_time
        self.status.trigger_type = trigger_type
        
        # Execute failsafe procedures
        self._execute_failsafe_procedures()
        
        # Notify callback if set
        if self.on_failsafe_triggered:
            self.on_failsafe_triggered(trigger_type, description)
    
    def _execute_failsafe_procedures(self):
        """Execute all failsafe procedures"""
        print("🚨 Executing failsafe procedures...")
        
        # 1. Deploy parachute immediately
        self._deploy_parachute()
        
        # 2. Activate emergency beacon
        self._activate_emergency_beacon()
        
        # 3. Start continuous GPS pinging
        self._start_continuous_gps_ping()
        
        # 4. Neutralize all controls
        self._neutralize_controls()
        
        print("✅ Failsafe procedures executed")
    
    def _deploy_parachute(self):
        """Deploy the emergency parachute"""
        print("🪂 Deploying emergency parachute...")
        
        self.status.parachute_deployed = True
        
        # Notify callback if set
        if self.on_parachute_deploy:
            self.on_parachute_deploy()
        
        print("✅ Parachute deployed")
    
    def _activate_emergency_beacon(self):
        """Activate emergency tracking beacon"""
        print("📡 Activating emergency beacon...")
        
        self.status.emergency_beacon_active = True
        
        # This would activate a radio beacon or cellular tracking
        # For now, just mark as active
        
        print("✅ Emergency beacon activated")
    
    def _start_continuous_gps_ping(self):
        """Start continuous GPS position broadcasting"""
        print("📍 Starting continuous GPS pinging...")
        
        # The GPS ping thread will handle this automatically
        # when failsafe is triggered
    
    def _neutralize_controls(self):
        """Neutralize all aircraft controls"""
        print("🎛️ Neutralizing aircraft controls...")
        
        # This would send neutral commands to all control surfaces
        # For now, just log the action
        
        print("✅ Controls neutralized")
    
    def _gps_ping_loop(self):
        """GPS ping loop for tracking"""
        while self.running:
            try:
                current_time = time.time()
                
                # If failsafe is active, ping more frequently
                if self.status.active:
                    ping_interval = 0.5  # 2 Hz when failsafe active
                else:
                    ping_interval = self.gps_ping_interval
                
                # Check if it's time to ping
                if (not self.status.last_gps_ping or 
                    current_time - self.status.last_gps_ping >= ping_interval):
                    
                    self._send_gps_ping()
                    self.status.last_gps_ping = current_time
                    self.status.gps_ping_count += 1
                
                time.sleep(0.1)
                
            except Exception as e:
                print(f"GPS ping error: {e}")
                time.sleep(1.0)
    
    def _send_gps_ping(self):
        """Send GPS position ping"""
        if not self.gps_history:
            return
        
        current_pos = self.gps_history[-1]
        
        ping_data = {
            'timestamp': time.time(),
            'position': {
                'latitude': current_pos.get('latitude', 0),
                'longitude': current_pos.get('longitude', 0),
                'altitude': current_pos.get('altitude', 0)
            },
            'failsafe_active': self.status.active,
            'parachute_deployed': self.status.parachute_deployed,
            'emergency_beacon': self.status.emergency_beacon_active,
            'ping_count': self.status.gps_ping_count
        }
        
        # Send ping via multiple methods for redundancy
        self._send_radio_ping(ping_data)
        self._send_cellular_ping(ping_data)
        self._send_satellite_ping(ping_data)
        
        # Notify callback if set
        if self.on_gps_ping:
            self.on_gps_ping(ping_data)
    
    def _send_radio_ping(self, ping_data: Dict):
        """Send GPS ping via radio"""
        # This would send data via radio transmitter
        # For now, just log the action
        pass
    
    def _send_cellular_ping(self, ping_data: Dict):
        """Send GPS ping via cellular network"""
        # This would send data via cellular modem
        # For now, just log the action
        pass
    
    def _send_satellite_ping(self, ping_data: Dict):
        """Send GPS ping via satellite"""
        # This would send data via satellite modem
        # For now, just log the action
        pass
    
    def update_gps_data(self, latitude: float, longitude: float, altitude: float, 
                        voltage: float = 12.0, planned_route: List = None):
        """Update GPS data for monitoring"""
        current_time = time.time()
        
        gps_entry = {
            'timestamp': current_time,
            'latitude': latitude,
            'longitude': longitude,
            'altitude': altitude,
            'voltage': voltage,
            'planned_route': planned_route
        }
        
        self.gps_history.append(gps_entry)
        
        # Keep only recent history
        if len(self.gps_history) > self.max_gps_history:
            self.gps_history = self.gps_history[-self.max_gps_history:]
        
        # Update last known position
        self.last_gps_position = gps_entry
    
    def update_flight_condition_data(self, flight_data: FlightConditionData):
        """Update flight condition data for monitoring"""
        current_time = time.time()
        
        # Store current flight condition
        self.current_flight_condition = flight_data
        
        # Add to history
        self.flight_condition_history.append(flight_data)
        
        # Keep only recent history
        if len(self.flight_condition_history) > self.max_flight_condition_history:
            self.flight_condition_history = self.flight_condition_history[-self.max_flight_condition_history:]
        
        # Check for dangerous flight conditions
        self._check_flight_condition_failsafes(flight_data)
    
    def _check_flight_condition_failsafes(self, flight_data: FlightConditionData):
        """Check for dangerous flight conditions that require immediate failsafe activation"""
        
        # Check for stall conditions
        if flight_data.is_stall_condition():
            print("🚨 STALL DETECTED - Activating failsafe!")
            self._trigger_failsafe(FailsafeTrigger.STALL_DETECTED, 
                                 "Aircraft stall detected - immediate recovery required")
            return
        
        # Check for flat spin conditions
        if flight_data.is_flat_spin_condition():
            print("🚨 FLAT SPIN DETECTED - Activating failsafe!")
            self._trigger_failsafe(FailsafeTrigger.FLAT_SPIN_DETECTED, 
                                 "Flat spin detected - critical flight condition")
            return
        
        # Check for excessive G-forces
        if flight_data.is_excessive_g_forces():
            print("🚨 EXCESSIVE G-FORCES DETECTED - Activating failsafe!")
            self._trigger_failsafe(FailsafeTrigger.EXCESSIVE_G_FORCES, 
                                 "Excessive G-forces detected - structural stress")
            return
        
        # Check for uncontrolled descent
        if flight_data.is_uncontrolled_descent():
            print("🚨 UNCONTROLLED DESCENT DETECTED - Activating failsafe!")
            self._trigger_failsafe(FailsafeTrigger.UNCONTROLLED_DESCENT, 
                                 "Uncontrolled descent detected - loss of control")
            return
        
        # Check for critical attitude
        if flight_data.is_critical_attitude():
            print("🚨 CRITICAL ATTITUDE DETECTED - Activating failsafe!")
            self._trigger_failsafe(FailsafeTrigger.CRITICAL_ATTITUDE, 
                                 "Critical attitude detected - extreme pitch/roll")
            return
        
        # Check for excessive rotation rates
        rotation_rates = flight_data.is_excessive_rotation_rate()
        if rotation_rates['roll_rate']:
            print("🚨 EXCESSIVE ROLL RATE DETECTED - Activating failsafe!")
            self._trigger_failsafe(FailsafeTrigger.EXCESSIVE_ROLL_RATE, 
                                 "Excessive roll rate detected - loss of control")
            return
        
        if rotation_rates['pitch_rate']:
            print("🚨 EXCESSIVE PITCH RATE DETECTED - Activating failsafe!")
            self._trigger_failsafe(FailsafeTrigger.EXCESSIVE_PITCH_RATE, 
                                 "Excessive pitch rate detected - loss of control")
            return
        
        if rotation_rates['yaw_rate']:
            print("🚨 EXCESSIVE YAW RATE DETECTED - Activating failsafe!")
            self._trigger_failsafe(FailsafeTrigger.EXCESSIVE_YAW_RATE, 
                                 "Excessive yaw rate detected - loss of control")
            return
    
    def manual_failsafe_trigger(self):
        """Manually trigger failsafe (for testing or emergency)"""
        print("🚨 Manual failsafe trigger activated")
        self._trigger_failsafe(FailsafeTrigger.EMERGENCY_MANUAL, "Manual emergency trigger")
    
    def reset_failsafe(self):
        """Reset the failsafe system (for testing)"""
        print("🔄 Resetting failsafe system...")
        
        self.status.active = False
        self.status.triggered_at = None
        self.status.trigger_type = None
        self.status.parachute_deployed = False
        self.status.emergency_beacon_active = False
        
        print("✅ Failsafe system reset")
    
    def get_failsafe_status(self) -> Dict:
        """Get current failsafe status"""
        # Get current flight condition status
        flight_condition_status = {}
        if self.current_flight_condition:
            flight_condition_status = {
                'stall_detected': self.current_flight_condition.is_stall_condition(),
                'flat_spin_detected': self.current_flight_condition.is_flat_spin_condition(),
                'excessive_g_forces': self.current_flight_condition.is_excessive_g_forces(),
                'uncontrolled_descent': self.current_flight_condition.is_uncontrolled_descent(),
                'critical_attitude': self.current_flight_condition.is_critical_attitude(),
                'rotation_rates': self.current_flight_condition.is_excessive_rotation_rate(),
                'current_attitude': {
                    'pitch': self.current_flight_condition.pitch,
                    'roll': self.current_flight_condition.roll,
                    'yaw': self.current_flight_condition.yaw
                },
                'current_rates': {
                    'pitch_rate': self.current_flight_condition.pitch_rate,
                    'roll_rate': self.current_flight_condition.roll_rate,
                    'yaw_rate': self.current_flight_condition.yaw_rate
                },
                'flight_dynamics': {
                    'airspeed': self.current_flight_condition.airspeed,
                    'altitude': self.current_flight_condition.altitude,
                    'vertical_speed': self.current_flight_condition.vertical_speed,
                    'angle_of_attack': self.current_flight_condition.angle_of_attack,
                    'g_force': self.current_flight_condition.g_force
                }
            }
        
        return {
            'active': self.status.active,
            'triggered_at': self.status.triggered_at,
            'trigger_type': self.status.trigger_type.value if self.status.trigger_type else None,
            'last_gps_ping': self.status.last_gps_ping,
            'gps_ping_count': self.status.gps_ping_count,
            'parachute_deployed': self.status.parachute_deployed,
            'emergency_beacon_active': self.status.emergency_beacon_active,
            'gps_history_count': len(self.gps_history),
            'monitoring_active': self.running,
            'flight_condition_monitoring': {
                'active': self.current_flight_condition is not None,
                'history_count': len(self.flight_condition_history),
                'current_status': flight_condition_status
            }
        }
    
    def set_custom_condition(self, trigger: FailsafeTrigger, threshold: float, 
                           time_window: float, description: str, critical: bool = True):
        """Set a custom failsafe condition"""
        custom_condition = FailsafeCondition(
            trigger=trigger,
            threshold=threshold,
            time_window=time_window,
            description=description,
            critical=critical
        )
        
        # Replace existing condition if same trigger type
        for i, condition in enumerate(self.conditions):
            if condition.trigger == trigger:
                self.conditions[i] = custom_condition
                print(f"Updated condition: {trigger.value}")
                return
        
        # Add new condition
        self.conditions.append(custom_condition)
        print(f"Added custom condition: {trigger.value}")

# Example usage and testing
if __name__ == "__main__":
    # Create failsafe system
    failsafe = FailsafeSystem()
    
    # Set callbacks
    def on_failsafe_triggered(trigger_type, description):
        print(f"Failsafe callback: {trigger_type.value} - {description}")
    
    def on_parachute_deploy():
        print("Parachute deployment callback")
    
    def on_gps_ping(ping_data):
        print(f"GPS ping: {ping_data['position']}")
    
    failsafe.on_failsafe_triggered = on_failsafe_triggered
    failsafe.on_parachute_deploy = on_parachute_deploy
    failsafe.on_gps_ping = on_gps_ping
    
    # Start monitoring
    failsafe.start_monitoring()
    
    try:
        # Simulate GPS data updates
        print("Simulating GPS data updates...")
        for i in range(20):
            # Simulate normal flight
            failsafe.update_gps_data(
                latitude=37.7749 + (i * 0.0001),
                longitude=-122.4194 + (i * 0.0001),
                altitude=50.0 + (i * 2.0),
                voltage=12.0
            )
            time.sleep(0.5)
        
        # Test manual failsafe trigger
        print("\nTesting manual failsafe trigger...")
        failsafe.manual_failsafe_trigger()
        
        # Let it run for a bit
        time.sleep(5.0)
        
        # Check status
        status = failsafe.get_failsafe_status()
        print(f"\nFailsafe status: {json.dumps(status, indent=2)}")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        failsafe.stop_monitoring()
        print("Failsafe system shutdown complete")
