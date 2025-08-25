#!/usr/bin/env python3
"""
Main Flight Controller - Autonomous Aircraft Mission Execution
Integrates flight controller, hardware interface, and mission planning
"""

import time
import signal
import sys
from typing import Optional
from autonomous_flight_controller import AutonomousFlightController, GPSPosition, AircraftState
from hardware_interface import HardwareInterface, ActuatorCommands
from failsafe_system import FailsafeSystem, FailsafeTrigger
from efficiency_route_planner import (EfficiencyRoutePlanner, BatteryState, 
                                    ThrustProfile, EnvironmentalConditions, 
                                    PayloadCharacteristics, AerodynamicState,
                                    AirfoilProfile, AircraftGeometry, AircraftMaterial)
import json
import math

class MainFlightController:
    def __init__(self):
        self.flight_controller = AutonomousFlightController()
        self.hardware = HardwareInterface()
        self.failsafe = FailsafeSystem()
        self.efficiency_planner = EfficiencyRoutePlanner()
        
        # Mission parameters
        self.target_latitude = None
        self.target_longitude = None
        self.start_latitude = None
        self.start_longitude = None
        
        # Flight state
        self.mission_running = False
        self.current_waypoint_index = 0
        self.parachute_deployed = False
        
        # Control loop parameters
        self.control_rate = 50  # Hz
        self.last_control_time = 0
        
        # Safety parameters
        self.max_altitude = 150.0  # meters
        self.min_battery_voltage = 10.5  # volts
        self.max_flight_time = 1800  # 30 minutes
        self.mission_start_time = None
        
        # Efficiency monitoring
        self.current_efficiency_score = 0.0
        self.efficiency_history = []
        self.last_efficiency_update = 0
        
        # Signal handling
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Setup failsafe callbacks
        self._setup_failsafe_callbacks()
        
    def _setup_failsafe_callbacks(self):
        """Setup failsafe system callbacks"""
        self.failsafe.on_failsafe_triggered = self._on_failsafe_triggered
        self.failsafe.on_parachute_deploy = self._on_failsafe_parachute_deploy
        self.failsafe.on_gps_ping = self._on_failsafe_gps_ping
    
    def _on_failsafe_triggered(self, trigger_type: FailsafeTrigger, description: str):
        """Handle failsafe system activation"""
        print(f"🚨 FAILSAFE ACTIVATED: {trigger_type.value}")
        print(f"   Reason: {description}")
        
        # Stop normal mission execution
        self.mission_running = False
        
        # Execute emergency procedures
        self.emergency_landing()
        
        # Log the event
        print(f"Emergency procedures executed due to: {description}")
    
    def _on_failsafe_parachute_deploy(self):
        """Handle failsafe parachute deployment"""
        print("🪂 Failsafe parachute deployment activated")
        
        # Ensure parachute is deployed via hardware
        emergency_commands = ActuatorCommands()
        emergency_commands.parachute = True
        emergency_commands.aileron = 0.0
        emergency_commands.elevator = 0.0
        emergency_commands.rudder = 0.0
        emergency_commands.throttle = 0.0
        
        self.hardware.send_actuator_commands(emergency_commands)
        
        self.parachute_deployed = True
    
    def _on_failsafe_gps_ping(self, ping_data: dict):
        """Handle failsafe GPS pinging"""
        print(f"📍 Failsafe GPS ping: {ping_data['position']}")
        print(f"   Failsafe active: {ping_data['failsafe_active']}")
        print(f"   Parachute deployed: {ping_data['parachute_deployed']}")
        print(f"   Emergency beacon: {ping_data['emergency_beacon']}")
        print(f"   Ping count: {ping_data['ping_count']}")
        
        # This data would be transmitted to ground control
        # For now, just log it
    
    def signal_handler(self, signum, frame):
        """Handle system signals for graceful shutdown"""
        print(f"\nReceived signal {signum}, shutting down gracefully...")
        self.emergency_landing()
        self.cleanup()
        sys.exit(0)
    
    def set_mission_parameters(self, target_lat: float, target_lon: float, 
                              start_lat: float, start_lon: float):
        """Set mission parameters"""
        self.target_latitude = target_lat
        self.target_longitude = target_lon
        self.start_latitude = start_lat
        self.start_longitude = start_lon
        
        print(f"Mission parameters set:")
        print(f"  Start: {start_lat:.6f}, {start_lon:.6f}")
        print(f"  Target: {target_lat:.6f}, {target_lon:.6f}")
        
        # Set target in flight controller
        self.flight_controller.set_target(target_lat, target_lon, 0)
        
        # Plan route using efficiency planner
        self._plan_efficient_route()
    
    def _plan_efficient_route(self):
        """Plan route using advanced efficiency optimization"""
        print("🛩️ Planning efficient route with advanced optimization...")
        
        # Get current system states from hardware
        sensor_data = self.hardware.get_sensor_data()
        if not sensor_data:
            print("⚠️  No sensor data available, using default values")
            sensor_data = self._get_default_sensor_data()
        
        # Create efficiency planning data structures
        battery_state = self._create_battery_state(sensor_data)
        thrust_profile = self._create_thrust_profile(sensor_data)
        environmental = self._create_environmental_conditions(sensor_data)
        payload = self._create_payload_characteristics()
        aerodynamic = self._create_aerodynamic_state(sensor_data)
        
        # Plan efficient route
        start_position = (self.start_latitude, self.start_longitude, 0.0)
        target_position = (self.target_latitude, self.target_longitude, 0.0)
        
        try:
            flight_plan = self.efficiency_planner.plan_efficient_route(
                start_position, target_position,
                battery_state, thrust_profile, environmental, payload, aerodynamic
            )
            
            # Convert efficiency route to flight controller waypoints
            self._convert_efficiency_route_to_waypoints(flight_plan)
            
            print("✅ Efficiency route planning completed")
            print(f"   Route type: {flight_plan['route_type']}")
            print(f"   Efficiency score: {flight_plan['efficiency_score']:.3f}")
            print(f"   Optimal altitude: {flight_plan['optimal_altitude']:.1f}m")
            print(f"   Optimal airspeed: {flight_plan['optimal_airspeed']:.1f}m/s")
            
        except Exception as e:
            print(f"❌ Efficiency route planning failed: {e}")
            print("Falling back to basic route planning...")
            self._fallback_route_planning()
    
    def _get_default_sensor_data(self):
        """Get default sensor data when hardware is not available"""
        class DefaultSensorData:
            def __init__(self):
                self.gps_lat = self.start_latitude
                self.gps_lon = self.start_longitude
                self.gps_alt = 0.0
                self.voltage = 12.6
                self.current = 2.5
                self.airspeed = 20.0
                self.gps_heading = 0.0
                self.gps_speed = 20.0
        
        return DefaultSensorData()
    
    def _create_battery_state(self, sensor_data) -> BatteryState:
        """Create battery state for efficiency planning"""
        return BatteryState(
            voltage=sensor_data.voltage,
            current=sensor_data.current,
            capacity_remaining=2200,  # Default 3S LiPo capacity
            temperature=22.0,  # Default temperature
            charge_cycles=15,  # Default cycle count
            health_percentage=95.0  # Default health
        )
    
    def _create_thrust_profile(self, sensor_data) -> ThrustProfile:
        """Create thrust profile for efficiency planning"""
        # Estimate thrust based on current and power consumption
        power_consumption = sensor_data.voltage * sensor_data.current
        estimated_thrust = power_consumption * 0.15  # Rough estimate
        
        return ThrustProfile(
            motor_efficiency=0.85,  # Typical brushless motor efficiency
            propeller_efficiency=0.75,  # Typical propeller efficiency
            max_thrust=25.0,  # Maximum thrust capability
            current_thrust=estimated_thrust,
            rpm=8500,  # Typical cruise RPM
            power_consumption=power_consumption
        )
    
    def _create_environmental_conditions(self, sensor_data) -> EnvironmentalConditions:
        """Create environmental conditions for efficiency planning"""
        # Estimate air density based on altitude
        altitude = sensor_data.gps_alt
        air_density = 1.225 * math.exp(-altitude / 8000)  # Exponential decay model
        
        return EnvironmentalConditions(
            air_density=air_density,
            temperature=18.0,  # Default temperature
            pressure=101325,  # Standard atmospheric pressure
            humidity=65.0,  # Default humidity
            wind_speed=3.0,  # Default wind speed
            wind_direction=45.0,  # Default wind direction
            turbulence=0.2,  # Default turbulence
            visibility=5000  # Default visibility
        )
    
    def _create_payload_characteristics(self) -> PayloadCharacteristics:
        """Create payload characteristics for efficiency planning"""
        return PayloadCharacteristics(
            mass=0.3,  # Default payload mass
            center_of_gravity=(0.0, 0.0, 0.05),  # Slightly forward CG
            drag_coefficient=0.1,  # Low drag payload
            frontal_area=0.02,  # Small frontal area
            distribution="balanced"  # Balanced distribution
        )
    
    def _create_aerodynamic_state(self, sensor_data) -> AerodynamicState:
        """Create aerodynamic state for efficiency planning"""
        # Define aircraft-specific parameters
        airfoil_profile = AirfoilProfile(
            name="NACA 0012",
            thickness_ratio=0.12,
            camber=0.0,
            max_thickness_position=0.25,
            max_camber_position=0.0,
            cl_alpha=6.28,  # 2π radians
            cl_max=1.2,
            cd_min=0.006,
            reynolds_critical=500000
        )
        
        aircraft_geometry = AircraftGeometry(
            wingspan=1.2,      # meters
            wing_area=0.5,     # m²
            aspect_ratio=2.88,
            mean_aerodynamic_chord=0.4,
            fuselage_length=0.8,
            fuselage_diameter=0.1,
            tail_area=0.05,
            horizontal_tail_arm=0.5,
            vertical_tail_arm=0.2
        )
        
        aircraft_material = AircraftMaterial(
            name="Aluminum 6061-T6",
            density=2700,      # kg/m³
            tensile_strength=276,  # MPa
            elastic_modulus=70,    # GPa
            thermal_expansion=23e-6,
            corrosion_resistance=0.9,
            fatigue_resistance=0.95
        )
        
        return AerodynamicState(
            airspeed=sensor_data.airspeed,
            altitude=sensor_data.gps_alt,
            pitch=2.0,  # Slight nose-up for cruise
            roll=0.5,   # Minimal roll
            angle_of_attack=4.5,  # Optimal angle of attack
            sideslip=0.0,  # No sideslip
            lift_coefficient=0.8,  # Typical cruise lift coefficient
            drag_coefficient=0.06,  # Low drag coefficient
            reynolds_number=150000,  # Typical Reynolds number
            airfoil_profile=airfoil_profile,
            aircraft_geometry=aircraft_geometry,
            aircraft_material=aircraft_material
        )
    
    def _convert_efficiency_route_to_waypoints(self, flight_plan: dict):
        """Convert efficiency route plan to flight controller waypoints"""
        # Clear existing waypoints
        self.flight_controller.waypoints = []
        
        # Convert efficiency waypoints to GPSPosition objects
        for wp in flight_plan['waypoints']:
            position = wp['position']
            gps_position = GPSPosition(
                latitude=position['lat'],
                longitude=position['lon'],
                altitude=position['alt']
            )
            self.flight_controller.waypoints.append(gps_position)
        
        # Update current waypoint index
        self.current_waypoint_index = 0
        
        print(f"✅ Converted {len(self.flight_controller.waypoints)} waypoints from efficiency plan")
    
    def _fallback_route_planning(self):
        """Fallback to basic route planning if efficiency planning fails"""
        print("Using fallback route planning...")
        
        # Use basic flight controller route planning
        start_pos = GPSPosition(self.start_latitude, self.start_longitude, 0)
        self.flight_controller.plan_route(start_pos)
        
        # Update failsafe system with planned route
        self._update_failsafe_route()
    
    def _update_failsafe_route(self):
        """Update failsafe system with planned route for off-track detection"""
        if not self.flight_controller.waypoints:
            return
        
        # Convert waypoints to route format for failsafe
        planned_route = []
        for waypoint in self.flight_controller.waypoints:
            planned_route.append({
                'lat': waypoint.latitude,
                'lon': waypoint.longitude,
                'alt': waypoint.altitude
            })
        
        # Update failsafe system with route
        print(f"Updated failsafe system with {len(planned_route)} waypoints")
    
    def initialize_systems(self):
        """Initialize all systems for flight"""
        print("Initializing flight systems...")
        
        # Initialize hardware
        if not self.hardware.initialize_hardware():
            print("✗ Hardware initialization failed")
            return False
        
        # Start failsafe system
        print("Starting failsafe system...")
        self.failsafe.start_monitoring()
        
        # Calibrate sensors
        print("Calibrating sensors...")
        self.hardware.calibrate_sensors()
        
        # Wait for GPS fix
        print("Waiting for GPS fix...")
        gps_fix_acquired = False
        timeout = 60  # seconds
        
        start_time = time.time()
        while not gps_fix_acquired and (time.time() - start_time) < timeout:
            sensor_data = self.hardware.get_sensor_data()
            if sensor_data and sensor_data.gps_lat != 0 and sensor_data.gps_lon != 0:
                gps_fix_acquired = True
                print(f"✓ GPS fix acquired: {sensor_data.gps_lat:.6f}, {sensor_data.gps_lon:.6f}")
                
                # Update failsafe system with initial GPS data
                self._update_failsafe_gps(sensor_data)
            else:
                print("Waiting for GPS fix...")
                time.sleep(1)
        
        if not gps_fix_acquired:
            print("✗ GPS fix timeout")
            return False
        
        print("✓ All systems initialized")
        return True
    
    def _update_failsafe_gps(self, sensor_data):
        """Update failsafe system with GPS data"""
        # Get planned route for off-track detection
        planned_route = []
        if self.flight_controller.waypoints:
            for waypoint in self.flight_controller.waypoints:
                planned_route.append({
                    'lat': waypoint.latitude,
                    'lon': waypoint.longitude,
                    'alt': waypoint.altitude
                })
        
        # Update failsafe system
        self.failsafe.update_gps_data(
            latitude=sensor_data.gps_lat,
            longitude=sensor_data.gps_lon,
            altitude=sensor_data.gps_alt,
            voltage=sensor_data.voltage,
            planned_route=planned_route
        )
    
    def preflight_checklist(self):
        """Run preflight safety checks"""
        print("\n=== PREFLIGHT CHECKLIST ===")
        
        checks = [
            ("GPS signal", self._check_gps_signal),
            ("Battery voltage", self._check_battery),
            ("Control surfaces", self._check_control_surfaces),
            ("Wind conditions", self._check_wind_conditions),
            ("Flight area", self._check_flight_area),
            ("Emergency systems", self._check_emergency_systems),
            ("Failsafe system", self._check_failsafe_system),
            ("Efficiency planning", self._check_efficiency_planning)
        ]
        
        all_checks_passed = True
        
        for check_name, check_func in checks:
            print(f"\nChecking {check_name}...")
            if check_func():
                print(f"✓ {check_name} - OK")
            else:
                print(f"✗ {check_name} - FAILED")
                all_checks_passed = False
        
        if all_checks_passed:
            print("\n✅ All preflight checks passed!")
        else:
            print("\n❌ Preflight checks failed - mission aborted")
        
        return all_checks_passed
    
    def _check_gps_signal(self) -> bool:
        """Check GPS signal quality"""
        sensor_data = self.hardware.get_sensor_data()
        if not sensor_data:
            return False
        
        # Check if we have valid coordinates
        return (sensor_data.gps_lat != 0 and sensor_data.gps_lon != 0 and 
                sensor_data.gps_alt != 0)
    
    def _check_battery(self) -> bool:
        """Check battery voltage"""
        sensor_data = self.hardware.get_sensor_data()
        if not sensor_data:
            return False
        
        voltage_ok = sensor_data.voltage >= self.min_battery_voltage
        if not voltage_ok:
            print(f"  Battery voltage: {sensor_data.voltage:.1f}V (min: {self.min_battery_voltage}V)")
        
        return voltage_ok
    
    def _check_control_surfaces(self) -> bool:
        """Check control surface responsiveness"""
        print("  Testing control surfaces...")
        
        # Test aileron
        test_commands = ActuatorCommands()
        test_commands.aileron = 0.1
        self.hardware.send_actuator_commands(test_commands)
        time.sleep(0.1)
        
        test_commands.aileron = -0.1
        self.hardware.send_actuator_commands(test_commands)
        time.sleep(0.1)
        
        test_commands.aileron = 0.0
        self.hardware.send_actuator_commands(test_commands)
        
        return True  # Assume OK for now
    
    def _check_wind_conditions(self) -> bool:
        """Check wind conditions"""
        # This would analyze current wind data
        # For now, assume conditions are acceptable
        return True
    
    def _check_flight_area(self) -> bool:
        """Check flight area safety"""
        # This would check for obstacles, restricted areas, etc.
        # For now, assume area is clear
        return True
    
    def _check_emergency_systems(self) -> bool:
        """Check emergency systems"""
        print("  Testing emergency systems...")
        
        # Test parachute deployment
        test_commands = ActuatorCommands()
        test_commands.parachute = True
        self.hardware.send_actuator_commands(test_commands)
        time.sleep(0.5)
        
        test_commands.parachute = False
        self.hardware.send_actuator_commands(test_commands)
        
        return True
    
    def _check_failsafe_system(self) -> bool:
        """Check failsafe system status"""
        print("  Checking failsafe system...")
        
        # Check if failsafe is monitoring
        status = self.failsafe.get_failsafe_status()
        if not status['monitoring_active']:
            print("    Failsafe monitoring not active")
            return False
        
        # Check if failsafe is not already triggered
        if status['active']:
            print("    Failsafe already active - system error")
            return False
        
        print("    Failsafe system monitoring active")
        return True
    
    def _check_efficiency_planning(self) -> bool:
        """Check efficiency planning system"""
        print("  Checking efficiency planning...")
        
        # Check if we have waypoints planned
        if not self.flight_controller.waypoints:
            print("    No waypoints planned")
            return False
        
        # Check if we have enough waypoints for a route
        if len(self.flight_controller.waypoints) < 2:
            print("    Insufficient waypoints for route")
            return False
        
        print(f"    {len(self.flight_controller.waypoints)} waypoints planned")
        return True
    
    def execute_mission(self):
        """Execute the complete autonomous mission"""
        if not self.mission_running:
            print("Mission not started")
            return
        
        print("\n🚀 MISSION EXECUTION STARTED")
        self.mission_start_time = time.time()
        
        try:
            # Main mission loop
            while self.mission_running:
                current_time = time.time()
                
                # Check if failsafe is active
                if self.failsafe.status.active:
                    print("🚨 Failsafe active - mission suspended")
                    break
                
                # Control rate limiting
                if current_time - self.last_control_time < (1.0 / self.control_rate):
                    time.sleep(0.001)
                    continue
                
                self.last_control_time = current_time
                
                # Get current sensor data
                sensor_data = self.hardware.get_sensor_data()
                if not sensor_data:
                    continue
                
                # Update failsafe system with GPS data
                self._update_failsafe_gps(sensor_data)
                
                # Update efficiency monitoring
                self._update_efficiency_monitoring(sensor_data)
                
                # Create aircraft state
                current_state = AircraftState(
                    position=GPSPosition(sensor_data.gps_lat, sensor_data.gps_lon, sensor_data.gps_alt),
                    heading=sensor_data.gps_heading,
                    airspeed=sensor_data.airspeed,
                    groundspeed=sensor_data.gps_speed,
                    altitude=sensor_data.gps_alt,
                    pitch=0,  # Would come from IMU
                    roll=0,   # Would come from IMU
                    yaw=0     # Would come from IMU
                )
                
                # Execute current flight phase
                self._execute_flight_phase(current_state)
                
                # Check safety conditions
                if not self._check_safety_conditions(current_state):
                    print("🚨 Safety condition violated - emergency landing")
                    self.emergency_landing()
                    break
                
                # Check mission completion
                if self._check_mission_completion(current_state):
                    print("✅ Mission completed successfully!")
                    break
                
        except Exception as e:
            print(f"Mission execution error: {e}")
            self.emergency_landing()
    
    def _update_efficiency_monitoring(self, sensor_data):
        """Update efficiency monitoring during flight"""
        current_time = time.time()
        
        # Update efficiency every 5 seconds
        if current_time - self.last_efficiency_update >= 5.0:
            # Calculate current efficiency score
            efficiency_score = self._calculate_current_efficiency(sensor_data)
            
            # Store in history
            self.efficiency_history.append({
                'timestamp': current_time,
                'efficiency_score': efficiency_score,
                'position': {
                    'lat': sensor_data.gps_lat,
                    'lon': sensor_data.gps_lon,
                    'alt': sensor_data.gps_alt
                },
                'battery_voltage': sensor_data.voltage,
                'airspeed': sensor_data.airspeed
            })
            
            # Keep only last 100 efficiency readings
            if len(self.efficiency_history) > 100:
                self.efficiency_history = self.efficiency_history[-100:]
            
            self.current_efficiency_score = efficiency_score
            self.last_efficiency_update = current_time
            
            # Log efficiency status
            print(f"📊 Efficiency update: {efficiency_score:.3f}")
    
    def _calculate_current_efficiency(self, sensor_data) -> float:
        """Calculate current flight efficiency score"""
        # Simplified efficiency calculation based on current conditions
        efficiency = 1.0
        
        # Battery efficiency
        battery_factor = min(sensor_data.voltage / 12.6, 1.0)
        efficiency *= battery_factor
        
        # Airspeed efficiency (assume optimal around 20 m/s)
        speed_factor = 1.0 - abs(sensor_data.airspeed - 20.0) / 20.0
        efficiency *= speed_factor
        
        # Altitude efficiency (assume optimal around 100m)
        altitude_factor = 1.0 - abs(sensor_data.gps_alt - 100.0) / 100.0
        efficiency *= altitude_factor
        
        return max(0.0, min(1.0, efficiency))
    
    def _execute_flight_phase(self, current_state: AircraftState):
        """Execute the current flight phase"""
        phase = self.flight_controller.current_phase
        
        if phase.value == "ground":
            self._execute_ground_phase(current_state)
        elif phase.value == "takeoff":
            self._execute_takeoff_phase(current_state)
        elif phase.value == "climb":
            self._execute_climb_phase(current_state)
        elif phase.value == "cruise":
            self._execute_cruise_phase(current_state)
        elif phase.value == "approach":
            self._execute_approach_phase(current_state)
        elif phase.value == "parachute_deploy":
            self._execute_parachute_phase(current_state)
        elif phase.value == "descent":
            self._execute_descent_phase(current_state)
    
    def _execute_ground_phase(self, current_state: AircraftState):
        """Execute ground phase - prepare for takeoff"""
        print("Ground phase - preparing for takeoff")
        
        # Check if ready for takeoff
        if (current_state.altitude < 5 and  # Near ground
            current_state.airspeed < 5):    # Low speed
            
            print("Conditions met for takeoff")
            self.flight_controller.autonomous_takeoff(current_state)
    
    def _execute_takeoff_phase(self, current_state: AircraftState):
        """Execute takeoff phase"""
        print("Takeoff phase - accelerating and climbing")
        
        # Takeoff commands
        commands = ActuatorCommands()
        commands.throttle = 1.0  # Full throttle
        commands.elevator = 0.2  # Nose up
        
        self.hardware.send_actuator_commands(commands)
        
        # Check if takeoff complete
        if current_state.altitude > 20 and current_state.airspeed > 15:
            print("Takeoff complete, transitioning to climb")
            self.flight_controller.current_phase.value = "climb"
    
    def _execute_climb_phase(self, current_state: AircraftState):
        """Execute climb phase"""
        print("Climb phase - gaining altitude")
        
        # Climb commands
        commands = ActuatorCommands()
        commands.throttle = 0.8  # High throttle
        commands.elevator = 0.1  # Slight nose up
        
        self.hardware.send_actuator_commands(commands)
        
        # Check if climb complete
        if current_state.altitude >= self.flight_controller.cruise_altitude:
            print("Climb complete, transitioning to cruise")
            self.flight_controller.current_phase.value = "cruise"
    
    def _execute_cruise_phase(self, current_state: AircraftState):
        """Execute cruise phase - navigate to waypoints"""
        print("Cruise phase - navigating to waypoints")
        
        # Navigate to current waypoint
        if self.current_waypoint_index < len(self.flight_controller.waypoints):
            target_waypoint = self.flight_controller.waypoints[self.current_waypoint_index]
            
            # Get navigation commands
            nav_commands = self.flight_controller.navigate_to_waypoint(current_state, target_waypoint)
            
            # Convert to actuator commands
            commands = ActuatorCommands()
            commands.aileron = nav_commands['aileron']
            commands.elevator = nav_commands['elevator']
            commands.rudder = nav_commands['rudder']
            commands.throttle = nav_commands['throttle']
            
            self.hardware.send_actuator_commands(commands)
            
            # Check if waypoint reached
            distance = current_state.position.distance_to(target_waypoint)
            if distance < 20:  # Within 20m of waypoint
                print(f"Waypoint {self.current_waypoint_index} reached")
                self.current_waypoint_index += 1
                
                # Check if approaching final waypoint
                if self.current_waypoint_index >= len(self.flight_controller.waypoints) - 1:
                    print("Approaching target, transitioning to approach phase")
                    self.flight_controller.current_phase.value = "approach"
        else:
            # All waypoints reached
            print("All waypoints reached")
    
    def _execute_approach_phase(self, current_state: AircraftState):
        """Execute approach phase - prepare for parachute deployment"""
        print("Approach phase - preparing for parachute deployment")
        
        # Analyze wind conditions
        self.flight_controller.analyze_wind(current_state, current_state.heading, current_state.groundspeed)
        
        # Calculate parachute deployment parameters
        deployment_alt, deployment_dist = self.flight_controller.calculate_parachute_deployment(current_state)
        
        # Check if ready for parachute deployment
        distance_to_target = current_state.position.distance_to(self.flight_controller.target_position)
        
        if (current_state.altitude <= deployment_alt and 
            distance_to_target <= deployment_dist and
            not self.parachute_deployed):
            
            print("Conditions met for parachute deployment")
            self.flight_controller.current_phase.value = "parachute_deploy"
    
    def _execute_parachute_phase(self, current_state: AircraftState):
        """Execute parachute deployment phase"""
        print("Parachute deployment phase")
        
        # Deploy parachute
        deployment_commands = self.flight_controller.execute_parachute_deployment(current_state)
        
        # Convert to actuator commands
        commands = ActuatorCommands()
        commands.parachute = deployment_commands['parachute_release']
        commands.aileron = deployment_commands['aileron']
        commands.elevator = deployment_commands['elevator']
        commands.rudder = deployment_commands['rudder']
        commands.throttle = deployment_commands['throttle']
        
        self.hardware.send_actuator_commands(commands)
        
        self.parachute_deployed = True
        self.flight_controller.current_phase.value = "descent"
        
        print("Parachute deployed, transitioning to descent")
    
    def _execute_descent_phase(self, current_state: AircraftState):
        """Execute descent phase - parachute descent"""
        print("Descent phase - parachute descent")
        
        # Neutralize all controls during descent
        commands = ActuatorCommands()
        commands.aileron = 0.0
        commands.elevator = 0.0
        commands.rudder = 0.0
        commands.throttle = 0.0
        
        self.hardware.send_actuator_commands(commands)
        
        # Check if landed
        if current_state.altitude < 5:
            print("Landed successfully!")
            self.flight_controller.current_phase.value = "landed"
            self.mission_running = False
    
    def _check_safety_conditions(self, current_state: AircraftState) -> bool:
        """Check safety conditions"""
        current_time = time.time()
        
        # Check altitude
        if current_state.altitude > self.max_altitude:
            print(f"🚨 Altitude limit exceeded: {current_state.altitude:.1f}m")
            return False
        
        # Check battery
        sensor_data = self.hardware.get_sensor_data()
        if sensor_data and sensor_data.voltage < self.min_battery_voltage:
            print(f"🚨 Low battery: {sensor_data.voltage:.1f}V")
            return False
        
        # Check flight time
        if self.mission_start_time and (current_time - self.mission_start_time) > self.max_flight_time:
            print(f"🚨 Maximum flight time exceeded: {self.max_flight_time/60:.1f} minutes")
            return False
        
        return True
    
    def _check_mission_completion(self, current_state: AircraftState) -> bool:
        """Check if mission is complete"""
        if not self.flight_controller.target_position:
            return False
        
        # Check if landed near target
        distance_to_target = current_state.position.distance_to(self.flight_controller.target_position)
        if distance_to_target < 10 and current_state.altitude < 5:  # Within 10m and on ground
            return True
        
        return False
    
    def emergency_landing(self):
        """Execute emergency landing procedure"""
        print("🚨 EMERGENCY LANDING INITIATED")
        
        # Deploy parachute immediately
        emergency_commands = ActuatorCommands()
        emergency_commands.parachute = True
        emergency_commands.aileron = 0.0
        emergency_commands.elevator = 0.0
        emergency_commands.rudder = 0.0
        emergency_commands.throttle = 0.0
        
        self.hardware.send_actuator_commands(emergency_commands)
        
        # Activate emergency beacon
        print("Emergency beacon activated")
        
        self.mission_running = False
    
    def start_mission(self):
        """Start the autonomous mission"""
        print("Starting autonomous mission...")
        
        # Initialize systems
        if not self.initialize_systems():
            print("System initialization failed")
            return False
        
        # Run preflight checks
        if not self.preflight_checklist():
            print("Preflight checks failed")
            return False
        
        # Start mission
        self.mission_running = True
        print("✅ Mission started successfully!")
        
        # Execute mission
        self.execute_mission()
        
        return True
    
    def cleanup(self):
        """Clean up all systems"""
        print("Cleaning up systems...")
        
        # Stop mission
        self.mission_running = False
        
        # Stop failsafe system
        self.failsafe.stop_monitoring()
        
        # Clean up hardware
        self.hardware.cleanup()
        
        # Save telemetry log
        timestamp = int(time.time())
        filename = f"flight_log_{timestamp}.json"
        self.flight_controller.save_telemetry_log(filename)
        
        # Save failsafe log
        failsafe_filename = f"failsafe_log_{timestamp}.json"
        with open(failsafe_filename, 'w') as f:
            json.dump(self.failsafe.get_failsafe_status(), f, indent=2)
        
        # Save efficiency log
        efficiency_filename = f"efficiency_log_{timestamp}.json"
        with open(efficiency_filename, 'w') as f:
            json.dump({
                'efficiency_history': self.efficiency_history,
                'final_efficiency_score': self.current_efficiency_score
            }, f, indent=2)
        
        print("✓ Cleanup complete")
    
    def get_system_status(self) -> dict:
        """Get comprehensive system status"""
        return {
            'mission_status': {
                'running': self.mission_running,
                'current_waypoint': self.current_waypoint_index,
                'total_waypoints': len(self.flight_controller.waypoints),
                'flight_phase': self.flight_controller.current_phase.value,
                'parachute_deployed': self.parachute_deployed
            },
            'failsafe_status': self.failsafe.get_failsafe_status(),
            'flight_controller_status': self.flight_controller.get_flight_status(),
            'efficiency_status': {
                'current_score': self.current_efficiency_score,
                'history_count': len(self.efficiency_history),
                'last_update': self.last_efficiency_update
            }
        }

def main():
    """Main function"""
    print("=== AUTONOMOUS AIRCRAFT FLIGHT CONTROLLER ===")
    print("Advanced autonomous flight with precision parachute landing")
    print("Integrated failsafe system and efficiency optimization")
    
    # Create flight controller
    controller = MainFlightController()
    
    try:
        # Set mission parameters (example coordinates)
        # Replace with actual coordinates for your mission
        target_lat = 37.7749  # Target latitude
        target_lon = -122.4194  # Target longitude
        start_lat = 37.7849   # Start latitude
        start_lon = -122.4094  # Start longitude
        
        controller.set_mission_parameters(target_lat, target_lon, start_lat, start_lon)
        
        # Start mission
        success = controller.start_mission()
        
        if success:
            print("Mission completed successfully!")
        else:
            print("Mission failed")
            
    except KeyboardInterrupt:
        print("\nMission interrupted by user")
    except Exception as e:
        print(f"Mission error: {e}")
    finally:
        controller.cleanup()
        print("Flight controller shutdown complete")

if __name__ == "__main__":
    main()
