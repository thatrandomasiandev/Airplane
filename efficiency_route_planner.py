#!/usr/bin/env python3
"""
Advanced Efficiency Route Planner for Autonomous Aircraft
Optimizes flight paths based on multiple real-world factors for maximum efficiency
"""

import math
import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from enum import Enum
import json

class FlightEfficiencyFactor(Enum):
    BATTERY_OPTIMIZATION = "battery_optimization"
    THRUST_EFFICIENCY = "thrust_efficiency"
    AERODYNAMIC_EFFICIENCY = "aerodynamic_efficiency"
    ENVIRONMENTAL_OPTIMIZATION = "environmental_optimization"
    PAYLOAD_OPTIMIZATION = "payload_optimization"
    ALTITUDE_OPTIMIZATION = "altitude_optimization"
    WEATHER_OPTIMIZATION = "weather_optimization"

@dataclass
class BatteryState:
    voltage: float  # Volts
    current: float  # Amperes
    capacity_remaining: float  # mAh
    temperature: float  # Celsius
    charge_cycles: int
    health_percentage: float  # 0-100%
    
    def get_power_available(self) -> float:
        """Calculate available power in watts"""
        return self.voltage * self.current
    
    def get_energy_remaining(self) -> float:
        """Calculate remaining energy in watt-hours"""
        return (self.capacity_remaining / 1000.0) * self.voltage
    
    def get_efficiency_factor(self) -> float:
        """Calculate battery efficiency factor (0-1)"""
        # Consider voltage, temperature, and health
        voltage_factor = min(self.voltage / 12.6, 1.0)  # Normalize to 3S LiPo
        temp_factor = 1.0 if 15 <= self.temperature <= 35 else 0.7  # Optimal temp range
        health_factor = self.health_percentage / 100.0
        
        return voltage_factor * temp_factor * health_factor

@dataclass
class ThrustProfile:
    motor_efficiency: float  # 0-1
    propeller_efficiency: float  # 0-1
    max_thrust: float  # Newtons
    current_thrust: float  # Newtons
    rpm: float  # Revolutions per minute
    power_consumption: float  # Watts
    
    def get_thrust_efficiency(self) -> float:
        """Calculate overall thrust efficiency"""
        return self.motor_efficiency * self.propeller_efficiency
    
    def get_power_to_thrust_ratio(self) -> float:
        """Calculate power efficiency (thrust per watt)"""
        if self.power_consumption > 0:
            return self.current_thrust / self.power_consumption
        return 0.0

@dataclass
class EnvironmentalConditions:
    air_density: float  # kg/m³
    temperature: float  # Celsius
    pressure: float  # Pa
    humidity: float  # %
    wind_speed: float  # m/s
    wind_direction: float  # degrees
    turbulence: float  # 0-1 scale
    visibility: float  # meters
    
    def get_air_density_factor(self) -> float:
        """Calculate air density factor affecting performance"""
        # Standard air density at sea level: 1.225 kg/m³
        return self.air_density / 1.225
    
    def get_wind_impact_factor(self) -> float:
        """Calculate wind impact on efficiency"""
        # Headwind reduces efficiency, tailwind increases it
        return 1.0 - (self.wind_speed * 0.1)  # Simplified model

@dataclass
class PayloadCharacteristics:
    mass: float  # kg
    center_of_gravity: Tuple[float, float, float]  # x, y, z in meters
    drag_coefficient: float
    frontal_area: float  # m²
    distribution: str  # "balanced", "forward", "aft"
    
    def get_payload_efficiency_impact(self) -> float:
        """Calculate payload impact on flight efficiency"""
        # Heavier payloads reduce efficiency
        mass_factor = 1.0 / (1.0 + self.mass * 0.1)
        
        # CG position affects stability and efficiency
        cg_x, cg_y, cg_z = self.center_of_gravity
        cg_factor = 1.0 if abs(cg_x) < 0.1 else 0.8  # Balanced CG is best
        
        # Drag impact
        drag_factor = 1.0 / (1.0 + self.drag_coefficient * self.frontal_area * 0.5)
        
        return mass_factor * cg_factor * drag_factor

@dataclass
class AirfoilProfile:
    name: str
    thickness_ratio: float  # t/c ratio
    camber: float  # percentage camber
    max_thickness_position: float  # x/c position of max thickness
    max_camber_position: float  # x/c position of max camber
    cl_alpha: float  # lift curve slope (per radian)
    cl_max: float  # maximum lift coefficient
    cd_min: float  # minimum drag coefficient
    reynolds_critical: float  # critical Reynolds number for transition
    
    def get_lift_coefficient(self, angle_of_attack: float, reynolds: float) -> float:
        """Calculate lift coefficient for given angle of attack and Reynolds number"""
        # Linear region (before stall)
        if abs(angle_of_attack) < 10:  # degrees
            cl = self.cl_alpha * math.radians(angle_of_attack)
        else:
            # Stall region - simplified model
            cl = self.cl_max * math.sin(math.radians(angle_of_attack))
        
        # Reynolds number correction
        if reynolds < self.reynolds_critical:
            cl *= 0.9  # Laminar flow penalty
        
        return max(-self.cl_max, min(self.cl_max, cl))
    
    def get_drag_coefficient(self, angle_of_attack: float, reynolds: float) -> float:
        """Calculate drag coefficient for given angle of attack and Reynolds number"""
        # Base drag
        cd_base = self.cd_min
        
        # Induced drag (varies with lift coefficient squared)
        cl = self.get_lift_coefficient(angle_of_attack, reynolds)
        cd_induced = (cl ** 2) / (math.pi * 4.0)  # Assuming aspect ratio of 4
        
        # Reynolds number correction
        if reynolds < self.reynolds_critical:
            cd_base *= 1.2  # Laminar flow penalty
        
        return cd_base + cd_induced

@dataclass
class AircraftGeometry:
    wingspan: float  # meters
    wing_area: float  # m²
    aspect_ratio: float  # wingspan² / wing_area
    mean_aerodynamic_chord: float  # meters
    fuselage_length: float  # meters
    fuselage_diameter: float  # meters
    tail_area: float  # m²
    horizontal_tail_arm: float  # meters (distance from CG to tail)
    vertical_tail_arm: float  # meters
    
    def get_induced_drag_factor(self) -> float:
        """Calculate induced drag factor based on geometry"""
        # Higher aspect ratio = lower induced drag
        return 1.0 / (1.0 + self.aspect_ratio * 0.1)
    
    def get_parasitic_drag_factor(self) -> float:
        """Calculate parasitic drag factor based on geometry"""
        # Wetted area ratio affects parasitic drag
        wetted_area = self.wing_area + (self.fuselage_length * self.fuselage_diameter * math.pi)
        wetted_ratio = wetted_area / self.wing_area
        return 1.0 + (wetted_ratio - 1.0) * 0.3

@dataclass
class AircraftMaterial:
    name: str
    density: float  # kg/m³
    tensile_strength: float  # MPa
    elastic_modulus: float  # GPa
    thermal_expansion: float  # 1/K
    corrosion_resistance: float  # 0-1 scale
    fatigue_resistance: float  # 0-1 scale
    
    def get_weight_factor(self) -> float:
        """Calculate weight impact factor"""
        # Normalize to aluminum density (2700 kg/m³)
        return self.density / 2700.0
    
    def get_structural_efficiency(self) -> float:
        """Calculate structural efficiency factor"""
        # Higher strength-to-weight ratio = better efficiency
        strength_to_weight = self.tensile_strength / self.density
        return min(1.0, strength_to_weight / 200.0)  # Normalize to typical values

@dataclass
class AerodynamicState:
    airspeed: float  # m/s
    altitude: float  # meters
    pitch: float  # degrees
    roll: float  # degrees
    angle_of_attack: float  # degrees
    sideslip: float  # degrees
    lift_coefficient: float
    drag_coefficient: float
    reynolds_number: float
    airfoil_profile: AirfoilProfile
    aircraft_geometry: AircraftGeometry
    aircraft_material: AircraftMaterial
    
    def get_lift_drag_ratio(self) -> float:
        """Calculate lift-to-drag ratio (efficiency metric)"""
        if self.drag_coefficient > 0:
            return self.lift_coefficient / self.drag_coefficient
        return 0.0
    
    def get_optimal_airspeed(self, aircraft_mass: float, air_density: float) -> float:
        """Calculate optimal airspeed for maximum efficiency"""
        # Optimal airspeed for maximum L/D ratio
        # Consider airfoil characteristics and aircraft geometry
        optimal_speed = math.sqrt((2 * aircraft_mass * 9.81) / 
                                 (air_density * self.aircraft_geometry.wing_area * 
                                  self.airfoil_profile.cl_alpha * 0.1))  # 0.1 rad ≈ 5.7°
        return optimal_speed
    
    def get_efficiency_score(self) -> float:
        """Calculate overall aerodynamic efficiency score"""
        # Normalize all factors to 0-1 range
        pitch_factor = 1.0 - abs(self.pitch) / 45.0  # 0° pitch is optimal
        roll_factor = 1.0 - abs(self.roll) / 45.0    # 0° roll is optimal
        aoa_factor = 1.0 - abs(self.angle_of_attack - 5.0) / 10.0  # 5° AOA is optimal
        
        # Airfoil efficiency
        airfoil_efficiency = self.airfoil_profile.cl_alpha / 6.0  # Normalize to typical value
        
        # Geometry efficiency
        geometry_efficiency = self.aircraft_geometry.get_induced_drag_factor()
        
        # Material efficiency
        material_efficiency = self.aircraft_material.get_structural_efficiency()
        
        # Combine factors with weights
        efficiency = (pitch_factor * 0.2 + roll_factor * 0.2 + aoa_factor * 0.2 + 
                     airfoil_efficiency * 0.2 + geometry_efficiency * 0.1 + 
                     material_efficiency * 0.1)
        
        return max(0.0, min(1.0, efficiency))
    
    def calculate_aerodynamic_forces(self, air_density: float) -> Dict[str, float]:
        """Calculate aerodynamic forces based on current state"""
        # Dynamic pressure
        q = 0.5 * air_density * (self.airspeed ** 2)
        
        # Lift force
        lift = q * self.aircraft_geometry.wing_area * self.lift_coefficient
        
        # Drag force
        drag = q * self.aircraft_geometry.wing_area * self.drag_coefficient
        
        # Side force (due to sideslip)
        side_force = q * self.aircraft_geometry.wing_area * self.sideslip * 0.1
        
        return {
            'lift': lift,
            'drag': drag,
            'side_force': side_force,
            'dynamic_pressure': q
        }

class EfficiencyRoutePlanner:
    def __init__(self):
        self.aircraft_parameters = {
            'wing_area': 0.5,      # m²
            'wing_span': 1.2,      # meters
            'aspect_ratio': 2.88,  # wing aspect ratio
            'mass_empty': 1.5,     # kg (empty aircraft)
            'max_payload': 0.5,    # kg
            'cruise_altitude': 100.0,  # meters
            'service_ceiling': 200.0,  # meters
        }
        
        # Efficiency weights for different factors
        self.efficiency_weights = {
            FlightEfficiencyFactor.BATTERY_OPTIMIZATION: 0.25,
            FlightEfficiencyFactor.THRUST_EFFICIENCY: 0.20,
            FlightEfficiencyFactor.AERODYNAMIC_EFFICIENCY: 0.25,
            FlightEfficiencyFactor.ENVIRONMENTAL_OPTIMIZATION: 0.15,
            FlightEfficiencyFactor.PAYLOAD_OPTIMIZATION: 0.10,
            FlightEfficiencyFactor.ALTITUDE_OPTIMIZATION: 0.05
        }
        
        # Performance curves and lookup tables
        self.performance_curves = self._initialize_performance_curves()
        
    def _initialize_performance_curves(self) -> Dict:
        """Initialize aircraft performance curves"""
        return {
            'power_curve': {
                'altitude': [0, 50, 100, 150, 200],
                'power_factor': [1.0, 0.95, 0.90, 0.85, 0.80]
            },
            'efficiency_curve': {
                'airspeed': [10, 15, 20, 25, 30],
                'efficiency': [0.7, 0.85, 0.95, 0.90, 0.75]
            },
            'battery_curve': {
                'voltage': [10.5, 11.1, 11.7, 12.3, 12.6],
                'efficiency': [0.6, 0.75, 0.85, 0.95, 1.0]
            }
        }
    
    def plan_efficient_route(self, start_pos: Tuple[float, float, float],
                           target_pos: Tuple[float, float, float],
                           battery_state: BatteryState,
                           thrust_profile: ThrustProfile,
                           environmental: EnvironmentalConditions,
                           payload: PayloadCharacteristics,
                           aerodynamic: AerodynamicState) -> Dict:
        """
        Plan the most efficient route considering all factors
        Returns: Complete route plan with efficiency metrics
        """
        print("🛩️ Planning efficient route with advanced optimization...")
        
        # Calculate base efficiency scores for each factor
        efficiency_scores = self._calculate_efficiency_scores(
            battery_state, thrust_profile, environmental, payload, aerodynamic
        )
        
        # Generate route candidates
        route_candidates = self._generate_route_candidates(start_pos, target_pos)
        
        # Evaluate each route for efficiency
        route_evaluations = []
        for route in route_candidates:
            evaluation = self._evaluate_route_efficiency(
                route, efficiency_scores, environmental, payload, aerodynamic
            )
            route_evaluations.append(evaluation)
        
        # Select optimal route
        optimal_route = self._select_optimal_route(route_evaluations)
        
        # Generate detailed flight plan
        flight_plan = self._generate_flight_plan(optimal_route, efficiency_scores)
        
        return flight_plan
    
    def _calculate_efficiency_scores(self, battery_state: BatteryState,
                                   thrust_profile: ThrustProfile,
                                   environmental: EnvironmentalConditions,
                                   payload: PayloadCharacteristics,
                                   aerodynamic: AerodynamicState) -> Dict:
        """Calculate efficiency scores for all factors"""
        scores = {}
        
        # Battery efficiency
        battery_efficiency = battery_state.get_efficiency_factor()
        scores[FlightEfficiencyFactor.BATTERY_OPTIMIZATION] = battery_efficiency
        
        # Thrust efficiency
        thrust_efficiency = thrust_profile.get_thrust_efficiency()
        scores[FlightEfficiencyFactor.THRUST_EFFICIENCY] = thrust_efficiency
        
        # Aerodynamic efficiency
        aero_efficiency = aerodynamic.get_efficiency_score()
        scores[FlightEfficiencyFactor.AERODYNAMIC_EFFICIENCY] = aero_efficiency
        
        # Environmental efficiency
        env_efficiency = (environmental.get_air_density_factor() * 
                         environmental.get_wind_impact_factor())
        scores[FlightEfficiencyFactor.ENVIRONMENTAL_OPTIMIZATION] = env_efficiency
        
        # Payload efficiency
        payload_efficiency = payload.get_payload_efficiency_impact()
        scores[FlightEfficiencyFactor.PAYLOAD_OPTIMIZATION] = payload_efficiency
        
        # Altitude efficiency
        altitude_efficiency = self._calculate_altitude_efficiency(
            aerodynamic.altitude, environmental.air_density
        )
        scores[FlightEfficiencyFactor.ALTITUDE_OPTIMIZATION] = altitude_efficiency
        
        return scores
    
    def _calculate_altitude_efficiency(self, altitude: float, air_density: float) -> float:
        """Calculate efficiency based on altitude and air density"""
        # Lower altitudes generally have higher air density (better efficiency)
        # But there's an optimal altitude for most aircraft
        optimal_altitude = 100.0  # meters
        
        altitude_factor = 1.0 - abs(altitude - optimal_altitude) / optimal_altitude
        density_factor = air_density / 1.225  # Normalize to sea level
        
        return (altitude_factor + density_factor) / 2.0
    
    def _generate_route_candidates(self, start_pos: Tuple[float, float, float],
                                 target_pos: Tuple[float, float, float]) -> List[Dict]:
        """Generate multiple route candidates for evaluation"""
        start_lat, start_lon, start_alt = start_pos
        target_lat, target_lon, target_alt = target_pos
        
        # Calculate direct distance and bearing
        distance = self._calculate_distance(start_lat, start_lon, target_lat, target_lon)
        bearing = self._calculate_bearing(start_lat, start_lon, target_lat, target_lon)
        
        routes = []
        
        # Route 1: Direct path (most efficient for short distances)
        routes.append({
            'type': 'direct',
            'waypoints': [
                {'lat': start_lat, 'lon': start_lon, 'alt': start_alt},
                {'lat': target_lat, 'lon': target_lon, 'alt': target_alt}
            ],
            'estimated_distance': distance,
            'estimated_time': distance / 20.0,  # Assuming 20 m/s cruise
            'efficiency_priority': 'speed'
        })
        
        # Route 2: Altitude-optimized path
        if distance > 500:  # For longer flights
            optimal_alt = self._calculate_optimal_cruise_altitude(start_alt, target_alt)
            routes.append({
                'type': 'altitude_optimized',
                'waypoints': [
                    {'lat': start_lat, 'lon': start_lon, 'alt': start_alt},
                    {'lat': start_lat, 'lon': start_lon, 'alt': optimal_alt},
                    {'lat': target_lat, 'lon': target_lon, 'alt': optimal_alt},
                    {'lat': target_lat, 'lon': target_lon, 'alt': target_alt}
                ],
                'estimated_distance': distance * 1.1,  # Slightly longer
                'estimated_time': distance * 1.1 / 22.0,  # Faster at optimal altitude
                'efficiency_priority': 'fuel_economy'
            })
        
        # Route 3: Wind-optimized path
        wind_optimized_route = self._generate_wind_optimized_route(
            start_pos, target_pos, bearing, distance
        )
        if wind_optimized_route:
            routes.append(wind_optimized_route)
        
        # Route 4: Terrain-avoidance path (if needed)
        terrain_route = self._generate_terrain_avoidance_route(
            start_pos, target_pos, distance
        )
        if terrain_route:
            routes.append(terrain_route)
        
        return routes
    
    def _generate_wind_optimized_route(self, start_pos: Tuple[float, float, float],
                                     target_pos: Tuple[float, float, float],
                                     bearing: float, distance: float) -> Optional[Dict]:
        """Generate wind-optimized route"""
        # This would analyze wind patterns and create a route that minimizes headwind
        # For now, return a simple wind-optimized route
        start_lat, start_lon, start_alt = start_pos
        target_lat, target_lon, target_alt = target_pos
        
        # Create waypoints that minimize crosswind
        mid_lat = (start_lat + target_lat) / 2
        mid_lon = (start_lon + target_lon) / 2
        
        return {
            'type': 'wind_optimized',
            'waypoints': [
                {'lat': start_lat, 'lon': start_lon, 'alt': start_alt},
                {'lat': mid_lat, 'lon': mid_lon, 'alt': start_alt + 20},
                {'lat': target_lat, 'lon': target_lon, 'alt': target_alt}
            ],
            'estimated_distance': distance * 1.15,
            'estimated_time': distance * 1.15 / 21.0,
            'efficiency_priority': 'wind_efficiency'
        }
    
    def _generate_terrain_avoidance_route(self, start_pos: Tuple[float, float, float],
                                        target_pos: Tuple[float, float, float],
                                        distance: float) -> Optional[Dict]:
        """Generate terrain-avoidance route if needed"""
        # This would analyze terrain data and create a route that avoids obstacles
        # For now, return None (no terrain avoidance needed)
        return None
    
    def _evaluate_route_efficiency(self, route: Dict, efficiency_scores: Dict,
                                 environmental: EnvironmentalConditions,
                                 payload: PayloadCharacteristics,
                                 aerodynamic: AerodynamicState) -> Dict:
        """Evaluate the efficiency of a specific route"""
        # Calculate route-specific efficiency factors
        route_efficiency = self._calculate_route_efficiency(
            route, efficiency_scores, environmental, payload, aerodynamic
        )
        
        # Calculate overall efficiency score
        overall_score = 0.0
        for factor, weight in self.efficiency_weights.items():
            if factor in efficiency_scores:
                overall_score += efficiency_scores[factor] * weight
        
        # Apply route-specific modifiers
        route_modifiers = self._calculate_route_modifiers(route, environmental)
        overall_score *= route_modifiers
        
        return {
            'route': route,
            'efficiency_score': overall_score,
            'route_efficiency': route_efficiency,
            'estimated_energy_consumption': self._estimate_energy_consumption(route),
            'estimated_flight_time': route['estimated_time'],
            'safety_score': self._calculate_safety_score(route),
            'reliability_score': self._calculate_reliability_score(route)
        }
    
    def _calculate_route_efficiency(self, route: Dict, efficiency_scores: Dict,
                                  environmental: EnvironmentalConditions,
                                  payload: PayloadCharacteristics,
                                  aerodynamic: AerodynamicState) -> Dict:
        """Calculate efficiency metrics for a specific route"""
        # Distance efficiency
        distance_efficiency = 1.0 / (1.0 + route['estimated_distance'] * 0.001)
        
        # Time efficiency
        time_efficiency = 1.0 / (1.0 + route['estimated_time'] * 0.1)
        
        # Altitude efficiency
        altitude_efficiency = 1.0
        for waypoint in route['waypoints']:
            if 'alt' in waypoint:
                alt_eff = self._calculate_altitude_efficiency(
                    waypoint['alt'], environmental.air_density
                )
                altitude_efficiency = min(altitude_efficiency, alt_eff)
        
        # Wind efficiency
        wind_efficiency = environmental.get_wind_impact_factor()
        
        return {
            'distance_efficiency': distance_efficiency,
            'time_efficiency': time_efficiency,
            'altitude_efficiency': altitude_efficiency,
            'wind_efficiency': wind_efficiency,
            'overall_route_efficiency': (distance_efficiency + time_efficiency + 
                                       altitude_efficiency + wind_efficiency) / 4.0
        }
    
    def _calculate_route_modifiers(self, route: Dict, 
                                 environmental: EnvironmentalConditions) -> float:
        """Calculate route-specific efficiency modifiers"""
        modifier = 1.0
        
        # Route type modifiers
        if route['type'] == 'direct':
            modifier *= 1.0  # No penalty
        elif route['type'] == 'altitude_optimized':
            modifier *= 0.95  # Slight penalty for complexity
        elif route['type'] == 'wind_optimized':
            modifier *= 0.90  # Penalty for longer route
        elif route['type'] == 'terrain_avoidance':
            modifier *= 0.85  # Penalty for obstacle avoidance
        
        # Environmental modifiers
        if environmental.turbulence > 0.5:
            modifier *= 0.9  # Turbulence reduces efficiency
        
        if environmental.visibility < 1000:
            modifier *= 0.95  # Poor visibility slightly reduces efficiency
        
        return modifier
    
    def _estimate_energy_consumption(self, route: Dict) -> float:
        """Estimate energy consumption for the route in watt-hours"""
        # Simplified energy estimation
        base_power = 50.0  # Watts (typical cruise power)
        flight_time = route['estimated_time']  # hours
        
        # Apply efficiency factors
        efficiency_factor = 0.8  # Typical efficiency
        
        energy_consumption = (base_power * flight_time) / efficiency_factor
        return energy_consumption
    
    def _calculate_safety_score(self, route: Dict) -> float:
        """Calculate safety score for the route (0-1)"""
        safety_score = 1.0
        
        # Penalize routes with many waypoints (more complex)
        if len(route['waypoints']) > 3:
            safety_score *= 0.9
        
        # Penalize very long routes
        if route['estimated_distance'] > 1000:
            safety_score *= 0.85
        
        # Penalize high-altitude routes
        max_alt = max(wp.get('alt', 0) for wp in route['waypoints'])
        if max_alt > 150:
            safety_score *= 0.9
        
        return safety_score
    
    def _calculate_reliability_score(self, route: Dict) -> float:
        """Calculate reliability score for the route (0-1)"""
        reliability_score = 1.0
        
        # Simpler routes are more reliable
        if route['type'] == 'direct':
            reliability_score *= 1.0
        elif route['type'] == 'altitude_optimized':
            reliability_score *= 0.95
        elif route['type'] == 'wind_optimized':
            reliability_score *= 0.9
        elif route['type'] == 'terrain_avoidance':
            reliability_score *= 0.85
        
        return reliability_score
    
    def _select_optimal_route(self, route_evaluations: List[Dict]) -> Dict:
        """Select the optimal route based on efficiency scores"""
        # Sort routes by overall efficiency score
        sorted_routes = sorted(route_evaluations, 
                              key=lambda x: x['efficiency_score'], 
                              reverse=True)
        
        optimal_route = sorted_routes[0]
        
        print(f"✅ Optimal route selected: {optimal_route['route']['type']}")
        print(f"   Efficiency score: {optimal_route['efficiency_score']:.3f}")
        print(f"   Estimated time: {optimal_route['estimated_flight_time']:.1f} hours")
        print(f"   Energy consumption: {optimal_route['estimated_energy_consumption']:.1f} Wh")
        
        return optimal_route
    
    def _generate_flight_plan(self, optimal_route: Dict, 
                             efficiency_scores: Dict) -> Dict:
        """Generate detailed flight plan from optimal route"""
        route = optimal_route['route']
        
        # Calculate optimal flight parameters
        optimal_altitude = self._calculate_optimal_cruise_altitude(
            route['waypoints'][0]['alt'], 
            route['waypoints'][-1]['alt']
        )
        
        optimal_airspeed = self._calculate_optimal_cruise_speed(
            optimal_altitude, efficiency_scores
        )
        
        # Generate waypoint commands
        waypoint_commands = []
        for i, waypoint in enumerate(route['waypoints']):
            command = {
                'waypoint_id': i,
                'position': waypoint,
                'altitude': waypoint.get('alt', optimal_altitude),
                'speed': optimal_airspeed,
                'actions': self._generate_waypoint_actions(i, len(route['waypoints']))
            }
            waypoint_commands.append(command)
        
        flight_plan = {
            'route_type': route['type'],
            'efficiency_score': optimal_route['efficiency_score'],
            'estimated_distance': route['estimated_distance'],
            'estimated_time': route['estimated_time'],
            'estimated_energy': optimal_route['estimated_energy_consumption'],
            'optimal_altitude': optimal_altitude,
            'optimal_airspeed': optimal_airspeed,
            'waypoints': waypoint_commands,
            'efficiency_breakdown': efficiency_scores,
            'route_efficiency': optimal_route['route_efficiency'],
            'safety_score': optimal_route['safety_score'],
            'reliability_score': optimal_route['reliability_score']
        }
        
        return flight_plan
    
    def _calculate_optimal_cruise_altitude(self, start_alt: float, 
                                         target_alt: float) -> float:
        """Calculate optimal cruise altitude for efficiency"""
        # Consider aircraft performance characteristics
        base_altitude = 100.0  # meters (typical efficient altitude)
        
        # Adjust based on start and target altitudes
        if start_alt > 50 or target_alt > 50:
            base_altitude = max(start_alt, target_alt) + 20
        
        # Ensure within aircraft limits
        base_altitude = min(base_altitude, self.aircraft_parameters['service_ceiling'])
        base_altitude = max(base_altitude, 20.0)  # Minimum safe altitude
        
        return base_altitude
    
    def _calculate_optimal_cruise_speed(self, altitude: float, 
                                      efficiency_scores: Dict) -> float:
        """Calculate optimal cruise speed for efficiency"""
        # Base optimal speed
        base_speed = 20.0  # m/s
        
        # Adjust based on altitude (higher altitude = higher speed)
        altitude_factor = 1.0 + (altitude - 100.0) / 1000.0
        base_speed *= altitude_factor
        
        # Adjust based on aerodynamic efficiency
        if FlightEfficiencyFactor.AERODYNAMIC_EFFICIENCY in efficiency_scores:
            aero_factor = efficiency_scores[FlightEfficiencyFactor.AERODYNAMIC_EFFICIENCY]
            base_speed *= (0.8 + aero_factor * 0.4)  # 0.8x to 1.2x range
        
        # Ensure within aircraft limits
        base_speed = min(base_speed, 30.0)  # Max speed
        base_speed = max(base_speed, 15.0)  # Min speed
        
        return base_speed
    
    def _generate_waypoint_actions(self, waypoint_index: int, 
                                 total_waypoints: int) -> List[str]:
        """Generate actions for each waypoint"""
        actions = []
        
        if waypoint_index == 0:
            actions.append('takeoff')
            actions.append('climb_to_cruise')
        elif waypoint_index == total_waypoints - 1:
            actions.append('descend_to_target')
            actions.append('land')
        else:
            actions.append('maintain_cruise')
            actions.append('monitor_efficiency')
        
        return actions
    
    def _calculate_distance(self, lat1: float, lon1: float, 
                          lat2: float, lon2: float) -> float:
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
    
    def _calculate_bearing(self, lat1: float, lon1: float, 
                          lat2: float, lon2: float) -> float:
        """Calculate bearing between two GPS coordinates"""
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        dlon = lon2_rad - lon1_rad
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360

# Example usage and testing
if __name__ == "__main__":
    # Create route planner
    planner = EfficiencyRoutePlanner()
    
    # Create sample data
    battery_state = BatteryState(
        voltage=12.3,
        current=3.2,
        capacity_remaining=2200,
        temperature=22.0,
        charge_cycles=15,
        health_percentage=95.0
    )
    
    thrust_profile = ThrustProfile(
        motor_efficiency=0.85,
        propeller_efficiency=0.75,
        max_thrust=25.0,
        current_thrust=15.0,
        rpm=8500,
        power_consumption=180.0
    )
    
    environmental = EnvironmentalConditions(
        air_density=1.15,
        temperature=18.0,
        pressure=101325,
        humidity=65.0,
        wind_speed=3.0,
        wind_direction=45.0,
        turbulence=0.2,
        visibility=5000
    )
    
    payload = PayloadCharacteristics(
        mass=0.3,
        center_of_gravity=(0.0, 0.0, 0.05),
        drag_coefficient=0.1,
        frontal_area=0.02,
        distribution="balanced"
    )
    
    # Define sample airfoil profiles
    airfoil_profile = AirfoilProfile(
        name="NACA 0012",
        thickness_ratio=0.12,
        camber=0.0,
        max_thickness_position=0.25,
        max_camber_position=0.0,
        cl_alpha=6.28, # 2π radians = 360 degrees
        cl_max=1.2,
        cd_min=0.006,
        reynolds_critical=500000
    )

    # Define sample aircraft geometry
    aircraft_geometry = AircraftGeometry(
        wingspan=1.2,
        wing_area=0.5,
        aspect_ratio=2.88,
        mean_aerodynamic_chord=0.4,
        fuselage_length=0.8,
        fuselage_diameter=0.1,
        tail_area=0.05,
        horizontal_tail_arm=0.5,
        vertical_tail_arm=0.2
    )

    # Define sample aircraft material
    aircraft_material = AircraftMaterial(
        name="Aluminum 6061-T6",
        density=2700,
        tensile_strength=276,
        elastic_modulus=70,
        thermal_expansion=23e-6,
        corrosion_resistance=0.9,
        fatigue_resistance=0.95
    )
    
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
        airfoil_profile=airfoil_profile,
        aircraft_geometry=aircraft_geometry,
        aircraft_material=aircraft_material
    )
    
    # Plan efficient route
    start_position = (37.7849, -122.4094, 0.0)
    target_position = (37.7749, -122.4194, 0.0)
    
    print("🛩️ Testing Advanced Efficiency Route Planner")
    print("=" * 60)
    
    flight_plan = planner.plan_efficient_route(
        start_position, target_position,
        battery_state, thrust_profile, environmental, payload, aerodynamic
    )
    
    print("\n📋 FLIGHT PLAN SUMMARY")
    print("=" * 60)
    print(f"Route Type: {flight_plan['route_type']}")
    print(f"Efficiency Score: {flight_plan['efficiency_score']:.3f}")
    print(f"Estimated Distance: {flight_plan['estimated_distance']:.1f} meters")
    print(f"Estimated Time: {flight_plan['estimated_time']:.2f} hours")
    print(f"Estimated Energy: {flight_plan['estimated_energy']:.1f} Wh")
    print(f"Optimal Altitude: {flight_plan['optimal_altitude']:.1f} meters")
    print(f"Optimal Airspeed: {flight_plan['optimal_airspeed']:.1f} m/s")
    print(f"Safety Score: {flight_plan['safety_score']:.3f}")
    print(f"Reliability Score: {flight_plan['reliability_score']:.3f}")
    
    print(f"\nWaypoints: {len(flight_plan['waypoints'])}")
    for i, wp in enumerate(flight_plan['waypoints']):
        pos = wp['position']
        print(f"  WP{i}: {pos['lat']:.6f}, {pos['lon']:.6f} at {pos['alt']:.1f}m")
        print(f"    Speed: {wp['speed']:.1f} m/s, Actions: {', '.join(wp['actions'])}")
    
    print("\n✅ Efficiency route planning completed!")
