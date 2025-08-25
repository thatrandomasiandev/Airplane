#!/usr/bin/env python3
"""
Hardware Interface for Autonomous Aircraft
Handles sensor reading, actuator control, and communication
"""

import time
import serial
import struct
from typing import Dict, List, Optional
import threading
import queue

class SensorData:
    def __init__(self):
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        self.gps_alt = 0.0
        self.gps_speed = 0.0
        self.gps_heading = 0.0
        
        self.imu_accel_x = 0.0
        self.imu_accel_y = 0.0
        self.imu_accel_z = 0.0
        
        self.imu_gyro_x = 0.0
        self.imu_gyro_y = 0.0
        self.imu_gyro_z = 0.0
        
        self.imu_mag_x = 0.0
        self.imu_mag_y = 0.0
        self.imu_mag_z = 0.0
        
        self.barometer_pressure = 0.0
        self.barometer_altitude = 0.0
        self.barometer_temperature = 0.0
        
        self.airspeed = 0.0
        self.voltage = 0.0
        self.current = 0.0
        
        self.timestamp = time.time()

class ActuatorCommands:
    def __init__(self):
        self.aileron = 0.0      # -1.0 to 1.0 (left to right)
        self.elevator = 0.0     # -1.0 to 1.0 (down to up)
        self.rudder = 0.0       # -1.0 to 1.0 (left to right)
        self.throttle = 0.0     # 0.0 to 1.0 (idle to full)
        self.parachute = False  # True to deploy
        self.landing_gear = False  # True to deploy
        self.flaps = 0.0        # 0.0 to 1.0 (retracted to fully extended)

class HardwareInterface:
    def __init__(self):
        self.sensors = SensorData()
        self.actuators = ActuatorCommands()
        
        # Communication queues
        self.sensor_queue = queue.Queue()
        self.actuator_queue = queue.Queue()
        
        # Serial connections
        self.gps_serial = None
        self.imu_serial = None
        self.actuator_serial = None
        
        # Threading
        self.running = False
        self.sensor_thread = None
        self.actuator_thread = None
        
        # Calibration data
        self.accel_bias = [0.0, 0.0, 0.0]
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.mag_bias = [0.0, 0.0, 0.0]
        
        # Configuration
        self.update_rate = 100  # Hz
        self.serial_timeout = 1.0
        
    def initialize_hardware(self):
        """Initialize all hardware connections"""
        print("Initializing hardware interfaces...")
        
        try:
            # Initialize GPS (UART)
            self.gps_serial = serial.Serial(
                port='/dev/ttyUSB0',  # Adjust port as needed
                baudrate=9600,
                timeout=self.serial_timeout
            )
            print("✓ GPS interface initialized")
            
        except Exception as e:
            print(f"✗ GPS initialization failed: {e}")
            self.gps_serial = None
        
        try:
            # Initialize IMU (I2C or UART)
            self.imu_serial = serial.Serial(
                port='/dev/ttyUSB1',  # Adjust port as needed
                baudrate=115200,
                timeout=self.serial_timeout
            )
            print("✓ IMU interface initialized")
            
        except Exception as e:
            print(f"✗ IMU initialization failed: {e}")
            self.imu_serial = None
        
        try:
            # Initialize actuator controller
            self.actuator_serial = serial.Serial(
                port='/dev/ttyUSB2',  # Adjust port as needed
                baudrate=115200,
                timeout=self.serial_timeout
            )
            print("✓ Actuator interface initialized")
            
        except Exception as e:
            print(f"✗ Actuator initialization failed: {e}")
            self.actuator_serial = None
        
        # Start sensor and actuator threads
        self.start_threads()
        
        return True
    
    def start_threads(self):
        """Start sensor reading and actuator control threads"""
        self.running = True
        
        # Sensor reading thread
        self.sensor_thread = threading.Thread(target=self._sensor_loop, daemon=True)
        self.sensor_thread.start()
        
        # Actuator control thread
        self.actuator_thread = threading.Thread(target=self._actuator_loop, daemon=True)
        self.actuator_thread.start()
        
        print("✓ Hardware threads started")
    
    def stop_threads(self):
        """Stop all hardware threads"""
        self.running = False
        
        if self.sensor_thread:
            self.sensor_thread.join(timeout=2.0)
        if self.actuator_thread:
            self.actuator_thread.join(timeout=2.0)
        
        print("✓ Hardware threads stopped")
    
    def _sensor_loop(self):
        """Main sensor reading loop"""
        while self.running:
            try:
                # Read GPS data
                if self.gps_serial:
                    self._read_gps()
                
                # Read IMU data
                if self.imu_serial:
                    self._read_imu()
                
                # Read other sensors
                self._read_barometer()
                self._read_airspeed()
                self._read_power()
                
                # Update timestamp
                self.sensors.timestamp = time.time()
                
                # Put data in queue for main thread
                self.sensor_queue.put(self.sensors)
                
                # Sleep to maintain update rate
                time.sleep(1.0 / self.update_rate)
                
            except Exception as e:
                print(f"Sensor loop error: {e}")
                time.sleep(0.1)
    
    def _actuator_loop(self):
        """Main actuator control loop"""
        while self.running:
            try:
                # Check for new actuator commands
                if not self.actuator_queue.empty():
                    new_commands = self.actuator_queue.get_nowait()
                    self._apply_actuator_commands(new_commands)
                
                # Send current actuator state
                if self.actuator_serial:
                    self._send_actuator_state()
                
                time.sleep(1.0 / self.update_rate)
                
            except Exception as e:
                print(f"Actuator loop error: {e}")
                time.sleep(0.1)
    
    def _read_gps(self):
        """Read GPS data from serial"""
        if not self.gps_serial or not self.gps_serial.in_waiting:
            return
        
        try:
            # Read GPS line (NMEA format)
            line = self.gps_serial.readline().decode('ascii').strip()
            
            if line.startswith('$GPRMC'):
                # Parse RMC sentence
                parts = line.split(',')
                if len(parts) >= 12 and parts[2] == 'A':  # Valid fix
                    # Extract latitude
                    lat_raw = float(parts[3])
                    lat_dir = parts[4]
                    lat_deg = int(lat_raw / 100)
                    lat_min = lat_raw - (lat_deg * 100)
                    self.sensors.gps_lat = lat_deg + (lat_min / 60.0)
                    if lat_dir == 'S':
                        self.sensors.gps_lat = -self.sensors.gps_lat
                    
                    # Extract longitude
                    lon_raw = float(parts[5])
                    lon_dir = parts[6]
                    lon_deg = int(lon_raw / 100)
                    lon_min = lon_raw - (lon_deg * 100)
                    self.sensors.gps_lon = lon_deg + (lon_min / 60.0)
                    if lon_dir == 'W':
                        self.sensors.gps_lon = -self.sensors.gps_lon
                    
                    # Extract speed and heading
                    self.sensors.gps_speed = float(parts[7]) * 0.514  # Convert knots to m/s
                    self.sensors.gps_heading = float(parts[8])
            
            elif line.startswith('$GPGGA'):
                # Parse GGA sentence for altitude
                parts = line.split(',')
                if len(parts) >= 15 and parts[6] != '':
                    self.sensors.gps_alt = float(parts[9])
                    
        except Exception as e:
            print(f"GPS parsing error: {e}")
    
    def _read_imu(self):
        """Read IMU data from serial"""
        if not self.imu_serial or not self.imu_serial.in_waiting:
            return
        
        try:
            # Read IMU data packet
            if self.imu_serial.in_waiting >= 24:  # 6 floats * 4 bytes
                data = self.imu_serial.read(24)
                
                # Unpack accelerometer data (3 floats)
                accel_x, accel_y, accel_z = struct.unpack('fff', data[:12])
                
                # Apply calibration and convert to m/s²
                self.sensors.imu_accel_x = accel_x - self.accel_bias[0]
                self.sensors.imu_accel_y = accel_y - self.accel_bias[1]
                self.sensors.imu_accel_z = accel_z - self.accel_bias[2]
                
                # Unpack gyroscope data (3 floats)
                gyro_x, gyro_y, gyro_z = struct.unpack('fff', data[12:24])
                
                # Apply calibration and convert to rad/s
                self.sensors.imu_gyro_x = gyro_x - self.gyro_bias[0]
                self.sensors.imu_gyro_y = gyro_y - self.gyro_bias[1]
                self.sensors.imu_gyro_z = gyro_z - self.gyro_bias[2]
                
        except Exception as e:
            print(f"IMU parsing error: {e}")
    
    def _read_barometer(self):
        """Read barometric pressure sensor"""
        # Simulate barometer reading (replace with actual sensor code)
        # This would typically read from I2C or SPI sensor
        try:
            # Simulated values for testing
            self.sensors.barometer_pressure = 101325.0  # Pa (sea level)
            self.sensors.barometer_temperature = 20.0   # °C
            
            # Calculate altitude from pressure
            # Using standard atmosphere model
            p0 = 101325.0  # Sea level pressure
            T0 = 288.15    # Sea level temperature (K)
            L = -0.0065    # Temperature lapse rate (K/m)
            g = 9.80665    # Gravitational acceleration (m/s²)
            R = 287.05     # Specific gas constant for air (J/kg·K)
            
            if self.sensors.barometer_pressure > 0:
                self.sensors.barometer_altitude = T0/L * (1 - (self.sensors.barometer_pressure/p0)**(L*R/g))
            else:
                self.sensors.barometer_altitude = 0.0
                
        except Exception as e:
            print(f"Barometer error: {e}")
    
    def _read_airspeed(self):
        """Read airspeed sensor"""
        # Simulate airspeed reading (replace with actual sensor code)
        # This would typically read from differential pressure sensor
        try:
            # Calculate airspeed from pitot tube pressure difference
            # For now, simulate based on throttle and altitude
            base_speed = self.actuators.throttle * 25.0  # Max 25 m/s
            altitude_factor = 1.0 - (self.sensors.barometer_altitude / 1000.0) * 0.1
            self.sensors.airspeed = base_speed * altitude_factor
            
        except Exception as e:
            print(f"Airspeed error: {e}")
    
    def _read_power(self):
        """Read power system sensors"""
        # Simulate power readings (replace with actual sensor code)
        try:
            # Simulated battery voltage and current
            self.sensors.voltage = 12.6  # V (3S LiPo)
            self.sensors.current = 2.5   # A
            
        except Exception as e:
            print(f"Power sensor error: {e}")
    
    def _apply_actuator_commands(self, commands: ActuatorCommands):
        """Apply actuator commands to hardware"""
        # Update internal state
        self.actuators.aileron = commands.aileron
        self.actuators.elevator = commands.elevator
        self.actuators.rudder = commands.rudder
        self.actuators.throttle = commands.throttle
        self.actuators.parachute = commands.parachute
        self.actuators.landing_gear = commands.landing_gear
        self.actuators.flaps = commands.flaps
        
        # Send commands to hardware
        if self.actuator_serial:
            self._send_actuator_commands(commands)
    
    def _send_actuator_commands(self, commands: ActuatorCommands):
        """Send actuator commands to hardware controller"""
        try:
            # Create command packet
            packet = struct.pack('ffffffBB', 
                commands.aileron,
                commands.elevator,
                commands.rudder,
                commands.throttle,
                commands.flaps,
                0.0,  # Reserved
                commands.parachute,
                commands.landing_gear
            )
            
            # Send packet
            self.actuator_serial.write(packet)
            self.actuator_serial.flush()
            
        except Exception as e:
            print(f"Actuator command error: {e}")
    
    def _send_actuator_state(self):
        """Send current actuator state for monitoring"""
        try:
            # Create state packet
            packet = struct.pack('ffffffBB', 
                self.actuators.aileron,
                self.actuators.elevator,
                self.actuators.rudder,
                self.actuators.throttle,
                self.actuators.flaps,
                0.0,  # Reserved
                self.actuators.parachute,
                self.actuators.landing_gear
            )
            
            # Send packet
            self.actuator_serial.write(packet)
            self.actuator_serial.flush()
            
        except Exception as e:
            print(f"Actuator state error: {e}")
    
    def get_sensor_data(self) -> Optional[SensorData]:
        """Get latest sensor data from queue"""
        try:
            return self.sensor_queue.get_nowait()
        except queue.Empty:
            return None
    
    def send_actuator_commands(self, commands: ActuatorCommands):
        """Send actuator commands to hardware"""
        self.actuator_queue.put(commands)
    
    def calibrate_sensors(self):
        """Calibrate IMU sensors"""
        print("Calibrating IMU sensors...")
        print("Keep the aircraft still and level...")
        
        # Collect calibration data
        accel_samples = []
        gyro_samples = []
        mag_samples = []
        
        sample_count = 100
        for i in range(sample_count):
            if self.sensor_queue.qsize() > 0:
                data = self.sensor_queue.get()
                accel_samples.append([data.imu_accel_x, data.imu_accel_y, data.imu_accel_z])
                gyro_samples.append([data.imu_gyro_x, data.imu_gyro_y, data.imu_gyro_z])
                mag_samples.append([data.imu_mag_x, data.imu_mag_y, data.imu_mag_z])
            
            time.sleep(0.01)
            if i % 20 == 0:
                print(f"Calibration progress: {i/sample_count*100:.0f}%")
        
        # Calculate biases
        if accel_samples:
            self.accel_bias = [
                sum(sample[0] for sample in accel_samples) / len(accel_samples),
                sum(sample[1] for sample in accel_samples) / len(accel_samples),
                sum(sample[2] for sample in accel_samples) / len(accel_samples) - 9.81  # Gravity correction
            ]
        
        if gyro_samples:
            self.gyro_bias = [
                sum(sample[0] for sample in gyro_samples) / len(gyro_samples),
                sum(sample[1] for sample in gyro_samples) / len(gyro_samples),
                sum(sample[2] for sample in gyro_samples) / len(gyro_samples)
            ]
        
        if mag_samples:
            self.mag_bias = [
                sum(sample[0] for sample in mag_samples) / len(mag_samples),
                sum(sample[1] for sample in mag_samples) / len(mag_samples),
                sum(sample[2] for sample in mag_samples) / len(mag_samples)
            ]
        
        print("✓ IMU calibration complete")
        print(f"  Accel bias: {self.accel_bias}")
        print(f"  Gyro bias: {self.gyro_bias}")
        print(f"  Mag bias: {self.mag_bias}")
    
    def emergency_stop(self):
        """Emergency stop - neutralize all controls"""
        emergency_commands = ActuatorCommands()
        emergency_commands.aileron = 0.0
        emergency_commands.elevator = 0.0
        emergency_commands.rudder = 0.0
        emergency_commands.throttle = 0.0
        emergency_commands.parachute = True  # Deploy parachute
        emergency_commands.landing_gear = True
        emergency_commands.flaps = 0.0
        
        self.send_actuator_commands(emergency_commands)
        print("🚨 EMERGENCY STOP - Controls neutralized, parachute deployed")
    
    def cleanup(self):
        """Clean up hardware connections"""
        self.stop_threads()
        
        if self.gps_serial:
            self.gps_serial.close()
        if self.imu_serial:
            self.imu_serial.close()
        if self.actuator_serial:
            self.actuator_serial.close()
        
        print("✓ Hardware interfaces cleaned up")

# Example usage
if __name__ == "__main__":
    # Initialize hardware interface
    hw = HardwareInterface()
    
    try:
        # Initialize hardware
        if hw.initialize_hardware():
            print("Hardware interface ready!")
            
            # Calibrate sensors
            hw.calibrate_sensors()
            
            # Test sensor reading
            print("\nTesting sensor reading...")
            for i in range(10):
                data = hw.get_sensor_data()
                if data:
                    print(f"GPS: {data.gps_lat:.6f}, {data.gps_lon:.6f}, Alt: {data.gps_alt:.1f}m")
                    print(f"Accel: {data.imu_accel_x:.2f}, {data.imu_accel_y:.2f}, {data.imu_accel_z:.2f} m/s²")
                    print(f"Gyro: {data.imu_gyro_x:.2f}, {data.imu_gyro_y:.2f}, {data.imu_gyro_z:.2f} rad/s")
                    print(f"Airspeed: {data.airspeed:.1f} m/s, Voltage: {data.voltage:.1f}V")
                    print("---")
                
                time.sleep(0.1)
            
            # Test actuator commands
            print("\nTesting actuator commands...")
            test_commands = ActuatorCommands()
            test_commands.aileron = 0.2
            test_commands.elevator = -0.1
            test_commands.throttle = 0.5
            
            hw.send_actuator_commands(test_commands)
            time.sleep(1.0)
            
            # Emergency stop test
            print("\nTesting emergency stop...")
            hw.emergency_stop()
            time.sleep(2.0)
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        hw.cleanup()
        print("Hardware interface shutdown complete")
