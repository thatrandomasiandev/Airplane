# 🛩️ Autonomous Aircraft Flight Controller

**Advanced autonomous aircraft with precision GPS navigation and parachute landing**

A sophisticated autonomous flight control system that can take off, navigate to predetermined GPS coordinates, analyze wind conditions, and deploy a parachute at precisely the right moment to land exactly on target.

## 🚀 Features

### **Core Capabilities**
- **Autonomous Takeoff** - Fully automated takeoff sequence with safety checks
- **GPS Navigation** - Precise waypoint navigation with route optimization
- **Wind Analysis** - Real-time wind speed and direction calculation
- **Precision Landing** - Calculated parachute deployment for target accuracy
- **Safety Systems** - Comprehensive safety monitoring and emergency procedures

### **Flight Phases**
1. **Ground** - Preflight checks and system initialization
2. **Takeoff** - Autonomous acceleration and climb
3. **Climb** - Altitude gain to cruise level
4. **Cruise** - Waypoint navigation and route following
5. **Approach** - Final approach and landing preparation
6. **Parachute Deploy** - Precision deployment calculation
7. **Descent** - Controlled parachute descent
8. **Landed** - Mission completion

### **Safety Features**
- Altitude limits and monitoring
- Battery voltage monitoring
- Maximum flight time limits
- Emergency parachute deployment
- Control surface testing
- GPS signal validation

## 🏗️ System Architecture

### **Software Components**
```
autonomous_flight_controller.py  # Core flight logic and navigation
hardware_interface.py            # Sensor reading and actuator control
main_flight_controller.py       # Mission execution and integration
```

### **Hardware Requirements**
- **Flight Computer** - Raspberry Pi 4 or similar
- **GPS Module** - U-blox NEO-8M or similar
- **IMU Sensor** - MPU6050 or similar (accelerometer + gyroscope)
- **Barometer** - BMP280 or similar (altitude + pressure)
- **Airspeed Sensor** - Pitot tube with differential pressure sensor
- **Servos** - Standard RC aircraft servos (aileron, elevator, rudder)
- **ESC** - Electronic speed controller for motor
- **Parachute System** - Deployable parachute with servo release
- **Power System** - 3S LiPo battery with voltage monitoring

### **Communication Interfaces**
- **UART** - GPS and IMU communication
- **I2C** - Barometer and additional sensors
- **PWM** - Servo and ESC control
- **GPIO** - Digital inputs/outputs for status

## 📋 Prerequisites

### **Hardware Setup**
1. **Aircraft Frame** - Fixed-wing RC aircraft (recommended: high-wing trainer)
2. **Flight Computer** - Raspberry Pi 4 with SD card
3. **Sensors** - GPS, IMU, barometer, airspeed
4. **Actuators** - Servos, motor, ESC, parachute release
5. **Power Distribution** - Battery, voltage regulator, power distribution board

### **Software Requirements**
- Python 3.8+
- Required Python packages (see requirements.txt)
- Linux-based operating system (Raspberry Pi OS recommended)

## 🛠️ Installation

### **1. Clone the Repository**
```bash
git clone <repository-url>
cd autonomous-aircraft
```

### **2. Install Dependencies**
```bash
pip install -r requirements.txt
```

### **3. Hardware Configuration**
```bash
# Configure serial ports for your hardware
# Edit hardware_interface.py with correct port assignments
```

### **4. GPS Configuration**
```bash
# Ensure GPS module is properly connected and configured
# Test GPS signal acquisition
```

## 🎯 Usage

### **Basic Mission Setup**

```python
from main_flight_controller import MainFlightController

# Create flight controller
controller = MainFlightController()

# Set mission parameters
target_lat = 37.7749    # Target latitude
target_lon = -122.4194  # Target longitude
start_lat = 37.7849     # Start latitude
start_lon = -122.4094   # Start longitude

controller.set_mission_parameters(target_lat, target_lon, start_lat, start_lon)

# Start mission
success = controller.start_mission()
```

### **Command Line Execution**
```bash
# Run the main flight controller
python main_flight_controller.py

# Run individual components for testing
python autonomous_flight_controller.py
python hardware_interface.py
```

### **Configuration Files**
Create configuration files for different aircraft types and mission profiles:

```yaml
# aircraft_config.yaml
aircraft:
  wing_area: 0.5          # m²
  mass: 2.0               # kg
  max_airspeed: 25.0      # m/s
  cruise_altitude: 100.0  # meters

flight_control:
  takeoff_speed: 15.0     # m/s
  climb_rate: 3.0         # m/s
  turn_radius: 50.0       # meters

safety:
  max_altitude: 150.0     # meters
  min_battery_voltage: 10.5  # volts
  max_flight_time: 1800   # seconds
```

## 🔧 Hardware Integration

### **Sensor Connections**

#### **GPS Module (UART)**
```python
# Connect to /dev/ttyUSB0 or /dev/ttyAMA0
# Baud rate: 9600
# NMEA protocol
```

#### **IMU Sensor (I2C)**
```python
# MPU6050 or similar
# I2C address: 0x68
# Provides: accelerometer, gyroscope, temperature
```

#### **Barometer (I2C)**
```python
# BMP280 or similar
# I2C address: 0x76 or 0x77
# Provides: pressure, temperature, altitude
```

### **Actuator Control**

#### **Servo Control**
```python
# Standard PWM servo control
# Frequency: 50Hz
# Pulse width: 1-2ms (0.5-2.5ms for extended range)
```

#### **Motor Control**
```python
# ESC control via PWM
# Frequency: 50Hz
# Pulse width: 1-2ms (idle to full throttle)
```

## 📊 Flight Data and Logging

### **Telemetry Data**
The system logs comprehensive flight data including:
- GPS position and altitude
- Aircraft attitude (pitch, roll, yaw)
- Airspeed and groundspeed
- Wind conditions
- Control surface positions
- Battery status
- Flight phase transitions

### **Data Export**
```python
# Save flight log to JSON
controller.flight_controller.save_telemetry_log("flight_log.json")

# Analyze flight data
import json
with open("flight_log.json", "r") as f:
    data = json.load(f)
```

## 🧪 Testing and Simulation

### **Hardware-in-the-Loop Testing**
```bash
# Test individual components
python -m pytest tests/

# Run hardware tests
python hardware_interface.py
```

### **Flight Simulation**
```python
# Use simulation mode for testing
# Disable actual hardware control
SIMULATION_MODE = True
```

## 🚨 Safety and Emergency Procedures

### **Emergency Landing**
The system automatically initiates emergency procedures when:
- Altitude limits exceeded
- Low battery voltage
- Maximum flight time exceeded
- System errors detected

### **Manual Override**
```python
# Emergency stop
controller.emergency_landing()

# Manual control override
controller.hardware.emergency_stop()
```

### **Fail-Safe Systems**
- **Parachute Deployment** - Automatic deployment on system failure
- **Engine Cut** - Immediate throttle reduction on safety violation
- **Control Neutralization** - All control surfaces returned to neutral
- **Emergency Beacon** - Activation of emergency tracking system

## 📈 Performance and Accuracy

### **Navigation Accuracy**
- **GPS Position**: ±3-5 meters
- **Altitude**: ±2-3 meters
- **Heading**: ±2-3 degrees
- **Landing Accuracy**: ±5-10 meters (with wind compensation)

### **Wind Compensation**
The system analyzes wind conditions and compensates for:
- Wind drift during flight
- Wind effects on parachute descent
- Optimal deployment timing
- Landing position prediction

### **Flight Performance**
- **Takeoff Distance**: 50-100 meters
- **Climb Rate**: 3-5 m/s
- **Cruise Speed**: 20-25 m/s
- **Descent Rate**: 2-3 m/s (parachute)
- **Maximum Range**: 5-10 km (depending on battery)

## 🔍 Troubleshooting

### **Common Issues**

#### **GPS Signal Problems**
```bash
# Check GPS connection
ls /dev/ttyUSB*
# Test GPS data
python -c "import serial; s=serial.Serial('/dev/ttyUSB0',9600); print(s.readline())"
```

#### **Sensor Calibration Issues**
```python
# Recalibrate IMU sensors
controller.hardware.calibrate_sensors()

# Check sensor readings
sensor_data = controller.hardware.get_sensor_data()
print(f"Accel: {sensor_data.imu_accel_x:.2f}, {sensor_data.imu_accel_y:.2f}, {sensor_data.imu_accel_z:.2f}")
```

#### **Control Surface Problems**
```python
# Test control surfaces
test_commands = ActuatorCommands()
test_commands.aileron = 0.2
controller.hardware.send_actuator_commands(test_commands)
```

### **Debug Mode**
```python
# Enable debug logging
import logging
logging.basicConfig(level=logging.DEBUG)

# Check system status
status = controller.get_flight_status()
print(json.dumps(status, indent=2))
```

## 📚 Advanced Features

### **Custom Flight Profiles**
```python
# Create custom flight patterns
class CustomFlightProfile:
    def __init__(self):
        self.waypoints = []
        self.flight_modes = []
    
    def add_waypoint(self, lat, lon, alt, speed):
        self.waypoints.append({
            'position': GPSPosition(lat, lon, alt),
            'speed': speed,
            'actions': []
        })
```

### **Wind Pattern Analysis**
```python
# Advanced wind analysis
def analyze_wind_patterns(wind_data):
    # Analyze wind patterns at different altitudes
    # Predict wind changes during flight
    # Optimize route for wind conditions
    pass
```

### **Machine Learning Integration**
```python
# ML-based flight optimization
from sklearn.ensemble import RandomForestRegressor

class MLFlightOptimizer:
    def __init__(self):
        self.model = RandomForestRegressor()
    
    def optimize_landing_parameters(self, flight_data):
        # Use ML to optimize parachute deployment
        pass
```

## 🤝 Contributing

### **Development Setup**
```bash
# Install development dependencies
pip install -r requirements-dev.txt

# Run tests
pytest

# Code formatting
black .
flake8 .
```

### **Code Standards**
- Follow PEP 8 style guidelines
- Add type hints for all functions
- Include comprehensive docstrings
- Write unit tests for new features

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## ⚠️ Disclaimer

**This software is for educational and research purposes only. Autonomous aircraft operation may be subject to local aviation regulations and restrictions. Always ensure compliance with applicable laws and safety requirements before operating autonomous aircraft.**

## 🆘 Support

For technical support and questions:
- Check the troubleshooting section
- Review the code documentation
- Open an issue on GitHub
- Contact the development team

---

**Happy Flying! 🛩️✈️**

*Built with ❤️ for autonomous aviation research and development*
