# PlzStandUp Robot Firmware

**📸 [View Robot Photos & Videos](https://photos.app.goo.gl/qP713SWyDZ1aFKTv6)**

A self-balancing robot firmware project built with Arduino framework and PlatformIO for the Arduino UNO R4 Minima.

## Overview

PlzStandUp is a two-wheeled self-balancing robot that uses an MPU6050 gyroscope/accelerometer sensor for orientation feedback and a PID controller to maintain balance. The robot automatically calibrates its balance point on startup and uses sophisticated motor control algorithms to stay upright.

## Hardware Requirements

### Microcontroller
- **Arduino UNO R4 Minima** (Renesas RA platform)

### Sensors
- **MPU6050** - 6-axis gyroscope/accelerometer for orientation sensing
- Connected to I2C pins (SDA/SCL)
- Interrupt pin connected to digital pin 2

### Motors & Control
- **Dual DC Motors** with H-bridge motor driver
- Motor control pins:
  - Left motor: PWM (pin 6), DIR1 (pin 8), DIR2 (pin 7)
  - Right motor: PWM (pin 5), DIR1 (pin 3), DIR2 (pin 4)
- Speed encoder inputs:
  - Left: INT (pin 10), PULSE (pin 9)
  - Right: INT (pin 11), PULSE (pin 12)

### Additional Components
- **Buzzer** (pin 13) - Audio feedback
- **LED** (pin 13) - Status indicator

## Features

- **Automatic Calibration**: Self-calibrates balance point on startup (20 readings)
- **PID Control**: Tuned PID controller with anti-windup and rate limiting
- **Fast Recovery**: Detects rapid tilt changes and applies recovery forces
- **Smooth Motor Control**: Rate-limited output changes prevent violent oscillations
- **Fall Detection**: Stops motors when tilt exceeds safe limits (±60°)
- **Debug Output**: Comprehensive serial monitoring of sensor data and control outputs
- **Motor Direction Reversing**: Easy configuration for different motor orientations

## Installation

### Prerequisites
- [PlatformIO](https://platformio.org/) installed
- Arduino UNO R4 Minima

### Setup
1. Clone this repository:
   ```bash
   git clone https://github.com/kenken64/plzstandupBot.git
   cd plzstandupBot/firmware
   ```

2. Install dependencies:
   ```bash
   pio lib install
   ```

3. Build the project:
   ```bash
   pio run
   ```

4. Upload to your Arduino:
   ```bash
   pio run -t upload
   ```

5. Monitor serial output:
   ```bash
   pio device monitor
   ```

## Configuration

### PID Tuning
The PID controller can be tuned by modifying these parameters in `main.cpp`:

```cpp
double Kp = 8.0;    // Proportional gain
double Ki = 0.05;   // Integral gain  
double Kd = 1.5;    // Derivative gain
```

### Motor Settings
- `motorSpeedFactor`: Adjust for motor differences (default: 1)
- `minStartSpeed`: Minimum speed to overcome motor stiction (default: 30)
- `maxNormalSpeed`: Maximum speed during normal operation (default: 60)
- `fallLimit`: Maximum tilt angle before stopping (default: 60°)

### Sensor Calibration
MPU6050 offsets are pre-configured but can be adjusted:

```cpp
mpu.setXGyroOffset(30);
mpu.setYGyroOffset(-32);
mpu.setZGyroOffset(47);
mpu.setZAccelOffset(1170);
```

## Usage

1. **Power on** the robot and place it upright
2. **Calibration phase**: The robot will automatically calibrate for ~20 readings
3. **Balancing mode**: Robot will attempt to maintain balance using PID control
4. **Monitor**: Use serial monitor (115200 baud) to view real-time debug information

### Serial Output
The robot outputs detailed telemetry:
```
Pitch: 179.85 | Setpoint: 180.12 | Error: -0.27 | Output: -15.23 | MOTOR_CMD: BACKWARD 30
```

## Project Structure

```
firmware/
├── src/
│   └── main.cpp          # Main firmware code
├── include/              # Header files
├── lib/                  # Project libraries
├── test/                 # Unit tests
├── logs/                 # Test session logs
├── platformio.ini        # PlatformIO configuration
└── README.md            # This file
```

## Dependencies

- **MPU6050 Library**: electroniccats/MPU6050@^1.4.4
- **PID Controller**: br3ttb/PID@^1.2.0
- **Arduino Framework**: Built-in I2C, serial communication

## Troubleshooting

### Robot moves in wrong direction
- Check motor wiring connections
- Uncomment the motor direction reversal code in `setMotorSpeeds()`

### Oscillation/instability
- Reduce `Kp` gain for less aggressive response
- Increase `Kd` gain for better damping
- Check mechanical balance and weight distribution

### DMP initialization failed
- Check MPU6050 wiring and I2C connections
- Verify power supply voltage (3.3V or 5V depending on module)

### Robot won't start balancing
- Ensure robot is placed upright during calibration
- Check that motors are connected and functional
- Verify motor driver power supply

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is open source. Please check the repository for license details.

## Author

Kenneth Phang (@kenken64)

## Acknowledgments

- MPU6050 library by ElectronicCats
- PID library by Brett Beauregard
- Arduino and PlatformIO communities
