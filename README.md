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
double Kp = 7.0;    // Proportional gain (gentler response)
double Ki = 0.03;   // Integral gain (reduced windup)  
double Kd = 1.2;    // Derivative gain (smoother damping)
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

## Latest Performance Analysis (monitor.log)

### Performance Visualization

![Robot Performance Analysis](monitor_plot.png)

*Real-time performance data showing pitch tracking, error dynamics, PID output correlation, and system recovery from a major disturbance event.*

### Key Metrics & Analysis

- **Source**: monitor.log (Sep 15, ~32,893 data points)
- **Duration**: Extended balancing session with recovery event
- **Comparison windows**: first 1000 samples vs last 1000 samples
- **Error metrics**:
  - Early mean absolute error ≈ 1.69°
  - Latest mean absolute error ≈ 4.45°
  - Overall session: MAE = 5.82°, StdDev = 11.41°, MaxErr = 79.37°
- **Tilt dynamics**:
  - Early mean TiltRate ≈ 4.72°/s
  - Latest mean TiltRate ≈ 0.16°/s
  - Peak disturbance: ~300°/s (major fall/recovery event)
- **Motor behavior**:
  - Early MotorSpeed StdDev ≈ 24.07 (highly variable, active balancing)
  - Latest MotorSpeed mean ≈ 33.9 with StdDev ≈ 3.01 (steady forward compensation)
  - Excellent PID-to-motor correlation throughout session

### Key Observations

**✅ System Strengths:**
- Successfully recovered from catastrophic fall (sample ~15,000)
- Maintained stable operation for 20,000+ samples post-recovery
- PID controller shows excellent response characteristics
- Motor commands perfectly correlate with control output

**⚠️ Areas for Improvement:**
- Persistent 5° steady-state error after recovery (calibration issue)
- No automatic recalibration after major disturbances
- Suggest increasing Ki slightly (0.03 → 0.05) for better steady-state performance

**Interpretation**: This session demonstrates robust system recovery capabilities. The robot successfully survived and recovered from a major fall, resuming stable operation with a persistent offset. This indicates the current control strategy is fundamentally sound but could benefit from automatic recalibration triggers after large disturbances.

Important: In contrast, the dedicated balancing session logs (for example logs/balancing_robot_20250915_003442.log) show excellent performance around a setpoint of ~180.22° with small steady-state error (typically ±0.20° to ±0.45°) and smooth, well-damped motor activity. This confirms the current control strategy and gains are capable of tight balance when calibration and conditions are nominal.

Current PID settings (see src/main.cpp): Kp = 7.0, Ki = 0.03, Kd = 1.2 (gentler response)

Recommendations based on current settings:
- Keep the above gains for general operation; they produce smooth, stable balance in the dedicated logs.
- If a future run shows a persistent ~4–5° offset like in this monitor.log:
  - Recalibrate the IMU at startup and ensure the robot is held upright during the 20 reading calibration window
  - Check battery voltage and mechanical friction/stiction
  - Optionally raise Ki slightly (e.g., Ki = 0.08) to reduce steady-state error; keep Kp, Kd unchanged
  - Verify setpoint auto-calibration is completing (you should see "Calibrated setpoint:" in the serial log)

### Performance Visualization Tool

A Python plotting script (`plot_monitor.py`) is included to visualize log data:

```bash
# Install dependencies (Ubuntu/Debian)
sudo apt install python3-matplotlib python3-numpy

# Plot monitor.log data
python3 plot_monitor.py monitor.log

# Plot any log file
python3 plot_monitor.py logs/balancing_robot_20250915_003442.log
```

The script generates comprehensive performance charts showing:
- **Pitch vs Setpoint tracking**
- **Error dynamics over time** 
- **PID output vs Motor speed correlation**
- **Tilt rate (system dynamics)**
- **Statistical summary** (MAE, StdDev, MaxErr)

How the above stats were computed (illustrative one-liners):
```bash path=null start=null
# Mean absolute error (first and last 1000 samples)
grep -o "Error: [0-9.-]*" monitor.log | head -n 1000 | sed 's/Error: //' | awk '{s+=($1<0?-$1:$1)} END{print s/NR}'
grep -o "Error: [0-9.-]*" monitor.log | tail -n 1000 | sed 's/Error: //' | awk '{s+=($1<0?-$1:$1)} END{print s/NR}'

# TiltRate stats
grep -o "TiltRate: [0-9.-]*" monitor.log | head -n 1000 | sed 's/TiltRate: //' | awk '{s+=$1; q+=$1*$1} END{print s/NR, sqrt(q/NR-(s/NR)^2)}'
grep -o "TiltRate: [0-9.-]*" monitor.log | tail -n 1000 | sed 's/TiltRate: //' | awk '{s+=$1; q+=$1*$1} END{print s/NR, sqrt(q/NR-(s/NR)^2)}'
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
