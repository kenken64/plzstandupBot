# Bluetooth Setup and Usage Guide

## Hardware Connections

### HC-05 Bluetooth Module Wiring
Connect the HC-05 module to your Arduino as follows:

```
HC-05 Module    Arduino Pin
-----------     -----------
VCC             5V or 3.3V
GND             GND
TXD             A0 (BT_RX_PIN)
RXD             A1 (BT_TX_PIN)
EN/KEY          Not used
STATE           Not used
```

### Optional: Bluetooth Disable Pin
- Connect pin A2 to ground to disable Bluetooth during uploads
- Leave A2 floating (unconnected) for normal Bluetooth operation

## Software Setup

### Arduino Code Features
The firmware now includes:
- SoftwareSerial Bluetooth communication
- Command processing from Python scripts
- Status reporting back to Python
- Remote control capabilities
- PID parameter tuning via Bluetooth
- Motor control commands

### Supported Commands
- `STATUS` - Get current robot status
- `FORWARD` - Move robot forward
- `BACKWARD` - Move robot backward
- `STOP` - Stop all motors
- `SPEED:xxx` - Set speed (0-255)
- `PID:kp,ki,kd` - Update PID parameters
- `CALIBRATE` - Recalibrate balance setpoint

## Python Script Usage

### Prerequisites
```bash
pip install pyserial
```

### Pairing the HC-05 Module

1. **Put HC-05 in pairing mode** (usually automatic)
2. **Find the module** from your computer:
   ```bash
   bluetoothctl
   scan on
   # Look for HC-05 or similar device
   # Note the MAC address (e.g., 98:D3:31:FB:4F:9E)
   ```

3. **Pair the device**:
   ```bash
   bluetoothctl
   pair 98:D3:31:FB:4F:9E
   trust 98:D3:31:FB:4F:9E
   ```

4. **Create serial connection**:
   ```bash
   sudo rfcomm connect 0 98:D3:31:FB:4F:9E &
   ```
   This creates `/dev/rfcomm0`

### Running the Test Script
```bash
cd python_control
./bluetooth_test.py
```

### Custom Python Integration
```python
from bluetooth_test import RobotBluetoothController

robot = RobotBluetoothController('/dev/rfcomm0')
if robot.connect():
    status = robot.get_status()
    robot.move_forward()
    robot.stop_motors()
    robot.disconnect()
```

## Troubleshooting

### Connection Issues
1. **Device not found**: Ensure HC-05 is powered and in pairing mode
2. **Permission denied**: Try `sudo rfcomm connect...`
3. **Port not found**: Check if `/dev/rfcomm0` exists
4. **No response**: Verify wiring and baud rate (9600)

### Arduino Upload Issues
- Ground pin A2 to disable Bluetooth during uploads
- Or disconnect HC-05 RXD pin temporarily
- Use Arduino IDE Serial Monitor to test basic functionality

### Communication Problems
- Check baud rate matches (Arduino: 9600, Python: 9600)
- Ensure proper line endings (`\n`)
- Verify HC-05 AT command settings if needed

## Advanced Configuration

### HC-05 AT Commands (Optional)
If you need to configure the HC-05 module:

1. Enter AT mode (connect KEY/EN pin to 3.3V before powering on)
2. Use 38400 baud rate for AT commands
3. Common commands:
   ```
   AT+NAME=RobotBot
   AT+PSWD=1234
   AT+UART=9600,0,0
   ```

### Multiple Device Support
To connect multiple robots, use different RFCOMM channels:
```bash
sudo rfcomm connect 0 MAC_ADDRESS_1 &
sudo rfcomm connect 1 MAC_ADDRESS_2 &
```

Then specify the port in Python:
```python
robot1 = RobotBluetoothController('/dev/rfcomm0')
robot2 = RobotBluetoothController('/dev/rfcomm1')
```

## Example Usage Scenarios

### Remote Control
```python
robot = RobotBluetoothController()
robot.connect()

# Move forward for 2 seconds
robot.move_forward()
time.sleep(2)
robot.stop_motors()
```

### PID Tuning
```python
robot = RobotBluetoothController()
robot.connect()

# Adjust PID parameters
robot.set_pid(8.0, 0.05, 1.5)
robot.get_status()  # Check new values
```

### Data Logging
```python
robot = RobotBluetoothController()
robot.connect()

while True:
    status = robot.get_status()
    # Log status data to file
    time.sleep(1)
```

## Safety Notes
- Always test in a safe, open area
- Keep the robot within Bluetooth range (typically 10 meters)
- Monitor battery levels during extended use
- Use emergency stop command if robot behaves unexpectedly
