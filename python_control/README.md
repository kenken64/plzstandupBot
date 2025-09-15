# 🤖 Balancing Robot - Complete Bluetooth Control System

A comprehensive system for controlling your balancing robot via Bluetooth with real-time telemetry, PID tuning, and advanced motor control.

## 🎯 What This System Provides

### ✅ Enhanced Firmware Features:
- **Fixed Motor Logic**: Corrected the motor control - both wheels spin same direction for forward/back
- **Improved PID Gains**: Better tuned gains (Kp=12.0, Ki=0.015, Kd=2.5) based on log analysis
- **Bluetooth Integration**: HC-05 module support with non-blocking communication
- **Real-time Telemetry**: Streams pitch, error, PID output, and control commands
- **Enhanced Steering**: Proper differential control for turning while balancing
- **Command Safety**: Auto-decay of commands, fall detection, emergency stop

### ✅ Python Control Interfaces:
- **Command-line Client**: Simple text-based control and monitoring
- **Advanced GUI**: Real-time visualization, PID tuning, data logging

## 🔧 Hardware Setup

### Required Components:
1. **HC-05 Bluetooth Module** (or similar)
2. **Arduino** (your existing robot controller)
3. **4 jumper wires** for connections

### Wiring Connections:
```
HC-05 Module  →  Arduino
VCC          →  5V or 3.3V (use jumper wire for easy disconnect)
GND          →  GND  
TX           →  A0 (Arduino pin)
RX           →  A1 (Arduino pin)
             →  A2 (optional: add jumper to GND for upload mode)
```

### Upload Safety:
The firmware includes upload-safe mode:
- **Normal operation**: Leave A2 pin floating (no connection)
- **Firmware upload**: Connect A2 to GND with jumper wire, then upload
- **Alternative**: Disconnect HC-05 VCC wire during uploads

**Important**: The HC-05 RX pin often requires 3.3V logic levels. If using 5V Arduino, add a voltage divider (two resistors) on the A1 → HC-05 RX connection.

## 📱 Software Setup

### 1. Install Python Dependencies
```bash
cd python_control
pip install -r requirements.txt
```

### 2. Pair Bluetooth Module
1. Power on your robot with HC-05 module
2. On your PC, go to Bluetooth settings
3. Pair with "HC-05" (default PIN: 1234 or 0000)
4. Note the COM port assigned (Windows) or device path (Linux)

### 3. Flash Enhanced Firmware
1. Upload the enhanced `main.cpp` to your Arduino (now includes all Bluetooth functionality)
2. Open Serial Monitor to verify initialization
3. Look for "DMP ready. Bluetooth balancing mode engaged."

## 🚀 Usage

### Command-Line Interface
```bash
# Auto-detect Bluetooth port
python robot_client.py

# Specify specific port
python robot_client.py /dev/rfcomm0    # Linux
python robot_client.py COM5           # Windows
```

**Available Commands:**
- `w/s` - Increase/decrease forward speed
- `a/d` - Steer left/right  
- `x` - Stop all movement
- `p` - Set PID gains interactively
- `t` - Show current telemetry
- `h` - Get robot help
- `q` - Quit

### Advanced GUI Interface
```bash
python robot_gui.py
```

**GUI Features:**
- **Real-time Control**: Speed/steering sliders
- **PID Tuning**: Live gain adjustment with sliders
- **Telemetry Display**: Pitch, error, rates, commands
- **Live Plotting**: Real-time graphs of robot behavior
- **Data Logging**: Save telemetry to CSV/JSON files
- **Keyboard Shortcuts**: Spacebar = emergency stop, R = reset

## 🎛️ Control Commands

### Robot Commands (sent via Bluetooth):
- `SPEED:value` - Set speed (-50 to 50)
- `STEER:value` - Set steering (-50 to 50) 
- `PID:kp,ki,kd` - Update PID gains
- `STOP` - Emergency stop
- `STATUS` - Get current robot status
- `HELP` - List available commands

### Telemetry Format:
```
TEL:pitch,setpoint,error,output,tiltRate,steering,speed
Example: TEL:179.45,180.00,-0.55,25.2,1.3,0.0,10.0
```

## 🔧 Tuning Your Robot

### Based on Your Log Analysis:
The current firmware includes improved PID gains based on your monitor.log analysis:

**Previous Issues:**
- Large steady-state errors (~4.45°)
- Sluggish tilt response 
- Motor output stuck at constant values
- Potential integral windup

**New Settings:**
- **Kp: 12.0** (increased from 7.0) - More responsive to errors
- **Ki: 0.015** (decreased from 0.03) - Reduced integral windup
- **Kd: 2.5** (increased from 1.2) - Better damping and reaction to changes

### Fine-tuning Process:
1. **Start Conservative**: Use the GUI to make small adjustments
2. **Test Stability**: Increase Kp until you see slight oscillations
3. **Add Damping**: Increase Kd to reduce oscillations
4. **Steady-State**: Carefully increase Ki if you have persistent offset

## 🚨 Safety Features

- **Fall Detection**: Motors stop when tilt > 60°
- **Command Decay**: Control commands automatically decay to prevent runaway
- **Emergency Stop**: Multiple ways to immediately stop robot
- **Connection Loss**: Robot stops if Bluetooth disconnects

## 🔍 Troubleshooting

### Connection Issues:
1. **Port not found**: Check Bluetooth pairing and device manager
2. **Permission denied**: On Linux, add user to `dialout` group
3. **No response**: Verify HC-05 wiring and baud rate (9600)

### Robot Behavior:
1. **Wrong direction**: Check motor wiring or reverse in firmware  
2. **Won't balance**: Verify MPU6050 calibration values
3. **Oscillations**: Reduce Kp or increase Kd
4. **Sluggish**: Increase Kp, verify deadband settings

### Performance Monitoring:
Use the GUI's real-time plots to observe:
- **Pitch tracking**: Should follow setpoint closely
- **Error magnitude**: Should be small and stable
- **PID output**: Should be smooth, not erratic
- **Tilt rate**: Should show quick responses to disturbances

## 📊 Data Analysis

The GUI can log all telemetry data for offline analysis:
1. Enable "Data Logging" in GUI
2. Run tests with different PID settings
3. Save data as CSV for analysis in Excel/Python
4. Compare error statistics and control smoothness

## 🎯 Next Steps

1. **Test the basic connection** with command-line client
2. **Verify motor directions** and balancing behavior  
3. **Use GUI for real-time tuning** while robot balances
4. **Log data** during different scenarios for analysis
5. **Experiment with steering** while balancing

## 📝 Technical Details

### Communication Protocol:
- **Baud Rate**: 9600 (HC-05 default)
- **Flow Control**: None
- **Data Format**: ASCII text commands, newline terminated
- **Telemetry Rate**: 10Hz (every 100ms)

### Performance Impact:
- **Balance Loop**: Still runs at 200Hz (5ms) - unchanged
- **Bluetooth Processing**: Non-blocking, minimal impact
- **Command Latency**: ~50-100ms typical over Bluetooth

This system transforms your balancing robot into a fully remote-controllable and tunable platform! 🤖✨
