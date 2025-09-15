#!/usr/bin/env python3
"""
Bluetooth Robot Client
Connects to the balancing robot via Bluetooth and provides basic control interface.
"""

import serial
import serial.tools.list_ports
import time
import threading
import sys
from datetime import datetime
import re

class BluetoothRobotClient:
    def __init__(self):
        self.connection = None
        self.connected = False
        self.telemetry_data = {}
        self.running = False
        self.telemetry_thread = None
        
    def find_bluetooth_port(self):
        """Find available serial ports that might be Bluetooth"""
        import os
        import glob
        
        bt_ports = []
        
        # First, check for rfcomm devices directly
        rfcomm_devices = glob.glob('/dev/rfcomm*')
        if rfcomm_devices:
            bt_ports.extend(rfcomm_devices)
            print(f"Found rfcomm devices: {rfcomm_devices}")
        
        # Also check regular serial ports for Bluetooth descriptions
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Look for common Bluetooth device names
            description = port.description.lower()
            if any(keyword in description for keyword in ['bluetooth', 'hc-05', 'hc-06', 'rfcomm']):
                bt_ports.append(port.device)
        
        # Remove duplicates while preserving order
        seen = set()
        unique_ports = []
        for port in bt_ports:
            if port not in seen:
                unique_ports.append(port)
                seen.add(port)
                
        return unique_ports
    
    def connect(self, port=None, baudrate=9600):
        """Connect to the robot via Bluetooth"""
        if port is None:
            # Try to find Bluetooth ports automatically
            bt_ports = self.find_bluetooth_port()
            if not bt_ports:
                print("No Bluetooth ports found. Available ports:")
                for port_info in serial.tools.list_ports.comports():
                    print(f"  {port_info.device}: {port_info.description}")
                return False
            port = bt_ports[0]
            
        try:
            print(f"Connecting to {port} at {baudrate} baud...")
            self.connection = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)  # Allow time for connection to establish
            
            # Test connection
            self.connection.write(b"STATUS\\n")
            time.sleep(0.5)
            
            if self.connection.in_waiting > 0:
                response = self.connection.readline().decode('utf-8', errors='ignore').strip()
                print(f"Robot responded: {response}")
                self.connected = True
                print(f"Successfully connected to robot on {port}")
                
                # Start telemetry thread
                self.running = True
                self.telemetry_thread = threading.Thread(target=self._telemetry_listener, daemon=True)
                self.telemetry_thread.start()
                
                return True
            else:
                print("No response from robot")
                self.connection.close()
                return False
                
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from robot"""
        self.running = False
        if self.connection and self.connection.is_open:
            self.send_command("STOP")  # Stop robot movement
            time.sleep(0.1)
            self.connection.close()
            self.connected = False
            print("Disconnected from robot")
    
    def send_command(self, command):
        """Send a command to the robot"""
        if not self.connected:
            print("Not connected to robot")
            return False
            
        try:
            cmd = command.strip() + "\\n"
            self.connection.write(cmd.encode('utf-8'))
            return True
        except Exception as e:
            print(f"Failed to send command: {e}")
            return False
    
    def _telemetry_listener(self):
        """Background thread to listen for telemetry data"""
        while self.running and self.connected:
            try:
                if self.connection.in_waiting > 0:
                    line = self.connection.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Parse telemetry data
                    if line.startswith("TEL:"):
                        self._parse_telemetry(line[4:])  # Remove "TEL:" prefix
                    elif line and not line.startswith("TEL:"):
                        # Print other robot messages
                        timestamp = datetime.now().strftime("%H:%M:%S")
                        print(f"[{timestamp}] Robot: {line}")
                        
                time.sleep(0.01)  # Small delay to prevent busy waiting
                
            except Exception as e:
                if self.running:  # Only print error if we're still supposed to be running
                    print(f"Telemetry error: {e}")
                break
    
    def _parse_telemetry(self, data):
        """Parse telemetry data from robot"""
        try:
            # Format: pitch,setpoint,error,output,tiltRate,steering,speed
            values = [float(x) for x in data.split(',')]
            
            if len(values) >= 7:
                self.telemetry_data = {
                    'timestamp': time.time(),
                    'pitch': values[0],
                    'setpoint': values[1],
                    'error': values[2],
                    'output': values[3],
                    'tilt_rate': values[4],
                    'steering': values[5],
                    'speed': values[6]
                }
                
        except (ValueError, IndexError) as e:
            pass  # Ignore malformed telemetry data
    
    def get_telemetry(self):
        """Get latest telemetry data"""
        return self.telemetry_data.copy() if self.telemetry_data else None
    
    def set_speed(self, speed):
        """Set robot speed (-50 to 50)"""
        speed = max(-50, min(50, speed))  # Clamp to valid range
        return self.send_command(f"SPEED:{speed}")
    
    def set_steering(self, steering):
        """Set robot steering (-50 to 50)"""
        steering = max(-50, min(50, steering))  # Clamp to valid range
        return self.send_command(f"STEER:{steering}")
    
    def stop(self):
        """Stop all robot movement"""
        return self.send_command("STOP")
    
    def set_pid(self, kp, ki, kd):
        """Set PID gains"""
        return self.send_command(f"PID:{kp},{ki},{kd}")
    
    def get_status(self):
        """Request robot status"""
        return self.send_command("STATUS")
    
    def help(self):
        """Get robot command help"""
        return self.send_command("HELP")


def main():
    """Main interactive control loop"""
    robot = BluetoothRobotClient()
    
    print("=== Bluetooth Robot Controller ===")
    
    # Try to connect
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = None
        
    if not robot.connect(port):
        print("Failed to connect to robot")
        return
    
    print("\\nConnected! Available commands:")
    print("  w/s - Forward/Backward speed")
    print("  a/d - Left/Right steering")
    print("  x   - Stop all movement")
    print("  p   - Set PID gains")
    print("  t   - Show telemetry")
    print("  h   - Help")
    print("  q   - Quit")
    print("\\nPress keys and Enter, or type commands directly:")
    
    try:
        speed = 0
        steering = 0
        
        while True:
            cmd = input("> ").strip().lower()
            
            if cmd == 'q' or cmd == 'quit':
                break
            elif cmd == 'w':
                speed = min(50, speed + 10)
                robot.set_speed(speed)
                print(f"Speed: {speed}")
            elif cmd == 's':
                speed = max(-50, speed - 10)
                robot.set_speed(speed)
                print(f"Speed: {speed}")
            elif cmd == 'a':
                steering = max(-50, steering - 10)
                robot.set_steering(steering)
                print(f"Steering: {steering}")
            elif cmd == 'd':
                steering = min(50, steering + 10)
                robot.set_steering(steering)
                print(f"Steering: {steering}")
            elif cmd == 'x':
                speed = 0
                steering = 0
                robot.stop()
                print("Stopped")
            elif cmd == 't':
                tel = robot.get_telemetry()
                if tel:
                    print(f"Telemetry:")
                    print(f"  Pitch: {tel['pitch']:.2f}°")
                    print(f"  Setpoint: {tel['setpoint']:.2f}°")
                    print(f"  Error: {tel['error']:.2f}°")
                    print(f"  PID Output: {tel['output']:.1f}")
                    print(f"  Tilt Rate: {tel['tilt_rate']:.1f}°/s")
                    print(f"  Commands - Speed: {tel['speed']:.1f}, Steering: {tel['steering']:.1f}")
                else:
                    print("No telemetry data available")
            elif cmd == 'h':
                robot.help()
            elif cmd == 'p':
                try:
                    kp = float(input("Enter Kp: "))
                    ki = float(input("Enter Ki: "))
                    kd = float(input("Enter Kd: "))
                    robot.set_pid(kp, ki, kd)
                    print(f"PID set to Kp={kp}, Ki={ki}, Kd={kd}")
                except ValueError:
                    print("Invalid PID values")
            elif cmd.startswith('speed:'):
                try:
                    speed = float(cmd[6:])
                    robot.set_speed(speed)
                    print(f"Speed set to {speed}")
                except ValueError:
                    print("Invalid speed value")
            elif cmd.startswith('steer:'):
                try:
                    steering = float(cmd[6:])
                    robot.set_steering(steering)
                    print(f"Steering set to {steering}")
                except ValueError:
                    print("Invalid steering value")
            elif cmd:
                # Send raw command to robot
                robot.send_command(cmd.upper())
                
    except KeyboardInterrupt:
        print("\\nInterrupted by user")
    
    finally:
        robot.disconnect()
        print("Goodbye!")


if __name__ == "__main__":
    main()
