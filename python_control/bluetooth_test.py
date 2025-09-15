#!/usr/bin/env python3
"""
Bluetooth Test Script for Arduino Robot
This script communicates with the Arduino robot via Bluetooth HC-05 module
"""

import serial
import time
import sys

class RobotBluetoothController:
    def __init__(self, port='/dev/rfcomm0', baudrate=9600):
        """
        Initialize Bluetooth connection to robot
        
        Args:
            port (str): Bluetooth serial port (Linux: /dev/rfcomm0, Windows: COMx)
            baudrate (int): Communication speed (should match Arduino)
        """
        self.port = port
        self.baudrate = baudrate
        self.connection = None
        self.connected = False
        
    def connect(self):
        """Establish Bluetooth connection to robot"""
        try:
            print(f"Attempting to connect to robot on {self.port}...")
            self.connection = serial.Serial(self.port, self.baudrate, timeout=2)
            time.sleep(2)  # Allow connection to stabilize
            self.connected = True
            print("✓ Connected to robot successfully!")
            
            # Read initial message from robot
            if self.connection.in_waiting:
                response = self.connection.readline().decode('utf-8').strip()
                print(f"Robot says: {response}")
                
            return True
        except serial.SerialException as e:
            print(f"✗ Connection failed: {e}")
            print("\nTroubleshooting tips:")
            print("1. Make sure the HC-05 module is paired with your computer")
            print("2. Check that the correct port is being used")
            print("3. Ensure the robot is powered on")
            print("4. Try: sudo rfcomm connect 0 XX:XX:XX:XX:XX:XX &")
            return False
        except Exception as e:
            print(f"✗ Unexpected error: {e}")
            return False
    
    def disconnect(self):
        """Close Bluetooth connection"""
        if self.connection and self.connected:
            self.connection.close()
            self.connected = False
            print("Disconnected from robot")
    
    def send_command(self, command):
        """
        Send command to robot and return response
        
        Args:
            command (str): Command to send to robot
            
        Returns:
            str: Robot's response or None if failed
        """
        if not self.connected:
            print("Not connected to robot!")
            return None
            
        try:
            # Clear any existing data in buffer by reading it
            while self.connection.in_waiting:
                self.connection.read(self.connection.in_waiting)
            
            # Send command
            self.connection.write((command + '\n').encode('utf-8'))
            print(f"→ Sent: {command}")
            
            # Read response (might be multiple lines)
            responses = []
            start_time = time.time()
            
            while time.time() - start_time < 5:  # Increased timeout to 5 seconds
                if self.connection.in_waiting:
                    response = self.connection.readline().decode('utf-8').strip()
                    if response:
                        responses.append(response)
                        print(f"← Received: {response}")
                        
                        # For GET_PID, look for the specific response format
                        if command == "GET_PID" and "PID_VALUES:" in response:
                            # Continue reading a bit more in case there are more lines
                            time.sleep(0.2)
                            while self.connection.in_waiting:
                                extra_response = self.connection.readline().decode('utf-8').strip()
                                if extra_response:
                                    responses.append(extra_response)
                                    print(f"← Received: {extra_response}")
                            break
                        
                        # Break if we get a complete status or simple response
                        elif response.startswith("RESPONSE:") or response.startswith("ERROR:") or response == "=== END STATUS ===":
                            break
                        
                        # For STATUS command, continue until END STATUS
                        elif command == "STATUS" and response == "=== END STATUS ===":
                            break
                            
                time.sleep(0.1)
            
            if not responses:
                print("⚠️ No response received from robot")
                return None
                
            return '\n'.join(responses)
            
        except Exception as e:
            print(f"Error sending command: {e}")
            return None
    
    def get_status(self):
        """Get current robot status"""
        return self.send_command("STATUS")
    
    def move_forward(self):
        """Move robot forward"""
        return self.send_command("FORWARD")
    
    def move_backward(self):
        """Move robot backward"""
        return self.send_command("BACKWARD")
    
    def stop_motors(self):
        """Stop robot motors"""
        return self.send_command("STOP")
    
    def set_speed(self, speed):
        """Set robot speed (0-255)"""
        return self.send_command(f"SPEED:{speed}")
    
    def set_pid(self, kp, ki, kd):
        """Set PID parameters"""
        return self.send_command(f"PID:{kp},{ki},{kd}")
    
    def get_pid(self):
        """Get current PID parameters"""
        response = self.send_command("GET_PID")
        print(f"🔍 DEBUG: Full response received: {repr(response)}")
        
        if response:
            lines = response.split('\n')
            print(f"🔍 DEBUG: Response lines: {lines}")
            
            for line in lines:
                print(f"🔍 DEBUG: Checking line: '{line}'")
                
                # Check for different possible response formats
                if "PID_VALUES:" in line:
                    print(f"🔍 DEBUG: Found PID_VALUES line: '{line}'")
                    
                    # Extract the PID values part after the colon
                    if ":" in line:
                        pid_part = line.split(":", 1)[1].strip()
                        print(f"🔍 DEBUG: Extracted PID part: '{pid_part}'")
                        
                        try:
                            kp, ki, kd = map(float, pid_part.split(','))
                            print(f"✅ Successfully parsed PID: Kp={kp}, Ki={ki}, Kd={kd}")
                            return {'kp': kp, 'ki': ki, 'kd': kd}
                        except ValueError as e:
                            print(f"❌ Error parsing PID values '{pid_part}': {e}")
                            return None
        
        print("❌ No valid PID response found")
        return None
    
    def calibrate(self):
        """Recalibrate robot balance"""
        return self.send_command("CALIBRATE")

def main():
    """Main test function"""
    print("=== Arduino Robot Bluetooth Test ===")
    
    # Initialize controller
    robot = RobotBluetoothController()
    
    # Connect to robot
    if not robot.connect():
        sys.exit(1)
    
    try:
        while True:
            print("\n=== Commands ===")
            print("1. Get Status")
            print("2. Move Forward")
            print("3. Move Backward") 
            print("4. Stop Motors")
            print("5. Set Speed")
            print("6. Set PID")
            print("7. Get PID")
            print("8. Calibrate")
            print("9. Custom Command")
            print("10. Exit")
            
            choice = input("\nEnter choice (1-10): ").strip()
            
            if choice == '1':
                robot.get_status()
            elif choice == '2':
                robot.move_forward()
            elif choice == '3':
                robot.move_backward()
            elif choice == '4':
                robot.stop_motors()
            elif choice == '5':
                speed = input("Enter speed (0-255): ")
                try:
                    speed = int(speed)
                    robot.set_speed(speed)
                except ValueError:
                    print("Invalid speed value")
            elif choice == '6':
                try:
                    kp = float(input("Enter Kp: "))
                    ki = float(input("Enter Ki: "))
                    kd = float(input("Enter Kd: "))
                    robot.set_pid(kp, ki, kd)
                except ValueError:
                    print("Invalid PID values")
            elif choice == '7':
                pid_values = robot.get_pid()
                if pid_values:
                    print(f"\nCurrent PID Values:")
                    print(f"  Kp: {pid_values['kp']}")
                    print(f"  Ki: {pid_values['ki']}")
                    print(f"  Kd: {pid_values['kd']}")
                else:
                    print("Failed to retrieve PID values")
            elif choice == '8':
                robot.calibrate()
            elif choice == '9':
                command = input("Enter custom command: ")
                robot.send_command(command)
            elif choice == '10':
                break
            else:
                print("Invalid choice")
                
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        robot.disconnect()
        print("Test completed")

if __name__ == "__main__":
    main()
