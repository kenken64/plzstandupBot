#!/usr/bin/env python3
"""
Debug script specifically for testing GET_PID command
"""

import serial
import time
import sys

def debug_get_pid():
    """Debug the GET_PID command specifically"""
    try:
        # Connect to robot
        print("🔌 Connecting to robot...")
        connection = serial.Serial('/dev/rfcomm0', 9600, timeout=2)
        time.sleep(2)
        print("✅ Connected!")
        
        # Clear buffer by reading existing data
        while connection.in_waiting:
            connection.read(connection.in_waiting)
        print("🧹 Buffer cleared")
        
        # Send GET_PID command
        print("\n📡 Sending GET_PID command...")
        connection.write(b'GET_PID\n')
        print("→ Sent: GET_PID")
        
        # Read all responses with detailed debugging
        responses = []
        start_time = time.time()
        
        print("\n👂 Listening for responses...")
        while time.time() - start_time < 10:  # 10 second timeout
            if connection.in_waiting > 0:
                raw_data = connection.readline()
                print(f"📥 Raw bytes received: {raw_data}")
                
                try:
                    decoded = raw_data.decode('utf-8').strip()
                    print(f"📝 Decoded text: '{decoded}'")
                    
                    if decoded:
                        responses.append(decoded)
                        
                        # Check if this looks like a PID response
                        if "PID" in decoded.upper():
                            print(f"🎯 Potential PID response: '{decoded}'")
                            
                        # If we see ACK, we know the command was received
                        if decoded.startswith("ACK:"):
                            print("✅ Command acknowledged by robot")
                            
                        # If we see RESPONSE, this is the actual response
                        if decoded.startswith("RESPONSE:"):
                            print("📋 Got response from robot")
                            
                except UnicodeDecodeError as e:
                    print(f"❌ Decode error: {e}")
                    print(f"   Raw data: {raw_data}")
            else:
                time.sleep(0.1)
                
        print(f"\n📊 Total responses received: {len(responses)}")
        for i, response in enumerate(responses):
            print(f"  {i+1}. '{response}'")
            
        # Try to parse PID values from any response
        print("\n🔍 Looking for PID values in responses...")
        for response in responses:
            if "PID" in response.upper() and ":" in response:
                parts = response.split(":", 1)
                if len(parts) == 2:
                    pid_part = parts[1].strip()
                    print(f"   Found potential PID data: '{pid_part}'")
                    
                    if "," in pid_part:
                        try:
                            values = pid_part.split(",")
                            kp = float(values[0])
                            ki = float(values[1]) if len(values) > 1 else 0
                            kd = float(values[2]) if len(values) > 2 else 0
                            print(f"   ✅ Successfully parsed: Kp={kp}, Ki={ki}, Kd={kd}")
                            return True
                        except (ValueError, IndexError) as e:
                            print(f"   ❌ Parse error: {e}")
        
        print("❌ No valid PID values found in responses")
        return False
        
    except serial.SerialException as e:
        print(f"❌ Serial connection error: {e}")
        return False
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return False
    finally:
        try:
            connection.close()
            print("🔌 Connection closed")
        except:
            pass

def test_basic_communication():
    """Test basic communication with simpler commands"""
    try:
        print("🔌 Testing basic communication...")
        connection = serial.Serial('/dev/rfcomm0', 9600, timeout=2)
        time.sleep(2)
        
        # Test with a simpler command first
        commands = ["STOP", "STATUS"]
        
        for cmd in commands:
            print(f"\n📡 Testing {cmd} command...")
            # Clear buffer by reading existing data
            while connection.in_waiting:
                connection.read(connection.in_waiting)
            connection.write((cmd + '\n').encode('utf-8'))
            
            start_time = time.time()
            while time.time() - start_time < 3:
                if connection.in_waiting:
                    response = connection.readline().decode('utf-8').strip()
                    if response:
                        print(f"← {response}")
                        if cmd == "STATUS" and response == "=== END STATUS ===":
                            break
                        elif cmd == "STOP" and response.startswith("RESPONSE:"):
                            break
                time.sleep(0.1)
                
        connection.close()
        print("✅ Basic communication test completed")
        
    except Exception as e:
        print(f"❌ Basic communication test failed: {e}")

if __name__ == "__main__":
    print("🔧 GET_PID Debug Tool")
    print("=" * 40)
    
    print("\n1️⃣ Testing basic communication first...")
    test_basic_communication()
    
    print("\n2️⃣ Testing GET_PID specifically...")
    success = debug_get_pid()
    
    if success:
        print("\n🎉 GET_PID command is working!")
    else:
        print("\n❌ GET_PID command needs debugging")
        print("\nPossible issues:")
        print("- Arduino code may not be responding to GET_PID")
        print("- Response format might be different than expected")
        print("- Timing issues with Bluetooth communication")
        print("- Robot might not be running the updated firmware")
