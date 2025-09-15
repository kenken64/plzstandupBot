#!/usr/bin/env python3
"""
Robot Response Diagnostic Tool
Identifies why the robot is not responding to Bluetooth commands
"""

import subprocess
import time
import sys
import os
import serial

HC05_MAC = "98:D3:33:80:67:7A"
RFCOMM_DEVICE = "/dev/rfcomm0"

def run_command(cmd, timeout=10):
    """Run a shell command and return result"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, 
                              text=True, timeout=timeout)
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "Command timed out"

def check_hardware_connection():
    """Check if HC-05 Bluetooth hardware is connected"""
    print("🔍 Checking HC-05 Hardware Connection...")
    
    # Check if HC-05 is paired and connected
    success, stdout, stderr = run_command(f"bluetoothctl info {HC05_MAC}")
    if not success:
        print("❌ HC-05 not found in Bluetooth devices")
        return False
    
    if "Connected: yes" in stdout:
        print("✅ HC-05 Bluetooth connected")
    else:
        print("⚠️ HC-05 paired but not connected")
    
    # Check signal strength if available
    if "RSSI" in stdout:
        rssi_line = [line for line in stdout.split('\n') if 'RSSI' in line]
        if rssi_line:
            print(f"📶 Signal strength: {rssi_line[0].strip()}")
    
    return True

def check_serial_communication():
    """Test raw serial communication with different parameters"""
    print("\n📡 Testing Serial Communication...")
    
    if not os.path.exists(RFCOMM_DEVICE):
        print(f"❌ {RFCOMM_DEVICE} not available")
        return False
    
    # Test different baud rates
    baud_rates = [9600, 115200, 38400, 19200]
    
    for baud in baud_rates:
        print(f"\n🔧 Testing baud rate: {baud}")
        try:
            ser = serial.Serial(RFCOMM_DEVICE, baud, timeout=2)
            print(f"✅ Serial port opened at {baud} baud")
            
            # Send different types of commands
            test_commands = [
                b'STATUS\\n',
                b'HELP\\n', 
                b'\\r\\n',  # Just carriage return
                b'AT\\r\\n',  # AT command (if in AT mode)
                b'\\n',  # Just newline
            ]
            
            for cmd in test_commands:
                print(f"   Sending: {cmd}")
                ser.write(cmd)
                ser.flush()
                time.sleep(0.5)
                
                # Check for response
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting)
                    print(f"   📥 Response: {repr(response)}")
                    ser.close()
                    return True, baud
            
            # Listen for spontaneous data (telemetry)
            print("   👂 Listening for telemetry...")
            for i in range(10):  # 1 second
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting)
                    print(f"   📥 Telemetry: {repr(response)}")
                    ser.close()
                    return True, baud
                time.sleep(0.1)
            
            ser.close()
            print(f"   ❌ No response at {baud} baud")
            
        except Exception as e:
            print(f"   ❌ Failed at {baud} baud: {e}")
    
    return False, None

def check_arduino_power():
    """Check if Arduino is powered and running"""
    print("\n🔋 Checking Arduino Power Status...")
    
    # Check if USB Arduino is connected (would indicate power)
    usb_devices = ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyACM1"]
    usb_found = False
    
    for device in usb_devices:
        if os.path.exists(device):
            print(f"🔌 USB Arduino found at: {device}")
            usb_found = True
            
            # Try to read from USB to see if firmware is running
            try:
                ser = serial.Serial(device, 115200, timeout=1)
                print("   Reading USB serial output...")
                for i in range(10):
                    if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            print(f"   📡 Arduino says: {line}")
                    time.sleep(0.1)
                ser.close()
            except Exception as e:
                print(f"   ⚠️ USB read failed: {e}")
            break
    
    if not usb_found:
        print("⚠️ No USB Arduino detected - running on battery power")
    
    return usb_found

def check_upload_mode():
    """Check if robot might be in upload mode"""
    print("\n🔧 Checking Upload Mode Status...")
    
    print("❓ Is pin A2 connected to GND?")
    print("   📌 If YES: Robot is in upload mode - Bluetooth disabled")
    print("   📌 If NO: Robot should have Bluetooth enabled")
    print()
    print("💡 To fix upload mode:")
    print("   1. Disconnect A2 from GND (remove jumper wire)")
    print("   2. Power cycle the robot")
    print("   3. Robot should now respond to Bluetooth")

def check_firmware_version():
    """Check if correct firmware is uploaded"""
    print("\n📋 Firmware Requirements Check...")
    
    # Check if current firmware has Bluetooth support
    firmware_path = "../src/main.cpp"
    if os.path.exists(firmware_path):
        with open(firmware_path, 'r') as f:
            content = f.read()
            
        bluetooth_features = [
            "SoftwareSerial bluetooth",
            "BT_RX_PIN A0",
            "BT_TX_PIN A1", 
            "bluetooth.begin(9600)",
            "processBluetooth()",
            "sendTelemetry()"
        ]
        
        missing_features = []
        for feature in bluetooth_features:
            if feature not in content:
                missing_features.append(feature)
        
        if not missing_features:
            print("✅ Firmware has all Bluetooth features")
        else:
            print("⚠️ Firmware missing features:")
            for feature in missing_features:
                print(f"   - {feature}")
    else:
        print("❌ Cannot find firmware source code")

def suggest_solutions():
    """Suggest solutions based on diagnostics"""
    print("\n🛠️ TROUBLESHOOTING SOLUTIONS:")
    print("=" * 50)
    
    print("\n1. 🔧 CHECK UPLOAD MODE (Most Common Issue)")
    print("   - Look for jumper wire connecting A2 to GND")
    print("   - If found, REMOVE the jumper wire")
    print("   - Power cycle the robot")
    print("   - Try connecting again")
    
    print("\n2. 🔌 CHECK WIRING")
    print("   - HC-05 TX → Arduino A0")  
    print("   - HC-05 RX → Arduino A1")
    print("   - HC-05 VCC → Arduino 5V")
    print("   - HC-05 GND → Arduino GND")
    
    print("\n3. 🔋 CHECK POWER")
    print("   - Ensure robot/Arduino is powered on")
    print("   - Check battery if running on battery")
    print("   - Look for LED blinks (3 blinks = ready)")
    
    print("\n4. 📱 UPLOAD FIRMWARE")
    print("   - Connect Arduino via USB")
    print("   - Ground A2 pin (upload mode)")
    print("   - Upload src/main.cpp")
    print("   - Remove A2 ground connection")
    print("   - Power cycle robot")

def main():
    """Main diagnostic function"""
    print("🤖 Robot Response Diagnostic Tool")
    print("=" * 40)
    print("Analyzing why robot is not responding...\n")
    
    # Step 1: Hardware connection
    hardware_ok = check_hardware_connection()
    
    # Step 2: Serial communication test
    if hardware_ok:
        comm_ok, working_baud = check_serial_communication()
        if comm_ok:
            print(f"\n🎉 SUCCESS: Robot responding at {working_baud} baud!")
            return
    
    # Step 3: Power check
    check_arduino_power()
    
    # Step 4: Upload mode check  
    check_upload_mode()
    
    # Step 5: Firmware check
    check_firmware_version()
    
    # Step 6: Solutions
    suggest_solutions()
    
    print("\n💡 Most likely cause: Robot is in UPLOAD MODE")
    print("🔧 Solution: Remove A2-to-GND connection and power cycle")

if __name__ == "__main__":
    main()
