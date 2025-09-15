#!/usr/bin/env python3
"""
Bluetooth Connection Helper for HC-05
Automatically establishes connection and creates rfcomm device
"""

import subprocess
import time
import sys
import os

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

def check_bluetooth_status():
    """Check if Bluetooth is available and HC-05 is paired"""
    print("🔍 Checking Bluetooth status...")
    
    # Check if bluetoothctl is available
    success, stdout, stderr = run_command("which bluetoothctl")
    if not success:
        print("❌ bluetoothctl not found. Install bluez-utils package.")
        return False
    
    # Check if HC-05 is known
    success, stdout, stderr = run_command(f"bluetoothctl info {HC05_MAC}")
    if not success:
        print(f"❌ HC-05 ({HC05_MAC}) not paired. Please pair it first.")
        return False
    
    if "Paired: yes" in stdout:
        print("✅ HC-05 is paired")
        return True
    else:
        print("❌ HC-05 is not paired")
        return False

def connect_bluetooth():
    """Connect to HC-05 via Bluetooth"""
    print("📡 Connecting to HC-05...")
    
    # Try to connect
    success, stdout, stderr = run_command(f"bluetoothctl connect {HC05_MAC}", timeout=15)
    
    if success or "Connected: yes" in stdout:
        print("✅ Bluetooth connection established")
        return True
    else:
        print(f"⚠️ Bluetooth connect returned: {stdout}")
        # Sometimes it still works even if command reports failure
        # Check actual connection status
        success, stdout, stderr = run_command(f"bluetoothctl info {HC05_MAC}")
        if "Connected: yes" in stdout:
            print("✅ Bluetooth connection confirmed")
            return True
        else:
            print("❌ Bluetooth connection failed")
            return False

def setup_rfcomm():
    """Setup RFCOMM serial connection"""
    print("🔌 Setting up RFCOMM connection...")
    
    # Check if rfcomm0 already exists
    if os.path.exists(RFCOMM_DEVICE):
        print(f"✅ {RFCOMM_DEVICE} already exists")
        return True
    
    # Try to bind RFCOMM
    success, stdout, stderr = run_command(f"sudo rfcomm bind 0 {HC05_MAC}")
    
    if success:
        print(f"✅ RFCOMM bound to {RFCOMM_DEVICE}")
        return True
    else:
        print(f"❌ RFCOMM bind failed: {stderr}")
        return False

def test_connection():
    """Test if the serial connection works"""
    print("🧪 Testing serial connection...")
    
    try:
        import serial
        ser = serial.Serial(RFCOMM_DEVICE, 9600, timeout=2)
        print(f"✅ Serial connection to {RFCOMM_DEVICE} established")
        
        # Send test command
        ser.write(b"STATUS\\n")
        time.sleep(1)
        
        # Try to read response
        response_received = False
        for i in range(5):
            if ser.in_waiting > 0:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    print(f"🤖 Robot response: {response}")
                    response_received = True
                    break
            time.sleep(0.5)
        
        ser.close()
        
        if response_received:
            print("✅ Robot communication successful!")
        else:
            print("⚠️ No response from robot (may still be calibrating)")
        
        return True
        
    except Exception as e:
        print(f"❌ Serial connection test failed: {e}")
        return False

def cleanup_rfcomm():
    """Clean up existing RFCOMM connections"""
    print("🧹 Cleaning up existing RFCOMM connections...")
    run_command("sudo rfcomm release 0", timeout=5)
    time.sleep(1)

def main():
    """Main connection establishment function"""
    print("🤖 HC-05 Bluetooth Connection Helper")
    print("=" * 40)
    
    # Clean up any existing connections first
    cleanup_rfcomm()
    
    # Step 1: Check Bluetooth status
    if not check_bluetooth_status():
        return False
    
    # Step 2: Connect via Bluetooth
    if not connect_bluetooth():
        return False
    
    # Wait a moment for connection to stabilize
    time.sleep(2)
    
    # Step 3: Setup RFCOMM
    if not setup_rfcomm():
        return False
    
    # Wait for device to be ready
    time.sleep(1)
    
    # Step 4: Test connection
    if not test_connection():
        return False
    
    print()
    print("🎉 Bluetooth connection established successfully!")
    print(f"📍 Robot is available at: {RFCOMM_DEVICE}")
    print("🎮 You can now use the robot control applications:")
    print("   python3 robot_gui.py")
    print("   python3 robot_client.py")
    print("   python3 test_connection.py")
    
    return True

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--cleanup":
        cleanup_rfcomm()
    else:
        success = main()
        sys.exit(0 if success else 1)
