#!/usr/bin/env python3
"""
Bluetooth Disconnection Helper for HC-05
Cleanly disconnects and cleans up Bluetooth connections
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

def stop_robot_processes():
    """Stop any running robot control processes"""
    print("🛑 Stopping robot control processes...")
    
    # Stop GUI processes
    run_command("pkill -f robot_gui.py", timeout=5)
    run_command("pkill -f robot_client.py", timeout=5)
    run_command("pkill -f test_connection.py", timeout=5)
    
    time.sleep(1)
    print("✅ Robot processes stopped")

def release_rfcomm():
    """Release RFCOMM serial connection"""
    print("📡 Releasing RFCOMM connection...")
    
    if os.path.exists(RFCOMM_DEVICE):
        success, stdout, stderr = run_command(f"sudo rfcomm release 0", timeout=5)
        if success:
            print(f"✅ RFCOMM released: {RFCOMM_DEVICE}")
        else:
            print(f"⚠️ RFCOMM release: {stderr}")
        time.sleep(1)
    else:
        print("ℹ️ No RFCOMM device to release")

def disconnect_bluetooth():
    """Disconnect from HC-05 via Bluetooth"""
    print("🔌 Disconnecting from HC-05...")
    
    # Check current connection status
    success, stdout, stderr = run_command(f"bluetoothctl info {HC05_MAC}")
    
    if "Connected: yes" in stdout:
        # Try to disconnect
        success, stdout, stderr = run_command(f"bluetoothctl disconnect {HC05_MAC}", timeout=10)
        
        if success or "Successful disconnected" in stdout:
            print("✅ Bluetooth disconnected successfully")
        else:
            print(f"⚠️ Bluetooth disconnect: {stdout}")
            
        time.sleep(1)
        
        # Verify disconnection
        success, stdout, stderr = run_command(f"bluetoothctl info {HC05_MAC}")
        if "Connected: no" in stdout:
            print("✅ Disconnection confirmed")
        else:
            print("⚠️ Device may still be connected")
    else:
        print("ℹ️ Device already disconnected")

def cleanup_all():
    """Complete cleanup of all connections"""
    print("🧹 Performing complete cleanup...")
    
    # Kill any remaining rfcomm processes
    run_command("sudo pkill -f rfcomm", timeout=5)
    
    # Remove any stale rfcomm devices
    for i in range(5):
        run_command(f"sudo rfcomm release {i}", timeout=2)
    
    time.sleep(1)
    print("✅ Cleanup completed")

def check_disconnection_status():
    """Check final status after disconnection"""
    print("\n📋 Final Status:")
    
    # Check Bluetooth connection
    success, stdout, stderr = run_command(f"bluetoothctl info {HC05_MAC}")
    if success:
        if "Connected: yes" in stdout:
            print("🔗 Bluetooth: Still connected")
        else:
            print("✅ Bluetooth: Disconnected")
    
    # Check RFCOMM devices
    if os.path.exists(RFCOMM_DEVICE):
        print(f"📡 RFCOMM: {RFCOMM_DEVICE} still exists")
    else:
        print("✅ RFCOMM: No devices present")
    
    # Check running processes
    success, stdout, stderr = run_command("ps aux | grep -E '(robot_|rfcomm)' | grep -v grep")
    if stdout.strip():
        print("🔄 Processes: Some robot processes still running")
        for line in stdout.strip().split('\n'):
            print(f"   {line}")
    else:
        print("✅ Processes: All robot processes stopped")

def main():
    """Main disconnection function"""
    print("🤖 HC-05 Bluetooth Disconnection Helper")
    print("=" * 40)
    
    # Step 1: Stop robot processes
    stop_robot_processes()
    
    # Step 2: Release RFCOMM connection
    release_rfcomm()
    
    # Step 3: Disconnect Bluetooth
    disconnect_bluetooth()
    
    # Step 4: Complete cleanup
    cleanup_all()
    
    # Step 5: Show final status
    check_disconnection_status()
    
    print()
    print("🎯 Disconnection completed!")
    print("💡 To reconnect later, use: python3 connect_bluetooth.py")

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--force":
        print("🔥 Force mode: Aggressive cleanup")
        cleanup_all()
    else:
        main()
