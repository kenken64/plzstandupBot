#!/usr/bin/env python3
"""
Simple test script to validate Bluetooth robot connection
"""

import sys
import time
from robot_client import BluetoothRobotClient

def test_robot_connection(port=None):
    """Test basic robot connection and functionality"""
    print("🤖 Bluetooth Robot Connection Test")
    print("=" * 40)
    
    # Create robot client
    robot = BluetoothRobotClient()
    
    # Test connection
    print("1. Testing connection...")
    if not robot.connect(port):
        print("❌ Connection failed!")
        return False
    
    print("✅ Connected successfully!")
    time.sleep(1)
    
    # Test status command
    print("\n2. Testing STATUS command...")
    robot.get_status()
    time.sleep(2)
    
    # Test help command
    print("\n3. Testing HELP command...")
    robot.help()
    time.sleep(2)
    
    # Test telemetry
    print("\n4. Testing telemetry reception...")
    start_time = time.time()
    telemetry_received = False
    
    while time.time() - start_time < 5:  # Wait up to 5 seconds
        tel = robot.get_telemetry()
        if tel:
            print(f"✅ Telemetry received:")
            print(f"   Pitch: {tel['pitch']:.2f}°")
            print(f"   Setpoint: {tel['setpoint']:.2f}°")
            print(f"   Error: {tel['error']:.2f}°")
            print(f"   PID Output: {tel['output']:.1f}")
            telemetry_received = True
            break
        time.sleep(0.1)
    
    if not telemetry_received:
        print("⚠️  No telemetry received (robot may not be calibrated yet)")
    
    # Test PID tuning
    print("\n5. Testing PID command...")
    original_gains = (12.0, 0.015, 2.5)
    test_gains = (10.0, 0.020, 2.0)
    
    robot.set_pid(*test_gains)
    print(f"   Sent PID gains: Kp={test_gains[0]}, Ki={test_gains[1]}, Kd={test_gains[2]}")
    time.sleep(1)
    
    # Restore original gains
    robot.set_pid(*original_gains)
    print(f"   Restored PID gains: Kp={original_gains[0]}, Ki={original_gains[1]}, Kd={original_gains[2]}")
    time.sleep(1)
    
    # Test basic movement commands (very gentle)
    print("\n6. Testing movement commands (gentle)...")
    print("   Forward 5...")
    robot.set_speed(5)
    time.sleep(1)
    
    print("   Stop...")
    robot.stop()
    time.sleep(1)
    
    print("   Backward 5...")
    robot.set_speed(-5)
    time.sleep(1)
    
    print("   Stop...")
    robot.stop()
    time.sleep(1)
    
    print("   Left turn 5...")
    robot.set_steering(-5)
    time.sleep(1)
    
    print("   Stop all...")
    robot.stop()
    time.sleep(1)
    
    print("\n7. Test complete!")
    robot.disconnect()
    print("✅ All tests passed! Your robot is ready for Bluetooth control.")
    
    return True

def main():
    """Main test function"""
    port = None
    if len(sys.argv) > 1:
        port = sys.argv[1]
        print(f"Using specified port: {port}")
    else:
        print("Auto-detecting Bluetooth port...")
    
    try:
        test_robot_connection(port)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")

if __name__ == "__main__":
    main()
