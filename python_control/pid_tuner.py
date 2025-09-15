#!/usr/bin/env python3
"""
PID Tuning Script for Arduino Robot
This script demonstrates how to get and set PID parameters via Bluetooth
"""

from bluetooth_test import RobotBluetoothController
import time
import sys

def display_pid_values(pid_values):
    """Display PID values in a formatted way"""
    if pid_values:
        print("\n" + "="*40)
        print("         CURRENT PID VALUES")
        print("="*40)
        print(f"  Kp (Proportional): {pid_values['kp']:.4f}")
        print(f"  Ki (Integral):     {pid_values['ki']:.4f}")
        print(f"  Kd (Derivative):   {pid_values['kd']:.4f}")
        print("="*40)
    else:
        print("❌ Failed to retrieve PID values")

def pid_tuning_session(robot):
    """Interactive PID tuning session"""
    print("\n🎛️  Starting PID Tuning Session")
    print("=" * 50)
    
    while True:
        # Get current PID values
        print("\n📊 Getting current PID values...")
        current_pid = robot.get_pid()
        display_pid_values(current_pid)
        
        if not current_pid:
            print("Cannot continue without PID values. Exiting.")
            return
        
        print("\nOptions:")
        print("1. Adjust Kp (Proportional)")
        print("2. Adjust Ki (Integral)")
        print("3. Adjust Kd (Derivative)")
        print("4. Set all PID values")
        print("5. Get robot status")
        print("6. Test current settings")
        print("7. Reset to defaults")
        print("8. Exit tuning session")
        
        choice = input("\nChoose option (1-8): ").strip()
        
        if choice == '1':
            try:
                new_kp = float(input(f"Current Kp: {current_pid['kp']:.4f}, Enter new Kp: "))
                robot.set_pid(new_kp, current_pid['ki'], current_pid['kd'])
                print(f"✅ Kp updated to {new_kp}")
            except ValueError:
                print("❌ Invalid Kp value")
        
        elif choice == '2':
            try:
                new_ki = float(input(f"Current Ki: {current_pid['ki']:.4f}, Enter new Ki: "))
                robot.set_pid(current_pid['kp'], new_ki, current_pid['kd'])
                print(f"✅ Ki updated to {new_ki}")
            except ValueError:
                print("❌ Invalid Ki value")
        
        elif choice == '3':
            try:
                new_kd = float(input(f"Current Kd: {current_pid['kd']:.4f}, Enter new Kd: "))
                robot.set_pid(current_pid['kp'], current_pid['ki'], new_kd)
                print(f"✅ Kd updated to {new_kd}")
            except ValueError:
                print("❌ Invalid Kd value")
        
        elif choice == '4':
            try:
                print(f"Current values: Kp={current_pid['kp']:.4f}, Ki={current_pid['ki']:.4f}, Kd={current_pid['kd']:.4f}")
                new_kp = float(input("Enter new Kp: "))
                new_ki = float(input("Enter new Ki: "))
                new_kd = float(input("Enter new Kd: "))
                robot.set_pid(new_kp, new_ki, new_kd)
                print(f"✅ PID updated to Kp={new_kp}, Ki={new_ki}, Kd={new_kd}")
            except ValueError:
                print("❌ Invalid PID values")
        
        elif choice == '5':
            print("\n📡 Getting robot status...")
            robot.get_status()
        
        elif choice == '6':
            print("\n🧪 Testing current PID settings...")
            print("Observe robot behavior for 10 seconds...")
            robot.get_status()
            time.sleep(10)
            robot.get_status()
        
        elif choice == '7':
            # Default PID values from the Arduino code
            default_kp = 7.0
            default_ki = 0.03
            default_kd = 1.2
            robot.set_pid(default_kp, default_ki, default_kd)
            print(f"✅ PID reset to defaults: Kp={default_kp}, Ki={default_ki}, Kd={default_kd}")
        
        elif choice == '8':
            break
        
        else:
            print("❌ Invalid option")
        
        # Small delay to let the robot process the command
        time.sleep(0.5)

def compare_pid_values(robot):
    """Compare current PID with defaults"""
    print("\n📋 PID Comparison")
    print("=" * 50)
    
    current_pid = robot.get_pid()
    if not current_pid:
        print("❌ Cannot retrieve current PID values")
        return
    
    # Default values from Arduino code
    defaults = {'kp': 7.0, 'ki': 0.03, 'kd': 1.2}
    
    print(f"{'Parameter':<12} {'Current':<10} {'Default':<10} {'Difference':<12}")
    print("-" * 50)
    
    for param in ['kp', 'ki', 'kd']:
        current = current_pid[param]
        default = defaults[param]
        diff = current - default
        status = "✓" if abs(diff) < 0.001 else "⚠️" if abs(diff) < 1.0 else "❌"
        
        print(f"{param.upper():<12} {current:<10.4f} {default:<10.4f} {diff:<+10.4f} {status}")

def main():
    """Main PID tuning application"""
    print("🤖 Arduino Robot PID Tuner")
    print("=" * 40)
    
    # Initialize robot connection
    robot = RobotBluetoothController()
    
    if not robot.connect():
        print("❌ Failed to connect to robot")
        sys.exit(1)
    
    try:
        while True:
            print("\n🎯 PID Tuner Main Menu")
            print("1. View current PID values")
            print("2. Compare with defaults")
            print("3. Start tuning session")
            print("4. Quick PID test")
            print("5. Exit")
            
            choice = input("\nChoose option (1-5): ").strip()
            
            if choice == '1':
                print("\n📊 Retrieving current PID values...")
                pid_values = robot.get_pid()
                display_pid_values(pid_values)
            
            elif choice == '2':
                compare_pid_values(robot)
            
            elif choice == '3':
                pid_tuning_session(robot)
            
            elif choice == '4':
                print("\n🚀 Quick PID Test")
                pid_values = robot.get_pid()
                if pid_values:
                    display_pid_values(pid_values)
                    print("\nGetting robot status before and after...")
                    robot.get_status()
                    time.sleep(5)
                    robot.get_status()
                else:
                    print("❌ Cannot perform test without PID values")
            
            elif choice == '5':
                break
            
            else:
                print("❌ Invalid choice")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    
    finally:
        robot.disconnect()
        print("👋 PID Tuner session ended")

if __name__ == "__main__":
    main()
