#!/bin/bash

# HC-05 Bluetooth Connection Script
# Connects to the HC-05 module and creates /dev/rfcomm0 for serial communication

# HC-05 MAC address (update if different)
HC05_MAC="98:D3:33:80:67:7A"
RFCOMM_DEVICE="0"

echo "🔗 HC-05 Bluetooth Connection Script"
echo "===================================="

# Check if Bluetooth is powered on
echo "📡 Checking Bluetooth status..."
if ! bluetoothctl show | grep -q "Powered: yes"; then
    echo "❌ Bluetooth is not powered on"
    echo "💡 Enable Bluetooth with: sudo bluetoothctl -- power on"
    exit 1
fi

echo "✅ Bluetooth is powered on"

# Check if HC-05 is paired
echo "🔍 Checking if HC-05 is paired..."
if ! bluetoothctl devices | grep -q "$HC05_MAC"; then
    echo "❌ HC-05 not found in paired devices"
    echo "💡 Please pair the HC-05 first:"
    echo "   bluetoothctl"
    echo "   scan on"
    echo "   pair $HC05_MAC"
    echo "   trust $HC05_MAC"
    exit 1
fi

echo "✅ HC-05 found in paired devices"

# Check if RFCOMM connection already exists
if sudo rfcomm show $RFCOMM_DEVICE 2>/dev/null | grep -q "connected"; then
    echo "⚠️  RFCOMM connection already active:"
    sudo rfcomm show $RFCOMM_DEVICE
    echo ""
    echo "🔄 To reconnect, first run: ./disconnect_bluetooth.sh"
    exit 0
fi

# Release any existing RFCOMM binding
echo "🧹 Cleaning up any existing RFCOMM bindings..."
sudo rfcomm release $RFCOMM_DEVICE 2>/dev/null || true

# Create RFCOMM connection
echo "🔌 Connecting to HC-05 ($HC05_MAC)..."
sudo rfcomm connect $RFCOMM_DEVICE $HC05_MAC &
RFCOMM_PID=$!

# Wait for connection to establish
echo "⏳ Waiting for connection to establish..."
sleep 3

# Check if connection was successful
if sudo rfcomm show $RFCOMM_DEVICE 2>/dev/null | grep -q "connected"; then
    echo "✅ Successfully connected to HC-05!"
    echo "📱 Device: /dev/rfcomm$RFCOMM_DEVICE"
    echo "🔗 Connection details:"
    sudo rfcomm show $RFCOMM_DEVICE
    
    # Check device permissions
    echo ""
    echo "🔐 Device permissions:"
    ls -la /dev/rfcomm$RFCOMM_DEVICE
    
    # Add user to dialout group if not already member
    if ! groups $USER | grep -q dialout; then
        echo "⚠️  Your user is not in the 'dialout' group"
        echo "💡 Add yourself to the group with: sudo usermod -a -G dialout $USER"
        echo "   Then log out and log back in for changes to take effect"
    fi
    
    echo ""
    echo "🎯 Ready to use! You can now run:"
    echo "   cd python_control"
    echo "   ./bluetooth_test.py"
    echo "   ./pid_tuner.py"
    echo ""
    echo "💡 To disconnect, run: ./disconnect_bluetooth.sh"
    
else
    echo "❌ Failed to establish connection"
    echo "🔧 Troubleshooting:"
    echo "   - Make sure the robot is powered on"
    echo "   - Check if HC-05 LED stops blinking (indicating connection)"
    echo "   - Verify HC-05 is within range"
    echo "   - Try running: bluetoothctl connect $HC05_MAC"
    
    # Kill the background process
    kill $RFCOMM_PID 2>/dev/null || true
    exit 1
fi
