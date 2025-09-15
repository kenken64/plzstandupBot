#!/bin/bash

# HC-05 Bluetooth Status Check Script
# Shows the current status of HC-05 connection and RFCOMM device

HC05_MAC="98:D3:33:80:67:7A"
RFCOMM_DEVICE="0"

echo "📊 HC-05 Bluetooth Status Check"
echo "==============================="

# Check Bluetooth controller status
echo "🔷 Bluetooth Controller:"
if bluetoothctl show | grep -q "Powered: yes"; then
    echo "   ✅ Powered: ON"
else
    echo "   ❌ Powered: OFF"
fi

# Check if HC-05 is paired
echo ""
echo "🔷 HC-05 Device ($HC05_MAC):"
if bluetoothctl devices | grep -q "$HC05_MAC"; then
    echo "   ✅ Paired: YES"
    
    # Get detailed device info
    device_info=$(bluetoothctl info $HC05_MAC)
    if echo "$device_info" | grep -q "Connected: yes"; then
        echo "   ✅ Connected: YES"
    else
        echo "   ⚠️  Connected: NO"
    fi
    
    if echo "$device_info" | grep -q "Trusted: yes"; then
        echo "   ✅ Trusted: YES"
    else
        echo "   ⚠️  Trusted: NO"
    fi
else
    echo "   ❌ Paired: NO"
    echo "   💡 Run: bluetoothctl pair $HC05_MAC"
fi

# Check RFCOMM connection
echo ""
echo "🔷 RFCOMM Connection:"
rfcomm_status=$(sudo rfcomm show $RFCOMM_DEVICE 2>/dev/null)
if [ $? -eq 0 ]; then
    if echo "$rfcomm_status" | grep -q "connected"; then
        echo "   ✅ Status: CONNECTED"
        echo "   📱 Device: /dev/rfcomm$RFCOMM_DEVICE"
        echo "   🔗 Details: $rfcomm_status"
        
        # Check device permissions
        if [ -e "/dev/rfcomm$RFCOMM_DEVICE" ]; then
            echo "   📄 Permissions: $(ls -l /dev/rfcomm$RFCOMM_DEVICE | awk '{print $1,$3,$4}')"
        fi
        
        # Check if user is in dialout group
        if groups $USER | grep -q dialout; then
            echo "   👤 User access: ✅ (in dialout group)"
        else
            echo "   👤 User access: ⚠️  (not in dialout group)"
            echo "      💡 Add with: sudo usermod -a -G dialout $USER"
        fi
        
    elif echo "$rfcomm_status" | grep -q "closed"; then
        echo "   ⚠️  Status: BOUND BUT CLOSED"
        echo "   💡 Run: ./connect_bluetooth.sh"
    else
        echo "   ❓ Status: UNKNOWN"
        echo "   📋 Raw: $rfcomm_status"
    fi
else
    echo "   ❌ Status: NO RFCOMM BINDING"
    echo "   💡 Run: ./connect_bluetooth.sh"
fi

# Check for active RFCOMM processes
echo ""
echo "🔷 RFCOMM Processes:"
rfcomm_procs=$(pgrep -f rfcomm | wc -l)
if [ $rfcomm_procs -gt 0 ]; then
    echo "   📊 Active processes: $rfcomm_procs"
    echo "   🔍 Process details:"
    ps aux | grep rfcomm | grep -v grep | sed 's/^/      /'
else
    echo "   📊 Active processes: 0"
fi

# Show available scripts
echo ""
echo "🔷 Available Commands:"
echo "   📡 Connect:    ./connect_bluetooth.sh"
echo "   🔌 Disconnect: ./disconnect_bluetooth.sh"
echo "   📊 Status:     ./bluetooth_status.sh (this script)"
echo ""
echo "🔷 Test Scripts:"
echo "   🧪 Basic test: cd python_control && ./bluetooth_test.py"
echo "   🎛️  PID tuner:  cd python_control && ./pid_tuner.py"

# HC-05 LED indicator guide
echo ""
echo "🔷 HC-05 LED Indicators:"
echo "   💡 Fast blinking:  Not connected"
echo "   💡 Slow blinking:  Paired but not connected"  
echo "   💡 Solid/Very slow: Connected and communicating"
