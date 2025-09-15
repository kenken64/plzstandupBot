#!/bin/bash

# HC-05 Bluetooth Disconnection Script
# Disconnects the HC-05 module and releases /dev/rfcomm0

RFCOMM_DEVICE="0"

echo "🔌 HC-05 Bluetooth Disconnection Script"
echo "======================================="

# Check if RFCOMM connection exists
if ! sudo rfcomm show $RFCOMM_DEVICE 2>/dev/null | grep -q "rfcomm$RFCOMM_DEVICE:"; then
    echo "ℹ️  No RFCOMM connection found on device $RFCOMM_DEVICE"
    echo "✅ Already disconnected"
    exit 0
fi

# Show current connection status
echo "📋 Current connection status:"
sudo rfcomm show $RFCOMM_DEVICE

# Kill any active RFCOMM processes
echo ""
echo "🛑 Terminating RFCOMM processes..."
sudo pkill -f "rfcomm.*connect.*$RFCOMM_DEVICE" || true

# Wait a moment for processes to terminate
sleep 1

# Release the RFCOMM device
echo "🔓 Releasing RFCOMM device..."
sudo rfcomm release $RFCOMM_DEVICE 2>/dev/null || true

# Wait a moment
sleep 1

# Verify disconnection
if sudo rfcomm show $RFCOMM_DEVICE 2>/dev/null | grep -q "connected"; then
    echo "⚠️  Connection still active, trying force release..."
    
    # Force kill any remaining processes
    sudo pkill -9 -f rfcomm 2>/dev/null || true
    sleep 1
    
    # Try release again
    sudo rfcomm release $RFCOMM_DEVICE 2>/dev/null || true
    sleep 1
fi

# Final status check
echo ""
echo "🔍 Final status check..."
if sudo rfcomm show $RFCOMM_DEVICE 2>/dev/null | grep -q "connected"; then
    echo "❌ Failed to disconnect completely"
    echo "📋 Current status:"
    sudo rfcomm show $RFCOMM_DEVICE
    echo ""
    echo "🔧 Manual cleanup may be required:"
    echo "   sudo pkill -9 rfcomm"
    echo "   sudo rfcomm release $RFCOMM_DEVICE"
    exit 1
else
    echo "✅ Successfully disconnected from HC-05!"
    echo "📱 /dev/rfcomm$RFCOMM_DEVICE is now available for new connections"
    echo ""
    echo "💡 To reconnect, run: ./connect_bluetooth.sh"
fi

# Check if device file still exists
if [ -e "/dev/rfcomm$RFCOMM_DEVICE" ]; then
    echo "📄 Device file /dev/rfcomm$RFCOMM_DEVICE still exists (this is normal)"
else
    echo "🗑️  Device file /dev/rfcomm$RFCOMM_DEVICE removed"
fi

echo ""
echo "🎯 Ready for Arduino uploads or new Bluetooth connections!"
