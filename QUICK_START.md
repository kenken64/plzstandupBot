# 🤖 Quick Start Guide

## **Easy Launch Options:**

### **🎮 GUI Control (Recommended)**
```bash
./start_robot_control.sh         # Auto-setup + GUI
./start_gui.sh                   # Quick GUI (minimal setup)
```

### **💻 Command Line**
```bash
./start_robot_control.sh cli     # CLI with auto-detect
./start_robot_control.sh cli /dev/rfcomm0  # CLI with specific port
```

### **🧪 Connection Test**
```bash
./start_robot_control.sh test    # Test Bluetooth connection
```

### **⚙️ Setup & Diagnostics**
```bash
./start_robot_control.sh setup   # Install dependencies
./start_robot_control.sh list    # List Bluetooth devices
```

---

## **💡 Pro Tips:**

### **First Time Setup:**
1. Connect HC-05 to Arduino (A0, A1, A2)
2. Upload enhanced `main.cpp` to Arduino
3. Run: `./start_robot_control.sh setup`
4. Pair HC-05 in Ubuntu Bluetooth settings
5. Run: `./start_robot_control.sh gui`

### **Upload Safety:**
- Ground pin A2 during firmware uploads
- Or disconnect HC-05 VCC temporarily

### **Troubleshooting:**
- **Permission errors**: Script will add you to `dialout` group
- **Dependencies missing**: Script will auto-install them
- **Port not found**: Use `./start_robot_control.sh list` to find ports
- **Connection issues**: Check Bluetooth pairing in Ubuntu settings

---

## **🎯 Robot Commands:**

**Via GUI:** Use sliders and buttons

**Via CLI:** Type these commands:
- `w/s` - Forward/backward speed
- `a/d` - Left/right steering  
- `x` - Emergency stop
- `p` - Tune PID gains
- `t` - Show telemetry
- `q` - Quit

**Direct Commands:**
- `SPEED:10` - Set speed to 10
- `STEER:-5` - Steer left 5
- `PID:12.0,0.015,2.5` - Set PID gains
- `STOP` - Stop all movement
- `STATUS` - Get robot status

---

## **📁 File Structure:**
```
/firmware/
├── src/main.cpp                 # Enhanced firmware (upload this!)
├── start_robot_control.sh       # Full-featured launcher
├── start_gui.sh                 # Quick GUI launcher
├── Robot_Control.desktop        # Desktop shortcut
└── python_control/              # Python control scripts
    ├── robot_gui.py             # Main GUI application
    ├── robot_client.py          # Command-line interface
    ├── test_connection.py       # Connection tester
    └── requirements.txt         # Python dependencies
```

**Ready to control your robot! 🚀**
