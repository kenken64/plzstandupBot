#!/bin/bash

# Balancing Robot Control Launcher
# Starts the GUI control interface with automatic setup

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTROL_DIR="$SCRIPT_DIR/python_control"

echo -e "${BLUE}🤖 Balancing Robot Control Launcher${NC}"
echo "=================================================="

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check Python installation
check_python() {
    echo -e "${YELLOW}🔍 Checking Python installation...${NC}"
    
    if command_exists python3; then
        PYTHON_CMD="python3"
        echo -e "${GREEN}✅ Python3 found: $(python3 --version)${NC}"
    elif command_exists python; then
        PYTHON_CMD="python"
        echo -e "${GREEN}✅ Python found: $(python --version)${NC}"
    else
        echo -e "${RED}❌ Python not found! Please install Python 3.7+${NC}"
        echo "Install with: sudo apt update && sudo apt install python3 python3-pip"
        exit 1
    fi
}

# Function to check and install dependencies
check_dependencies() {
    echo -e "${YELLOW}🔍 Checking Python dependencies...${NC}"
    
    # Change to control directory
    cd "$CONTROL_DIR"
    
    # Check if requirements.txt exists
    if [ ! -f "requirements.txt" ]; then
        echo -e "${RED}❌ requirements.txt not found in $CONTROL_DIR${NC}"
        exit 1
    fi
    
    # Check for virtual environment
    VENV_DIR="$CONTROL_DIR/venv"
    if [ ! -d "$VENV_DIR" ]; then
        echo -e "${YELLOW}📦 Creating virtual environment...${NC}"
        $PYTHON_CMD -m venv "$VENV_DIR"
        echo -e "${GREEN}✅ Virtual environment created${NC}"
    fi
    
    # Activate virtual environment
    source "$VENV_DIR/bin/activate"
    PYTHON_CMD="$VENV_DIR/bin/python"
    
    # Check if dependencies are installed
    missing_deps=()
    while IFS= read -r line; do
        # Skip comments and empty lines
        [[ $line =~ ^#.*$ ]] && continue
        [[ -z $line ]] && continue
        [[ $line == "tkinter" ]] && continue  # tkinter is built-in, skip
        
        # Extract package name (before >= or ==)
        package=$(echo "$line" | sed 's/[><=].*//')
        
        # Handle package name mapping (install name vs import name)
        import_name="$package"
        case "$package" in
            "pyserial") import_name="serial" ;;
        esac
        
        if ! $PYTHON_CMD -c "import $import_name" 2>/dev/null; then
            missing_deps+=("$line")
        fi
    done < requirements.txt
    
    if [ ${#missing_deps[@]} -ne 0 ]; then
        echo -e "${YELLOW}📦 Installing missing dependencies in virtual environment...${NC}"
        $PYTHON_CMD -m pip install --upgrade pip
        $PYTHON_CMD -m pip install "${missing_deps[@]}"
        echo -e "${GREEN}✅ Dependencies installed in virtual environment${NC}"
    else
        echo -e "${GREEN}✅ All dependencies satisfied${NC}"
    fi
}

# Function to check Bluetooth permissions
check_bluetooth() {
    echo -e "${YELLOW}🔍 Checking Bluetooth permissions...${NC}"
    
    # Check if user is in dialout group (needed for serial port access)
    if groups $USER | grep -q '\bdialout\b'; then
        echo -e "${GREEN}✅ User has serial port permissions${NC}"
    else
        echo -e "${YELLOW}⚠️  Adding user to dialout group for serial port access...${NC}"
        echo "This requires sudo permission:"
        sudo usermod -a -G dialout $USER
        echo -e "${YELLOW}⚠️  Please log out and log back in for group changes to take effect${NC}"
        echo "Or run: newgrp dialout"
    fi
}

# Function to list available Bluetooth devices
list_bluetooth_devices() {
    echo -e "${YELLOW}🔍 Scanning for Bluetooth devices...${NC}"
    
    # Check if bluetoothctl is available
    if command_exists bluetoothctl; then
        echo "Paired Bluetooth devices:"
        timeout 5 bluetoothctl -- paired-devices 2>/dev/null || echo "No paired devices or bluetoothctl not accessible"
    fi
    
    # List available serial ports
    echo -e "\n${YELLOW}Available serial ports:${NC}"
    
    # Look for Arduino and Bluetooth serial devices
    arduino_ports=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -5)
    bluetooth_ports=$(ls /dev/rfcomm* 2>/dev/null | head -5)
    
    if [ -n "$arduino_ports" ]; then
        echo -e "${GREEN}Arduino/USB devices:${NC}"
        for port in $arduino_ports; do
            echo "  $port"
        done
    fi
    
    if [ -n "$bluetooth_ports" ]; then
        echo -e "${GREEN}Bluetooth serial devices:${NC}"
        for port in $bluetooth_ports; do
            echo "  $port"
        done
    fi
    
    if [ -z "$arduino_ports" ] && [ -z "$bluetooth_ports" ]; then
        echo "No Arduino or Bluetooth serial devices found."
        echo "Make sure your robot is connected and/or Bluetooth is paired."
    fi
}

# Function to start the control interface
start_control() {
    local interface="$1"
    
    cd "$CONTROL_DIR"
    
    # Ensure we're using virtual environment Python
    VENV_DIR="$CONTROL_DIR/venv"
    if [ -d "$VENV_DIR" ]; then
        source "$VENV_DIR/bin/activate"
        PYTHON_CMD="$VENV_DIR/bin/python"
    fi
    
    case "$interface" in
        "gui"|"")  
            echo -e "${GREEN}🚀 Starting GUI Control Interface...${NC}"
            echo "Close the GUI window to exit."
            echo ""
            $PYTHON_CMD robot_gui.py
            ;;
        "cli")
            echo -e "${GREEN}🚀 Starting Command-Line Interface...${NC}"
            echo "Type 'q' to quit when connected."
            echo ""
            $PYTHON_CMD robot_client.py "$2"
            ;;
        "test")
            echo -e "${GREEN}🧪 Starting Connection Test...${NC}"
            echo ""
            $PYTHON_CMD test_connection.py "$2"
            ;;
        *)
            echo -e "${RED}❌ Unknown interface: $interface${NC}"
            show_usage
            exit 1
            ;;
    esac
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTION] [PORT]"
    echo ""
    echo "Options:"
    echo "  gui         Start GUI control interface (default)"
    echo "  cli [port]  Start command-line interface"
    echo "  test [port] Test Bluetooth connection"
    echo "  list        List available Bluetooth devices"
    echo "  setup       Install dependencies and check setup"
    echo "  help        Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                    # Start GUI (auto-detect port)"
    echo "  $0 gui                # Start GUI explicitly"
    echo "  $0 cli                # Start CLI with auto-detect"
    echo "  $0 cli /dev/rfcomm0   # Start CLI with specific port"
    echo "  $0 test               # Test connection"
    echo "  $0 list               # Show available devices"
}

# Main script logic
main() {
    case "$1" in
        "help"|"-h"|"--help")
            show_usage
            exit 0
            ;;
        "list")
            list_bluetooth_devices
            exit 0
            ;;
        "setup")
            check_python
            check_dependencies
            check_bluetooth
            echo -e "${GREEN}✅ Setup complete!${NC}"
            exit 0
            ;;
        "gui"|"cli"|"test"|"")
            # Setup and run
            check_python
            check_dependencies
            check_bluetooth
            list_bluetooth_devices
            echo ""
            start_control "$1" "$2"
            ;;
        *)
            echo -e "${RED}❌ Unknown option: $1${NC}"
            echo ""
            show_usage
            exit 1
            ;;
    esac
}

# Trap Ctrl+C for clean exit
trap 'echo -e "\n${YELLOW}🛑 Control interface stopped${NC}"; exit 0' INT

# Check if control directory exists
if [ ! -d "$CONTROL_DIR" ]; then
    echo -e "${RED}❌ Control directory not found: $CONTROL_DIR${NC}"
    echo "Make sure you're running this script from the firmware directory."
    exit 1
fi

# Run main function with all arguments
main "$@"
