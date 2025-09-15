#!/bin/bash
# Quick GUI launcher - just starts the GUI with minimal setup

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTROL_DIR="$SCRIPT_DIR/python_control"

cd "$CONTROL_DIR"

# Check if virtual environment exists, if so use it
VENV_DIR="$CONTROL_DIR/venv"
if [ -d "$VENV_DIR" ]; then
    echo "Using virtual environment..."
    source "$VENV_DIR/bin/activate"
    python robot_gui.py
else
    echo "Virtual environment not found. Please run: ./start_robot_control.sh setup"
    echo "Trying with system Python..."
    python3 robot_gui.py
fi
