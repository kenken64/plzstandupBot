#!/usr/bin/env python3
"""
Advanced Bluetooth Robot GUI Controller
Provides a graphical interface for controlling the balancing robot with:
- Real-time telemetry visualization
- PID gain tuning sliders
- Joystick-style control
- Data logging and plotting
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import numpy as np
import threading
import time
import json
from datetime import datetime
from collections import deque
import sys
import os

# Add the current directory to path to import robot_client
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from robot_client import BluetoothRobotClient

class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Balancing Robot Controller")
        self.root.geometry("1200x800")
        
        # Robot client
        self.robot = BluetoothRobotClient()
        self.connected = False
        
        # Data storage for plotting
        self.max_data_points = 1000
        self.timestamps = deque(maxlen=self.max_data_points)
        self.pitch_data = deque(maxlen=self.max_data_points)
        self.setpoint_data = deque(maxlen=self.max_data_points)
        self.error_data = deque(maxlen=self.max_data_points)
        self.output_data = deque(maxlen=self.max_data_points)
        self.tilt_rate_data = deque(maxlen=self.max_data_points)
        
        # Control variables
        self.speed_var = tk.DoubleVar(value=0)
        self.steering_var = tk.DoubleVar(value=0)
        self.kp_var = tk.DoubleVar(value=12.0)
        self.ki_var = tk.DoubleVar(value=0.015)
        self.kd_var = tk.DoubleVar(value=2.5)
        
        # GUI state
        self.logging_enabled = tk.BooleanVar(value=False)
        self.log_data = []
        
        self.setup_gui()
        self.setup_plots()
        
        # Start GUI update loop
        self.update_gui()
        
    def setup_gui(self):
        """Setup the main GUI layout"""
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(1, weight=1)
        
        # Connection frame
        self.setup_connection_frame(main_frame)
        
        # Control frame
        self.setup_control_frame(main_frame)
        
        # Telemetry frame
        self.setup_telemetry_frame(main_frame)
        
        # Plot frame
        self.setup_plot_frame(main_frame)
        
    def setup_connection_frame(self, parent):
        """Setup connection controls"""
        conn_frame = ttk.LabelFrame(parent, text="Connection", padding="5")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky=tk.W)
        self.port_var = tk.StringVar(value="Auto-detect")
        port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, state="readonly", width=20)
        port_combo.grid(row=0, column=1, padx=(5, 10))
        # Populate available ports
        import serial.tools.list_ports
        ports = ["Auto-detect"] + [port.device for port in serial.tools.list_ports.comports()]
        port_combo['values'] = ports
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5)
        
        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=3, padx=(10, 0))
        
    def setup_control_frame(self, parent):
        """Setup robot control interface"""
        control_frame = ttk.LabelFrame(parent, text="Robot Control", padding="5")
        control_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        
        # Speed control
        ttk.Label(control_frame, text="Speed:").grid(row=0, column=0, sticky=tk.W)
        speed_scale = ttk.Scale(control_frame, from_=-50, to=50, orient=tk.HORIZONTAL,
                               variable=self.speed_var, command=self.on_speed_change)
        speed_scale.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=5)
        self.speed_label = ttk.Label(control_frame, text="0.0")
        self.speed_label.grid(row=0, column=2, padx=5)
        
        # Steering control
        ttk.Label(control_frame, text="Steering:").grid(row=1, column=0, sticky=tk.W, pady=(10, 0))
        steering_scale = ttk.Scale(control_frame, from_=-50, to=50, orient=tk.HORIZONTAL,
                                  variable=self.steering_var, command=self.on_steering_change)
        steering_scale.grid(row=1, column=1, sticky=(tk.W, tk.E), padx=5, pady=(10, 0))
        self.steering_label = ttk.Label(control_frame, text="0.0")
        self.steering_label.grid(row=1, column=2, padx=5, pady=(10, 0))
        
        # Control buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=2, column=0, columnspan=3, pady=10)
        
        ttk.Button(button_frame, text="Stop", command=self.stop_robot).grid(row=0, column=0, padx=5)
        ttk.Button(button_frame, text="Reset", command=self.reset_controls).grid(row=0, column=1, padx=5)
        
        # PID Tuning
        pid_frame = ttk.LabelFrame(control_frame, text="PID Tuning", padding="5")
        pid_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(10, 0))
        
        # Kp
        ttk.Label(pid_frame, text="Kp:").grid(row=0, column=0, sticky=tk.W)
        kp_scale = ttk.Scale(pid_frame, from_=0, to=30, orient=tk.HORIZONTAL,
                            variable=self.kp_var, command=self.on_pid_change)
        kp_scale.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=5)
        self.kp_label = ttk.Label(pid_frame, text="12.0")
        self.kp_label.grid(row=0, column=2, padx=5)
        
        # Ki
        ttk.Label(pid_frame, text="Ki:").grid(row=1, column=0, sticky=tk.W)
        ki_scale = ttk.Scale(pid_frame, from_=0, to=0.1, orient=tk.HORIZONTAL,
                            variable=self.ki_var, command=self.on_pid_change)
        ki_scale.grid(row=1, column=1, sticky=(tk.W, tk.E), padx=5)
        self.ki_label = ttk.Label(pid_frame, text="0.015")
        self.ki_label.grid(row=1, column=2, padx=5)
        
        # Kd
        ttk.Label(pid_frame, text="Kd:").grid(row=2, column=0, sticky=tk.W)
        kd_scale = ttk.Scale(pid_frame, from_=0, to=10, orient=tk.HORIZONTAL,
                            variable=self.kd_var, command=self.on_pid_change)
        kd_scale.grid(row=2, column=1, sticky=(tk.W, tk.E), padx=5)
        self.kd_label = ttk.Label(pid_frame, text="2.5")
        self.kd_label.grid(row=2, column=2, padx=5)
        
        # Configure column weights
        control_frame.columnconfigure(1, weight=1)
        pid_frame.columnconfigure(1, weight=1)
        
    def setup_telemetry_frame(self, parent):
        """Setup telemetry display"""
        telemetry_frame = ttk.LabelFrame(parent, text="Telemetry", padding="5")
        telemetry_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(10, 0))
        
        # Create telemetry labels
        self.tel_labels = {}
        labels = ["Pitch", "Setpoint", "Error", "PID Output", "Tilt Rate", "Speed Cmd", "Steer Cmd"]
        
        for i, label in enumerate(labels):
            ttk.Label(telemetry_frame, text=f"{label}:").grid(row=i, column=0, sticky=tk.W, pady=2)
            value_label = ttk.Label(telemetry_frame, text="--", font=("Consolas", 10))
            value_label.grid(row=i, column=1, sticky=tk.W, padx=10, pady=2)
            self.tel_labels[label.lower().replace(" ", "_")] = value_label
        
        # Data logging controls
        log_frame = ttk.LabelFrame(telemetry_frame, text="Data Logging", padding="5")
        log_frame.grid(row=len(labels), column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(10, 0))
        
        ttk.Checkbutton(log_frame, text="Enable Logging", variable=self.logging_enabled).grid(row=0, column=0, sticky=tk.W)
        ttk.Button(log_frame, text="Save Log", command=self.save_log).grid(row=0, column=1, padx=10)
        ttk.Button(log_frame, text="Clear Log", command=self.clear_log).grid(row=0, column=2, padx=10)
        
        self.log_status = ttk.Label(log_frame, text="0 records")
        self.log_status.grid(row=1, column=0, columnspan=3, pady=5)
        
    def setup_plot_frame(self, parent):
        """Setup real-time plotting"""
        plot_frame = ttk.LabelFrame(parent, text="Real-time Plots", padding="5")
        plot_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(10, 0))
        
        # Create matplotlib figure
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 6))
        self.fig.tight_layout(pad=3.0)
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure plot frame weights
        plot_frame.columnconfigure(0, weight=1)
        plot_frame.rowconfigure(0, weight=1)
        
    def setup_plots(self):
        """Initialize plot configuration"""
        # Plot 1: Pitch and Setpoint
        self.ax1.set_title("Robot Balance")
        self.ax1.set_ylabel("Angle (degrees)")
        self.ax1.grid(True)
        self.ax1.legend(["Pitch", "Setpoint", "Error"])
        
        # Plot 2: PID Output and Tilt Rate
        self.ax2.set_title("Control Signals")
        self.ax2.set_ylabel("Output / Rate")
        self.ax2.set_xlabel("Time (s)")
        self.ax2.grid(True)
        self.ax2.legend(["PID Output", "Tilt Rate"])
        
        # Initialize empty lines
        self.pitch_line, = self.ax1.plot([], [], 'b-', label='Pitch')
        self.setpoint_line, = self.ax1.plot([], [], 'g--', label='Setpoint')
        self.error_line, = self.ax1.plot([], [], 'r-', label='Error')
        self.output_line, = self.ax2.plot([], [], 'm-', label='PID Output')
        self.tilt_rate_line, = self.ax2.plot([], [], 'c-', label='Tilt Rate')
        
    def toggle_connection(self):
        """Toggle robot connection"""
        if not self.connected:
            port = None if self.port_var.get() == "Auto-detect" else self.port_var.get()
            if self.robot.connect(port):
                self.connected = True
                self.connect_btn.config(text="Disconnect")
                self.status_label.config(text="Connected", foreground="green")
            else:
                messagebox.showerror("Connection Error", "Failed to connect to robot")
        else:
            self.robot.disconnect()
            self.connected = False
            self.connect_btn.config(text="Connect")
            self.status_label.config(text="Disconnected", foreground="red")
    
    def on_speed_change(self, value):
        """Handle speed slider change"""
        speed = float(value)
        self.speed_label.config(text=f"{speed:.1f}")
        if self.connected:
            self.robot.set_speed(speed)
    
    def on_steering_change(self, value):
        """Handle steering slider change"""
        steering = float(value)
        self.steering_label.config(text=f"{steering:.1f}")
        if self.connected:
            self.robot.set_steering(steering)
    
    def on_pid_change(self, value):
        """Handle PID gain changes"""
        kp = self.kp_var.get()
        ki = self.ki_var.get()
        kd = self.kd_var.get()
        
        self.kp_label.config(text=f"{kp:.3f}")
        self.ki_label.config(text=f"{ki:.3f}")
        self.kd_label.config(text=f"{kd:.3f}")
        
        if self.connected:
            self.robot.set_pid(kp, ki, kd)
    
    def stop_robot(self):
        """Emergency stop"""
        if self.connected:
            self.robot.stop()
        self.speed_var.set(0)
        self.steering_var.set(0)
    
    def reset_controls(self):
        """Reset all controls to default"""
        self.speed_var.set(0)
        self.steering_var.set(0)
        if self.connected:
            self.robot.stop()
    
    def save_log(self):
        """Save logged data to file"""
        if not self.log_data:
            messagebox.showwarning("No Data", "No data to save")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("CSV files", "*.csv"), ("All files", "*.*")]
        )
        
        if filename:
            try:
                if filename.endswith('.csv'):
                    import csv
                    with open(filename, 'w', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(['timestamp', 'pitch', 'setpoint', 'error', 'output', 'tilt_rate', 'steering', 'speed'])
                        for record in self.log_data:
                            writer.writerow([
                                record['timestamp'], record['pitch'], record['setpoint'],
                                record['error'], record['output'], record['tilt_rate'],
                                record['steering'], record['speed']
                            ])
                else:
                    with open(filename, 'w') as f:
                        json.dump(self.log_data, f, indent=2)
                
                messagebox.showinfo("Save Successful", f"Data saved to {filename}")
            except Exception as e:
                messagebox.showerror("Save Error", f"Failed to save data: {e}")
    
    def clear_log(self):
        """Clear logged data"""
        self.log_data.clear()
        self.timestamps.clear()
        self.pitch_data.clear()
        self.setpoint_data.clear()
        self.error_data.clear()
        self.output_data.clear()
        self.tilt_rate_data.clear()
    
    def update_gui(self):
        """Main GUI update loop"""
        if self.connected:
            # Get telemetry data
            telemetry = self.robot.get_telemetry()
            
            if telemetry:
                # Update telemetry display
                self.tel_labels['pitch'].config(text=f"{telemetry['pitch']:.2f}°")
                self.tel_labels['setpoint'].config(text=f"{telemetry['setpoint']:.2f}°")
                self.tel_labels['error'].config(text=f"{telemetry['error']:.2f}°")
                self.tel_labels['pid_output'].config(text=f"{telemetry['output']:.1f}")
                self.tel_labels['tilt_rate'].config(text=f"{telemetry['tilt_rate']:.1f}°/s")
                self.tel_labels['speed_cmd'].config(text=f"{telemetry['speed']:.1f}")
                self.tel_labels['steer_cmd'].config(text=f"{telemetry['steering']:.1f}")
                
                # Update plot data
                current_time = time.time()
                if not self.timestamps or current_time - self.timestamps[-1] > 0.05:  # 20Hz update
                    self.timestamps.append(current_time)
                    self.pitch_data.append(telemetry['pitch'])
                    self.setpoint_data.append(telemetry['setpoint'])
                    self.error_data.append(telemetry['error'])
                    self.output_data.append(telemetry['output'])
                    self.tilt_rate_data.append(telemetry['tilt_rate'])
                    
                    # Log data if enabled
                    if self.logging_enabled.get():
                        self.log_data.append(telemetry.copy())
                        self.log_status.config(text=f"{len(self.log_data)} records")
                
                # Update plots
                self.update_plots()
        
        # Schedule next update
        self.root.after(50, self.update_gui)  # 20Hz update rate
    
    def update_plots(self):
        """Update real-time plots"""
        if len(self.timestamps) < 2:
            return
        
        # Convert timestamps to relative seconds
        base_time = self.timestamps[0]
        time_axis = [(t - base_time) for t in self.timestamps]
        
        # Update plot 1: Balance data
        self.pitch_line.set_data(time_axis, list(self.pitch_data))
        self.setpoint_line.set_data(time_axis, list(self.setpoint_data))
        self.error_line.set_data(time_axis, list(self.error_data))
        
        # Update plot 2: Control signals
        self.output_line.set_data(time_axis, list(self.output_data))
        self.tilt_rate_line.set_data(time_axis, list(self.tilt_rate_data))
        
        # Auto-scale axes
        if time_axis:
            for ax in [self.ax1, self.ax2]:
                ax.set_xlim(max(0, time_axis[-1] - 30), time_axis[-1] + 1)  # Show last 30 seconds
                ax.relim()
                ax.autoscale_view()
        
        # Refresh canvas
        self.canvas.draw_idle()
    
    def on_closing(self):
        """Handle application closing"""
        if self.connected:
            self.robot.disconnect()
        self.root.destroy()


def main():
    """Main application entry point"""
    root = tk.Tk()
    app = RobotGUI(root)
    
    # Handle window closing
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    # Bind keyboard shortcuts
    def on_key_press(event):
        key = event.keysym.lower()
        if key == 'space':
            app.stop_robot()
        elif key == 'r':
            app.reset_controls()
    
    root.bind('<Key>', on_key_press)
    root.focus_set()  # Enable keyboard input
    
    root.mainloop()


if __name__ == "__main__":
    main()
