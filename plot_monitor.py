#!/usr/bin/env python3
"""
Plot monitor.log data from PlzStandUp robot
Usage: python3 plot_monitor.py [logfile]
"""

import re
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def parse_log_line(line):
    """Parse a single log line and extract metrics"""
    data = {}
    
    # Extract pitch
    pitch_match = re.search(r'Pitch: ([\d.-]+)', line)
    if pitch_match:
        data['pitch'] = float(pitch_match.group(1))
    
    # Extract setpoint
    setpoint_match = re.search(r'Setpoint: ([\d.-]+)', line)
    if setpoint_match:
        data['setpoint'] = float(setpoint_match.group(1))
    
    # Extract error
    error_match = re.search(r'Error: ([-\d.]+)', line)
    if error_match:
        data['error'] = float(error_match.group(1))
    
    # Extract output
    output_match = re.search(r'Output: ([-\d.]+)', line)
    if output_match:
        data['output'] = float(output_match.group(1))
    
    # Extract tilt rate
    tilt_match = re.search(r'TiltRate: ([\d.-]+)', line)
    if tilt_match:
        data['tilt_rate'] = float(tilt_match.group(1))
    
    # Extract motor speed
    motor_match = re.search(r'MotorSpeed: ([-\d]+)', line)
    if motor_match:
        data['motor_speed'] = int(motor_match.group(1))
    
    return data

def plot_monitor_data(logfile):
    """Parse log file and create plots"""
    print(f"Parsing {logfile}...")
    
    # Data storage
    timestamps = []
    pitch = []
    setpoint = []
    error = []
    output = []
    tilt_rate = []
    motor_speed = []
    
    # Parse the log file
    with open(logfile, 'r') as f:
        sample_count = 0
        for line_num, line in enumerate(f):
            if 'Pitch:' in line:  # Only process lines with sensor data
                data = parse_log_line(line)
                
                if 'pitch' in data:  # Valid data line
                    timestamps.append(sample_count)
                    pitch.append(data.get('pitch', np.nan))
                    setpoint.append(data.get('setpoint', np.nan))
                    error.append(data.get('error', np.nan))
                    output.append(data.get('output', np.nan))
                    tilt_rate.append(data.get('tilt_rate', np.nan))
                    motor_speed.append(data.get('motor_speed', 0))
                    sample_count += 1
                    
                    if sample_count % 1000 == 0:
                        print(f"  Processed {sample_count} samples...")
    
    print(f"Parsed {sample_count} data points")
    
    # Convert to numpy arrays for easier plotting
    timestamps = np.array(timestamps)
    pitch = np.array(pitch)
    setpoint = np.array(setpoint)
    error = np.array(error)
    output = np.array(output)
    tilt_rate = np.array(tilt_rate)
    motor_speed = np.array(motor_speed)
    
    # Create subplots
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle(f'PlzStandUp Robot Performance Analysis - {os.path.basename(logfile)}', fontsize=16)
    
    # Plot 1: Pitch vs Setpoint
    axes[0,0].plot(timestamps, pitch, 'b-', label='Actual Pitch', alpha=0.7, linewidth=0.5)
    axes[0,0].plot(timestamps, setpoint, 'r--', label='Setpoint', linewidth=1)
    axes[0,0].set_ylabel('Angle (degrees)')
    axes[0,0].set_title('Pitch vs Setpoint')
    axes[0,0].legend()
    axes[0,0].grid(True, alpha=0.3)
    
    # Plot 2: Error over time
    axes[0,1].plot(timestamps, error, 'g-', linewidth=0.5)
    axes[0,1].axhline(y=0, color='k', linestyle='-', alpha=0.3)
    axes[0,1].set_ylabel('Error (degrees)')
    axes[0,1].set_title('Tracking Error')
    axes[0,1].grid(True, alpha=0.3)
    
    # Plot 3: PID Output and Motor Speed
    ax3 = axes[1,0]
    ax3.plot(timestamps, output, 'purple', label='PID Output', linewidth=0.7)
    ax3.set_ylabel('PID Output', color='purple')
    ax3.set_title('PID Output vs Motor Speed')
    ax3.grid(True, alpha=0.3)
    
    # Secondary y-axis for motor speed
    ax3_motor = ax3.twinx()
    ax3_motor.plot(timestamps, motor_speed, 'orange', label='Motor Speed', alpha=0.8, linewidth=0.7)
    ax3_motor.set_ylabel('Motor Speed', color='orange')
    
    # Plot 4: Tilt Rate (derivative-like signal)
    axes[1,1].plot(timestamps, tilt_rate, 'red', linewidth=0.5)
    axes[1,1].set_ylabel('Tilt Rate (deg/s)')
    axes[1,1].set_xlabel('Sample Number')
    axes[1,1].set_title('Tilt Rate (System Dynamics)')
    axes[1,1].grid(True, alpha=0.3)
    
    # Add sample number labels to all x-axes
    for ax in [axes[0,0], axes[0,1], axes[1,0]]:
        ax.set_xlabel('Sample Number')
    
    # Calculate and display stats
    valid_errors = error[~np.isnan(error)]
    if len(valid_errors) > 0:
        mean_abs_error = np.mean(np.abs(valid_errors))
        std_error = np.std(valid_errors)
        max_error = np.max(np.abs(valid_errors))
        
        stats_text = f'Stats: MAE={mean_abs_error:.2f}°, StdDev={std_error:.2f}°, MaxErr={max_error:.2f}°'
        fig.text(0.5, 0.02, stats_text, ha='center', fontsize=12, 
                bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
    
    plt.tight_layout()
    plt.subplots_adjust(top=0.93, bottom=0.1)
    
    # Save plot
    plot_filename = logfile.replace('.log', '_plot.png')
    plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
    print(f"Plot saved as: {plot_filename}")
    
    # Show plot
    plt.show()

def main():
    """Main function"""
    if len(sys.argv) > 1:
        logfile = sys.argv[1]
    else:
        logfile = 'monitor.log'
    
    if not os.path.exists(logfile):
        print(f"Error: Log file '{logfile}' not found!")
        print("Usage: python3 plot_monitor.py [logfile]")
        return
    
    try:
        plot_monitor_data(logfile)
    except Exception as e:
        print(f"Error plotting data: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
