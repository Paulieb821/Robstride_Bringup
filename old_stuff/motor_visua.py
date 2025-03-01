import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# Data storage for plotting
class MotorDataBuffer:
    def __init__(self, buffer_size=500):
        self.buffer_size = buffer_size
        self.time_data = deque(maxlen=buffer_size)
        self.motor_data = {}  # Dictionary to store data for each motor
    
    def initialize_motor(self, motor_id):
        if motor_id not in self.motor_data:
            self.motor_data[motor_id] = {
                'position': deque(maxlen=self.buffer_size),
                'velocity': deque(maxlen=self.buffer_size),
                'torque': deque(maxlen=self.buffer_size),
                'position_ref': deque(maxlen=self.buffer_size),
                'velocity_ref': deque(maxlen=self.buffer_size)
            }
    
    def add_data_point(self, time_point, motor_id, pos, vel, torque, pos_ref, vel_ref):
        # If this is the first data point, initialize time
        if not self.time_data:
            self.time_data.append(0)
        else:
            self.time_data.append(time_point)
        
        self.initialize_motor(motor_id)
        self.motor_data[motor_id]['position'].append(pos)
        self.motor_data[motor_id]['velocity'].append(vel)
        self.motor_data[motor_id]['torque'].append(torque)
        self.motor_data[motor_id]['position_ref'].append(pos_ref)
        self.motor_data[motor_id]['velocity_ref'].append(vel_ref)

# Visualization setup
class MotorVisualizer:
    def __init__(self, motor_ids):
        self.motor_ids = motor_ids
        self.data_buffer = MotorDataBuffer()
        self.start_time = time.time()
        
        # Create figure and subplots based on number of motors
        num_motors = len(motor_ids)
        self.fig, self.axes = plt.subplots(num_motors, 3, figsize=(15, 5*num_motors))
        if num_motors == 1:
            self.axes = [self.axes]  # Make axes indexable for single motor case
        
        # Initialize plots
        self.lines = {}
        for i, motor_id in enumerate(motor_ids):
            self.lines[motor_id] = {
                'position': self.axes[i][0].plot([], [], 'b-', label='Actual Position')[0],
                'position_ref': self.axes[i][0].plot([], [], 'r--', label='Reference Position')[0],
                'velocity': self.axes[i][1].plot([], [], 'b-', label='Actual Velocity')[0],
                'velocity_ref': self.axes[i][1].plot([], [], 'r--', label='Reference Velocity')[0],
                'torque': self.axes[i][2].plot([], [], 'g-', label='Command Torque')[0]
            }
            
            # Configure subplot titles and labels
            self.axes[i][0].set_title(f'Motor {motor_id} Position')
            self.axes[i][0].set_xlabel('Time (s)')
            self.axes[i][0].set_ylabel('Position (rad)')
            self.axes[i][0].grid(True)
            self.axes[i][0].legend()
            
            self.axes[i][1].set_title(f'Motor {motor_id} Velocity')
            self.axes[i][1].set_xlabel('Time (s)')
            self.axes[i][1].set_ylabel('Velocity (rad/s)')
            self.axes[i][1].grid(True)
            self.axes[i][1].legend()
            
            self.axes[i][2].set_title(f'Motor {motor_id} Torque')
            self.axes[i][2].set_xlabel('Time (s)')
            self.axes[i][2].set_ylabel('Torque (Nm)')
            self.axes[i][2].grid(True)
            self.axes[i][2].legend()
        
        plt.tight_layout()
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=50, blit=True)
    
    def update_plot(self, frame):
        lines_to_update = []
        
        for motor_id in self.motor_ids:
            if motor_id in self.data_buffer.motor_data:
                idx = self.motor_ids.index(motor_id)
                
                # Get time data
                time_data = list(self.data_buffer.time_data)
                
                # Update position plots
                self.lines[motor_id]['position'].set_data(time_data, self.data_buffer.motor_data[motor_id]['position'])
                self.lines[motor_id]['position_ref'].set_data(time_data, self.data_buffer.motor_data[motor_id]['position_ref'])
                
                # Update velocity plots
                self.lines[motor_id]['velocity'].set_data(time_data, self.data_buffer.motor_data[motor_id]['velocity'])
                self.lines[motor_id]['velocity_ref'].set_data(time_data, self.data_buffer.motor_data[motor_id]['velocity_ref'])
                
                # Update torque plot
                self.lines[motor_id]['torque'].set_data(time_data, self.data_buffer.motor_data[motor_id]['torque'])
                
                # Adjust axis limits dynamically
                for i, ax_type in enumerate(['position', 'velocity', 'torque']):
                    if time_data:
                        self.axes[idx][i].set_xlim(max(0, time_data[-1] - 10), max(10, time_data[-1]))
                        
                        if ax_type == 'position':
                            data_max = max(max(self.data_buffer.motor_data[motor_id]['position'], default=0),
                                          max(self.data_buffer.motor_data[motor_id]['position_ref'], default=0))
                            data_min = min(min(self.data_buffer.motor_data[motor_id]['position'], default=0),
                                          min(self.data_buffer.motor_data[motor_id]['position_ref'], default=0))
                        elif ax_type == 'velocity':
                            data_max = max(max(self.data_buffer.motor_data[motor_id]['velocity'], default=0),
                                          max(self.data_buffer.motor_data[motor_id]['velocity_ref'], default=0))
                            data_min = min(min(self.data_buffer.motor_data[motor_id]['velocity'], default=0),
                                          min(self.data_buffer.motor_data[motor_id]['velocity_ref'], default=0))
                        else:  # torque
                            data_max = max(self.data_buffer.motor_data[motor_id]['torque'], default=0)
                            data_min = min(self.data_buffer.motor_data[motor_id]['torque'], default=0)
                        
                        padding = max(0.1, (data_max - data_min) * 0.1) if data_max != data_min else 0.1
                        self.axes[idx][i].set_ylim(data_min - padding, data_max + padding)
                
                # Add all lines to the update list
                lines_to_update.extend([
                    self.lines[motor_id]['position'],
                    self.lines[motor_id]['position_ref'],
                    self.lines[motor_id]['velocity'],
                    self.lines[motor_id]['velocity_ref'],
                    self.lines[motor_id]['torque']
                ])
        
        return lines_to_update
    
    def show(self):
        plt.show(block=False)