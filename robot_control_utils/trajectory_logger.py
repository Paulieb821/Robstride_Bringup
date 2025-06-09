import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import os
import json
from typing import List, Dict, Optional
import pandas as pd

class TrajectoryLogger:
    def __init__(self, motor_ids: List[int], kp: float, kp2:float,  kd: float, kd2: float, save_dir: str = "trajectory_logs"):
        self.motor_ids = motor_ids
        self.kp = kp
        self.kd = kd
        self.kp2 = kp2
        self.kd2 = kd2
        self.save_dir = save_dir
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Create save directory if it doesn't exist
        self.run_dir = os.path.join(save_dir, f"KpOne-{kp}_kdOne-{kd}_KpTwo-{kp2}_KdTwo-{kd2}_{self.timestamp}")
        os.makedirs(self.run_dir, exist_ok=True)
        
        # Initialize data storage
        self.data = {
            'time': [],
            'position': {id: [] for id in motor_ids},
            'velocity': {id: [] for id in motor_ids},
            'torque': {id: [] for id in motor_ids},
            'position_ref': {id: [] for id in motor_ids},
            'velocity_ref': {id: [] for id in motor_ids},
            'errors': {id: [] for id in motor_ids}
        }
        
        # Performance metrics
        self.metrics = {id: {} for id in motor_ids}
        
    def log_data(self, time: float, feedback_data: Dict[int, Dict], ref_data: Dict[int, Dict]):
        """Log data for each motor"""
        self.data['time'].append(time)
        
        for motor_id in self.motor_ids:
            if motor_id in feedback_data:
                self.data['position'][motor_id].append(feedback_data[motor_id]['angle'])
                self.data['velocity'][motor_id].append(feedback_data[motor_id]['velocity'])
                self.data['torque'][motor_id].append(feedback_data[motor_id]['torque'])
                self.data['position_ref'][motor_id].append(ref_data[motor_id]['position'])
                self.data['velocity_ref'][motor_id].append(ref_data[motor_id]['velocity'])
                self.data['errors'][motor_id].append(feedback_data[motor_id]['errors'])
    
    def calculate_metrics(self):
        """Calculate performance metrics for each motor"""
        for motor_id in self.motor_ids:
            pos = np.array(self.data['position'][motor_id])
            pos_ref = np.array(self.data['position_ref'][motor_id])
            vel = np.array(self.data['velocity'][motor_id])
            vel_ref = np.array(self.data['velocity_ref'][motor_id])
            time = np.array(self.data['time'])
            
            # Calculate position error
            pos_error = pos - pos_ref
            
            # Calculate metrics
            self.metrics[motor_id] = {
                'max_overshoot': np.max(np.abs(pos_error)),
                'rise_time': self._calculate_rise_time(time, pos, pos_ref),
                'settling_time': self._calculate_settling_time(time, pos_error),
                'steady_state_error': np.mean(np.abs(pos_error[-100:])),  # Last 100 points
                'max_velocity_error': np.max(np.abs(vel - vel_ref)),
                'rms_position_error': np.sqrt(np.mean(pos_error**2)),
                'rms_velocity_error': np.sqrt(np.mean((vel - vel_ref)**2))
            }
    
    def _calculate_rise_time(self, time, pos, pos_ref):
        """Calculate rise time (time to reach 90% of final value)"""
        final_pos = pos_ref[-1]
        target = 0.9 * final_pos
        for i, p in enumerate(pos):
            if p >= target:
                return time[i]
        return np.nan
    
    def _calculate_settling_time(self, time, error, threshold=0.05):
        """Calculate settling time (time to stay within threshold)"""
        for i in range(len(error)):
            if np.all(np.abs(error[i:]) < threshold):
                return time[i]
        return np.nan
    
    def plot_results(self):
        """Generate and save plots for each motor"""
        for motor_id in self.motor_ids:
            fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 12))
            time = np.array(self.data['time'])
            
            # Position plot
            ax1.plot(time, self.data['position'][motor_id], label='Actual')
            ax1.plot(time, self.data['position_ref'][motor_id], label='Reference', linestyle='--')
            if motor_id == 2:
                ax1.set_title(f'Motor {motor_id} Position (Kp={self.kp2}, Kd={self.kd2})')
            else:
                ax1.set_title(f'Motor {motor_id} Position (Kp={self.kp}, Kd={self.kd})')
            ax1.set_ylabel('Position (rad)')
            ax1.legend()
            ax1.grid(True)
            
            # Velocity plot
            ax2.plot(time, self.data['velocity'][motor_id], label='Actual')
            ax2.plot(time, self.data['velocity_ref'][motor_id], label='Reference', linestyle='--')
            ax2.set_title('Velocity')
            ax2.set_ylabel('Velocity (rad/s)')
            ax2.legend()
            ax2.grid(True)
            
            # Torque plot
            ax3.plot(time, self.data['torque'][motor_id], label='Torque')
            ax3.set_title('Torque')
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Torque (Nm)')
            ax3.legend()
            ax3.grid(True)
            
            plt.tight_layout()
            plt.savefig(os.path.join(self.run_dir, f'motor_{motor_id}_plots.png'))
            plt.close()
    
    def save_metrics(self):
        """Save metrics to a JSON file"""
        metrics_file = os.path.join(self.run_dir, 'metrics.json')
        with open(metrics_file, 'w') as f:
            json.dump({
                'kp': self.kp,
                'kd': self.kd,
                'timestamp': self.timestamp,
                'metrics': self.metrics
            }, f, indent=4)
    
    def save_raw_data(self):
        """Save raw data to CSV files"""
        for motor_id in self.motor_ids:
            df = pd.DataFrame({
                'time': self.data['time'],
                'position': self.data['position'][motor_id],
                'velocity': self.data['velocity'][motor_id],
                'torque': self.data['torque'][motor_id],
                'position_ref': self.data['position_ref'][motor_id],
                'velocity_ref': self.data['velocity_ref'][motor_id]
            })
            df.to_csv(os.path.join(self.run_dir, f'motor_{motor_id}_data.csv'), index=False)
    
    def finalize(self):
        """Calculate metrics, generate plots, and save all data"""
        self.calculate_metrics()
        self.plot_results()
        self.save_metrics()
        self.save_raw_data() 