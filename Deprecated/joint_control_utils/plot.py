
import matplotlib.pyplot as plt
import numpy as np


class Plot:

    def __init__(self, data):
        '''
        self.data = nested dictionary containty all the telemetry
        Format = {motor_id: {"time": [], "position": [], "velocity": [], "torque": [], "pos_ref":[], "vel_ref":[], "vel_pred":[], "pos_pred":[]} for motor_id in motor_ids}        

        '''
        self.data = data

    def plot_motor_data(self, K=None, L=None):
        '''
        K = controller gains (npdarray or list)
        L = estimated gains (npdarray or list)
        failure_count = default none, otherwise pass in the count
        '''
        for id, _ in self.data.items():
            if len(self.data[id]['time']) > 0:
                print("Current id : ", id)
                print("length of self.data[1][time] : ", len(self.data[id]["time"]))

                # Create vector of times between each sample and print
                time_vector = np.diff(self.data[id]["time"])
                mean_time_vector = np.mean(time_vector)
                stdev_time_vector = np.std(time_vector)
                print("Mean time vector : ", mean_time_vector*1000, " ms")
                print("Standard deviation time vector : ", stdev_time_vector*1000, " ms")
                print("Last value of time vector : ", self.data[id]["time"][-1])

                plt.figure(figsize=(12, 8))

                # position plot
                plt.subplot(3, 1, 1)
                plt.scatter(self.data[id]["time"], self.data[id]["position"], label="Position Raw", color='blue', s=5)
                plt.plot(self.data[id]["time"], self.data[id]["position"], label="Position", color='blue')
                plt.plot(self.data[id]["time"], self.data[id]["pos_ref"], label="Position Ref", color='black')
                plt.scatter(self.data[id]["time"], self.data[id]["pos_pred"], label="Position Pred Raw", color='red', s=5)
                plt.plot(self.data[id]["time"], self.data[id]["pos_pred"], label="Position Pred", color='red')

                # velocity plot
                plt.subplot(3, 1, 2)
                plt.scatter(self.data[id]["time"], self.data[id]["velocity"], label="Velocity Raw", color='blue', s=5)
                plt.plot(self.data[id]["time"], self.data[id]["velocity"], label="Velocity", color='blue')
                plt.plot(self.data[id]["time"], self.data[id]["vel_ref"], label="Velocity Ref", color='black')
                plt.scatter(self.data[id]["time"], self.data[id]["vel_pred"], label="Velocity Pred Raw", color='red', s=5)
                plt.plot(self.data[id]["time"], self.data[id]["vel_pred"], label="Velocity Pred", color='red')
                
                plt.subplot(3, 1, 3)
                torque_bound_pos = np.ones(len(self.data[id]["time"])) * 0.014
                torque_bound_neg = np.ones(len(self.data[id]["time"])) * -0.014
                plt.scatter(self.data[id]["time"], self.data[id]["torque"], label="Torque", color='red', s=5)
                plt.step(self.data[id]["time"], self.data[id]["torque"], label="Torque", color='green')
                plt.plot(self.data[id]["time"], torque_bound_pos, label="Torque Bound Pos", linestyle='--', color='black')
                plt.plot(self.data[id]["time"], torque_bound_neg, label="Torque Bound Neg", linestyle='--', color='black')
                # plt.plot(self.data[1]["time"], self.data[1]["raw_torque"], label="Command Raw Torque", color='purple')  UNCOMMENT IF RAW TORQUE NEEDED
                plt.title("Motor Torque")
                plt.xlabel("Time (s)")
                plt.ylabel("Torque (Nm)")
                plt.grid()
                plt.legend()

            # Gains display

            # plt.tight_layout()
            # s = f"Kp = {round(K[0], 3)}, Kd = {round(K[1], 3)}, Ki = {round(K[2], 2)}"
            # plt.gcf().text(0.01, 0.97, s, fontsize=12, ha='left')
            plt.show()
    


'''
initialization reference when porting to paul's code : 

from plot import Plot

1) initialize the data dictionary same as it was : 
data = {motor_id: {"time": [], "position": [], "velocity": [], "torque": [], 
"pos_ref":[], "vel_ref":[], "vel_pred":[], "pos_pred":[]} for motor_id in motor_ids}

2) Run control loop and log data as usual

3) Replace all the plotting mehod with this : 
plotter = Plot(data)
plotter.plot_motor_data(K, L, failure_count)

'''
