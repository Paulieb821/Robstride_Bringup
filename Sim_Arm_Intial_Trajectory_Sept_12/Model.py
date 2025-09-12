import mujoco
import mujoco.viewer
import numpy as np
import os
import time

# ------------------------
# CONFIGURATION SECTION
# ------------------------

# Relative path to your MJCF or XML model
xml_path = "robot.xml"  

# Initial joint angles, this can be changed to any valid configuration for simulation
# [J1, J2, J3, J4] 
#starting_joint_angles = np.array([-.979, -1.75, -2.967, -0.924])
#starting_joint_angles = np.array([1.72, -1.75, .107, 2.72])
#starting_joint_angles = np.array([2.97, .873, 2.967, 3.14])
#starting_joint_angles = np.array([-2.84, -1.75, .115, 2.68])
#starting_joint_angles = np.array([2.9, -1.75, 2.97, -0.9160])
#starting_joint_angles = np.array([-2.58, -1.75, .132, 2.69])
#starting_joint_angles = np.array([-2.58, -.873, -.126, 3.14])
#starting_joint_angles = np.array([-2.967, -1.745, -2.967, -0.960])
#starting_joint_angles = np.array([-2.967, -1.745, -2.967, -0.960])
starting_joint_angles = np.array([-2.967, -1.745, -2.967, -0.960])
#starting_joint_angles = np.array([-2.967, -1.745, -2.967, -0.960])
# ------------------------
# CORE SETUP
# ------------------------
# Will be using Radians for this
# Get the full path+
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)

# Load the model and create data
model = mujoco.MjModel.from_xml_path(abspath)
data = mujoco.MjData(model)

# Set the initial joint configuration
data.qpos[:len(starting_joint_angles)] = starting_joint_angles
mujoco.mj_forward(model, data)  # Update kinematics after setting qpos

# ------------------------
# JOINT CONTROL FUNCTION
# ------------------------

'''def assign_joint_angles(model, data, start_time):
    """
    Function to assign joint angles each time step.
    Reads current joint angles, modifies them, and writes back.
    """

    # Read current joint angles
    joint_angles = data.qpos.copy()

    # Get elapsed time
    t = time.time() - start_time

    # ---- MODIFY JOINT ANGLES BELOW ----
    # Example: simple oscillation (disabled here)
    joint_angles[0] = 0.5*np.sin(t)
    joint_angles[1] = 0.5*np.cos(t)

    # -----------------------------------

    # Assign back to data
    data.qpos[:len(joint_angles)] = joint_angles
    mujoco.mj_forward(model, data)  # Apply kinematic update
'''
# ------------------------
# MAIN SIMULATION LOOP
# ------------------------
def get_joint_limits(): 
    """ Defines the joint mobility in Radians """
    return{ 
        0: [-2.967, 2.967], 
        1: [-1.745, 0.873], 
        2: [-2.967, 2.967], 
        3: [-0.960, 3.142], 
    }
def home_single_joint (model, data, viewer, joint_idx, target = 0.0, speed = 1.0): 
    ''' model: Static robot description
        data: current postions and velocities
        viewer: real time visualization
        joint_idx = determines which joint to move first
        target: (Final position ofzero is desired)
        speed: 1 = normal, 0.5 = half speed
        '''
    joint_limits = get_joint_limits()
    #Get current position of the joint that is being moved
    
    #will be used to calulated the difference needed to move the joint to get a desired position of zero
    start_pos = data.qpos[joint_idx] 

    #Calculate the angular distance needed 
    distance = abs(target - start_pos)

    #check if the distance from desired position is close to 0 
    if distance < 0.01: 
        print (f" Joint {joint_idx} already at target")
        return
    print(f" Moving joint {joint_idx} from {start_pos:.3f} to {target:.3f}")

    # Calculate the number of steps to get the joint moving
    #Guarantess that the steps are atleast 30 steps
    steps = max(30, int(distance / (speed * 0.02)))

    #Execute the smooth motion for each joint
    for i in range(steps + 1):
        if not viewer.is_running():
            break 
        
    
        #Gives percentage of the step count progress
        progress = i / steps 
        #takes the step size and uses a for loop to take the progress index to be able to correctley determine the size
        current_pos = start_pos + progress * (target - start_pos) 

        #This is a use for saftey, clips the value do the system is not damaged
        low, high = joint_limits[joint_idx] 
        current_pos = np.clip(current_pos, low, high)

        data.qpos[joint_idx] = current_pos
        mujoco.mj_forward(model, data)

        viewer.sync()

        #sleeps for 20ms this keeps the robot moving or taking a step sizes at 50Hz

        time.sleep(0.02)

    
    final_pos = data.qpos[joint_idx]
    if abs(final_pos - target) < 0.01: 
        print (f"Joint {joint_idx} successfully homed to target position {target:.3f}")
    else:
        print(f"Joint {joint_idx} did NOT reach target position. Final position: {final_pos:.3f}")

#Homing Sequence
def safe_homing_sequence(model, data, viewer):
    initial_positions = data.qpos [:4].copy()
    print(f"Starting homing sequence from: {initial_positions}")
    #Takes the joints and aligns them from outside in
    homing_order = [3,2,1,0]
    for joint_idx in homing_order: 
        print (f"Homing joint J{joint_idx +1}")
        home_single_joint(model, data, viewer, joint_idx, target=0.0)
        time.sleep(0.3)
        final_positions = data.qpos[:4].copy() 
        print(f"Homing complete")
#Keeps the arm set to that position in the intial state
def assign_joint_angles(model,data, start_time):
    data.qpos[:4] = [0.0, 0.0 , 0.0, 0.0]
    mujoco.mj_forward(model, data)


 #Main simulation loop
def run_kinematic_sim():
    sim_start_time = time.time()
    homing_complete = False 

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            if not homing_complete: 
                safe_homing_sequence(model, data, viewer)
                print("Homing complete")
            # Step kinematically (no physics, just forward computation)
            assign_joint_angles(model, data, sim_start_time)

            # You can sleep to slow down visualization if needed
            viewer.sync()
            mujoco.mj_step1(model, data)  # Only computes kinematics

if __name__ == "__main__":
    run_kinematic_sim()