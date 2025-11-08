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
# NUM_TESTS: This variable controls how many times the collision test will run.
# Each test starts from a new random position.
NUM_TESTS = 1000

# CORE SETUP
# ------------------------
# Will be using Radians for this
# Get the full path to the XML file so the script can find it
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)

# model: This object loads and holds the static description of your robot from the XML file.
# It contains all the definitions for bodies, joints, geometry, etc. It doesn't change.
model = mujoco.MjModel.from_xml_path(abspath)
# data: This object holds the dynamic state of the simulation (e.g., joint positions, velocities).
# This is what gets updated on every step of the simulation.
data = mujoco.MjData(model)


# ------------------------
# FUNCTIONS
# ------------------------
def get_joint_limits():
    """ Defines the joint mobility in Radians.
        These values should match the 'range' attribute of the <joint> tags in robot.xml. """
    return {
        0: [-2.967, 2.967], # Joint 1 (J1)
        1: [-1.745, 0.873], # Joint 2 (J2)
        2: [-2.967, 2.967], # Joint 3 (J3)
        3: [-0.960, 3.142], # Joint 4 (J4)
    }

def generate_random_start_pos(edge_buffer = .6):
    """Generates a random set of joint angles within the robot's limits."""
    joint_limits = get_joint_limits()
    num_joints = len(joint_limits)
    # Create a NumPy array to hold the random angles
    random_angles = np.zeros(num_joints)
    # For each joint, pick a random value between its low and high limit
    for i in range(num_joints):
        low, high = joint_limits[i]

        if np.random.rand() < .5:
            random_angles[i] = np.random.uniform(low, low + edge_buffer)
        else:        
            random_angles[i] = np.random.uniform(high - edge_buffer, high)
        random_angles[i] = np.clip(random_angles[i], low, high)
    return random_angles
#speed should be set bewteen 1 and 10, this is much faster for testing
def home_single_joint(model, data, viewer, joint_idx, collision_counter, target=0.0, speed=200.0):
    ''' model: Static robot description
        data: current postions and velocities
        viewer: real time visualization
        joint_idx: determines which joint to move
        collision_counter: a dictionary {'count': 0} used to track collisions
        target: the final desired angle for the joint (0.0 for home)
        speed: controls the speed of the movement
        '''
    joint_limits = get_joint_limits()
    # Get the joint's current angle from the simulation data
    start_pos = data.qpos[joint_idx]

    # Calculate how far the joint needs to move
    distance = abs(target - start_pos)

    # If the joint is already very close to the target, do nothing.
    if distance < 0.01:
        return
    print(f" Moving joint {joint_idx} from {start_pos:.3f} to {target:.3f}")

    # Calculate the number of small steps needed to create a smooth motion (modified to 15 for fast testing)
    steps = max(3, int(distance / (speed * 0.02)))

    # This loop executes the smooth motion step-by-step
    for i in range(steps + 1):
        if not viewer.is_running():
            break

        # 'progress' goes from 0.0 to 1.0 to smoothly interpolate the position
        progress = i / steps
        # Calculate the joint's new position for this specific step
        current_pos = start_pos + progress * (target - start_pos)
        # Ensure the new position does not exceed the joint's physical limits
        low, high = joint_limits[joint_idx]
        current_pos = np.clip(current_pos, low, high)
        # Apply the new position to the simulation data
        data.qpos[joint_idx] = current_pos
        
        #Run a full physics step. This is required to update collision information.
        mujoco.mj_step(model, data)
        
        # data.ncon is a built-in MuJoCo variable for the number of contacts
        if data.ncon > 0:
            collision_counter['count'] += 1
        
        # Refresh the viewer window to show the robot's new position
        viewer.sync()

        # A short pause to make the animation viewable at a consistent speed
        time.sleep(0.02)

#Homing Sequence
def safe_homing_sequence(model, data, viewer, collision_counter):
    """ This function defines the plan for homing the robot.
        It calls home_single_joint for each joint, one after the other. """
    initial_positions = data.qpos[:4].copy()
    print(f"Starting homing sequence from: {initial_positions}")
    # Defines the order to move the joints
    homing_order = [1, 3, 2, 0]
    for joint_idx in homing_order:
        if not viewer.is_running():
            break
        print(f"Homing joint J{joint_idx + 1}")
        # Call the function to move the individual joint, passing the counter
        home_single_joint(model, data, viewer, joint_idx, collision_counter, target=0.0)
        time.sleep(0.3) # Pause between moving each joint
    final_positions = data.qpos[:4].copy()
    print(f"Homing complete for this test")

def run_collisions_checker():
    """ This is the main function that runs the entire test. """
    # Initialize master counters to keep a running total across all tests.
    total_collisions_all_tests = 0
    tests_with_collisions = 0

    # Launch the MuJoCo viewer window
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # This is the main test loop, which runs for NUM_TESTS iterations.
        for i in range(NUM_TESTS):
            if not viewer.is_running():
                break
            print(f"\n ---- Starting Test {i + 1}/{NUM_TESTS}----")

            # Phase 1: Find a valid starting position that is not already in a collision.
            while viewer.is_running():
                start_pos = generate_random_start_pos()
                data.qpos[:len(start_pos)] = start_pos
                mujoco.mj_step(model, data) # Step once to check for initial collisions

                if data.ncon == 0:
                    print(f"Found valid start position: {np.round(start_pos, 2)}")
                    viewer.sync()
                    time.sleep(.1) # Pause to show the start position
                    break # Exit this loop and proceed to the test
                else:
                    print("Invalid starting position, re-generating...")
                    time.sleep(0.1)

            if not viewer.is_running():
                break

            # Phase 2: Run the homing sequence and count any collisions that occur.
            # A new counter is created for each test.
            test_collision_counter = {'count': 0}
            safe_homing_sequence(model, data, viewer, test_collision_counter)

            # Phase 3: Report the results of the test and update the master counters.
            if test_collision_counter['count'] > 0:
                print(f"  RESULT: Collision detected! ({test_collision_counter['count']} contact steps)")
                total_collisions_all_tests += test_collision_counter['count']
                tests_with_collisions += 1
            else:
                print("  RESULT: Success, no collisions")
            time.sleep(1)
    
    # After all tests are finished, print the final summary.
    print("\n----- COLLISION TEST SUMMARY -----")
    print(f"Total tests run: {NUM_TESTS}")
    print(f"Number of tests that had collisions: {tests_with_collisions}")


if __name__ == "__main__":
    # Calls the main test function.
    run_collisions_checker()

