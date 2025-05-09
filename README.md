# Robstride_Bringup
Contains scripts for motor zeroing, logging, gripper control, trajectory tracking, and teleoperation for K-scale third arm project

## WARNINGS - READ THOROUGHLY
- **If testing something new, keep a finger on the batter power switch to kill robot**
- **Test with two people whenever possible - one on computer/controls and one with a finger on the switch and holding the arm in their other hand**
- **Never unplug robot to turn off, always use switch (unless emergency). Unplugging while power is still on can fry the motors**
- **All control scripts have workspace limit protections - do not edit the reachable sphere parameters unless you know what you're doing**
- **Any changes made to the speed of the arm (in trajectory or keyboard tracking scripts), should start slow and increase incrementally**
- **If you have a question about anything, message Paul or AK before running the robot**


## CAN
- We use this [usb-CAN adapter](https://www.amazon.com/Jhoinrch-Converter-Open-Source-Hardware-Operating/dp/B0CRB8KXWL/ref=sr_1_3_pp?crid=2TOMURKJ6RLS6&dib=eyJ2IjoiMSJ9.VmvPEm2hDw4oIZAfdMfKXOgBgSreX0hzCgJ8iS9IKcLoMuTvTR9ctdDYposcZSUz6W2gSIJGXJq8SSpbf7_5_k9KDV7fDmnWCFRRwozhPAND3OErpvqYpLAavLxXPZvsNhPZ0ApMQYxnta9JyJ1CgK3Xs0X4DGFx640zoipGYeBWBkoWuHXTaRxx7_AIYJX-iFivyaLDO-kqXwhZKMIFEj8J38Kzkp9e_xYhl3DdXmI.omQZ2FeuNyarA5OplfDQQ25Hh30MUsHrm9Cpi_ClcJU&dib_tag=se&keywords=can+to+usb&qid=1746804792&sprefix=can+to+us%2Caps%2C165&sr=8-3), not the one that comes with the motors: 
- To bring up CAN run:
    1. ```sudo ip link set can0 type can bitrate 1000000 loopback off```
    2. ```sudo ifconfig can0 up```
- If encountering isses, remove and re-insert CAN adapter. You may need to restart CAN using commands above. You may also need to play with the Boot switch on the adapter (but do this last)

## Motor Zeroing and Logging Scripts
- Run zeroing using ```motor_zeroing.py```
- Run logging using ```motor_logging.py```
- Both have an list of CAN ID's on top to control what gets zeroed/logged
- **If you're new to using the arm, please log joint states before running any control**
- **=If joints get stuck in a position hold mode after running a control script, running either of these scripts will make them spin freely again**

## Adding New Robot Models
- Place in folder ```Robstride_Bringup/robot_models``` (see examples already in folder)
- Latest arm model is ```Sim_Arm_4DOF_May_25```
- Control scripts have a field at the top of them where you can enter a directory to your model

## Control Script General Notes
- We're still working on a "start from anywhere" sequence. For now, use this manual startup sequence
    1. Hold the arm at it's zero position
    2. Run the zeroing script
    3. Run logging to ensure zero worked
    4. Start script, while continuing to hold arm at zero position
    5. Once arm stiffens, you can let go
- This is what zero position looks like:

![Zero Position Pose](/images/zero_pos.png)

## Trajectory Tracking
- Run using ```trajectory_tracking.py```
- Keep the arm still for a length of time using ```traj.addHold(time)```
- Add a point to move to using ```traj.addLinearMove_3dof(np.array([x, y, z]), time)```
- Note: [0,0,0] is roughly where motor 1 is located
- **If time permits, please add a reference point, then test, then add a point, then retest, etc. to make sure there is no weird behavior at each point**
- **Keep a few seconds of holding at the beginning of your trajectory to have time to get out of the way**


## Teleoperation
- Position control based teleop: ```keyboard_tracking_pos_based.py```
- Velocity control based teleop: ```keyboard_tracking_vel_based.py```
- Both controllers can be used interchangably, but each has their strengths:
    - Position based control generally works better when moving through free space or when vibration resistance is important
    - Velocity based control is better in pushing scenarios. It also starts up and fails safer, **making it good for new users**
- Controls 
    - Use WASD and Up/Down arrows to control end effector motion through space
    - Use 1 and 2 to rotate joints through null space while end effector position stays fix
    - Use Space to snap the gripper open or closed
    - Use O and C to slowly open or close the gripper

