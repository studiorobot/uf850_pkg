import os
import sys
import time
import matplotlib.pyplot as plt

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

# Connect to the robot arm
arm = XArmAPI("192.168.1.227")

# Enable motion and set initial state
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
time.sleep(1)

# Move to an initial position
arm.set_position(*[500.0, 0.0, 500.0, 180, 0, 0], wait=True)

# Initialize lists to store joint torques and timestamps
joint_torques = []
timestamps = []
start_time = time.time()  # Record the start time for timestamps

try:
    while True:
        # Get joint torques
        ret, torques = arm.get_joints_torque()
        if torques:
            joint_torques.append(torques)  # Store torque values for all joints
            timestamps.append(time.time() - start_time)  # Store elapsed time
        print(torques)
        time.sleep(0.1)  # Adjust sampling rate as needed

except KeyboardInterrupt:
    print("Exiting loop...")

finally:
    arm.set_mode(0)
    arm.set_state(state=0)
    time.sleep(1)
    arm.set_position(*[180.0, 0.0, 500.0, 180, 0, 0], wait=True)

    arm.disconnect()

    # Plot the joint torque data
    joint_torques = list(zip(*joint_torques))  # Transpose to separate torques for each joint
    plt.figure(figsize=(12, 8))

    for i in range(len(joint_torques)):  # Assuming a 6-DOF robot arm
        plt.plot(timestamps, joint_torques[i], label=f'Joint {i+1}')

    plt.xlabel('Time (s)')
    plt.ylabel('Torque (Nm)')
    plt.title('Joint Torques Over Time')
    plt.legend()
    plt.grid()
    plt.show()
