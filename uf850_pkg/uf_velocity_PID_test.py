import os
import sys
import time
import matplotlib.pyplot as plt
from simple_pid import PID

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

arm = XArmAPI("192.168.1.227")

arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
time.sleep(1)
# arm.move_gohome(wait=True)
arm.set_position(*[180.0, 0.0, 500.0, 180, 0, 0], wait=True)

# set cartesian velocity control mode
arm.set_mode(5)
arm.set_state(0)
time.sleep(1)

positions = []
timestamps = []
start_time = time.time()  # Record the start time for timestamps

err_max = 0.1


try:
    desired_pose_x = 150

    x_pid = PID(0.05, 0.0, 0.0, setpoint=desired_pose_x)

    failure, current_pose = arm.get_position()
    if not failure:
        current_pose_x = current_pose[0]
        error = desired_pose_x - current_pose_x

    # while True:
    while abs(error) > err_max:
        vx = x_pid(current_pose_x)

        arm.vc_set_cartesian_velocity([0, 0, 0, vx, 0, 0])

        failure, current_pose = arm.get_position()

        if not failure:
            current_pose_x = current_pose[0]
            error = desired_pose_x - current_pose_x
            positions.append(current_pose[:3])  # Store only X, Y, Z positions
            timestamps.append(time.time() - start_time)  # Store elapsed time
        print(current_pose)


except KeyboardInterrupt:
    print("Exiting loop...")

finally:
    # Stop the robot motion
    arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0])

    # arm.set_mode(0)
    # arm.set_state(state=0)
    # time.sleep(1)
    # arm.set_position(*[180.0, 0.0, 500.0, 180, 0, 0], wait=True)

    arm.disconnect()

    # Plot the position data
    positions = list(zip(*positions))  # Transpose to separate X, Y, Z components
    plt.figure(figsize=(10, 6))

    plt.plot(timestamps, positions[0], label='X Position')
    plt.plot(timestamps, positions[1], label='Y Position')
    plt.plot(timestamps, positions[2], label='Z Position')

    plt.xlabel('Time (s)')
    plt.ylabel('Position (mm)')
    plt.title('Robot Arm Position Over Time')
    plt.legend()
    plt.grid()
    plt.show()
