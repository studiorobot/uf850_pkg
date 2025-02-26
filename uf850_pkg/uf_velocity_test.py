import os
import sys
import time
import matplotlib.pyplot as plt

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

arm = XArmAPI("192.168.1.227")

# arm.motion_enable(enable=True)``
# arm.set_mode(0)
# arm.set_state(state=0)

# arm.move_gohome(wait=True)

# arm.set_position(*[200, 0, 200, 180, 0, 0], wait=True)

# arm.set_mode(1)
# arm.set_state(0)
# time.sleep(0.1)

# while arm.connected and arm.state != 4:
#     for i in range(300):
#         x = 200 + i
#         mvpose = [x, 0, 200, 180, 0, 0]
#         ret = arm.set_servo_cartesian(mvpose, speed=100, mvacc=2000)
#         print('set_servo_cartesian, ret={}'.format(ret))
#         time.sleep(0.01)
#     for i in range(300):
#         x = 500 - i
#         mvpose = [x, 0, 200, 180, 0, 0]
#         ret = arm.set_servo_cartesian(mvpose, speed=100, mvacc=2000)
#         print('set_servo_cartesian, ret={}'.format(ret))
#         time.sleep(0.01)

# arm.disconnect()


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

try:
    while True:
        # Move in positive X direction
        arm.vc_set_cartesian_velocity([100, 0, 0, 0, 0, 0])
        failure, current_pose = arm.get_position()
        if not failure:
            positions.append(current_pose[:3])  # Store only X, Y, Z positions
            timestamps.append(time.time() - start_time)  # Store elapsed time
        print(current_pose)
        time.sleep(0.2)

        # Move in negative X direction
        arm.vc_set_cartesian_velocity([-100, 0, 0, 0, 0, 0])
        failure, current_pose = arm.get_position()
        if not failure:
            positions.append(current_pose[:3])  # Store only X, Y, Z positions
            timestamps.append(time.time() - start_time)  # Store elapsed time
        print(current_pose)
        time.sleep(0.2)

except KeyboardInterrupt:
    print("Exiting loop...")

finally:
    # Stop the robot motion
    arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0])

    arm.set_mode(0)
    arm.set_state(state=0)
    time.sleep(1)
    arm.set_position(*[180.0, 0.0, 500.0, 180, 0, 0], wait=True)

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
