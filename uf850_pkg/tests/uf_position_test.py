import os
import sys
import time
import matplotlib.pyplot as plt

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

arm = XArmAPI("192.168.1.227")

arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
# arm.set_world_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# arm.move_gohome(wait=True)

try:
    # Offset Eef taken brush into account
    # arm.set_tcp_offset([0.0, 0.0, 110.0, 0.0, 0.0, 0.0])
    # time.sleep(0.5)
    x_offset = 489.1
    y_offset = -78.9
    z_offset = 106.2
    rx_offset = 93.9
    ry_offset = 0.5
    rz_offset = 0.9

    # arm.set_position(*[x_offset, y_offset, z_offset, rx_offset, ry_offset, rz_offset], wait=True)
    # arm.set_position(*[300, 0, 500, 180, 0, 0], wait=True)
    
    # Offset world with respect to canvas
    arm.set_world_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(1)
    failure, current_pose = arm.get_position()
    print(current_pose)
    arm.set_world_offset([-x_offset, -z_offset, y_offset, -rx_offset, -ry_offset, -rz_offset])
    time.sleep(1)
    failure, current_pose = arm.get_position()
    print(current_pose)

    # while arm.connected and arm.state != 4:
        # arm.set_position(*[0.0, 0.0, 20, -180, 0, 90], wait=True)
        # failure, current_pose = arm.get_position()
        # print(current_pose)
        # time.sleep(0.1)

except KeyboardInterrupt:
    # arm.move_gohome(wait=True)
    
    arm.set_world_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(0.1)
    # arm.set_tcp_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # time.sleep(0.1)


    arm.disconnect()

    print("Exiting")

finally:
    # arm.move_gohome(wait=True)
    
    arm.set_world_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(0.1)
    # arm.set_tcp_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # time.sleep(0.1)


    arm.disconnect()

    print("Exiting")