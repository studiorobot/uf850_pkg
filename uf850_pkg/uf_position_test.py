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

arm.move_gohome(wait=True)

arm.set_position(*[180.0, 0.0, 500.0, 180, 0, 0], wait=True)

arm.set_mode(1)
arm.set_state(0)
time.sleep(0.1)

while arm.connected and arm.state != 4:
    mvpose = [180.0, 0.0, 500.0, 180, 0, 0]
    ret = arm.set_servo_cartesian(mvpose, speed=25)
    failure, current_pose = arm.get_position()
    print(current_pose)
    time.sleep(2)

arm.disconnect()