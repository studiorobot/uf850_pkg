import os
import sys
import time
import json

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

arm = XArmAPI("192.168.1.227")

try:
    print("Homing the robot")
    print()
    arm.set_world_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)

    # arm.move_gohome(wait=True)

    input("Remove the brush from robot [ENTER]")
    print()
    arm.set_tcp_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    arm.set_world_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(0.1)

    input("Setting robot to MANUAL MODE [ENTER]")
    print()
    arm.set_mode(2)
    arm.set_state(0)

    arm.start_record_trajectory()

    while True:
        print("Move robot to and against canvas origin")
        record = input("Press [ENTER]")
        print()
        break

    arm.stop_record_trajectory()
    arm.set_mode(0)
    arm.set_state(0)
    
    failure, current_pose = arm.get_position()
    arm.set_position(*[
        current_pose[0],
        current_pose[1],
        current_pose[2],
        current_pose[3],
        0,
        0,
    ], wait=True)


    input("Verifying coordinate frame [ENTER]")
    print()

    # Offset world with respect to canvas
    arm.set_world_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(1)
    failure, current_pose = arm.get_position()
    print(current_pose)

    rx_offset = 180 - current_pose[3]
    ry_offset = 0.0
    rz_offset = 0.0

    print("Rotating the frame to match, see frames on web app")
    arm.set_world_offset([0, 0, 0, rx_offset, ry_offset, rz_offset])
    time.sleep(1)
    failure, current_pose = arm.get_position()
    print(current_pose)
    
    x_offset = -current_pose[0]
    y_offset = -current_pose[1]
    z_offset = -current_pose[2]

    print("Translating the frame to match, see frames on web app")
    arm.set_world_offset([x_offset, y_offset, z_offset, rx_offset, ry_offset, rz_offset])
    time.sleep(1)
    failure, current_pose = arm.get_position()
    print(current_pose)

    print()
    print("If the pose is approximately [0.0, 0.0, 0.0, 180, 0, 0] then it is correct")
    print()
    time.sleep(1)

    input("Save to File? [ENTER / Ctrl+C]")
    offsets = {
        "x_offset": x_offset,
        "y_offset": y_offset,
        "z_offset": z_offset,
        "rx_offset": rx_offset,
        "ry_offset": ry_offset,
        "rz_offset": rz_offset
    }

    json_file_path = f"../config/canvas_frame_offset.json"
    
    with open(json_file_path, "w") as f:
        json.dump(offsets, f, indent=4)

except KeyboardInterrupt:
    arm.set_tcp_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    arm.set_world_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(0.1)
    arm.disconnect()
    print("Exiting")

finally:
    arm.set_tcp_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    arm.set_world_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(0.1)
    arm.disconnect()
    print("Exiting")