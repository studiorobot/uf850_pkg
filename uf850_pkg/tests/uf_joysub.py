#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
import numpy as np
from joy2uf850.angle_conversion import euler_from_quaternion, get_quaternion_from_euler
from xarm.wrapper import XArmAPI
import time

##########################################################
#################### Copyright 2022 ######################
################ by Peter Schaldenbrand ##################
### The Robotics Institute, Carnegie Mellon University ###
################ All rights reserved. ####################
##########################################################

# Modified by David Ho - University of Michigan Robotics

class AuraJoyNode(Node):
    '''
        Low-level action functionality of the robot.
        This is an abstract class, see its children for usage.
    '''
    def __init__(self, debug=False, node_name="painting"):
        super().__init__(node_name)
        self.debug_bool = debug

    def debug(self, msg):
        if self.debug_bool:
            self.get_logger().info(msg)

    def good_morning_robot(self):
        raise NotImplementedError("This method must be implemented")

    def good_night_robot(self):
        raise NotImplementedError("This method must be implemented")

    def go_to_cartesian_pose(self, positions, orientations, precise=False):
        raise NotImplementedError("This method must be implemented")


class XArm(AuraJoyNode):
    '''
        Low-level action functionality of the robot.
        This is an abstract class, see its children for usage.
    '''
    def __init__(self, ip, debug=False):
        super().__init__(debug=debug, node_name="xarm_node")
        self.arm = XArmAPI(ip)

        # Start robot initialization on startup
        self.good_morning_robot()

        # Initialize joystick state variables
        self.joystick_axes = []
        self.joystick_buttons = []

        # Subscribe to the /joy topic
        self.create_subscription(Joy, '/joy', self.joystick_callback, 10)

    def good_morning_robot(self):
        self.get_logger().info("I'm waking up...")
        self.arm.motion_enable(enable=True)
        self.arm.reset(wait=True)
        
        self.arm.set_mode(0)
        self.arm.reset(wait=True)

        self.arm.set_state(state=0)
        self.arm.reset(wait=True)

        # Going to Home Position
        self.arm.set_position(*[180.0, 0.0, 500.0, 180, 0, 0], wait=True)

        # self.arm.set_mode(1)
        # self.arm.set_state(0)
        # time.sleep(0.1)

        # Set Jerk & Speed
        self.arm.set_tcp_jerk (10000)
        # self.arm.set_joint_jerk(28647, is_radian=True)
        # self.arm.clean_conf()

        # Initialize Current position and orientation of the arm
        failure, current_pose = self.arm.get_position()
        current_pose = np.round(current_pose, decimals=2)
        self.curr_x, self.curr_y, self.curr_z, self.curr_roll, self.curr_pitch, self.curr_yaw = current_pose
        
        if failure:
            self.get_logger().error("Failed to get current pose from xArm.")
            return


        self.get_logger().info("I'm ready to go!")

    def good_night_robot(self):
        self.get_logger().info("Shutting down robot...")
        self.arm.disconnect()

    def go_to_cartesian_pose(self, positions, orientations, speed=100):
        positions = np.array(positions)
        orientations = np.array(orientations)
        
        if len(positions.shape) == 1:
            positions = positions[None, :]
            orientations = orientations[None, :]
        
        for i in range(len(positions)):
            x, y, z = positions[i][0], positions[i][1], positions[i][2]
            
            q = orientations[i]
            euler = euler_from_quaternion(q[0], q[1], q[2], q[3])
            roll, pitch, yaw = 180.0, 0.0, 0.0
            
            wait = True
            failure, state = self.arm.get_position()
            
            if not failure:
                curr_x, curr_y, curr_z = state[0], state[1], state[2]
                dist = ((x - curr_x)**2 + (y - curr_y)**2 + (z - curr_z)**2)**0.5
                
                if dist < 5:
                    wait = False
                    speed = 100
            
            try:
                result = self.arm.set_position(
                    x=x,
                    y=y,
                    z=z,
                    roll=roll,
                    pitch=pitch,
                    yaw=yaw,
                    speed=speed,
                    wait=wait
                )
                
                if result:
                    self.get_logger().warn("Failed to go to pose. Resetting.")
                    self.arm.clean_error()
                    self.good_morning_robot()
                    self.arm.set_position(
                        x=x,
                        y=y,
                        z=z + 5,
                        roll=roll,
                        pitch=pitch,
                        yaw=yaw,
                        speed=speed,
                        wait=True
                    )
                    self.arm.set_position(
                        x=x,
                        y=y,
                        z=z,
                        roll=roll,
                        pitch=pitch,
                        yaw=yaw,
                        speed=speed,
                        wait=True
                    )
            except Exception as e:
                self.good_morning_robot()
                self.get_logger().error(f"Cannot go to position: {e}")

    def map_value(self, value):
        """
        Maps a value to a specific range:
        - [-1, 0) -> -1
        - [0] -> 0
        - (0, 1] -> 1
        
        :param value: Input float value in the range [-1, 1]
        :return: Mapped integer value
        :raises ValueError: If the input value is out of range
        """
        if value < 0:
            return -1
        elif value == 0:
            return 0
        else:
            return 1
    
    def joystick_callback(self, msg: Joy):
        """
        Callback function for processing joystick inputs.
        
        Args:
            msg (Joy): The joystick message containing axes and button states.
        """
        # Update joystick states
        self.joystick_axes = msg.axes
        self.joystick_buttons = msg.buttons
        
        if len(self.joystick_axes) < 8 or len(self.joystick_buttons) < 11:
            self.get_logger().error(f"Joystick input has insufficient data: {len(self.joystick_axes)} axes, {len(self.joystick_buttons)} buttons")
            return

        # Mapping to Xbox Joystick axes
        LEFT_STICK_LR   = self.joystick_axes[0]
        LEFT_STICK_FB   = self.joystick_axes[1]
        LEFT_TRIGGER    = self.joystick_axes[2]
        RIGHT_STICK_LR  = self.joystick_axes[3]
        RIGHT_STICK_FB  = self.joystick_axes[4]
        RIGHT_TRIGGER   = self.joystick_axes[5]
        CROSS_KEY_LR    = self.joystick_axes[6]
        CROSS_KEY_FB    = self.joystick_axes[7]

        # Mapping to Xbox Joystick buttons
        BTN_A           = self.joystick_buttons[0]
        BTN_B           = self.joystick_buttons[1]
        BTN_X           = self.joystick_buttons[2]
        BTN_Y           = self.joystick_buttons[3]
        BTN_LB          = self.joystick_buttons[4]
        BTN_RB          = self.joystick_buttons[5]
        BTN_BACK        = self.joystick_buttons[6]
        BTN_START       = self.joystick_buttons[7]
        BTN_POWER       = self.joystick_buttons[8]
        BTN_STICK_LEFT  = self.joystick_buttons[9]
        BTN_STICK_RIGHT = self.joystick_buttons[10]

        # Map joystick inputs to Cartesian commands
        try:
            speed = 2

            # Axes mapping for linear motion (left stick + triggers)
            linear_x = - self.map_value(LEFT_STICK_FB) * speed # Forward/backward (left stick up/down)
            linear_y = - self.map_value(LEFT_STICK_LR) * speed # Left/right (left stick left/right)
            linear_z = (self.map_value(RIGHT_TRIGGER) - self.map_value(LEFT_TRIGGER))/2 * speed # Triggers for up/down

            self.get_logger().info("                 ")
            self.get_logger().info("------------------------------")
            self.get_logger().info(f"Linear x: {linear_x}")
            self.get_logger().info(f"Linear y: {linear_y}")
            self.get_logger().info(f"Linear z: {linear_z}")

            # Axes mapping for angular motion (right stick + bumpers)

            angular_x = RIGHT_STICK_LR * 1.0  # Pitch (right stick left/right)
            angular_y = -RIGHT_STICK_FB * 1.0  # Roll (right stick up/down)
            angular_z = (BTN_LB - BTN_RB) * 1.0  # Yaw (LB/RB buttons)

            # print("Current Pose")
            # print(current_pose)

            # Update position and orientation based on joystick inputs
            self.curr_x += linear_x
            self.curr_y += linear_y
            self.curr_z += linear_z

            self.curr_roll  += angular_x
            self.curr_pitch += angular_y
            self.curr_yaw   += angular_z

            self.get_logger().info("------------------------------")
            self.get_logger().info(f"Current x: {self.curr_x}")
            self.get_logger().info(f"Current y: {self.curr_y}")
            self.get_logger().info(f"Current z: {self.curr_z}")
            self.get_logger().info("------------------------------")

            self.get_logger().info("                 ")


            # print("New Pose")
            # print(new_x, new_y, new_z, new_roll, new_pitch, new_yaw)

            # Send command to move the arm incrementally - use go_to_cartesian_pose
            # new_position = [new_x, new_y, new_z]
            # new_position = [self.curr_x/1000, self.curr_y/1000, self.curr_z/1000]
            # new_orientation = get_quaternion_from_euler(new_roll, new_pitch, new_yaw)
            # new_orientation = get_quaternion_from_euler(self.curr_roll, self.curr_pitch, self.curr_yaw)

            # # USE set_position
            result = self.arm.set_position(
                x = self.curr_x,
                y = self.curr_y,
                z = self.curr_z,
                roll  = 180, # self.curr_roll,
                pitch = 0,   # self.curr_pitch,
                yaw   = 0,   # self.curr_yaw,
                radius=5,
                speed=1000,
                mvacc=50000,
                wait=False
            )

            # mvpose = [self.curr_x, self.curr_y, self.curr_z, 180, 0, 0]
            # result = self.arm.set_servo_cartesian(mvpose, speed=100, mvacc=50000)

            if result:
                self.get_logger().warn("Failed to execute joystick command.")

        except Exception as e:
            self.get_logger().error(f"Error processing joystick input: {e}")

def toggle_cartesian(args=None):
    rclpy.init(args=args)

    arm_node = XArm(ip="192.168.1.227", debug=True)

    positions_ud = np.array([[0.5, 0, 0.5],
                            [0.5, 0, 0.7],
                            [0.5, 0, 0.5],
                            [0.5, 0, 0.6],
                            [0.5, 0, 0.5]])
    
    positions_fb = np.array([[0.5, 0, 0.5],
                            [0.7, 0, 0.5],
                            [0.5, 0, 0.5],
                            [0.6, 0, 0.5],
                            [0.5, 0, 0.5]])
    
    positions_lr = np.array([[0.5, 0, 0.5],
                            [0.5, 0.1, 0.5],
                            [0.5, 0, 0.5],
                            [0.5, -0.1, 0.5],
                            [0.5, 0, 0.5]])

    orientations = np.array([[1, 0, 0, 0],
                            [1, 0, 0, 0],
                            [1, 0, 0, 0],
                            [1, 0, 0, 0],
                            [1, 0, 0, 0]])

    # arm_node.go_to_cartesian_pose(positions_fb, orientations)
    # arm_node.go_to_cartesian_pose(positions_lr, orientations)
    arm_node.go_to_cartesian_pose(positions_ud, orientations)

    # failure, current_pose = arm_node.arm.get_position()

    # print(current_pose)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        arm_node = XArm(ip="192.168.1.227", debug=True)
        rclpy.spin(arm_node)
    except KeyboardInterrupt:
        arm_node.arm.disconnect()
        arm_node.destroy_node()
        rclpy.shutdown()
    finally:
        arm_node.arm.disconnect()
        arm_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
