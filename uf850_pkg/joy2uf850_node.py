#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Header, Bool
import numpy as np
from xarm.wrapper import XArmAPI
from uf850_pkg.useful_math_functions import get_euler_from_quaternion, get_quaternion_from_euler

##########################################################
#################### Copyright 2025 ######################
##################### by David Ho ########################
########### The University of Michigan Robotics ##########
################ All rights reserved. ####################
##########################################################

class Joy2UF850(Node):
    def __init__(self):
        super().__init__("joy2uf850_node")


        # Create a subscription for end effector position
        self.joint_state_sub = self.create_subscription(PoseStamped, '/eef_pose', self.eef_state_callback, 10)

        # Create a publisher for velocity commands
        self.vel_cmd_pub = self.create_publisher(TwistStamped, "/end_effector_vel_cmd", 10)

        # Create a subscription for joystick
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joystick_callback, 10)

        # Declaring end effector pose as None
        self._eef_state = None

    def eef_state_callback(self, msg:PoseStamped):
        self.eef_x = msg.pose.position.x
        self.eef_y = msg.pose.position.y
        self.eef_z = msg.pose.position.z

        self.eef_qx = msg.pose.orientation.x
        self.eef_qy = msg.pose.orientation.y
        self.eef_qz = msg.pose.orientation.z
        self.eef_qw = msg.pose.orientation.w

        self.eef_roll, self.eef_pitch, self.eef_yaw = get_euler_from_quaternion(self.eef_qx, self.eef_qy, self.eef_qz, self.eef_qw)

        self._eef_state = 0
        # self.get_logger().info("")
        # self.get_logger().info("------------------------------")
        # self.get_logger().info(f"Eef x: {self.eef_x}")
        # self.get_logger().info(f"Eef y: {self.eef_y}")
        # self.get_logger().info(f"Eef z: {self.eef_z}")
        # self.get_logger().info(f"Eef rx: {self.eef_roll}")
        # self.get_logger().info(f"Eef ry: {self.eef_pitch}")
        # self.get_logger().info(f"Eef rz: {self.eef_yaw}")

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

        if BTN_POWER:
            return
        
        # Declare some useful parameters:
        linear_speed = 100
        angular_speed = 25
        z_0 = 500.0
        z_max = 252.0
        error_threshold = 0.1

        if self._eef_state is None:
            return

        vx = - (LEFT_STICK_FB) * linear_speed # Forward/backward (left stick up/down)
        vy = - (LEFT_STICK_LR) * linear_speed # Left/right (left stick left/right)
        
        # vz = (RIGHT_TRIGGER - LEFT_TRIGGER)/2 * linear_speed

        if RIGHT_TRIGGER == 1:              # NOT Pressed
            if abs(self.eef_z - z_0) < error_threshold:
                self.get_logger().error(f"At z_0")
                vz = 0
            else:
                vz = np.clip((z_0 - self.eef_z), -100, 100)
                self.get_logger().error(f"vz: {vz}")
        else:
            if abs(self.eef_z - z_max) < 0.5:
                self.get_logger().error(f"At z_max")
                vz = 0
            else:
                vz = np.clip((RIGHT_TRIGGER - 1) * (self.eef_z - z_max), -100, 100)

        wx = RIGHT_STICK_LR * angular_speed  # Pitch (right stick left/right)
        wy = -RIGHT_STICK_FB * angular_speed  # Roll (right stick up/down)
        wz = (BTN_LB - BTN_RB) * angular_speed  # Yaw (LB/RB buttons)

        self.send_vel_cmd(vx=vx, vy=vy, vz=vz, wx=wx, wy=wy, wz=wz)


    def send_vel_cmd(self, vx=0.0, vy=0.0, vz=0.0, wx=0.0, wy=0.0, wz=0.0):
        velocity_msg = TwistStamped()
        velocity_msg.header = Header()
        velocity_msg.header.stamp = self.get_clock().now().to_msg()
        velocity_msg.header.frame_id = "end_effector_cartesian_velocity"

        velocity_msg.twist.linear.x = float(vx)
        velocity_msg.twist.linear.y = float(vy)
        velocity_msg.twist.linear.z = float(vz)

        velocity_msg.twist.angular.x = float(wx)
        velocity_msg.twist.angular.y = float(wy)
        velocity_msg.twist.angular.z = float(wz)
        
        self.vel_cmd_pub.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        joy2uf850_node = Joy2UF850()
        rclpy.spin(joy2uf850_node)
    except KeyboardInterrupt:
        joy2uf850_node.destroy_node()
        rclpy.shutdown()
    finally:
        joy2uf850_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()