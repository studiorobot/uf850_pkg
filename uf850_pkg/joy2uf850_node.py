#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import numpy as np
from xarm.wrapper import XArmAPI

##########################################################
#################### Copyright 2025 ######################
##################### by David Ho ########################
########### The University of Michigan Robotics ##########
################ All rights reserved. ####################
##########################################################

class Joy2UF850(Node):
    def __init__(self):
        super().__init__("joystick to uf850")

        # Create a subscription for joystick
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joystick_callback, 10)

        # Create a subscription for end effector position
        self.joint_state_sub = self.create_subscription(JointState, '/uf850_joint_states', self.joint_state_callback, 10)

        # Create a publisher for velocity commands
        self.vel_cmd_pub = self.create_publisher(TwistStamped, "/end_effector_vel_cmd", 10)
        frequency = 50      # Hz
        self.create_timer(1/frequency, self.send_vel_cmd)

    def joint_state_callback(self, msg:JointState):
        return

    def joystick_callback(self, msg: Joy):
        return
    
    def send_vel_cmd(self):
        return


def main(args=None):
    rclpy.init(args=args)
    node = Joy2UF850()
    rclpy.spin(node)
    rclpy.shutdown()
