#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Header
from rclpy.qos import QoSProfile
import numpy as np
from uf850_pkg.angle_conversion import euler_from_quaternion, get_quaternion_from_euler
from xarm.wrapper import XArmAPI
import time

##########################################################
#################### Copyright 2025 ######################
##################### by David Ho ########################
########### The University of Michigan Robotics ##########
################ All rights reserved. ####################
##########################################################

class UF850(Node):
    '''
        Low-level action functionality of the robot.
        This is an abstract class, see its children for usage.
    '''
    def __init__(self, debug=False, node_name="uf850_node"):
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

class XArm(UF850):
    '''
        Low-level action functionality of the robot.
        This is an abstract class, see its children for usage.
    '''
    def __init__(self, ip, debug=False):
        super().__init__(debug=debug, node_name="arm_node")
        self.arm = XArmAPI(ip)

        # Start robot initialization on startup
        self.good_morning_robot()

        # Create a publisher for joint torques
        self.state_pub_ = self.create_publisher(JointState, '/uf850_joint_states', 10)

        # Create a timer to periodically publish joint torques
        frequency = 50      # Hz
        self.create_timer(1/frequency, self.send_joint_state)

    def good_morning_robot(self):
        self.get_logger().info("I'm waking up...")
        self.arm.motion_enable(enable=True)
        self.arm.reset(wait=True)
        
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        self.arm.reset(wait=True)

        # Going to Home Position
        self.arm.set_position(*[180.0, 0.0, 500.0, 180, 0, 0], wait=True)

        # Initialize Current position and orientation of the arm
        failure, current_pose = self.arm.get_position()
        current_pose = np.round(current_pose, decimals=2)
        self.curr_x, self.curr_y, self.curr_z, self.curr_roll, self.curr_pitch, self.curr_yaw = current_pose
        
        if failure:
            self.get_logger().error("Failed to get current pose from xArm.")
            return


        self.get_logger().info("I'm ready to go!")

    def good_night_robot(self):
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        self.arm.reset(wait=True)

        # Going to Home Position
        self.arm.set_position(*[148.8, 0.0, 237.9, 180, 0, 0], wait=True)

        self.get_logger().info("Shutting down robot...")
        self.arm.disconnect()

    def send_joint_state(self):
        """
        Publishes the current joint state of the UF850 robot.
        """
        state_msg = JointState()
        state_msg.header = Header()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = "base_link"
        state_msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    
        # Get current joint state from the arm
        failure, joint_states = self.arm.get_joint_states()

        state_msg.position = joint_states[0]  # Joint positions (in degrees)
        state_msg.velocity = joint_states[1]  # Joint velocities (degrees/s)
        state_msg.effort = joint_states[2]  # Joint efforts (Nm ?)

        # Publish the message
        self.state_pub_.publish(state_msg)

        if failure:
            self.get_logger().error("Failed to get joint states from UF850.")
            return


def main(args=None):
    rclpy.init(args=args)
    
    try:
        arm_node = XArm(ip="192.168.1.227", debug=True)
        rclpy.spin(arm_node)
    except KeyboardInterrupt:
        arm_node.good_night_robot()
        arm_node.destroy_node()
        rclpy.shutdown()
    finally:
        arm_node.good_night_robot()
        arm_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
