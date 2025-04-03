#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Header, Bool
import numpy as np
import time
from xarm.wrapper import XArmAPI
from uf850_pkg.useful_math_functions import get_euler_from_quaternion, get_quaternion_from_euler
import json
from ament_index_python.packages import get_package_share_directory
import os

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
        self.joint_state_pub = self.create_publisher(JointState, '/uf850_joint_states', 10)
        # Create a publisher for end effector pose
        self.eef_pose_pub = self.create_publisher(PoseStamped, '/eef_pose', 10)
        # Create a timer to periodically publish joint torques
        frequency = 50      # Hz
        self.create_timer(1/frequency, self.publish_robot_state)

        # Create a subscriber for end effector velocity commands
        self.vel_sub = self.create_subscription(TwistStamped, "/end_effector_vel_cmd", self.subscribe_vel_cmd, 10)

    def good_morning_robot(self):
        self.get_logger().info("I'm waking up...")
        self.arm.motion_enable(enable=True)
        self.arm.reset(wait=True)
        
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        time.sleep(1)

        # Read from JSON to get offset
        package_share_dir = get_package_share_directory('uf850_pkg')
        json_file_path = os.path.join(package_share_dir, 'config', 'canvas_frame_offset.json')

        try:
            # Load offsets from JSON file
            with open(json_file_path, "r") as f:
                offsets = json.load(f)
            
           
                x_offset = offsets["x_offset"]
                y_offset = offsets["y_offset"]
                z_offset = offsets["z_offset"]
                rx_offset = offsets["rx_offset"]
                ry_offset = offsets["ry_offset"]
                rz_offset = offsets["rz_offset"]
            
        except FileNotFoundError:
            raise SystemExit("Error: canvas_frame_offset.json not found - run calibration first")
        
        # Going to Home Position
        self.arm.set_position(*[180.0, 0.0, 500.0, 180, 0, 0], wait=True)
        time.sleep(1)
        self.arm.set_position(*[500.0, 0.0, 500.0, -(rx_offset - 180) , ry_offset, rz_offset], wait=True)
        time.sleep(1)

        # Offset Eef taken brush into account
        self.arm.set_tcp_offset([0.0, 0.0, 110.0, 0.0, 0.0, 0.0], wait=True)

        # Offset World
        self.arm.set_world_offset([x_offset, y_offset, z_offset, rx_offset, ry_offset, rz_offset], wait=True)

        # Initialize Current position and orientation of the arm
        failure, current_pose = self.arm.get_position()
        current_pose = np.round(current_pose, decimals=2)
        self.curr_x, self.curr_y, self.curr_z, self.curr_roll, self.curr_pitch, self.curr_yaw = current_pose
        
        if failure:
            self.get_logger().error("Failed to get current pose from xArm.")
            return

        # set cartesian velocity control mode
        self.get_logger().info("Switching mode!")
        self.arm.set_mode(5)
        self.arm.set_state(0)
        time.sleep(1)

        self.get_logger().info("I'm ready to go!")

    def good_night_robot(self):
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        self.arm.reset(wait=True)

        # Going to Home Position
        self.arm.set_tcp_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.arm.set_world_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.arm.set_position(*[148.8, 0.0, 237.9, 180, 0, 0], wait=True)
        time.sleep(1)

        self.get_logger().info("Shutting down robot...")
        self.arm.disconnect()

    def publish_robot_state(self):
        self.publish_joint_state()
        self.publish_end_effector_pose()
    
    def publish_joint_state(self):
        """
        Publishes the current joint state of the UF850 robot.
        """
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = "base_link"
        joint_state_msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    
        # Get current joint state from the arm
        failure, joint_states = self.arm.get_joint_states()

        joint_state_msg.position = joint_states[0]  # Joint positions (in degrees)
        joint_state_msg.velocity = joint_states[1]  # Joint velocities (degrees/s)
        joint_state_msg.effort = joint_states[2]  # Joint efforts (Nm ?)

        # Publish the message
        self.joint_state_pub.publish(joint_state_msg)

        if failure:
            self.get_logger().error("Failed to get joint states from UF850.")
            return
        
    def publish_end_effector_pose(self):
        """
        Publishes the current end effector pose of the UF850 robot.
        """
        eef_pose_msg = PoseStamped()
        eef_pose_msg.header = Header()
        eef_pose_msg.header.stamp = self.get_clock().now().to_msg()
    
        # Get current joint state from the arm
        failure, end_effector_pose = self.arm.get_position()
        x, y, z, roll, pitch, yaw = end_effector_pose


        eef_pose_msg.pose.position.x = x
        eef_pose_msg.pose.position.y = y
        eef_pose_msg.pose.position.z = z

        # eef_pose_msg.pose.orientation.x = roll
        # eef_pose_msg.pose.orientation.y = pitch
        # eef_pose_msg.pose.orientation.z = yaw

        qx, qy, qz, qw = get_quaternion_from_euler(roll, pitch, yaw)
        eef_pose_msg.pose.orientation.x = qx
        eef_pose_msg.pose.orientation.y = qy
        eef_pose_msg.pose.orientation.z = qz
        eef_pose_msg.pose.orientation.w = qw

        # Publish the message
        self.eef_pose_pub.publish(eef_pose_msg)

        if failure:
            self.get_logger().error("Failed to get end effector pose from UF850.")
            return

    def subscribe_vel_cmd(self, msg: TwistStamped):
        """
        Subscribe and move arm with velocity commands
        """
        x = msg.twist.linear.x
        y = msg.twist.linear.y
        z = msg.twist.linear.z
        rx = msg.twist.angular.x
        ry = msg.twist.angular.y
        rz = msg.twist.angular.z

        # self.get_logger().info("Velocity commands go brrrrrrrrrrrrr")
        # self.get_logger().info("------------------------------")
        # self.get_logger().info(f"Linear x: {x}")
        # self.get_logger().info(f"Linear y: {y}")
        # self.get_logger().info(f"Linear z: {z}")
        # self.get_logger().info("------------------------------")
        # self.get_logger().info(f"Angular rx: {rx}")
        # self.get_logger().info(f"Angular ry: {ry}")
        # self.get_logger().info(f"Angular rz: {rz}")
        # self.get_logger().info("------------------------------")
        # self.get_logger().info("                 ")

        self.arm.vc_set_cartesian_velocity([x, y, z, rx, ry, rz])

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
