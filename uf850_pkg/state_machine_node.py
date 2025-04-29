#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped, PoseStamped, Pose, PoseArray
from std_msgs.msg import Header, Bool, Int8
import numpy as np
from sensor_msgs.msg import JointState
import time
from uf850_pkg.useful_math_functions import get_euler_from_quaternion, get_quaternion_from_euler
import json
from ament_index_python.packages import get_package_share_directory
import os
from xarm.wrapper import XArmAPI
from std_msgs.msg import String, Float32
import math

##########################################################
#################### Copyright 2025 ######################
##################### by David Ho ########################
########### The University of Michigan Robotics ##########
################ All rights reserved. ####################
##########################################################

class IdleState():
    """
    Idling
    """
    def __init__(self,node):
        self.node = node #state machine node, sharing with other class
        
        self.next_state = None

    def execute(self):
        self.next_state = None
        self.node.get_logger().info("Current State: IDLE")
        self.node.get_logger().info("")

        if self.node._eef_state is not None:
            self.node.get_logger().info("Aligning Axis ...")
            self.node.arm.set_mode(0)
            self.node.arm.set_state(state=0)
            time.sleep(1)

            failure, end_effector_pose = self.node.arm.get_position()
            eef_x, eef_y, eef_z, eef_rx, eef_ry, eef_rz = end_effector_pose
            self.node.arm.set_position(*[eef_x, eef_y, eef_z, 180, 0, 0], wait=True)
            
            # set cartesian velocity control mode
            self.node.get_logger().info("Switching to velocity control mode!")
            self.node.arm.set_mode(5)
            self.node.arm.set_state(0)
            time.sleep(1)

        while rclpy.ok() and self.next_state is None and not self.node.requested_state:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.node.requested_state or self.next_state
    
    def on_text_received(self,msg):
        self.node.get_logger().info(f'IdleState Voice Commmand received;  {msg.data}')
        if msg.data == 'draw':
            self.node.requested_state = 'PAINTING'
        elif msg.data == 'ChangePaint':
            self.node.requested_state = 'CHANGE PAINT'
        
    
class GoHomeState():
    """
    Go to Home Position
    """
    def __init__(self,node):
        self.node = node #state machine node, sharing with other class
        self.next_state = None

    def execute(self):
        self.next_state = None

        self.node.get_logger().info("Current State: GO HOME")
        self.node.get_logger().info("Moving Robot Home")
        self.node.get_logger().info("")
        # TODO: Add Home Pose here:
        to_canvas = self.node.is_already_in_canvas_frame
        self.go_home(to_canvas)
        self.next_state = "IDLE"
        
        while rclpy.ok() and self.next_state is None and not self.node.requested_state:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.node.requested_state or self.next_state
    
    def go_home(self, to_canvas):
        self.node.arm.set_mode(0)
        self.node.arm.set_state(state=0)
        time.sleep(1)
        if to_canvas:
            self.node.arm.set_position(*[0.0, 0.0, 75.4, 180, 0, 0], wait=True)
        else:
            self.node.arm.set_position(*[160.0, 160.0, 340.0, 180, 0, 0], wait=True)
        
        # set cartesian velocity control mode
        self.node.get_logger().info("Switching to velocity control mode!")
        self.node.arm.set_mode(5)
        self.node.arm.set_state(0)
        time.sleep(1)


class MoveState():
    """
    Receive Joystick Axes Input and Move Robot
    """
    def __init__(self,node):
        self.node = node #state machine node, sharing with other class
        self.next_state = None
        self.timeout_duration = 5.0  # Timeout in seconds for inactivity
        self.last_activity = time.time()

    def execute(self):
        self.next_state = None
        self.node.get_logger().info("Current State: MOVE")
        self.last_activity_time = time.time()  # Reset on entry

        while rclpy.ok() and self.next_state is None and not self.node.requested_state:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if self.node.is_already_in_canvas_frame:
                self.velocity_control_canvas()
            else:
                self.velocity_control_pallete()

            # Check for inactivity timeout
            if time.time() - self.last_activity > self.timeout_duration:
                self.node.get_logger().info("No input detected, transitioning to IDLE")
                self.node.arm.vc_set_cartesian_velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                return 'IDLE'

        return self.node.requested_state
    
    def on_text_received(self,msg):
        self.node.get_logger().info(f'MoveState Voice Commmand received;  {msg.data}')
        if msg.data == 'draw':
            self.node.requested_state = 'PAINTING'
        elif msg.data == 'ChangePaint':
            self.node.requested_state = 'CHANGE PAINT'
                                    
    
    def velocity_control_canvas(self):
        if self.node.is_joystick_active() and self.node._eef_state is not None:
            self.last_activity = time.time()
            # Perform movement logic (e.g., send velocity commands)
            # Declare some useful parameters:
            linear_speed = 25
            angular_speed = 10

            ########################## MAPPING LINEAR MOVEMENT #################################

            vx = (self.node.LEFT_STICK_LR) * linear_speed
            vy = - (self.node.LEFT_STICK_FB) * linear_speed

            z_max = 10
            z_0 = z_max + 40
            if self.node.RIGHT_TRIGGER == 1:              # NOT Pressed
                if abs(self.node.eef_z - z_0) < 0.1:
                    vz = 0
                else:   # GO UP
                    vz = np.clip((z_0 - self.node.eef_z), -200, 200)
            else:
                if abs(self.node.eef_z - z_max) < 0.1:
                    vz = 0
                else:   # GO DOWN
                    vz = np.clip((self.node.RIGHT_TRIGGER - 1) * (self.node.eef_z - z_max), -50, 50)


            ########################## MAPPING ORIENTATION #################################
            wx = self.node.RIGHT_STICK_FB * angular_speed
            wy = self.node.RIGHT_STICK_LR * angular_speed

            wz = (self.node.BTN_LB - self.node.BTN_RB) * angular_speed  # Yaw (LB/RB buttons)

            self.node.arm.vc_set_cartesian_velocity([vx, vy, vz, wx, wy, wz])

        else:
            self.node.arm.vc_set_cartesian_velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def velocity_control_pallete(self):
        if self.node.is_joystick_active() and self.node._eef_state is not None:
            self.last_activity = time.time()
            # Perform movement logic (e.g., send velocity commands)
            # Declare some useful parameters:
            linear_speed = 25
            angular_speed = 10

            ########################## MAPPING LINEAR MOVEMENT #################################

            vx = (self.node.LEFT_STICK_LR) * linear_speed
            vy = -(self.node.LEFT_STICK_FB) * linear_speed
            vz = (self.node.RIGHT_TRIGGER - self.node.LEFT_TRIGGER) * linear_speed

            ########################## MAPPING ORIENTATION #################################
            wx = self.node.RIGHT_STICK_FB * angular_speed
            wy = self.node.RIGHT_STICK_LR * angular_speed
            wz = (self.node.BTN_LB - self.node.BTN_RB) * angular_speed  # Yaw (LB/RB buttons)

            self.node.arm.vc_set_cartesian_velocity([vx, vy, vz, wx, wy, wz])

        else:
            self.node.arm.vc_set_cartesian_velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

class PaintingState():
    """
    Change to Canvas Coordinates
    """
    def __init__(self,node):
        self.node = node #state machine node, sharing with other class
        
        self.next_state = None

    def execute(self):
        self.next_state = "GO HOME"

        self.node.get_logger().info("Current State: PAINTING")
        self.node.get_logger().info("Switched to Canvas Coord")
        # TODO: Add Coord switching logic here
        self.node.switch_frame(to_canvas=True)
        self.node.is_already_in_canvas_frame = True
        
        while rclpy.ok() and self.next_state is None and not self.node.requested_state:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.node.requested_state or self.next_state
    
class ChangePaintState():
    """
    Change to Palette Coordinates
    """
    def __init__(self,node):
        self.node = node #state machine node, sharing with other class
        
        self.next_state = None

    def execute(self):

        self.node.get_logger().info("Current State: CHANGE PAINT")
        self.node.get_logger().info("Switched to Palette Coord")
        # TODO: Add Coord switching logic here
        self.node.switch_frame(to_canvas=False)
        self.node.is_already_in_canvas_frame = False
        
        self.next_state = "GO HOME"

        while rclpy.ok() and self.next_state is None and not self.node.requested_state:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        return self.node.requested_state or self.next_state


class PlanState():
    """
    Receive biometric inputs, voice inputs, and CoFRIDA strokes and execute them
    """
    def __init__(self,node):
        self.node = node #state machine node, sharing with other class
        self.next_state = None
        self.timeout_duration = 10.0  # Timeout in seconds for inactivity
        self.last_activity = time.time()
        self.pause = False

    def execute(self):
        self.next_state = None
        self.node.get_logger().info("Current State: PLAN")
        self.last_activity_time = time.time()  # Reset on entry

        while rclpy.ok() and self.next_state is None and not self.node.requested_state:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            # Switch to canvas frame
            self.node.switch_frame(to_canvas=False)
            self.node.is_already_in_canvas_frame = False

            # TODO: modify this section!!
            self.node.get_logger().info("hello there")
            if not self.node.is_already_in_canvas_frame:

                if self.node.arousal_state == 1: # person has elevated heartrate
                    # random chance to continue or not? im not sure
                    if np.random.rand() > 0: # TODO: Change this threshold to be in the options.json as a parameter!
                        # currently always goes home when arousal state = 1
                        self.node.get_logger().info("Arousal state = 1, SKIP stroke")
                        return 'GO HOME'
                    else:
                        self.node.get_logger().info("Arousal state = 1, CONTINUING stroke")

                self.node.get_logger().info("current depth is : %f" % (self.node.depth))
                if self.node.depth <= self.node.initial_depth: # person in canvas
                    self.node.get_logger().info("Canvas Occupied, SKIP stroke: %f" % (self.node.initial_depth))
                    return 'GO HOME'

                # basically just execute the stroke?
                self.go_to_cartesian_pose()
                self.node.get_logger().info("finished cartesian pose")
                self.next_state = "GO HOME" #after finished stroke, return to idle state
            else:
                self.node.get_logger().info("PLAN STATE - ERROR: robot is not in palette frame!")
            # Check for inactivity timeout
            if time.time() - self.last_activity > self.timeout_duration:
                self.node.get_logger().info("No input detected, transitioning to IDLE")
                self.node.arm.vc_set_cartesian_velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                return 'GO HOME'

        return self.node.requested_state
    
    def on_text_received(self,msg):
        self.node.get_logger().info(f'PlanState Voice Commmand received;  {msg.data}')
        if msg.data == 'draw':
            self.node.requested_state = 'PAINTING'
        elif msg.data == 'ChangePaint':
            self.node.requested_state = 'CHANGE PAINT'
        elif msg.data == 'stop':
            self.node.get_logger().info("set pause to be ture")
            self.pause = True

    
    def go_to_cartesian_pose(self): #self, positions, orientations, speed=50
        # positions in meters
        # for waypoint in self.node.next_stroke:
        #     positions = waypoint.position
        #     orientations = waypoint.orientation
        #     positions, orientations = np.array(positions), np.array(orientations)

        # if len(positions.shape) == 1:
        #     positions = positions[None,:]
        #     orientations = orientations[None,:]
        self.node.get_logger().info("starting paint strokes")
        for pose in self.node.next_stroke:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if self.pause == True:
                self.node.get_logger().info("stop command received")
                self.pause = False
                self.node.arm.vc_set_cartesian_velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                break
        
            self.node.get_logger().info("current depth is : %f" % (self.node.depth))
            if self.node.depth <= self.node.initial_depth: # person in canvas
                self.node.get_logger().info("Canvas Occupied, SKIP stroke: %f" % (self.node.initial_depth))
                break
            
            position = pose.position
            orientation = pose.orientation
            # ------
            x,y,z = position.y, position.x, position.z # i changed x and y index 0 1
            x,y,z = x*1000, y*-1000, z*1000 #m to mm # I changed y to multiple by 1000 instead of -1000
            # q = orientations[i]
            self.node.get_logger().info("going to position x %f, y %f" % (x, y))


            # euler= self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w) #quaternion.as_quat_array(orientations[i])
            roll, pitch, yaw = 180, 0, 0 #euler[0], euler[1], euler[2]
            # https://github.com/xArm-Developer/xArm-Python-SDK/blob/0fd107977ee9e66b6841ea9108583398a01f227b/xarm/x3/xarm.py#L214
            
            wait = True 
            failure, state = self.node.arm.get_position()
            # self.node.get_logger().info("get arm position: failure %f" % (failure))

            if not failure:
                curr_x, curr_y, curr_z = state[0], state[1], state[2]
                # print('curr', curr_y, y)
                dist = ((x-curr_x)**2 + (y-curr_y)**2 + (z-curr_z)**2)**0.5
                # print('dist', dist)
                # Dist in mm
                if dist < 5:
                    wait=False
                    speed=100
                    # print('less')

            # self.node.get_logger().info("entering try")

            self.node.arm.set_mode(0)
            self.node.arm.set_state(state=0)

            try:
                # self.node.get_logger().info("try")
                r = self.node.arm.set_position(*[x, y, z, roll, pitch, yaw], wait=wait)

                # self.node.get_logger().info("set position")
                if r:
                    self.node.get_logger().info("failed to go to pose, resetting.")
                    self.node.arm.clean_error()
                    self.good_morning_robot()
                    # self.node.arm.set_position(
                    #         x=x, y=y, z=z+5, roll=roll, pitch=pitch, yaw=yaw,
                    #         speed=speed, wait=True
                    # )
                    self.node.arm.set_position(
                            *[x, y, z+5, roll, pitch, yaw], wait=True
                    )
                    # self.node.arm.set_position(
                    #         x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw,
                    #         speed=speed, wait=True
                    # )
                    self.node.arm.set_position(
                        *[x, y, z, roll, pitch, yaw], wait=wait
                    )
            except Exception as e:
                self.good_morning_robot()
                print('Cannot go to position', e)

    def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians


    def good_morning_robot(self):
        # self.node.arm.set_tcp_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # self.node.arm.set_world_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        print("good_morning_robot in plan state")
        self.node.arm.motion_enable(enable=True)
        self.node.arm.reset(wait=True)
        self.node.arm.set_mode(0)
        self.node.arm.reset(wait=True)
        self.node.arm.set_state(state=0)

        self.node.arm.reset(wait=True)


    # def good_night_robot(self):
    #     self.node.arm.set_tcp_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    #     self.node.arm.set_world_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    #     self.node.arm.disconnect()


class RefreshPaintState():
    """
    Rinse and get new paint
    """
    def __init__(self,node):
        self.node = node #state machine node, sharing with other class
        self.next_state = None

    def execute(self):

        self.node.get_logger().info("Current State: CHANGE PAINT")
        self.node.get_logger().info("Switched to Palette Coord")
        
        self.node.switch_frame(to_canvas=False)
        self.node.is_already_in_canvas_frame = False
        
        # TODO: insert logic to go to water location, rag location, paint location
        
        self.next_state = "GO HOME"

        while rclpy.ok() and self.next_state is None and not self.node.requested_state:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        return self.node.requested_state or self.next_state

class StateMachineNode(Node):
    def __init__(self, ip):
        super().__init__('state_machine_node')
        self.get_logger().info("state machine node started")
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
        
        # Create a subscription for joystick
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joystick_callback, 10)

        # Create a subscription for voice
        self.voice_sub = self.create_subscription(String, 'voice_text', self.on_text_received, 10)

        # Create a subscription for biometric
        self.biometric_sub = self.create_subscription(Int8, 'arousal_label', self.biometric_callback, 10)
        self.arousal_state = False

        # Create a subscription for depth camera
        self.cofrida_sub = self.create_subscription(Float32, '/object_depth', self.depth_callback, 10)
        self.initial_depth = 0.0
        self.depth_count = 0
        self.depth = 100.0 # Track next requested stroke

        # Create a subscription for CoFRIDA
        self.cofrida_sub = self.create_subscription(PoseArray, "/frida_stroke_vec", self.cofrida_callback, 10)
        self.next_stroke = None # Track next requested stroke

        self.prev_btn_y = 0  # Track previous button state
        self.prev_btn_b = 0  # Track previous button state
        self.requested_state = None

        self.stroke_count = 0 # keep track of strokes until need to refresh the paint

        self.states = {
            'IDLE': IdleState(self),
            'PAINTING': PaintingState(self),            # move to vertical canvas coords
            'MOVE': MoveState(self),                    # move based on joystick control
            'CHANGE PAINT': ChangePaintState(self),     # move to flat canvas coords
            'GO HOME': GoHomeState(self),               # return to home position
            'PLAN': PlanState(self),                    # execute cofrida plans
            'REFRESH PAINT': RefreshPaintState(self)    # rinse and get more paint
        }

        self.current_state = 'CHANGE PAINT'
        self.is_already_in_canvas_frame = False
        self._eef_state = None

    def switch_frame(self, to_canvas):
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        time.sleep(0.1)

        self.arm.set_world_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], wait=True)
        time.sleep(0.1)

        if to_canvas:
            # Offset World
            self.arm.set_world_offset([self.x_offset, self.y_offset, self.z_offset, self.rx_offset, self.ry_offset, self.rz_offset], wait=True)
            time.sleep(0.1)

        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        time.sleep(0.1)
        

    def good_morning_robot(self):
        self.get_logger().info("I'm waking up...")
        self.arm.motion_enable(enable=True)
        self.arm.reset(wait=True)
        
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        time.sleep(1)
        # Going to Home Position
        self.arm.set_position(*[180.0, 0.0, 500.0, 180, 0, 0], wait=True)

        # Offset Eef taken brush into account
        # self.arm.set_tcp_offset([0.0, 0.0, 110.0, 0.0, 0.0, 0.0], wait=True)

        # Read from JSON to get offset
        package_share_dir = get_package_share_directory('uf850_pkg')
        json_file_path = os.path.join(package_share_dir, 'config', 'canvas_frame_offset.json')

        try:
            # Load offsets from JSON file
            with open(json_file_path, "r") as f:
                offsets = json.load(f)
            
        
                self.x_offset = offsets["x_offset"]
                self.y_offset = offsets["y_offset"]
                self.z_offset = offsets["z_offset"]
                self.rx_offset = offsets["rx_offset"]
                self.ry_offset = offsets["ry_offset"]
                self.rz_offset = offsets["rz_offset"]
            
        except FileNotFoundError:
            raise SystemExit("Error: canvas_frame_offset.json not found - run calibration first")
        

        # Move arm to safe location to avoid self collision
        # self.arm.set_position(*[-self.x_offset, 0.0, 600, -(self.rx_offset - 180) , self.ry_offset, self.rz_offset], wait=True)
        # time.sleep(1)

        # self.get_logger().info("Offseting world frame to canvas...")
        # # self.switch_frame(to_canvas=False)
        # self.switch_frame(to_canvas=True)

        # self.get_logger().info("Moving to canvas...")
        # self.arm.set_position(*[0.0, 0.0, 75.4, 180.0, 0.0, 0.0], wait=True)
        # time.sleep(1)

        # # set cartesian velocity control mode
        # self.get_logger().info("Switching to velocity control mode!")
        # self.arm.set_mode(5)
        # self.arm.set_state(0)
        # time.sleep(1)

        # self.get_logger().info("I'm ready to go!")

    def good_night_robot(self):
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        self.arm.reset(wait=True)

        # Going to Home Position
        self.arm.set_tcp_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], wait=True)
        self.arm.set_world_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], wait=True)
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
        self.eef_x, self.eef_y, self.eef_z, self.eef_rx, self.eef_ry, self.eef_rz = end_effector_pose

        eef_pose_msg.pose.position.x = self.eef_x
        eef_pose_msg.pose.position.y = self.eef_y
        eef_pose_msg.pose.position.z = self.eef_z

        # eef_pose_msg.pose.orientation.x = roll
        # eef_pose_msg.pose.orientation.y = pitch
        # eef_pose_msg.pose.orientation.z = yaw

        qx, qy, qz, qw = get_quaternion_from_euler(self.eef_rx, self.eef_ry, self.eef_rz)
        eef_pose_msg.pose.orientation.x = qx
        eef_pose_msg.pose.orientation.y = qy
        eef_pose_msg.pose.orientation.z = qz
        eef_pose_msg.pose.orientation.w = qw

        # Publish the message
        self.eef_pose_pub.publish(eef_pose_msg)

        self._eef_state = 0

        if failure:
            self.get_logger().error("Failed to get end effector pose from UF850.")
            return

    def is_joystick_active(self):
        """Returns True if any joystick input exceeds deadzones"""
        if not self.joystick_axes:
            return False
        
        STICK_DEADZONE = 0.05
        TRIGGER_DEADZONE = 0.95  # Since triggers rest at 1.0 when neutral

        active_axes = [
            self.LEFT_STICK_LR,
            self.LEFT_STICK_FB,
            self.RIGHT_STICK_LR,
            self.RIGHT_STICK_FB,
            self.CROSS_KEY_LR,
            self.CROSS_KEY_FB
        ]

        # Check sticks and cross keys (axes 0,1,3,4,6,7)
        stick_active = any(abs(axis) > STICK_DEADZONE for axis in active_axes)
        
        # Check triggers (axes 2 and 5)
        trigger_active = (
            abs(self.LEFT_TRIGGER - 1.0) > TRIGGER_DEADZONE or
            abs(self.RIGHT_TRIGGER - 1.0) > TRIGGER_DEADZONE
        )
        
        return stick_active or trigger_active

    def joystick_callback(self, msg: Joy):
        """
        Callback function for processing joystick inputs.
        
        Args:
            msg (Joy): The joystick message containing axes and button states.
        """

        if self.requested_state:
            return  # Ignore new requests during transitions

        self.joystick_axes = msg.axes
        self.joystick_buttons = msg.buttons
        self.last_joystick_update = time.time()
        
        if len(self.joystick_axes) < 8 or len(self.joystick_buttons) < 11:
            self.get_logger().error(f"Joystick input has insufficient data: {len(self.joystick_axes)} axes, {len(self.joystick_buttons)} buttons")
            return

        # Mapping to Xbox Joystick axes
        self.LEFT_STICK_LR   = self.joystick_axes[0]
        self.LEFT_STICK_FB   = self.joystick_axes[1]
        self.LEFT_TRIGGER    = self.joystick_axes[2]
        self.RIGHT_STICK_LR  = self.joystick_axes[3]
        self.RIGHT_STICK_FB  = self.joystick_axes[4]
        self.RIGHT_TRIGGER   = self.joystick_axes[5]
        self.CROSS_KEY_LR    = self.joystick_axes[6]
        self.CROSS_KEY_FB    = self.joystick_axes[7]

        # Mapping to Xbox Joystick buttons
        self.BTN_A           = self.joystick_buttons[0]
        self.BTN_B           = self.joystick_buttons[1]
        self.BTN_X           = self.joystick_buttons[2]
        self.BTN_Y           = self.joystick_buttons[3]
        self.BTN_LB          = self.joystick_buttons[4]
        self.BTN_RB          = self.joystick_buttons[5]
        self.BTN_BACK        = self.joystick_buttons[6]
        self.BTN_START       = self.joystick_buttons[7]
        self.BTN_POWER       = self.joystick_buttons[8]
        self.BTN_STICK_LEFT  = self.joystick_buttons[9]
        self.BTN_STICK_RIGHT = self.joystick_buttons[10]

        # Y BUTTON SWITCH BETWEEN COORD FRAMES
        #-------------------------------------------------------------
        current_btn_y = self.BTN_Y
        if current_btn_y == 1 and self.prev_btn_y == 0:
            if not self.is_already_in_canvas_frame:
                self.requested_state = 'PAINTING'
                # self.get_logger().info("Y Received 1")
            else:
                self.requested_state = 'CHANGE PAINT' 
                # self.get_logger().info("Y Received 2")
        self.prev_btn_y = current_btn_y  # Update previous state
        #=============================================================

        # B BUTTON MOVE THE ROBOT HOME
        #-------------------------------------------------------------
        current_btn_b = self.BTN_B
        if current_btn_b == 1 and self.prev_btn_b == 0:
            self.requested_state = 'GO HOME'
            # self.get_logger().info("B Received 1")
        self.prev_btn_b = current_btn_b  # Update previous state
        #=============================================================

        # JOY AXES MOVE THE ROBOT
        # ------------------------------------------------------------
        if self.is_joystick_active() and self.current_state != 'MOVE':
            self.requested_state = 'MOVE'
        
        #=============================================================

        # JOY AXES MOVE THE ROBOT
        # ------------------------------------------------------------
        if self.is_joystick_active() and self.current_state != 'MOVE':
            self.requested_state = 'MOVE'
        
        #=============================================================

    def on_text_received(self, msg):
        state_instance = self.states[self.current_state]
        state_instance.on_text_received(msg)
        #self.get_logger().info(f'data received')

    def biometric_callback(self, msg: Int8):
        if msg.data == 1:
            self.get_logger().info("received arousal = 1")
            self.arousal_state = True
        else:
            self.arousal_state = False

    def depth_callback(self, msg: Float32):
        self.depth_count += 1
        if self.depth_count < 10:
            self.initial_depth += msg.data
        if self.depth_count == 10:
            self.initial_depth = self.initial_depth / 10.0
            self.get_logger().info("SETTING INITIAL DEPTH %f" % (self.initial_depth))
        self.get_logger().info(f'Depth received: {msg.data}', throttle_duration_sec=2)
        self.depth = msg.data

    def cofrida_callback(self, msg: PoseArray):
        # given the pose array of a stroke (consisting of a few waypoints), execute those waypoints in canvas frame then return to the idle state
        # self.requested_state = 'PAINTING'   # move to canvas frame
        self.next_stroke = msg.poses        # put requested stroke in next_stroke for planning state to use
        self.requested_state = 'PLAN'       # go to planning state
        


    def run(self):
        try:
            while rclpy.ok():
                state_instance = self.states[self.current_state]
                
                if self.requested_state:
                    self.current_state = self.requested_state
                    self.requested_state = None
                    # self.get_logger().info(f"State changed to {self.current_state}")
                    continue  # Restart loop with new state
                
                next_state = state_instance.execute()

                if next_state:
                    self.current_state = next_state
                    # self.get_logger().info("state switched!")
                else:
                    break
        except KeyboardInterrupt:
            self.get_logger().info("State machine interrupted.")

def main(args=None):
    rclpy.init(args=args)
    state_machine_node = StateMachineNode(ip="192.168.1.227")
    executor = rclpy.executors.MultiThreadedExecutor()

    try:
        executor.add_node(state_machine_node)
        # Run your state machine logic
        state_machine_node.run()
    except KeyboardInterrupt:
        pass  # Let finally block handle cleanup
    except Exception as e:
        state_machine_node.get_logger().error(f"Critical error: {e}")
    finally:
        # Cleanup sequence
        executor.shutdown()
        state_machine_node.good_night_robot()
        state_machine_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()