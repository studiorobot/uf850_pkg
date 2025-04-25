#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Header, Bool
import numpy as np
from sensor_msgs.msg import JointState
import time
# from xarm.wrapper import XArmAPI
from uf850_pkg.useful_math_functions import get_euler_from_quaternion, get_quaternion_from_euler

##########################################################
#################### Copyright 2025 ######################
##################### by David Ho ########################
########### The University of Michigan Robotics ##########
################ All rights reserved. ####################
##########################################################

# class UF850(Node):
#     '''
#         Low-level action functionality of the robot.
#         This is an abstract class, see its children for usage.
#     '''
#     def __init__(self, debug=False, node_name="uf850_node"):
#         super().__init__(node_name)
#         self.debug_bool = debug

#     def debug(self, msg):
#         if self.debug_bool:
#             self.get_logger().info(msg)

#     def good_morning_robot(self):
#         raise NotImplementedError("This method must be implemented")

#     def good_night_robot(self):
#         raise NotImplementedError("This method must be implemented")

#     def go_to_cartesian_pose(self, positions, orientations, precise=False):
#         raise NotImplementedError("This method must be implemented")
    
# class XArm(UF850):
#     '''
#         Low-level action functionality of the robot.
#         This is an abstract class, see its children for usage.
#     '''
#     def __init__(self, ip, debug=False):
#         super().__init__(debug=debug, node_name="arm_node")
#         self.arm = XArmAPI(ip)

#         # Start robot initialization on startup
#         self.good_morning_robot()

#         # Create a publisher for joint torques
#         self.joint_state_pub = self.create_publisher(JointState, '/uf850_joint_states', 10)
#         # Create a publisher for end effector pose
#         self.eef_pose_pub = self.create_publisher(PoseStamped, '/eef_pose', 10)
#         # Create a subscription for joystick
#         self.joy_sub = self.create_subscription(Joy, "/joy", self.joystick_callback, 10)
        
#         self._eef_state = None

#         self.current_state = "idle"
#         self.next_state = "idle"

#     def good_morning_robot(self):
#         self.get_logger().info("I'm waking up...")
#         self.arm.motion_enable(enable=True)
#         self.arm.reset(wait=True)
        
#         self.arm.set_mode(0)
#         self.arm.set_state(state=0)
#         time.sleep(1)

#         # Going to Home Position
#         self.arm.set_position(*[180.0, 0.0, 500.0, 180, 0, 0], wait=True)
#         time.sleep(1)
#         self.arm.set_position(*[500.0, 0.0, 500.0, 95, 0, 0], wait=True)
#         time.sleep(1)

#         # Offset Eef taken brush into account
#         self.arm.set_tcp_offset([0.0, 0.0, 110.0, 0.0, 0.0, 0.0])

#         x_offset = 575.0
#         y_offset = -235.0
#         z_offset = 300.0
#         rx_offset = 87.5
#         ry_offset = 0.0
#         rz_offset = 90.0

#         # Offset world with respect to canvas
#         self.arm.set_world_offset([-z_offset, -x_offset, -y_offset, rx_offset, ry_offset, rz_offset])

#         # Initialize Current position and orientation of the arm
#         failure, current_pose = self.arm.get_position()
#         current_pose = np.round(current_pose, decimals=2)
#         self.curr_x, self.curr_y, self.curr_z, self.curr_roll, self.curr_pitch, self.curr_yaw = current_pose
        
#         if failure:
#             self.get_logger().error("Failed to get current pose from xArm.")
#             return

#         # set cartesian velocity control mode
#         self.get_logger().info("Switching mode!")
#         self.arm.set_mode(5)
#         self.arm.set_state(0)
#         time.sleep(1)

#         self.get_logger().info("I'm ready to go!")

#     def good_night_robot(self):
#         self.arm.set_mode(0)
#         self.arm.set_state(state=0)
#         self.arm.reset(wait=True)

#         # Going to Home Position
#         self.arm.set_tcp_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#         self.arm.set_world_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#         self.arm.set_position(*[148.8, 0.0, 237.9, 180, 0, 0], wait=True)
#         time.sleep(1)

#         self.get_logger().info("Shutting down robot...")
#         self.arm.disconnect()

#     def publish_robot_state(self):
#         self.publish_joint_state()
#         self.publish_end_effector_pose()
    
#     def publish_joint_state(self):
#         """
#         Publishes the current joint state of the UF850 robot.
#         """
#         joint_state_msg = JointState()
#         joint_state_msg.header = Header()
#         joint_state_msg.header.stamp = self.get_clock().now().to_msg()
#         joint_state_msg.header.frame_id = "base_link"
#         joint_state_msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    
#         # Get current joint state from the arm
#         failure, joint_states = self.arm.get_joint_states()

#         joint_state_msg.position = joint_states[0]  # Joint positions (in degrees)
#         joint_state_msg.velocity = joint_states[1]  # Joint velocities (degrees/s)
#         joint_state_msg.effort = joint_states[2]  # Joint efforts (Nm ?)

#         # Publish the message
#         self.joint_state_pub.publish(joint_state_msg)

#         if failure:
#             self.get_logger().error("Failed to get joint states from UF850.")
#             return
        
#     def publish_end_effector_pose(self):
#         """
#         Publishes the current end effector pose of the UF850 robot.
#         """
#         eef_pose_msg = PoseStamped()
#         eef_pose_msg.header = Header()
#         eef_pose_msg.header.stamp = self.get_clock().now().to_msg()
    
#         # Get current joint state from the arm
#         failure, end_effector_pose = self.arm.get_position()
#         self.x, self.y, self.z, self.roll, self.pitch, self.yaw = end_effector_pose


#         eef_pose_msg.pose.position.x = self.x
#         eef_pose_msg.pose.position.y = self.y
#         eef_pose_msg.pose.position.z = self.z

#         # eef_pose_msg.pose.orientation.x = roll
#         # eef_pose_msg.pose.orientation.y = pitch
#         # eef_pose_msg.pose.orientation.z = yaw

#         qx, qy, qz, qw = get_quaternion_from_euler(self.roll, self.pitch, self.yaw)
#         eef_pose_msg.pose.orientation.x = qx
#         eef_pose_msg.pose.orientation.y = qy
#         eef_pose_msg.pose.orientation.z = qz
#         eef_pose_msg.pose.orientation.w = qw

#         self._eef_state = 0

#         # Publish the message
#         self.eef_pose_pub.publish(eef_pose_msg)

#         if failure:
#             self.get_logger().error("Failed to get end effector pose from UF850.")
#             return

#     def set_next_state(self, state):
#         """!
#         @brief      Sets the next state.

#             This is in a different thread than run so we do nothing here and let run handle it on the next iteration.

#         @param      state  a string representing the next state.
#         """
#         self.next_state = state

#     def state_machine(self, msg: Joy):
#         # Update joystick states
#         self.joystick_axes = msg.axes
#         self.joystick_buttons = msg.buttons
        
#         if len(self.joystick_axes) < 8 or len(self.joystick_buttons) < 11:
#             self.get_logger().error(f"Joystick input has insufficient data: {len(self.joystick_axes)} axes, {len(self.joystick_buttons)} buttons")
#             return

#         # Mapping to Xbox Joystick axes
#         LEFT_STICK_LR   = self.joystick_axes[0]
#         LEFT_STICK_FB   = self.joystick_axes[1]
#         LEFT_TRIGGER    = self.joystick_axes[2]
#         RIGHT_STICK_LR  = self.joystick_axes[3]
#         RIGHT_STICK_FB  = self.joystick_axes[4]
#         RIGHT_TRIGGER   = self.joystick_axes[5]
#         CROSS_KEY_LR    = self.joystick_axes[6]
#         CROSS_KEY_FB    = self.joystick_axes[7]

#         # Mapping to Xbox Joystick buttons
#         BTN_A           = self.joystick_buttons[0]
#         BTN_B           = self.joystick_buttons[1]
#         BTN_X           = self.joystick_buttons[2]
#         BTN_Y           = self.joystick_buttons[3]
#         BTN_LB          = self.joystick_buttons[4]
#         BTN_RB          = self.joystick_buttons[5]
#         BTN_BACK        = self.joystick_buttons[6]
#         BTN_START       = self.joystick_buttons[7]
#         BTN_POWER       = self.joystick_buttons[8]
#         BTN_STICK_LEFT  = self.joystick_buttons[9]
#         BTN_STICK_RIGHT = self.joystick_buttons[10]

#         ### STATE MACHINE ###

#         if self.next_state == "go_home":
#             self.go_home()

#         if self.next_state == "idle":
#             self.idle()

#         if self.next_state == "estop":
#             self.estop()

#         if self.next_state == "calibrate":
#             self.calibrate()

#         if self.next_state == "moving":
#             self.moving(LEFT_STICK_LR, LEFT_STICK_FB, RIGHT_STICK_LR, RIGHT_STICK_FB, RIGHT_TRIGGER, BTN_LB, BTN_RB)

#         if self.next_state == "realign":
#             self.realign()

#         if self.next_state == "wrap_up":
#             self.wrap_up()

#         if self.next_state == "re_ink":
#             self.re_ink()

#         if self.next_state == "manual":
#             self.manual()

#         self.publish_robot_state()
    
#     def moving(self, LEFT_STICK_LR, LEFT_STICK_FB, RIGHT_STICK_LR, RIGHT_STICK_FB, RIGHT_TRIGGER, BTN_LB, BTN_RB):
#         """
#         Callback function for processing joystick inputs.
        
#         """
#         # Declare some useful parameters:
#         linear_speed = 25
#         angular_speed = 10

#         if self._eef_state is None:
#             return

#         ########################## MAPPING LINEAR MOVEMENT #################################

#         vx = (LEFT_STICK_FB) * linear_speed # Forward/backward (left stick up/down)
#         vy = (LEFT_STICK_LR) * linear_speed # Left/right (left stick left/right)

#         z_0 = 40.0
#         z_max = z_0 - 100
#         if RIGHT_TRIGGER == 1:              # NOT Pressed
#             if abs(self.eef_z - z_0) < 0.5:
#                 vz = 0
#             else:   # GO UP
#                 vz = np.clip((z_0 - self.eef_z), -200, 200)
#         else:
#             if abs(self.eef_z - z_max) < 0.5:
#                 vz = 0
#             else:   # GO DOWN
#                 vz = np.clip((RIGHT_TRIGGER - 1) * (self.eef_z - z_max), -50, 50)


#         ########################## MAPPING ORIENTATION #################################
#         wx = -RIGHT_STICK_LR * angular_speed
#         wy = RIGHT_STICK_FB * angular_speed

#         wz = (BTN_LB - BTN_RB) * angular_speed  # Yaw (LB/RB buttons)

#         self.arm.vc_set_cartesian_velocity([vx, vy, vz, wx, wy, wz])



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

        while rclpy.ok() and self.next_state is None and not self.node.requested_state:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.node.requested_state or self.next_state
    
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
        # TODO: Add Coord switching logic here
        self.next_state = "IDLE"
        
        while rclpy.ok() and self.next_state is None and not self.node.requested_state:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.node.requested_state or self.next_state

class MoveState():
    """
    Receive Joystick Axes Input and Move Robot
    """
    def __init__(self,node):
        self.node = node #state machine node, sharing with other class
        self.next_state = None
        self.timeout_duration = 1.5  # Timeout in seconds for inactivity
        self.last_activity = time.time()


    def execute(self):
        self.next_state = None
        self.node.get_logger().info("Current State: MOVE")
        self.last_activity_time = time.time()  # Reset on entry

        while rclpy.ok() and self.next_state is None and not self.node.requested_state:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if self.node.is_joystick_active():
                self.last_activity = time.time()
                # Perform movement logic (e.g., send velocity commands)
                self.node.get_logger().info(f"Moving robot: X={self.node.LEFT_STICK_LR}, Y={self.node.LEFT_STICK_FB}")
                self.node.get_logger().info(f"Moving robot: RX={self.node.RIGHT_STICK_LR}, RY={self.node.RIGHT_STICK_FB}")
                self.node.get_logger().info(f"Moving robot: UP={self.node.LEFT_TRIGGER}, RY={self.node.RIGHT_TRIGGER}")
                self.node.get_logger().info("")

                # TODO: Add movement logic (e.g., publishing Twist messages)

            # Check for inactivity timeout
            elif time.time() - self.last_activity > self.timeout_duration:
                self.node.get_logger().info("No input detected, transitioning to IDLE")
                return 'IDLE'

        return self.node.requested_state

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
        self.node.is_already_in_canvas_frame = False
        
        self.next_state = "GO HOME"

        while rclpy.ok() and self.next_state is None and not self.node.requested_state:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        return self.node.requested_state or self.next_state
    
class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        self.get_logger().info("state machine node started")
        
        # Create a subscription for joystick
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joystick_callback, 10)

        self.prev_btn_y = 0  # Track previous button state
        self.prev_btn_b = 0  # Track previous button state
        self.requested_state = None

        self.states = {
            'IDLE': IdleState(self),
            'PAINTING': PaintingState(self),
            'MOVE': MoveState(self),
            'CHANGE PAINT': ChangePaintState(self),
            'GO HOME': GoHomeState(self),
        }

        self.current_state = 'CHANGE PAINT'
        self.is_already_in_canvas_frame = False

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

    def run(self):
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

def main(args=None):
    rclpy.init(args=args)
    
    # try:
    #     arm_node = XArm(ip="192.168.1.227", debug=True)
    #     rclpy.spin(arm_node)
    # except KeyboardInterrupt:
    #     arm_node.good_night_robot()
    #     arm_node.destroy_node()
    #     rclpy.shutdown()
    # finally:
    #     arm_node.good_night_robot()
    #     arm_node.destroy_node()
    #     rclpy.shutdown()

    state_machine_node = StateMachineNode()
    
    try:
        # Start state machine execution
        state_machine_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        state_machine_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()