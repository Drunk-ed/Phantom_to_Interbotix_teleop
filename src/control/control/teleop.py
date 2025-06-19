#!/usr/bin/env python3
# -----------------------------------------------------------------------------------
# This script maps Geomagic Touch stylus movements and buttons to an Interbotix RX200
# robotic arm using ROS 2. The code is thoroughly commented to help you understand
# how each component works — from coordinate transformations to gripper and joint control.
#
# READ THE COMMENTS: Helpful explanations are provided throughout the code to
# make it easier to follow and modify.
# -----------------------------------------------------------------------------------

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from omni_msgs.msg import OmniButtonEvent
from sensor_msgs.msg import JointState

# ------------------------------
# Transformation Matrix
# ------------------------------

# Homogeneous transformation matrix to convert a 3D point from the Geomagic Touch coordinate frame
# into the Interbotix RX200 robot's frame
T = np.array([
    [1,  0,  0,  0.0],   # X stays the same
    [0,  1,  0, -0.2],   # Y is shifted by -0.2m
    [0,  0,  1,  0.15],  # Z is shifted up by 0.15m
    [0,  0,  0,  1.0]
])

# Apply the transformation to a 3D point from the Geomagic
def transform_point(p_geomagic):
    # Add a fixed offset in Z and make it homogeneous
    p_hom = np.array([p_geomagic[0], p_geomagic[1], p_geomagic[2]+0.06, 1.0])
    # Apply the transformation
    p_transformed = T @ p_hom
    return p_transformed[:3]  # Return x, y, z (ignore the homogeneous component)


class PhantomPoseToInterbotix(Node):
    def __init__(self, bot):
        super().__init__('phantom_to_interbotix_node')

        # Interbotix robot interface
        self.bot = bot
        self.bot.gripper.release()  # Open gripper on startup

        # Timestamp tracking for throttling (limit callback to 50Hz)
        self.last_pose_time = self.get_clock().now()
        self.last_joint_time = self.get_clock().now()

        # Whether white button on stylus is pressed
        self.white_button_pressed = False

        # ------------------------------
        # ROS2 Topic Subscriptions
        # ------------------------------

        # Pose of the Geomagic stylus tip
        self.subscription = self.create_subscription(
            PoseStamped,
            '/phantom/pose',
            self.pose_callback,
            10
        )
        self.get_logger().info("Subscribed to /phantom/pose")

        # Button press events from the stylus
        self.subscription = self.create_subscription(
            OmniButtonEvent,
            '/phantom/button',
            self.button_callback,
            10
        )

        # Stylus joint states (used for wrist control)
        self.joint_sub = self.create_subscription(
            JointState,
            'phantom/joint_states',
            self.joint_callback,
            10
        )


    def button_callback(self, msg: OmniButtonEvent):
        # Get button states
        grey = msg.grey_button
        white = msg.white_button

        # Grey button = close gripper (grasp)
        if grey == 1:
            self.bot.gripper.grasp(delay=0.0)
            self.get_logger().info("Gripper closed.")
        else:
            self.bot.gripper.release(delay=0.0)

        # Save white button state for use in other callbacks
        self.white_button_pressed = white == 1

    def pose_callback(self, msg: PoseStamped):
        # Limit callback to 50Hz
        now = self.get_clock().now()
        if (now - self.last_pose_time).nanoseconds < 20_000_000:
            return
        self.last_pose_time = now

        # Only move end-effector when white button is NOT pressed
        if not self.white_button_pressed:
            pos = msg.pose.position
            p_geomagic = [pos.x, pos.y, pos.z]

            # Transform stylus tip to robot frame
            p_inter = transform_point(p_geomagic)

            # Command robot to move end-effector to the new XYZ
            # Orientation is kept fixed (roll = pitch = 0)
            self.bot.arm.set_ee_pose_components(
                x=p_inter[0],
                y=p_inter[1],
                z=p_inter[2],
                roll=0.0,
                pitch=0.0
            )



    def joint_callback(self, msg: JointState):
        # Limit callback to 50Hz
        now = self.get_clock().now()
        if (now - self.last_joint_time).nanoseconds < 20_000_000:
            return
        self.last_joint_time = now

        # Only update joints if white button IS pressed
        if self.white_button_pressed:
            # Map joint names to positions
            name_to_pos = dict(zip(msg.name, msg.position))

            # Extract roll and pitch from stylus joint state
            roll = name_to_pos.get('roll', 0.0)
            pitch = name_to_pos.get('pitch', 0.0)

            # Adjust pitch offset and clamp within robot joint limits
            pitch += 2.30
            pitch = max(-1.74, min(2.14, pitch))

            # Set wrist_angle (pitch) and wrist_rotate (roll) joints
            self.bot.arm.set_single_joint_position(
                joint_name='wrist_angle',
                position=pitch
            )

            self.bot.arm.set_single_joint_position(
                joint_name='wrist_rotate',
                position=roll + 2.61
            )

            # Note: do not update XYZ here (only joints)


            
            # Do NOT update set_ee_pose_components here (position is handled in pose_callback)



# Main function: initializes ROS 2 and starts the node
def main(args=None):
    rclpy.init(args=args)

    # Create Interbotix robot interface
    bot = InterbotixManipulatorXS(
        robot_model='rx200',
        group_name='arm',
        gripper_name='gripper'
    )

    robot_startup()  # Enable motors, home the robot

    node = PhantomPoseToInterbotix(bot)

    try:
        rclpy.spin(node)  # Start ROS event loop
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        robot_shutdown()  # Disable motors
        rclpy.shutdown()
