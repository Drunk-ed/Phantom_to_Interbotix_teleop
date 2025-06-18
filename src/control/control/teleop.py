#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from math import atan2, asin, degrees
from omni_msgs.msg import OmniButtonEvent
from tf_transformations import euler_from_quaternion
import tf_transformations
from sensor_msgs.msg import JointState

# Homogeneous transformation matrix from Geomagic to Interbotix
T = np.array([
    [1,  0,  0,  0.0],
    [0,  1,  0, -0.2],
    [0,  0,  1,  0.15],
    [0,  0,  0,  1.0]
])

def transform_point(p_geomagic):
    p_hom = np.array([p_geomagic[0], p_geomagic[1], p_geomagic[2]+0.06, 1.0])
    p_transformed = T @ p_hom
    return p_transformed[:3]

class PhantomPoseToInterbotix(Node):
    def __init__(self, bot):
        super().__init__('phantom_to_interbotix_node')
        self.bot = bot
        self.bot.gripper.release()
        self.last_pose_time = self.get_clock().now()
        self.last_joint_time = self.get_clock().now()
        


        self.white_button_pressed = False  # Track white button state

        self.subscription = self.create_subscription(
            PoseStamped,
            '/phantom/pose',
            self.pose_callback,
            10
        )
        self.get_logger().info("Subscribed to /phantom/pose")

        self.subscription = self.create_subscription(
            OmniButtonEvent,
            '/phantom/button',
            self.button_callback,
            10
        )
        self.joint_sub = self.create_subscription(
            JointState,
            'phantom/joint_states',
            self.joint_callback,
            10
        )

    def button_callback(self, msg: OmniButtonEvent):
        grey = msg.grey_button
        white = msg.white_button

        if grey == 1:
            self.bot.gripper.grasp(delay=0.0)
            self.get_logger().info("Gripper closed.")
        else:
            self.bot.gripper.release(delay=0.0)

        # Update white button state
        self.white_button_pressed = white == 1

    def pose_callback(self, msg: PoseStamped):
        now = self.get_clock().now()
        if (now - self.last_pose_time).nanoseconds < 20_000_000:  # 50 Hz
            return
        self.last_pose_time = now

        if self.white_button_pressed == False:
            pos = msg.pose.position
            p_geomagic = [pos.x, pos.y, pos.z]
            p_inter = transform_point(p_geomagic)

            self.bot.arm.set_ee_pose_components(
                x=p_inter[0],
                y=p_inter[1],
                z=p_inter[2],
                roll=0.0,
                pitch=0.0
            )


    def joint_callback(self, msg: JointState):
        now = self.get_clock().now()
        if (now - self.last_joint_time).nanoseconds < 20_000_000:  # 50 Hz
            return
        self.last_joint_time = now
        if self.white_button_pressed == True:
            name_to_pos = dict(zip(msg.name, msg.position))

            roll = name_to_pos.get('roll', 0.0)
            pitch = name_to_pos.get('pitch', 0.0)

            pitch += 2.30
            pitch = max(-1.74, min(2.14, pitch))

            self.bot.arm.set_single_joint_position(
                joint_name='wrist_angle',
                position=pitch
            )

            # Always update wrist_rotate and wrist_angle
            self.bot.arm.set_single_joint_position(
                joint_name='wrist_rotate',
                position=roll + 2.61
            )

            
            # Do NOT update set_ee_pose_components here (position is handled in pose_callback)



def main(args=None):
    rclpy.init(args=args)

    # Start Interbotix Robot
    bot = InterbotixManipulatorXS(
        robot_model='rx200',
        group_name='arm',
        gripper_name='gripper'
    )
    robot_startup()

    node = PhantomPoseToInterbotix(bot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        robot_shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
