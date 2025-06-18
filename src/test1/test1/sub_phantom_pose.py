import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from math import atan2, asin, sqrt, degrees
from tf_transformations import euler_from_quaternion

class PhantomPose(Node):
    def __init__(self):
        super().__init__('phantom_pose_subscriber')

        self.subscription = self.create_subscription(
            PoseStamped,
            '/phantom/pose',
            self.pose_callback,
            10
        )

        self.get_logger().info("Subscribed to /phantom/pose")

    def pose_callback(self, msg: PoseStamped):
        position = msg.pose.position
        orientation = msg.pose.orientation

        # Compute roll, pitch, yaw manually (in degrees)
        roll, pitch, yaw = self.quaternion_to_rpy_from_tf(orientation)

        self.get_logger().info(
            f"\nPosition: x={position.x:.4f}, y={position.y:.4f}, z={position.z:.4f}\n"
            f"Orientation (RPY): roll={roll:.2f}°, pitch={pitch:.2f}°, yaw={yaw:.2f}°"
        )

    def quaternion_to_rpy_from_tf(self, quat):
        """
        Converts a geometry_msgs/Quaternion to RPY (in degrees),
        using the standard intrinsic X–Y–Z (roll–pitch–yaw) sequence.
        """
        # Note: tf wants [x, y, z, w]
        q = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, yaw = euler_from_quaternion(q, axes='sxyz')

        # Convert to degrees
        roll = degrees(roll)
        pitch = degrees(pitch)
        yaw = degrees(yaw)

        # Optional: clamp if you still want −90…+90°
        roll  = max(-90, min(90, roll))
        pitch = max(-90, min(90, pitch))
        yaw   = max(-90, min(90, yaw))

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = PhantomPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
