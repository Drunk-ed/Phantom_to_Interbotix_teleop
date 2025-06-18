import rclpy
from rclpy.node import Node
from omni_msgs.msg import OmniFeedback

class PhantomForceFeedback(Node):
    def __init__(self):
        super().__init__('phantom_force_feedback')

        self.force_sub = self.create_subscription(
            OmniFeedback,
            '/phantom/force_feedback',
            self.force_callback,
            10
        )
        self.get_logger().info("Subscribed to /phantom/force_feedback")

    def force_callback(self, msg: OmniFeedback):


        self.get_logger().info(f"i heard: '{msg}'")


def main(args=None):
    rclpy.init(args=args)
    node = PhantomForceFeedback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
