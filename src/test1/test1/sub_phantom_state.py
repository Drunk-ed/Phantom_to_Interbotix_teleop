import rclpy
from rclpy.node import Node
from omni_msgs.msg import OmniState

class PhantomStateSubscriber(Node):
    def __init__(self):
        super().__init__('phantom_state_subscriber')

        self.subscription = self.create_subscription(
            OmniState,
            '/phantom/state',
            self.state_callback,
            10
        )
        self.get_logger().info("Subscribed to /phantom/state")

    def state_callback(self, msg: OmniState):
        self.get_logger().info(f"i heard: '{msg}'")

def main(args=None):
    rclpy.init(args=args)
    node = PhantomStateSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
