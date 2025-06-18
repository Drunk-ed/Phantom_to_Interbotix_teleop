import rclpy
from rclpy.node import Node
from omni_msgs.msg import OmniButtonEvent

class PhantomButton(Node):
    def __init__(self):
        super().__init__('phantom_Button')

        self.button_sub = self.create_subscription(
            OmniButtonEvent,
            '/phantom/button',
            self.button_callback,
            10
        )

        self.get_logger().info("Subscribed to /phantom/button")
   

    def button_callback(self, msg: OmniButtonEvent):
        self.get_logger().info(f" i heard: '{msg}'")
                


def main(args=None):
    rclpy.init(args=args)
    node = PhantomButton()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
