import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class PhantomJointState(Node):
    def __init__(self):
        super().__init__('phantom_Joint_State')

        self.button_sub = self.create_subscription(
            JointState,
            '/phantom/joint_states',
            self.joint_callback,
            10
        )

        self.get_logger().info("Subscribed to /phantom/joint_state")
   

    def joint_callback(self, msg: JointState):
        self.get_logger().info(f" i heard: '{msg}'")
                


def main(args=None):
    rclpy.init(args=args)
    node = PhantomJointState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
