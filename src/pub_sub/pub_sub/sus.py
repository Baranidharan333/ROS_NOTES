import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String

class NumberSubscriber(Node):
    def __init__(self):
        super().__init__('number_subscriber')

        # Create subscriptions
        self.create_subscription(Int32, 'int_topic', self.int_callback, 10)
        self.create_subscription(Float32, 'float_topic', self.float_callback, 10)
        self.create_subscription(String, 'string_topic', self.string_callback, 10)

        self.get_logger().info('Subscriber node started, waiting for messages...')

    def int_callback(self, msg):
        self.get_logger().info(f'Received Int: {msg.data}')

    def float_callback(self, msg):
        self.get_logger().info(f'Received Float: {msg.data}')

    def string_callback(self, msg):
        self.get_logger().info(f'Received String: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = NumberSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
