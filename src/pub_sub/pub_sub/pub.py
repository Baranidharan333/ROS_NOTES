import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String

class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')

        # Create publishers for int, float, and string
        self.int_pub = self.create_publisher(Int32, 'int_topic', 10)
        self.float_pub = self.create_publisher(Float32, 'float_topic', 10)
        self.string_pub = self.create_publisher(String, 'string_topic', 10)

        # Initialize counter
        self.counter = 1

        # Create a timer to publish every second
        self.timer = self.create_timer(1.0, self.publish_numbers)

    def publish_numbers(self):
        # Prepare messages
        int_msg = Int32()
        int_msg.data = self.counter

        float_msg = Float32()
        float_msg.data = float(self.counter)

        string_msg = String()
        string_msg.data = str(self.counter)

        # Publish messages
        self.int_pub.publish(int_msg)
        self.float_pub.publish(float_msg)
        self.string_pub.publish(string_msg)

        # Log info
        self.get_logger().info(f'Published: Int={int_msg.data}, Float={float_msg.data}, String="{string_msg.data}"')

        # Increment counter
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
