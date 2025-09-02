import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker_node')

        # declare parameters with defaults (must declare before use)
        self.declare_parameter('message', 'Hello, ROS2!')
        self.declare_parameter('publish_rate', 1.0)  # Hz

        # publisher on topic 'chatter'
        self.pub = self.create_publisher(String, 'chatter', 10)

        # use parameter value for timer period
        publish_rate = float(self.get_parameter('publish_rate').value)
        period = 1.0 / publish_rate if publish_rate > 0 else 1.0
        self.timer = self.create_timer(period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        # read the (possibly updated) parameter value each callback
        msg.data = self.get_parameter('message').value
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()