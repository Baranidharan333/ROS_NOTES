import rclpy
from rclpy.node import Node
from custom_bringup.msg import CustomMsg

class CustomPublisher(Node):
    def __init__(self):
        super().__init__('custom_publisher')
        self.publisher_ = self.create_publisher(CustomMsg, 'custom_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = CustomMsg()
        msg.datai = self.counter
        msg.dataf = float(self.counter) * 1.1
        msg.datas = f"Message number {self.counter}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.datai}, {msg.dataf:.2f}, '{msg.datas}'")
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = CustomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
