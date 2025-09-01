import rclpy
from rclpy.node import Node
from custom_bringup.msg import CustomMsg

class CustomSubscriber(Node):
    def __init__(self):
        super().__init__('custom_subscriber')
        self.subscription = self.create_subscription(
            CustomMsg,
            'custom_topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(
            f"Received: datai={msg.datai}, dataf={msg.dataf:.2f}, datas='{msg.datas}'"
        )

def main(args=None):
    rclpy.init(args=args)
    node = CustomSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
