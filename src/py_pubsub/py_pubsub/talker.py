# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String

# class Talker(Node):
#     def __init__(self):
#         super().__init__('talker_node')

#         # declare parameters with defaults (must declare before use)
#         self.declare_parameter('message', 'Hello, ROS2!')
#         self.declare_parameter('publish_rate', 1.0)  # Hz

#         # publisher on topic 'chatter'
#         self.pub = self.create_publisher(String, 'chatter', 10)

#         # use parameter value for timer period
#         publish_rate = float(self.get_parameter('publish_rate').value)
#         period = 1.0 / publish_rate if publish_rate > 0 else 1.0
#         self.timer = self.create_timer(period, self.timer_callback)

#     def timer_callback(self):
#         msg = String()
#         # read the (possibly updated) parameter value each callback
#         msg.data = self.get_parameter('message').value
#         self.pub.publish(msg)
#         self.get_logger().info(f'Publishing: "{msg.data}"')

# def main(args=None):
#     rclpy.init(args=args)
#     node = Talker()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import os
import yaml

class Talker(Node):
    def __init__(self):
        super().__init__('talker_node')

        # Get package share directory
        package_share_dir = get_package_share_directory('py_pubsub')
        param_file = os.path.join(package_share_dir, 'config', 'talker_params.yaml')

        # Load parameters from YAML file
        if os.path.exists(param_file):
            with open(param_file, 'r') as file:
                params = yaml.safe_load(file)
                # ROS2 params are under the node name key in YAML
                node_params = params.get('talker_node', {}).get('ros__parameters', {})
                for param_name, param_value in node_params.items():
                    self.declare_parameter(param_name, param_value)
        else:
            self.get_logger().error(f'Parameter file not found: {param_file}')

        # publisher on topic 'chatter'
        self.pub = self.create_publisher(String, 'chatter', 10)

        # use parameter value for timer period
        publish_rate = float(self.get_parameter('publish_rate').value)
        period = 1.0 / publish_rate if publish_rate > 0 else 1.0
        self.timer = self.create_timer(period, self.timer_callback)

    def timer_callback(self):
        msg = String()
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


if __name__ == '__main__':
    main()
