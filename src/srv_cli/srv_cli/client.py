import rclpy
from rclpy.node import Node
from custom_bringup.srv import CustomSrv 
class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(CustomSrv, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')


    def send_request(self, a, b):
        request = CustomSrv .Request()
        request.a = a
        request.b = b
        return request


    def response_callback(self, future):
        response = future.result()
        self.get_logger().info(f'Result: {response.sum}')

def main():
    rclpy.init()
    node = AddTwoIntsClient()
    a = int(input("Enter the num for a : "))
    b = int(input("Enter the num for b : "))
    future = node.client.call_async(node.send_request(a, b))

    rclpy.spin_until_future_complete(node, future)  # Wait for response
    response = future.result()
    node.get_logger().info(f'Result: {response.sum}')
    rclpy.spin(node)
    rclpy.shutdown()