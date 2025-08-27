# add_two_ints_service.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Built-in service type

class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        self.get_logger().info(f'Request: {request.a} + {request.b}')
        response.sum = request.a + request.b
        return response

def main():
    rclpy.init()
    node = AddTwoIntsService()
    rclpy.spin(node)
    rclpy.shutdown()
