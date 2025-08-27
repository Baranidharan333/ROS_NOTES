# # add_two_ints_client.py
# import rclpy
# from rclpy.node import Node
# from example_interfaces.srv import AddTwoInts

# class AddTwoIntsClient(Node):
#     def __init__(self):
#         super().__init__('add_two_ints_client')
#         self.client = self.create_client(AddTwoInts, 'add_two_ints')
#         while not self.client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Waiting for service...')


#     def send_request(self, a, b):
#         request = AddTwoInts.Request()
#         request.a = a
#         request.b = b
#         future = self.client.call_async(request)
#         future.add_done_callback(self.response_callback)

#     def response_callback(self, future):
#         response = future.result()
#         self.get_logger().info(f'Result: {response.sum}')

# def main():
#     rclpy.init()
#     node = AddTwoIntsClient()
#     count = 0
#     while count <= 3 :
#         a = int(input("Enter thr num for a : "))
#         b = int(input("Enter thr num for a : "))
#         node.send_request(a, b)
#         count += 1
#     rclpy.spin(node)
#     rclpy.shutdown()


# add_two_ints_client.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')


    def send_request(self, a, b):
        request = AddTwoInts.Request()
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