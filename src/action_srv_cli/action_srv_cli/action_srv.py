import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from action_tutorials_interfaces.action import Fibonacci
import time

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal request: {goal_handle.request.order}')

        sequence = [0, 1]
        feedback_msg = Fibonacci.Feedback()

        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i - 1])
            feedback_msg.partial_sequence = sequence
            self.get_logger().info(f'Feedback: {sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        self.get_logger().info(f'Result: {sequence}')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
