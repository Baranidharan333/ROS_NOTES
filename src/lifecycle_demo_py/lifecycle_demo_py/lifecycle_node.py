import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

class ManagedTalker(LifecycleNode):
    def __init__(self):
        super().__init__('managed_talker')
        self.pub = None
        self.timer = None
        self.count = 0

    # Called when we transition UNCONFIGURED -> INACTIVE
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')
        self.pub = self.create_publisher(String, 'chatter', 10)
        return TransitionCallbackReturn.SUCCESS

    # Called when we transition INACTIVE -> ACTIVE
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating...')
        # Start publishing only in ACTIVE state
        self.timer = self.create_timer(1.0, self._on_timer)
        return TransitionCallbackReturn.SUCCESS

    # Called when we transition ACTIVE -> INACTIVE
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating...')
        if self.timer:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
        return TransitionCallbackReturn.SUCCESS

    # Called when we transition INACTIVE -> UNCONFIGURED
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        if self.timer:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
        if self.pub:
            self.destroy_publisher(self.pub)
            self.pub = None
        self.count = 0
        return TransitionCallbackReturn.SUCCESS

    # Called when we transition from any state -> FINALIZED
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS

    def _on_timer(self):
        msg = String()
        msg.data = f'Hello #{self.count}'
        self.count += 1
        if self.pub is not None:
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ManagedTalker()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
