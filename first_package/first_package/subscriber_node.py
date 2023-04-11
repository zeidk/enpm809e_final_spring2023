'''
_summary_
'''

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


class SubscriberNode(Node):
    '''
    Simple class to create a ROS2 node

    Args:
        Node (class): ROS2 node class
    '''

    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name} node running')
        cb_group0 = MutuallyExclusiveCallbackGroup()
        cb_group1 = MutuallyExclusiveCallbackGroup()

        # Subscribers
        self._subscriber1 = self.create_subscription(String, 'leia',
                                                     self._subscriber1_callback,
                                                     10, callback_group=cb_group0)
        self._subscriber2 = self.create_subscription(String, 'leia',
                                                     self._subscriber2_callback,
                                                     10, callback_group=cb_group1)
        self._subscriber3 = self.create_subscription(String, 'leia',
                                                     self._subscriber3_callback,
                                                     10, callback_group=cb_group1)

    def _subscriber1_callback(self, message):
        self.get_logger().info(f"Cb1: I heard: '{message.data}'")
        while True:
            pass

    def _subscriber2_callback(self, message):
        self.get_logger().info(f"Cb2: I heard: '{message.data}'")

    def _subscriber3_callback(self, message):
        self.get_logger().info(f"Cb3: I heard: '{message.data}'")


def main(args=None):
    '''
    Main function to create a ROS2 node

    Args:
        args (Any, optional): ROS2 arguments. Defaults to None.
    '''
    rclpy.init(args=args)
    node = SubscriberNode('subscriber_node')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        print()
        node.get_logger().info('Beginning demo, end with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down.\n')
    node.destroy_node()

    # rclpy.init(args=args)
    # node = SubscriberNode('subscriber_node')
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()
