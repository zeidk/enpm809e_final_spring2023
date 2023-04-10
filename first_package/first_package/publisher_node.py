'''
_summary_
'''

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from rcl_interfaces.msg import SetParametersResult


class PublisherNode(Node):
    '''
    Simple class to create a ROS2 node

    Args:
        Node (class): ROS2 node class
    '''

    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name} node running')

        # Parameters
        self.declare_parameter('timer_period', 1.0)
        self.declare_parameter('jedi', "Obi-Wan Kenobi")

        self._timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self._jedi = self.get_parameter('jedi').get_parameter_value().string_value

        self.add_on_set_parameters_callback(self._parameters_callback)

        self._timer = self.create_timer(self._timer_period,
                                        self._timer_callback)

        self._counter = 0

        # Publishers
        self._publisher = self.create_publisher(String, 'leia', qos_profile=10)

    def _parameters_callback(self, parameters):
        for parameter in parameters:
            if parameter.name == "jedi":
                self._jedi = parameter.value
            else:
                raise ValueError(f'unknown parameter {parameter.name}')
        return SetParametersResult(successful=True)

    def _timer_callback(self):
        self._counter += 1
        text = f"{self._counter}: Please help me {self._jedi}. You're my only hope."
        message = String()
        message.data = text
        self._publisher.publish(message)
        self.get_logger().info(text)


def main(args=None):
    '''
    Main function to create a ROS2 node

    Args:
        args (Any, optional): ROS2 arguments. Defaults to None.
    '''
    rclpy.init(args=args)
    node = PublisherNode('publisher_node')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
