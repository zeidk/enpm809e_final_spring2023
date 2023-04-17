import rclpy
from rclpy.node import Node
from enpm809e_interfaces.srv import AddTwoInts


class ServerInterface(Node):
    '''
    Class showing how to start a ROS server.

    Args:
        Node (rclpy.node.Node): Parent class for ROS nodes

    Raises:
        KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
    '''

    def __init__(self, node_name):
        super().__init__(node_name)

        # Service server
        self._service = self.create_service(AddTwoInts, 'enpm809e_add_two_ints', self._service_callback)

    def _service_callback(self, request, response):
        '''
        Service callback function to add two integers.

        Args:
            request (AddTwoInts.Request): Request object
            response (AddTwoInts.Response): Response object

        Returns:
            AddTwoInts.Response: Response object
        '''
        response.sum = request.a + request.b
        self.get_logger().info(f'Received request: {request.a} + {request.b}')
        return response


def main(args=None):
    '''
    Main function to run the node.
    '''

    rclpy.init(args=args)
    node = ServerInterface('server_interface')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
