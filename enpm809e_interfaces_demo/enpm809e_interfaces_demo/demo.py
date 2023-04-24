import time
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from enpm809e_interfaces.msg import WeatherStation
from enpm809e_interfaces.srv import AddTwoInts


class DemoInterface(Node):
    '''
    Class showing how to use ROS2 interfaces.

    Args:
        Node (rclpy.node.Node): Parent class for ROS nodes

    Raises:
        KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
    '''

    def __init__(self, node_name):
        super().__init__(node_name)

        service_group = MutuallyExclusiveCallbackGroup()
        timer_group = MutuallyExclusiveCallbackGroup()

        # Service client
        self._client = self.create_client(AddTwoInts, 'enpm809e_add_two_ints')
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self._request = AddTwoInts.Request()
        # self._client_timer = self.create_timer(3.0, self._client_callback, callback_group=timer_group)
        response = self.add_two_ints(1, 2)
        self.get_logger().info(f'Response: {response.sum}')

        # Publisher
        self._publisher = self.create_publisher(WeatherStation, 'weather_forecast', 10)
        self._publisher_timer = self.create_timer(1.0, self._publisher_callback)
        self.get_logger().info(f'{node_name} node running')
        self._msg = WeatherStation()

        self._sub = self.create_subscription(WeatherStation, 'weather_forecast', self._sub_callback, 10)

    def _sub_callback(self, msg):
        if msg.weather == WeatherStation.SUNNY:
            self.get_logger().info('Sunny')
            
    def _client_callback(self):
        '''
        Timer callback function to call a service
        '''
        self.get_logger().info('Sending request: 1 + 2')
        response = self.add_two_ints(1, 2)
        self.get_logger().info(f'Response: {response.sum}')

    def _publisher_callback(self):
        '''
        Timer callback function to publish weather forecast
        '''
        self._msg.weather = WeatherStation.SUNNY
        self._msg.day = WeatherStation.MONDAY
        self._msg.time = time.time()
        self._publisher.publish(self._msg)
        self.get_logger().info('Publishing weather forecast')

    def add_two_ints(self, first_int, second_int):
        '''
        Function to add two integers using a service.

        Args:
            first_int (int): First integer
            second_int (int): Second integer

        Raises:
            KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
        '''

        # Request

        self._request.a = first_int
        self._request.b = second_int

        # Call service
        future = self._client.call_async(self._request)

        # Wait for response
        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        return future.result()


def main(args=None):
    '''
    Main function to run the node.
    '''
    rclpy.init(args=args)
    node = DemoInterface('demo_interfaces')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass

    node.destroy_node()
    rclpy.shutdown()
