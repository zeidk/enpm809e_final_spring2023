'''
Node showcasing the use of Nav2
'''

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations


class NavigationDemo(Node):
    '''
    Class showing the use of the Navigation stack

    Args:
        Node (class): ROS2 node class
    '''

    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'{node_name} node running')

        self._basic_navigator = BasicNavigator()
        # --- Wait for Nav2 ---
        self._basic_navigator.waitUntilNav2Active()

    def run(self):
        '''
        Run the demo
        '''
        self.set_initial_pose()
        self.go_to_pose(-13.426668, 0.0, 3.14159)
        # self.follow_waypoints()

    def set_initial_pose(self):
        '''
        Set the initial pose of the robot
        '''
        initial_pose = self.create_pose_stamped(-15, -10.000000, 0.0)
        self._basic_navigator.setInitialPose(initial_pose)

    def create_pose_stamped(self, position_x, position_y, yaw_angle):
        '''
        Create a PoseStamped message

        Args:
            position_x (float): X coordinate
            position_y (float): Y coordinate
            rotation_z (float): yaw angle

        Returns:
            PoseStamped: PoseStamped message
        '''
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw_angle)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self._basic_navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = position_x
        goal_pose.pose.position.y = position_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        return goal_pose

    def go_to_pose(self, position_x, position_y, yaw_angle):
        '''
        Action request to go to a pose

        Args:
            position_x (float): X coordinate
            position_y (float): Y coordinate
            rotation_z (float): yaw angle
        '''
        goal_pose = self.create_pose_stamped(position_x, position_y, yaw_angle)
        self._basic_navigator.goToPose(goal_pose)
        while not self._basic_navigator.isTaskComplete():
            feedback = self._basic_navigator.getFeedback()
            print(f'Feedback: {feedback}')

        print(self._basic_navigator.getResult())

    def follow_waypoints(self):
        '''
        Action request to follow a list of waypoints

        Args:
            waypoints (list): list of waypoints
        '''

        waypoint1 = self.create_pose_stamped(3.5, 1.0, 1.57)
        waypoint2 = self.create_pose_stamped(2.0, 2.5, 3.14)
        waypoint3 = self.create_pose_stamped(0.5, 1.0, 0.0)

        waypoints = [waypoint1, waypoint2, waypoint3]

        for _ in range(3):
            self._basic_navigator.followWaypoints(waypoints)

            while not self._basic_navigator.isTaskComplete():
                feedback = self._basic_navigator.getFeedback()
                print(f'Feedback: {feedback}')
        print(self._basic_navigator.getResult())


def main(args=None):
    '''
    Main function to create a ROS2 node

    Args:
        args (Any, optional): ROS2 arguments. Defaults to None.
    '''
    rclpy.init(args=args)
    node = NavigationDemo('nav2_demo_node')
    node.run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
