from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

parameter_file = os.path.join(
    get_package_share_directory('first_package'),
    'config',
    'params.yaml')


def generate_launch_description():
    '''
    Function that returns a LaunchDescription object

    Returns:
        LaunchDescription: LaunchDescription object
    '''
    launch_description = LaunchDescription()
    
    publisher_node = Node(
        package="first_package",
        executable="publisher_node",
        parameters=[parameter_file])
    
    subscriber_node = Node(
        package="first_package",
        executable="subscriber_node")
    
    launch_description.add_action(publisher_node)
    launch_description.add_action(subscriber_node)
    return launch_description
