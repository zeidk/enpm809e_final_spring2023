#!/usr/bin/env python3
'''
To test this script, run the following commands in separate terminals:

- ros2 launch robot_commander robot_commander.launch.py
- ros2 run robot_commander floor_robot_main.py
- ros2 launch ariac_gazebo ariac.launch.py trial_name:=final
'''

import rclpy
from rclpy.executors import MultiThreadedExecutor
from robot_commander.robot_commander_interface import RobotCommanderInterface


def main(args=None):
    '''
    Main function for the floor robot.
    '''
    rclpy.init(args=args)
    node = RobotCommanderInterface()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
