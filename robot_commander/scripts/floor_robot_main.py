#!/usr/bin/env python3
'''
To test this script, run the following commands in separate terminals:

- ros2 launch robot_commander robot_commander.launch.py
- ros2 run robot_commander floor_robot_main.py
- ros2 launch ariac_gazebo ariac.launch.py trial_name:=kitting
'''

import rclpy
from robot_commander.competition_interface import CompetitionInterface
from rclpy.executors import MultiThreadedExecutor


def main(args=None):
    rclpy.init(args=args)
    interface = CompetitionInterface()
    interface.start_competition()

    executor = MultiThreadedExecutor()
    executor.add_node(interface)
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass

    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
