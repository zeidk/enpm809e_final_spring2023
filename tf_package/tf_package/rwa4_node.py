'''
Example of a tf listener node
'''

import rclpy
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Pose
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException
import PyKDL


class Transforms(Node):
    '''
    Class to listen to a tf transform

    Args:
        Node (rclpy.node.Node): Node class
    '''

    def __init__(self, reference_frame, target_frame):
        super().__init__('tf_node')
        self._reference_frame = reference_frame
        self._target_frame = target_frame
        self.get_logger().info(f"Transforming from {self._reference_frame} to {self._target_frame}")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._transform_broadcaster = TransformBroadcaster(self)
        # self._listener_timer = self.create_timer(0.33, self._listener_callback)  # 30 Hz = 0.333s
        # self._broadcaster_timer = self.create_timer(0.33, self._broadcaster_callback)  # 30 Hz = 0.333s
        self._kdl_timer = self.create_timer(0.33, self._kdl_callback)  # 30 Hz = 0.333s

    def _listener_callback(self):
        '''
        Callback function for the timer listener
        '''
        try:
            transform = self._tf_buffer.lookup_transform(self._target_frame,
                                                         self._reference_frame,
                                                         rclpy.time.Time())

            self.get_logger().info("--LISTENER--")
            self.get_logger().info(
                f"position: {transform.transform.translation.x}, {transform.transform.translation.y}, {transform.transform.translation.z}")

            self.get_logger().info(
                f"orientation: {transform.transform.rotation.x}, {transform.transform.rotation.y}, {transform.transform.rotation.z}, {transform.transform.rotation.w}")

        except LookupException as exception:
            self.get_logger().error(f'failed to get transform {repr(exception)}')

    def _broadcaster_callback(self):
        '''
        Callback function for the timer broadcaster
        '''

        tfs = TransformStamped()
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id = "left_bins_camera_frame"
        tfs.child_frame_id = "block"
        tfs.transform.translation.x = 1.0765655693809077
        tfs.transform.translation.y = -0.15500023907735283
        tfs.transform.translation.z = -0.5660059651822175

        tfs.transform.rotation.x = -0.005935743917295585
        tfs.transform.rotation.y = -0.7093057462876493
        tfs.transform.rotation.z = -0.0059888422212663805
        tfs.transform.rotation.w = 0.70485052244878

        self._transform_broadcaster.sendTransform(tfs)

    def _kdl_callback(self):
        '''
        Callback function for the timer kdl
        '''

        camera_pose = Pose()
        camera_pose.position.x = -2.286
        camera_pose.position.y = -2.96
        camera_pose.position.z = 1.8
        camera_pose.orientation.x = -0.7071045443232116
        camera_pose.orientation.y = 0.0
        camera_pose.orientation.z = 0.7071090180427863
        camera_pose.orientation.w = 0.0

        block_pose = Pose()
        block_pose.position.x = 1.0765655693809077
        block_pose.position.y = -0.15500023907735283
        block_pose.position.z = -0.5660059651822175
        block_pose.orientation.x = -0.005935743917295585
        block_pose.orientation.y = -0.7093057462876493
        block_pose.orientation.z = -0.0059888422212663805
        block_pose.orientation.w = 0.70485052244878

        self.get_logger().info("--Blue Battery--")
        block_world = self._multiply_pose(camera_pose, block_pose)
        self.get_logger().info(
            f"position: {block_world.position.x}, {block_world.position.y}, {block_world.position.z}")

        self.get_logger().info(
            f"orientation: {block_world.orientation.x}, {block_world.orientation.y}, {block_world.orientation.z}, {block_world.orientation.w}")
    
        camera_pose.position.x = -2.286
        camera_pose.position.y = 2.96
        camera_pose.position.z = 1.8
        camera_pose.orientation.x = -0.7071045443232116
        camera_pose.orientation.y = 0.0
        camera_pose.orientation.z = 0.7071090180427863
        camera_pose.orientation.w = 0.0

        block_pose.position.x = 1.0771061786688785
        block_pose.position.y = 0.1550001572716883
        block_pose.position.z = -0.5660066901751861
        block_pose.orientation.x = -0.0009553969169289469
        block_pose.orientation.y = -0.7060971377747207
        block_pose.orientation.z = -0.0009619129764371631
        block_pose.orientation.w = 0.708113687176363

        self.get_logger().info("--Purple Pump--")
        block_world = self._multiply_pose(camera_pose, block_pose)
        self.get_logger().info(
            f"position: {block_world.position.x}, {block_world.position.y}, {block_world.position.z}")

        self.get_logger().info(
            f"orientation: {block_world.orientation.x}, {block_world.orientation.y}, {block_world.orientation.z}, {block_world.orientation.w}")
       

        # Tray 3
        camera_pose.position.x = -1.3
        camera_pose.position.y = -5.8
        camera_pose.position.z = 1.8
        camera_pose.orientation.x = -0.5000006633870012
        camera_pose.orientation.y = -0.49999933659210455
        camera_pose.orientation.z = 0.5000038267902595
        camera_pose.orientation.w = -0.499996173200466 

        block_pose.position.x = 1.0650078679976955
        block_pose.position.y = 0.43000709828110867
        block_pose.position.z = 0.03999614880500818
        block_pose.orientation.x = 0.49999971518961417
        block_pose.orientation.y = -0.4999997643707076
        block_pose.orientation.z = -0.49999712140291297
        block_pose.orientation.w = 0.500003399016789

        self.get_logger().info("--Tray--")
        block_world = self._multiply_pose(camera_pose, block_pose)
        self.get_logger().info(
            f"position: {block_world.position.x}, {block_world.position.y}, {block_world.position.z}")

        self.get_logger().info(
            f"orientation: {block_world.orientation.x}, {block_world.orientation.y}, {block_world.orientation.z}, {block_world.orientation.w}")



    def _multiply_pose(self, pose1: Pose, pose2: Pose) -> Pose:
        '''
        Use KDL to multiply two poses together.
        Args:
            pose1 (Pose): Pose of the first frame
            pose2 (Pose): Pose of the second frame
        Returns:
            Pose: Pose of the resulting frame
        '''

        orientation1 = pose1.orientation
        frame1 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(orientation1.x, orientation1.y, orientation1.z, orientation1.w),
            PyKDL.Vector(pose1.position.x, pose1.position.y, pose1.position.z))

        orientation2 = pose2.orientation
        frame2 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(orientation2.x, orientation2.y, orientation2.z, orientation2.w),
            PyKDL.Vector(pose2.position.x, pose2.position.y, pose2.position.z))

        frame3 = frame1 * frame2

        # return the resulting pose from frame3
        pose = Pose()
        pose.position.x = frame3.p.x()
        pose.position.y = frame3.p.y()
        pose.position.z = frame3.p.z()

        q = frame3.M.GetQuaternion()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose


def main(args=None):
    '''
    Main function to create a ROS2 node
    '''

    rclpy.init(args=args)

    node = Transforms("block", "world")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
