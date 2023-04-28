import rclpy
from rclpy.time import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
    BreakBeamStatus as BreakBeamStatusMsg,
    AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
    Part as PartMsg,
    PartPose as PartPoseMsg,
    Order as OrderMsg,
    AssemblyPart as AssemblyPartMsg,
    AGVStatus as AGVStatusMsg,
    AssemblyTask as AssemblyTaskMsg,
    VacuumGripperState,
)

from ariac_msgs.srv import (
    MoveAGV,
    VacuumGripperControl,
    ChangeGripper
)

from geometry_msgs.msg import (
    Pose
)

from std_srvs.srv import Trigger
from competitor_interfaces.srv import (
    ExitToolChanger,
    EnterToolChanger,
    PickupTray,
    PickupPart,
    MoveTrayToAGV,
    PlaceTrayOnAGV,
    RetractFromAGV,
    MovePartToAGV,
    PlacePartInTray
)

from competitor_interfaces.msg import Robots as RobotsMsg

from robot_commander.utils import (
    multiply_pose,
    rpy_from_quaternion,
    rad_to_deg_str,
    AdvancedLogicalCameraImage,
    quaternion_from_euler,
    Order,
    KittingTask,
    CombinedTask,
    AssemblyTask,
    KittingPart
)


class CompetitionInterface(Node):
    '''
    Class for a competition interface node.

    Args:
        Node (rclpy.node.Node): Parent class for ROS nodes

    Raises:
        KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
    '''
    _competition_states = {
        CompetitionStateMsg.IDLE: 'idle',
        CompetitionStateMsg.READY: 'ready',
        CompetitionStateMsg.STARTED: 'started',
        CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionStateMsg.ENDED: 'ended',
    }
    '''Dictionary for converting CompetitionState constants to strings'''

    _part_colors = {
        PartMsg.RED: 'red',
        PartMsg.BLUE: 'blue',
        PartMsg.GREEN: 'green',
        PartMsg.ORANGE: 'orange',
        PartMsg.PURPLE: 'purple',
    }
    '''Dictionary for converting Part color constants to strings'''

    _part_colors_emoji = {
        PartMsg.RED: '🟥',
        PartMsg.BLUE: '🟦',
        PartMsg.GREEN: '🟩',
        PartMsg.ORANGE: '🟧',
        PartMsg.PURPLE: '🟪',
    }
    '''Dictionary for converting Part color constants to emojis'''

    _part_types = {
        PartMsg.BATTERY: 'battery',
        PartMsg.PUMP: 'pump',
        PartMsg.REGULATOR: 'regulator',
        PartMsg.SENSOR: 'sensor',
    }
    '''Dictionary for converting Part type constants to strings'''

    _destinations = {
        AGVStatusMsg.KITTING: 'kitting station',
        AGVStatusMsg.ASSEMBLY_FRONT: 'front assembly station',
        AGVStatusMsg.ASSEMBLY_BACK: 'back assembly station',
        AGVStatusMsg.WAREHOUSE: 'warehouse',
    }
    '''Dictionary for converting AGVDestination constants to strings'''

    _stations = {
        AssemblyTaskMsg.AS1: 'assembly station 1',
        AssemblyTaskMsg.AS2: 'assembly station 2',
        AssemblyTaskMsg.AS3: 'assembly station 3',
        AssemblyTaskMsg.AS4: 'assembly station 4',
    }
    '''Dictionary for converting AssemblyTask constants to strings'''

    _gripper_states = {
        True: 'enabled',
        False: 'disabled'
    }
    '''Dictionary for converting VacuumGripperState constants to strings'''

    def __init__(self):
        super().__init__('competition_interface')

        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )

        self.set_parameters([sim_time])

        timer_group = MutuallyExclusiveCallbackGroup()
        service_group = MutuallyExclusiveCallbackGroup()

        self._kit_completed = False

        # timer
        self._robot_action_timer = self.create_timer(1, self._robot_action_timer_callback,
                                                     callback_group=timer_group)

        # Service client for starting the competition
        self._start_competition_client = self.create_client(Trigger, '/ariac/start_competition')

        # Service client for changing the gripper type
        self._place_tray = self.create_client(
            PlaceTrayOnAGV, '/competitor/floor_robot/place_tray_on_agv',
            callback_group=service_group)

        self._change_gripper_type_client = self.create_client(
            ChangeGripper, '/ariac/floor_robot_change_gripper',
            callback_group=service_group)

        # Service client for entering the gripper slot
        self._goto_tool_changer_client = self.create_client(
            EnterToolChanger, '/competitor/floor_robot/enter_tool_changer',
            callback_group=service_group)

        # Service client for exiting the gripper slot for the floor robot
        self._exit_gripper_slot_client = self.create_client(
            ExitToolChanger, '/competitor/floor_robot/exit_tool_changer',
            callback_group=service_group)

        # Service client for moving the floor robot to the home position
        self._move_floor_robot_home_client = self.create_client(
            Trigger, '/competitor/floor_robot/go_home',
            callback_group=service_group)

        self._pickup_tray_client = self.create_client(
            PickupTray, '/competitor/floor_robot/pickup_tray',
            callback_group=service_group)

        self._move_tray_to_agv_client = self.create_client(
            MoveTrayToAGV, '/competitor/floor_robot/move_tray_to_agv',
            callback_group=service_group)

        # Subscriber to the competition state topic
        self._competition_state_sub = self.create_subscription(
            CompetitionStateMsg,
            '/ariac/competition_state',
            self._competition_state_cb,
            10)

        # Store the state of the competition
        self._competition_state: CompetitionStateMsg = None

        # Subscriber to the break beam status topic
        self._break_beam0_sub = self.create_subscription(
            BreakBeamStatusMsg,
            '/ariac/sensors/breakbeam_0/status',
            self._breakbeam0_cb,
            qos_profile_sensor_data)

        # Store the number of parts that crossed the beam
        self._conveyor_part_count = 0

        # Store whether the beam is broken
        self._object_detected = False

        # Subscriber to the logical camera topic
        self._advanced_camera0_sub = self.create_subscription(
            AdvancedLogicalCameraImageMsg,
            '/ariac/sensors/advanced_camera_0/image',
            self._advanced_camera0_cb,
            qos_profile_sensor_data)

        # Store each camera image as an AdvancedLogicalCameraImage object
        self._camera_image: AdvancedLogicalCameraImage = None

        # Subscriber to the order topic
        self.orders_sub = self.create_subscription(
            OrderMsg,
            '/ariac/orders',
            self._orders_cb,
            10)

        # Flag for parsing incoming orders
        self._parse_incoming_order = False

        # List of orders
        self._orders = []

        # Subscriber to the floor gripper state topic
        self._floor_robot_gripper_state_sub = self.create_subscription(
            VacuumGripperState,
            '/ariac/floor_robot_gripper_state',
            self._floor_robot_gripper_state_cb,
            qos_profile_sensor_data)

        # Service client for turning on/off the vacuum gripper on the floor robot
        self._floor_gripper_enable = self.create_client(
            VacuumGripperControl,
            "/ariac/floor_robot_enable_gripper")

        # Attribute to store the current state of the floor robot gripper
        self._floor_robot_gripper_state = VacuumGripperState()

        # Service client for moving the ceiling robot to the home position
        self._move_ceiling_robot_home = self.create_client(
            Trigger, '/competitor/move_ceiling_robot_home')

        self._retract_from_agv_client = self.create_client(
            RetractFromAGV, '/competitor/floor_robot/retract_from_agv')

        self._pickup_part_client = self.create_client(
            PickupPart, '/competitor/floor_robot/pickup_part')

        self._move_part_to_agv_client = self.create_client(
            MovePartToAGV, '/competitor/floor_robot/move_part_to_agv')

        self._place_part_in_tray_client = self.create_client(
            PlacePartInTray, '/competitor/floor_robot/place_part_in_tray')

    def _robot_action_timer_callback(self):
        '''
        Callback for the timer that triggers the robot actions
        '''

        if self._kit_completed:
            return

        # move robot home
        self.move_robot_home("floor_robot")
        # self.move_robot_home("ceiling_robot")
        
        # change gripper type
        self.goto_tool_changer("floor_robot", "kts1", "trays")
        self.retract_from_gripper_slot("floor_robot", "kts1", "trays")

        # Pose of tray in the world frame
        quaternion = quaternion_from_euler(0, 0, 3.141591)
        tray_pose = Pose()
        tray_pose.position.x = -0.870000
        tray_pose.position.y = -5.840000
        tray_pose.position.z = 0.734990
        tray_pose.orientation.x = quaternion[0]
        tray_pose.orientation.y = quaternion[1]
        tray_pose.orientation.z = quaternion[2]
        tray_pose.orientation.w = quaternion[3]
        
        # Pose of a purple pump in the world frame
        quaternion = quaternion_from_euler(0, 0, 3.141591)
        purple_pump_pose = Pose()
        purple_pump_pose.position.x = -2.079999
        purple_pump_pose.position.y = 2.805001
        purple_pump_pose.position.z = 0.723487
        purple_pump_pose.orientation.x = quaternion[0]
        purple_pump_pose.orientation.y = quaternion[1]
        purple_pump_pose.orientation.z = quaternion[2]
        purple_pump_pose.orientation.w = quaternion[3]
        
        # Pose of a blue battery in the world frame
        quaternion = quaternion_from_euler(0, 0, 3.141591)
        blue_battery_pose = Pose()
        blue_battery_pose.position.x = -2.080001
        blue_battery_pose.position.y = -2.445000
        blue_battery_pose.position.z = 0.723434
        blue_battery_pose.orientation.x = quaternion[0]
        blue_battery_pose.orientation.y = quaternion[1]
        blue_battery_pose.orientation.z = quaternion[2]
        blue_battery_pose.orientation.w = quaternion[3]

        # pick and place tray
        self.pickup_tray("floor_robot", 3, tray_pose, "kts1")
        self.move_tray_to_agv("floor_robot", tray_pose, "agv1")
        self.place_tray("floor_robot", 3, "agv1")
        self.retract_from_agv("floor_robot", "agv1")

        # change gripper type
        self.goto_tool_changer("floor_robot", "kts2", "parts")
        self.retract_from_gripper_slot("floor_robot", "kts2", "parts")

        # pick and place purple pump
        self.pickup_part("floor_robot", purple_pump_pose, PartMsg.PUMP, PartMsg.PURPLE, "right_bins")
        self.move_part_to_agv("floor_robot", purple_pump_pose, "agv1", 2)
        self.place_part_in_tray("floor_robot", "agv1", 2)
        self.retract_from_agv("floor_robot", "agv1")

        # move robot home
        self.move_robot_home("floor_robot")

        # pick and place blue battery
        self.pickup_part("floor_robot", blue_battery_pose, PartMsg.BATTERY, PartMsg.BLUE, "left_bins")
        self.move_part_to_agv("floor_robot", blue_battery_pose, "agv1", 1)
        self.place_part_in_tray("floor_robot", "agv1", 1)
        self.retract_from_agv("floor_robot", "agv1")

        # move robot home
        self.move_robot_home("floor_robot")

        # move agv to warehouse
        self.move_agv_to_warehouse("agv1")
        self._kit_completed = True

    def place_part_in_tray(self, robot, agv, quadrant):
        '''
        Places a part in the tray

        Args:
            robot (str): Robot to use
            agv (str): AGV to place the part in
            quadrant (int): Quadrant of the tray to place the part in
        '''
        request = PlacePartInTray.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        elif robot == "ceiling_robot":
            request.robot = RobotsMsg.CEILING_ROBOT

        request.agv = agv
        request.quadrant = quadrant

        future = self._place_part_in_tray_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            self.get_logger().warn('Interrupted while waiting for the service. Exiting...')
            return

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Part positioned above tray')
                self.get_logger().warn('Deactivate gripper')
                self.set_floor_robot_gripper_state(False)
        else:
            self.get_logger().warn(f'Service call failed {future.exception()}')

    def move_part_to_agv(self, robot, part_pose, agv, quadrant):
        '''
        Moves the part to the AGV

        Args:
            robot (str): Robot to use
            part_pose (Pose): Pose of the part
            agv (str): AGV to move to
            quadrant (int): Quadrant of the AGV to move to
        '''
        request = MovePartToAGV.Request()

        if robot == "floor_robot":
            request.robot = MovePartToAGV.Request.FLOOR_ROBOT
        elif robot == "ceiling_robot":
            request.robot = MovePartToAGV.Request.CEILING_ROBOT

        request.part_pose = part_pose
        # request.part_type = part_type
        # request.part_color = part_color
        request.agv = agv
        request.quadrant = quadrant

        # self.get_logger().warn('Before service call')
        future = self._move_part_to_agv_client.call_async(request)
        # self.get_logger().warn('After service call')

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            self.get_logger().warn('Interrupted while waiting for the service. Exiting...')
            return

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Part moved to AGV')
        else:
            self.get_logger().warn(f'Service call failed {future.exception()}')

    def _orders_cb(self, msg: Order):
        '''Callback for the topic /ariac/orders
        Arguments:
            msg -- Order message
        '''
        order = Order(msg)
        self._orders.append(order)
        if self._parse_incoming_order:
            self.get_logger().info(self._parse_order(order))

    def _advanced_camera0_cb(self, msg: AdvancedLogicalCameraImageMsg):
        '''Callback for the topic /ariac/sensors/advanced_camera_0/image

        Arguments:
            msg -- AdvancedLogicalCameraImage message
        '''
        self._camera_image = AdvancedLogicalCameraImage(msg.part_poses,
                                                        msg.tray_poses,
                                                        msg.sensor_pose)

    def _breakbeam0_cb(self, msg: BreakBeamStatusMsg):
        '''Callback for the topic /ariac/sensors/breakbeam_0/status

        Arguments:
            msg -- BreakBeamStatusMsg message
        '''
        if not self._object_detected and msg.object_detected:
            self._conveyor_part_count += 1

        self._object_detected = msg.object_detected

    def _competition_state_cb(self, msg: CompetitionStateMsg):
        '''Callback for the topic /ariac/competition_state
        Arguments:
            msg -- CompetitionState message
        '''
        # Log if competition state has changed
        if self._competition_state != msg.competition_state:
            state = CompetitionInterface._competition_states[msg.competition_state]
            self.get_logger().info(f'Competition state is: {state}', throttle_duration_sec=1.0)

        self._competition_state = msg.competition_state

    def _floor_robot_gripper_state_cb(self, msg: VacuumGripperState):
        '''Callback for the topic /ariac/floor_robot_gripper_state

        Arguments:
            msg -- VacuumGripperState message
        '''
        self._floor_robot_gripper_state = msg

    def start_competition(self):
        '''Function to start the competition.
        '''
        self.get_logger().info('Waiting for competition to be ready')

        # Wait for competition to be ready
        while self._competition_state != CompetitionStateMsg.READY:
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                return

        self.get_logger().info('Competition is ready. Starting...')

        # Check if service is available
        if not self._start_competition_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Service \'/ariac/start_competition\' is not available.')
            return

        # Create trigger request and call starter service
        request = Trigger.Request()
        future = self._start_competition_client.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Started competition.')
        else:
            self.get_logger().warn('Unable to start competition')

    def parse_advanced_camera_image(self, image: AdvancedLogicalCameraImage) -> str:
        '''
        Parse an AdvancedLogicalCameraImage message and return a string representation.
        '''

        if len(image._part_poses) == 0:
            return 'No parts detected'

        output = '\n\n'
        for i, part_pose in enumerate(image._part_poses):
            part_pose: PartPoseMsg
            output += '==========================\n'
            part_color = CompetitionInterface._part_colors[part_pose.part.color].capitalize()
            part_color_emoji = CompetitionInterface._part_colors_emoji[part_pose.part.color]
            part_type = CompetitionInterface._part_types[part_pose.part.type].capitalize()
            output += f'Part {i+1}: {part_color_emoji} {part_color} {part_type}\n'
            output += '--------------------------\n'
            output += 'Camera Frame\n'
            output += '--------------------------\n'

            output += '  Position:\n'
            output += f'    x: {part_pose.pose.position.x:.3f} (m)\n'
            output += f'    y: {part_pose.pose.position.y:.3f} (m)\n'
            output += f'    z: {part_pose.pose.position.z:.3f} (m)\n'

            roll, pitch, yaw = rpy_from_quaternion(part_pose.pose.orientation)
            output += '  Orientation:\n'
            output += f'    roll: {rad_to_deg_str(roll)}\n'
            output += f'    pitch: {rad_to_deg_str(pitch)}\n'
            output += f'    yaw: {rad_to_deg_str(yaw)}\n'

            part_world_pose = multiply_pose(image._sensor_pose, part_pose.pose)
            output += '--------------------------\n'
            output += 'World Frame\n'
            output += '--------------------------\n'

            output += '  Position:\n'
            output += f'    x: {part_world_pose.position.x:.3f} (m)\n'
            output += f'    y: {part_world_pose.position.y:.3f} (m)\n'
            output += f'    z: {part_world_pose.position.z:.3f} (m)\n'

            roll, pitch, yaw = rpy_from_quaternion(part_world_pose.orientation)
            output += '  Orientation:\n'
            output += f'    roll: {rad_to_deg_str(roll)}\n'
            output += f'    pitch: {rad_to_deg_str(pitch)}\n'
            output += f'    yaw: {rad_to_deg_str(yaw)}\n'

            output += '==========================\n\n'

        return output

    def _parse_kitting_task(self, kitting_task: KittingTask):
        '''
        Parses a KittingTask object and returns a string representation.
        Args:
            kitting_task (KittingTask): KittingTask object to parse
        Returns:
            str: String representation of the KittingTask object
        '''
        output = 'Type: Kitting\n'
        output += '==========================\n'
        output += f'AGV: {kitting_task.agv_number}\n'
        output += f'Destination: {CompetitionInterface._destinations[kitting_task.destination]}\n'
        output += f'Tray ID: {kitting_task.tray_id}\n'
        output += 'Products:\n'
        output += '==========================\n'

        quadrants = {1: "Quadrant 1: -",
                     2: "Quadrant 2: -",
                     3: "Quadrant 3: -",
                     4: "Quadrant 4: -"}

        for i in range(1, 5):
            product: KittingPart
            for product in kitting_task.parts:
                if i == product.quadrant:
                    part_color = CompetitionInterface._part_colors[product.part.color].capitalize()
                    part_color_emoji = CompetitionInterface._part_colors_emoji[product.part.color]
                    part_type = CompetitionInterface._part_types[product.part.type].capitalize()
                    quadrants[i] = f'Quadrant {i}: {part_color_emoji} {part_color} {part_type}'
        output += f'\t{quadrants[1]}\n'
        output += f'\t{quadrants[2]}\n'
        output += f'\t{quadrants[3]}\n'
        output += f'\t{quadrants[4]}\n'

        return output

    def _parse_assembly_task(self, assembly_task: AssemblyTask):
        '''
        Parses an AssemblyTask object and returns a string representation.

        Args:
            assembly_task (AssemblyTask): AssemblyTask object to parse

        Returns:
            str: String representation of the AssemblyTask object
        '''
        output = 'Type: Assembly\n'
        output += '==========================\n'
        if len(assembly_task.agv_numbers) == 1:
            output += f'AGV: {assembly_task.agv_number[0]}\n'
        elif len(assembly_task.agv_numbers) == 2:
            output += f'AGV(s): [{assembly_task.agv_numbers[0]}, {assembly_task.agv_numbers[1]}]\n'
        output += f'Station: {self._stations[assembly_task.station].title()}\n'
        output += 'Products:\n'
        output += '==========================\n'

        product: AssemblyPartMsg
        for product in assembly_task.parts:
            part_color = CompetitionInterface._part_colors[product.part.color].capitalize()
            part_color_emoji = CompetitionInterface._part_colors_emoji[product.part.color]
            part_type = CompetitionInterface._part_types[product.part.type].capitalize()

            output += f'Part: {part_color_emoji} {part_color} {part_type}\n'

            output += '  Position:\n'
            output += f'    x: {product.assembled_pose.pose.position.x:.3f} (m)\n'
            output += f'    y: {product.assembled_pose.pose.position.y:.3f} (m)\n'
            output += f'    z: {product.assembled_pose.pose.position.z:.3f} (m)\n'

            roll, pitch, yaw = rpy_from_quaternion(product.assembled_pose.pose.orientation)
            output += '  Orientation:\n'
            output += f'    roll: {rad_to_deg_str(roll)}\n'
            output += f'    pitch: {rad_to_deg_str(pitch)}\n'
            output += f'    yaw: {rad_to_deg_str(yaw)}\n'

            output += '  Install direction:\n'
            output += f'    x: {product.install_direction.x:.1f}\n'
            output += f'    y: {product.install_direction.y:.1f}\n'
            output += f'    z: {product.install_direction.z:.1f}\n'

        return output

    def _parse_combined_task(self, combined_task: CombinedTask):
        '''
        Parses a CombinedTask object and returns a string representation.

        Args:
            combined_task (CombinedTask): CombinedTask object to parse

        Returns:
            str: String representation of the CombinedTask object
        '''

        output = 'Type: Combined\n'
        output += '==========================\n'
        output += f'Station: {self._stations[combined_task.station].title()}\n'
        output += 'Products:\n'
        output += '==========================\n'

        product: AssemblyPartMsg
        for product in combined_task.parts:
            part_color = CompetitionInterface._part_colors[product.part.color].capitalize()
            part_color_emoji = CompetitionInterface._part_colors_emoji[product.part.color]
            part_type = CompetitionInterface._part_types[product.part.type].capitalize()

            output += f'Part: {part_color_emoji} {part_color} {part_type}\n'

            output += '  Position:\n'
            output += f'    x: {product.assembled_pose.pose.position.x:.3f} (m)\n'
            output += f'    y: {product.assembled_pose.pose.position.y:.3f} (m)\n'
            output += f'    z: {product.assembled_pose.pose.position.z:.3f} (m)\n'

            roll, pitch, yaw = rpy_from_quaternion(product.assembled_pose.pose.orientation)
            output += '  Orientation:\n'
            output += f'    roll: {rad_to_deg_str(roll)}\n'
            output += f'    pitch: {rad_to_deg_str(pitch)}\n'
            output += f'    yaw: {rad_to_deg_str(yaw)}\n'

            output += '  Install direction:\n'
            output += f'    x: {product.install_direction.x:.1f}\n'
            output += f'    y: {product.install_direction.y:.1f}\n'
            output += f'    z: {product.install_direction.z:.1f}\n'

        return output

    def _parse_order(self, order: Order):
        '''Parse an order message and return a string representation.
        Args:
            order (Order) -- Order message
        Returns:
            String representation of the order message
        '''
        output = '\n\n==========================\n'
        output += f'Received Order: {order.order_id}\n'
        output += f'Priority: {order.order_priority}\n'

        if order.order_type == OrderMsg.KITTING:
            output += self._parse_kitting_task(order.order_task)
        elif order.order_type == OrderMsg.ASSEMBLY:
            output += self._parse_assembly_task(order.order_task)
        elif order.order_type == OrderMsg.COMBINED:
            output += self._parse_combined_task(order.order_task)
        else:
            output += 'Type: Unknown\n'
        return output

    def lock_agv_tray(self, num):
        '''
        Lock the tray of an AGV and parts on the tray. 
        This will prevent tray and parts from moving during transport.
        Args:
            num (int):  AGV number
        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''

        # Create a client to send a request to the `/ariac/agv{num}_lock_tray` service
        tray_locker = self.create_client(
            Trigger,
            f'/ariac/agv{num}_lock_tray'
        )

        # Build the request
        request = Trigger.Request()
        # Send the request
        future = tray_locker.call_async(request)

        # Wait for the response
        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        # Check the response
        if future.result().success:
            self.get_logger().info(f'Locked AGV{num}\'s tray')
        else:
            self.get_logger().warn('Unable to lock tray')

    def move_agv_to_station(self, num, station):
        '''
        Move an AGV to an assembly station.
        Args:
            num (int): AGV number
            station (int): Assembly station number
        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''

        # Create a client to send a request to the `/ariac/move_agv` service.
        mover = self.create_client(
            MoveAGV,
            f'/ariac/move_agv{num}')

        # Create a request object.
        request = MoveAGV.Request()

        # Set the request location.
        if station in [AssemblyTaskMsg.AS1, AssemblyTaskMsg.AS3]:
            request.location = MoveAGV.Request.ASSEMBLY_FRONT
        else:
            request.location = MoveAGV.Request.ASSEMBLY_BACK

        # Send the request.
        future = mover.call_async(request)

        # Wait for the server to respond.
        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        # Check the result of the service call.
        if future.result().success:
            self.get_logger().info(f'Moved AGV{num} to {self._stations[station]}')
        else:
            self.get_logger().warn(future.result().message)

    def move_agv_to_warehouse(self, agv):
        '''
        Move an AGV to the warehouse.
        Args:
            agv (str): AGV number
        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''

        # Create a client to send a request to the `/ariac/move_agv` service.
        mover = self.create_client(
            MoveAGV,
            f'/ariac/move_{agv}')

        # Create a request object.
        request = MoveAGV.Request()

        request.location = MoveAGV.Request.WAREHOUSE

        # Send the request.
        future = mover.call_async(request)

        # Wait for the server to respond.
        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        # Check the result of the service call.
        if future.result().success:
            self.get_logger().info(f'Moved AGV{agv} to {self._stations[MoveAGV.Request.WAREHOUSE]}')
        else:
            self.get_logger().warn(future.result().message)

    def set_floor_robot_gripper_state(self, state):
        '''Set the gripper state of the floor robot.

        Arguments:
            state -- True to enable, False to disable

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''
        if self._floor_robot_gripper_state.enabled == state:
            self.get_logger().warn(f'Gripper is already {self._gripper_states[state]}')
            return

        request = VacuumGripperControl.Request()
        request.enable = state

        future = self._floor_gripper_enable.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result().success:
            self.get_logger().info(f'Changed gripper state to {self._gripper_states[state]}')
        else:
            self.get_logger().warn('Unable to change gripper state')

    def wait(self, duration):
        '''Wait for a specified duration.

        Arguments:
            duration -- Duration to wait in seconds

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''
        start = self.get_clock().now()

        while self.get_clock().now() <= start + Duration(seconds=duration):
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt as kb_error:
                raise KeyboardInterrupt from kb_error

    def change_gripper(self, gripper_type):
        '''
        Change the gripper type.

        Args:
            gripper_type (str): Type of gripper to change to ('trays' or 'parts')

        Raises:
            KeyboardInterrupt: _description_
        '''
        request = ChangeGripper.Request()

        if gripper_type == 'trays':
            request.gripper_type = ChangeGripper.Request.TRAY_GRIPPER
        elif gripper_type == 'parts':
            request.gripper_type = ChangeGripper.Request.PART_GRIPPER

        future = self._change_gripper_type_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result().success:
            self.get_logger().info(f'Gripper was changed to {gripper_type}')
        else:
            self.get_logger().warn('Unable to change gripper')

    def place_tray(self, robot, tray_id, agv):
        '''
        Place a tray on an AGV.

        Args:
            robot (str): Name of the robot to move
            agv (str): Name of the AGV to place the tray on
        '''
        request = PlaceTrayOnAGV.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        elif robot == "ceiling_robot":
            request.robot = RobotsMsg.CEILING_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.agv = agv
        request.tray_id = tray_id

        # self.get_logger().warn('Before service call')
        future = self._place_tray.call_async(request)
        # self.get_logger().warn('After service call')

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Tray placed on AGV')
                self.get_logger().warn('Deactivate gripper')
                self.set_floor_robot_gripper_state(False)
        else:
            self.get_logger().warn(f'Service call failed {future.exception()}')

    def move_tray_to_agv(self, robot, tray_pose, agv_num):
        '''
        Move the robot to an AGV.

        Args:
            robot (str): Name of the robot to move
            tray_pose (Pose): Pose of the tray in the world frame
            station (str): Name of the tray station
            agv_num (str): Number of the AGV

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''
        request = MoveTrayToAGV.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        elif robot == "ceiling_robot":
            request.robot = RobotsMsg.CEILING_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.tray_pose = tray_pose
        request.agv = agv_num

        future = self._move_tray_to_agv_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot moved tray to AGV')
        else:
            self.get_logger().warn(f'Service call failed {future.exception()}')

    def pickup_tray(self, robot, tray_id, tray_pose, station):
        '''
        Move the robot to a tray.

        Args:
            tray_pose (Pose): Pose of the tray in the world frame
            station (str): tray station
        '''
        request = PickupTray.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        elif robot == "ceiling_robot":
            request.robot = RobotsMsg.CEILING_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.tray_station = station
        request.tray_id = tray_id
        request.tray_pose = tray_pose

        future = self._pickup_tray_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot positioned over tray')
                self.get_logger().warn('Activate gripper')
                self.set_floor_robot_gripper_state(True)
        else:
            self.get_logger().warn(f'Service call failed {future.exception()}')

    def retract_from_gripper_slot(self, robot, station, gripper_type):
        '''
        Exit the gripper slot.

        Args:
            station (str): Gripper station
            gripper_type (str): Gripper type

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''
        request = ExitToolChanger.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        elif robot == "ceiling_robot":
            request.robot = RobotsMsg.CEILING_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.station = station
        request.gripper_type = gripper_type

        future = self._exit_gripper_slot_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot retracted from tool changer')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to retract from tool changer')

    def retract_from_agv(self, robot, agv):
        '''
        Retracts the robot from the AGV

        Args:
            robot (str): Robot to use
            agv (str): AGV to retract from

        Raises:
            KeyboardInterrupt: Raised if the user presses Ctrl+C
        '''
        request = RetractFromAGV.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        elif robot == "ceiling_robot":
            request.robot = RobotsMsg.CEILING_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.agv = agv

        future = self._retract_from_agv_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            self.get_logger().warn('Interrupted while waiting for the service. Exiting...')
            return

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot retracted from AGV')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to retract from AGV')

    def goto_tool_changer(self, robot, station, gripper_type):
        '''
        Move the end effector inside the gripper slot.

        Args:
            station (str): Gripper station name
            gripper_type (str): Gripper type

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''

        self.get_logger().info('Move inside gripper slot service called')

        request = EnterToolChanger.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        elif robot == "ceiling_robot":
            request.robot = RobotsMsg.CEILING_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.station = station
        request.gripper_type = gripper_type

        future = self._goto_tool_changer_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot is at the tool changer')
                self.change_gripper(gripper_type)
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move the robot to the tool changer')

    def pickup_part(self, robot, part_pose, part_type, part_color, bin_side):
        '''
        Moves the robot to the part

        Args:
            robot (str): Robot to use
            part_pose (Pose): Pose of the part in the world frame
            part_type (int): Type of the part
            bin_side (str): Bin side to move to

        Raises:
            KeyboardInterrupt: Raised if the user presses Ctrl+C
        '''
        self.set_floor_robot_gripper_state(True)

        request = PickupPart.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        elif robot == "ceiling_robot":
            request.robot = RobotsMsg.CEILING_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.part_pose = part_pose
        request.part_type = part_type
        request.part_color = part_color
        request.bin_side = bin_side

        future = self._pickup_part_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            self.get_logger().warn('Interrupted while waiting for the service. Exiting...')
            return

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot is above part')

        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move robot above part')

    def move_robot_home(self, robot_name):
        '''Move one of the robots to its home position.

        Arguments:
            robot_name -- Name of the robot to move home
        '''
        request = Trigger.Request()

        if robot_name == 'floor_robot':
            if not self._move_floor_robot_home_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Robot commander node not running')
                return

            future = self._move_floor_robot_home_client.call_async(request)

        elif robot_name == 'ceiling_robot':
            if not self._move_ceiling_robot_home.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Robot commander node not running')
                return
            future = self._move_ceiling_robot_home.call_async(request)
        else:
            self.get_logger().error(f'Robot name: ({robot_name}) is not valid')
            return

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Moved {robot_name} to home position')
        else:
            self.get_logger().warn(future.result().message)
