#include "robot_commander/floor_robot_commander.hpp"

FloorRobotCommander::FloorRobotCommander() : Node("floor_robot_commander"),
                                             node_(std::make_shared<rclcpp::Node>("example_group_node")),
                                             executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()),
                                             planning_scene_()
{
    // Use upper joint velocity and acceleration limits
    // start of testings

    auto mgi_options = moveit::planning_interface::MoveGroupInterface::Options(
        "floor_robot",
        "robot_description");

    floor_robot_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, mgi_options);
    // if (floor_robot_->startStateMonitor())
    // {
    //     RCLCPP_INFO(this->get_logger(), "Floor Robot State Monitor Started");
    // }
    // else
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Floor Robot State Monitor Failed to Start");
    // }
    // end of testings
    floor_robot_->setMaxAccelerationScalingFactor(1.0);
    floor_robot_->setMaxVelocityScalingFactor(1.0);

    // Subscribe to topics
    // rclcpp::SubscriptionOptions options;

    // topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // options.callback_group = topic_cb_group_;

    competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>(
        "/ariac/competition_state", 1,
        std::bind(&FloorRobotCommander::competition_state_cb, this, std::placeholders::_1));

    kts1_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/kts1_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&FloorRobotCommander::kts1_camera_cb, this, std::placeholders::_1));

    kts2_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/kts2_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&FloorRobotCommander::kts2_camera_cb, this, std::placeholders::_1));

    left_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/left_bins_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&FloorRobotCommander::left_bins_camera_cb, this, std::placeholders::_1));

    right_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/right_bins_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&FloorRobotCommander::right_bins_camera_cb, this, std::placeholders::_1));

    floor_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
        "/ariac/floor_robot_gripper_state", rclcpp::SensorDataQoS(),
        std::bind(&FloorRobotCommander::floor_gripper_state_cb, this, std::placeholders::_1));

    // Initialize service clients
    quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>("/ariac/perform_quality_check");
    floor_robot_tool_changer_ = this->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper");
    floor_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/floor_robot_enable_gripper");

    // Register services

    floor_robot_place_tray_on_agv_srv_ = create_service<competitor_interfaces::srv::PlaceTrayOnAGV>(
        "/competitor/floor_robot/place_tray_on_agv",
        std::bind(
            &FloorRobotCommander::FloorRobotPlaceTrayOnAGVServiceCallback, this,
            std::placeholders::_1, std::placeholders::_2));

    floor_robot_move_home_srv_ = create_service<std_srvs::srv::Trigger>(
        "/competitor/floor_robot/go_home",
        std::bind(
            &FloorRobotCommander::FloorRobotMoveHomeServiceCallback, this,
            std::placeholders::_1, std::placeholders::_2));

    floor_robot_retract_from_agv_ = create_service<competitor_interfaces::srv::RetractFromAGV>(
        "/competitor/floor_robot/retract_from_agv",
        std::bind(
            &FloorRobotCommander::FloorRobotRetractFromAGVServiceCallback, this,
            std::placeholders::_1, std::placeholders::_2));

    floor_robot_enter_tool_changer_srv_ = create_service<competitor_interfaces::srv::EnterToolChanger>(
        "/competitor/floor_robot/enter_tool_changer",
        std::bind(
            &FloorRobotCommander::FloorRobotEnterToolChangerServiceCallback, this,
            std::placeholders::_1, std::placeholders::_2));

    floor_robot_exit_gripper_slot_srv_ = create_service<competitor_interfaces::srv::ExitToolChanger>(
        "/competitor/floor_robot/exit_tool_changer",
        std::bind(
            &FloorRobotCommander::FloorRobotExitToolChangerServiceCallback, this,
            std::placeholders::_1, std::placeholders::_2));

    floor_robot_pickup_tray_srv_ = create_service<competitor_interfaces::srv::PickupTray>(
        "/competitor/floor_robot/pickup_tray",
        std::bind(
            &FloorRobotCommander::FloorRobotPickupTrayServiceCallback, this,
            std::placeholders::_1, std::placeholders::_2));

    floor_robot_pickup_part_srv_ = create_service<competitor_interfaces::srv::PickupPart>(
        "/competitor/floor_robot/pickup_part",
        std::bind(
            &FloorRobotCommander::FloorRobotPickupPartServiceCallback, this,
            std::placeholders::_1, std::placeholders::_2));

    floor_robot_move_tray_to_agv_srv_ = create_service<competitor_interfaces::srv::MoveTrayToAGV>(
        "/competitor/floor_robot/move_tray_to_agv",
        std::bind(
            &FloorRobotCommander::MoveTrayToAGVServiceCallback, this,
            std::placeholders::_1, std::placeholders::_2));

    floor_robot_move_part_to_agv_srv_ = create_service<competitor_interfaces::srv::MovePartToAGV>(
        "/competitor/floor_robot/move_part_to_agv",
        std::bind(
            &FloorRobotCommander::FloorRobotMovePartToAGVServiceCallback, this,
            std::placeholders::_1, std::placeholders::_2));

    floor_robot_place_part_in_tray_srv_ = create_service<competitor_interfaces::srv::PlacePartInTray>(
        "/competitor/floor_robot/place_part_in_tray",
        std::bind(
            &FloorRobotCommander::FloorRobotPlacePartInTrayServiceCallback, this,
            std::placeholders::_1, std::placeholders::_2));

    AddModelsToPlanningScene();

    // Start of testings
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]()
                                   { this->executor_->spin(); });
    // End of testings
    RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

FloorRobotCommander::~FloorRobotCommander()
{
    floor_robot_->~MoveGroupInterface();
}

//--- Service callbacks ---//

void FloorRobotCommander::FloorRobotPlacePartInTrayServiceCallback(
    competitor_interfaces::srv::PlacePartInTray::Request::SharedPtr req_,
    competitor_interfaces::srv::PlacePartInTray::Response::SharedPtr res_)
{

    if (req_->robot != competitor_interfaces::msg::Robots::FLOOR_ROBOT)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Wrong robot specified in service call");
        res_->success = false;
        return;
    }

    if (FloorRobotPlacePartInTray(req_->agv, req_->quadrant))
    {
        res_->success = true;
    }
    else
    {
        res_->success = false;
    }
}

void FloorRobotCommander::FloorRobotPickupPartServiceCallback(
    competitor_interfaces::srv::PickupPart::Request::SharedPtr req_,
    competitor_interfaces::srv::PickupPart::Response::SharedPtr res_)
{
    if (req_->robot != competitor_interfaces::msg::Robots::FLOOR_ROBOT)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Wrong robot specified in service call");
        res_->success = false;
        return;
    }

    if (FloorRobotPickupPart(req_->part_pose, req_->part_type, req_->part_color, req_->bin_side))
    {
        res_->success = true;
    }
    else
    {
        res_->success = false;
    }
}

void FloorRobotCommander::FloorRobotPickupTrayServiceCallback(
    competitor_interfaces::srv::PickupTray::Request::SharedPtr req_,
    competitor_interfaces::srv::PickupTray::Response::SharedPtr res_)
{
    if (req_->robot != competitor_interfaces::msg::Robots::FLOOR_ROBOT)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Wrong robot specified in service call");
        res_->success = false;
        return;
    }

    if (FloorRobotPickupTray(req_->tray_id, req_->tray_pose, req_->tray_station))
    {
        res_->success = true;
    }
    else
    {
        res_->success = false;
    }
}

void FloorRobotCommander::MoveTrayToAGVServiceCallback(
    competitor_interfaces::srv::MoveTrayToAGV::Request::SharedPtr req_,
    competitor_interfaces::srv::MoveTrayToAGV::Response::SharedPtr res_)
{
    if (req_->robot != competitor_interfaces::msg::Robots::FLOOR_ROBOT)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Wrong robot specified in service call");
        res_->success = false;
        return;
    }

    if (FloorRobotMoveTrayToAGV(req_->tray_pose, req_->agv))
    {
        res_->success = true;
    }
    else
    {
        res_->success = false;
    }
}

void FloorRobotCommander::FloorRobotEnterToolChangerServiceCallback(
    competitor_interfaces::srv::EnterToolChanger::Request::SharedPtr req_,
    competitor_interfaces::srv::EnterToolChanger::Response::SharedPtr res_)
{
    if (req_->robot != competitor_interfaces::msg::Robots::FLOOR_ROBOT)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Wrong robot specified in service call");
        res_->success = false;
        return;
    }

    if (FloorRobotEnterToolChanger(req_->station, req_->gripper_type))
    {
        res_->success = true;
    }
    else
    {
        res_->success = false;
    }
}

//--- Service callbacks ---//

void FloorRobotCommander::FloorRobotMovePartToAGVServiceCallback(
    competitor_interfaces::srv::MovePartToAGV::Request::SharedPtr req_,
    competitor_interfaces::srv::MovePartToAGV::Response::SharedPtr res_)
{
    if (req_->robot != competitor_interfaces::msg::Robots::FLOOR_ROBOT)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Wrong robot specified in service call");
        res_->success = false;
        return;
    }

    if (FloorRobotMovePartToAGV(req_->part_pose,  req_->agv))
    {
        res_->success = true;
    }
    else
    {
        res_->success = false;
    }
}

void FloorRobotCommander::FloorRobotPlaceTrayOnAGVServiceCallback(
    competitor_interfaces::srv::PlaceTrayOnAGV::Request::SharedPtr req_,
    competitor_interfaces::srv::PlaceTrayOnAGV::Response::SharedPtr res_)
{
    if (req_->robot != competitor_interfaces::msg::Robots::FLOOR_ROBOT)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Wrong robot specified in service call");
        res_->success = false;
        return;
    }

    if (FloorRobotPlaceTrayOnAGV(req_->agv, req_->tray_id))
    {
        res_->success = true;
    }
    else
    {
        res_->success = false;
    }
}

void FloorRobotCommander::FloorRobotExitToolChangerServiceCallback(
    competitor_interfaces::srv::ExitToolChanger::Request::SharedPtr req_,
    competitor_interfaces::srv::ExitToolChanger::Response::SharedPtr res_)
{
    if (req_->robot != competitor_interfaces::msg::Robots::FLOOR_ROBOT)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Wrong robot specified in service call");
        res_->success = false;
        return;
    }

    if (FloorRobotExitToolChanger(req_->station, req_->gripper_type))
    {
        res_->success = true;
    }
    else
    {
        res_->success = false;
    }
}

void FloorRobotCommander::FloorRobotRetractFromAGVServiceCallback(
    competitor_interfaces::srv::RetractFromAGV::Request::SharedPtr req_,
    competitor_interfaces::srv::RetractFromAGV::Response::SharedPtr res_)
{
    if (req_->robot != competitor_interfaces::msg::Robots::FLOOR_ROBOT)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Wrong robot specified in service call");
        res_->success = false;
        return;
    }

    if (FloorRobotRetractFromAGV(req_->agv))
    {
        res_->success = true;
    }
    else
    {
        res_->success = false;
    }
}

//--- Service callbacks ---//
void FloorRobotCommander::FloorRobotMoveHomeServiceCallback(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res)
{
    (void)req; // remove unused parameter warning
    floor_robot_->setNamedTarget("home");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(floor_robot_->plan(plan));

    if (success)
    {
        if (static_cast<bool>(floor_robot_->execute(plan)))
        {
            res->success = true;
        }
        else
        {
            res->success = false;
            res->message = "Trajectory execution failed";
        }
    }
    else
    {
        res->message = "Unable to generate trajectory";
        res->success = false;
    }
}

void FloorRobotCommander::competition_state_cb(
    const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg)
{
    competition_state_ = msg->competition_state;
}

void FloorRobotCommander::kts1_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!kts1_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from kts1 camera");
        kts1_camera_received_data = true;
    }

    kts1_trays_ = msg->tray_poses;
    kts1_camera_pose_ = msg->sensor_pose;
}

void FloorRobotCommander::kts2_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!kts2_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from kts2 camera");
        kts2_camera_received_data = true;
    }

    kts2_trays_ = msg->tray_poses;
    kts2_camera_pose_ = msg->sensor_pose;
}

void FloorRobotCommander::left_bins_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!left_bins_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from left bins camera");
        left_bins_camera_received_data = true;
    }

    left_bins_parts_ = msg->part_poses;
    left_bins_camera_pose_ = msg->sensor_pose;
}

void FloorRobotCommander::right_bins_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!right_bins_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from right bins camera");
        right_bins_camera_received_data = true;
    }

    right_bins_parts_ = msg->part_poses;
    right_bins_camera_pose_ = msg->sensor_pose;
}

void FloorRobotCommander::floor_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{
    floor_gripper_state_ = *msg;
}

geometry_msgs::msg::Pose FloorRobotCommander::MultiplyPose(
    geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
{
    KDL::Frame f1;
    KDL::Frame f2;

    tf2::fromMsg(p1, f1);
    tf2::fromMsg(p2, f2);

    KDL::Frame f3 = f1 * f2;

    return tf2::toMsg(f3);
}

void FloorRobotCommander::LogPose(geometry_msgs::msg::Pose p)
{
    tf2::Quaternion q(
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    roll *= 180 / M_PI;
    pitch *= 180 / M_PI;
    yaw *= 180 / M_PI;

    RCLCPP_INFO(get_logger(), "(X: %.2f, Y: %.2f, Z: %.2f, R: %.2f, P: %.2f, Y: %.2f)",
                p.position.x, p.position.y, p.position.z,
                roll, pitch, yaw);
}

geometry_msgs::msg::Pose FloorRobotCommander::BuildPose(
    double x, double y, double z, geometry_msgs::msg::Quaternion orientation)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = orientation;

    return pose;
}

geometry_msgs::msg::Pose FloorRobotCommander::FrameWorldPose(std::string frame_id)
{
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::Pose pose;

    try
    {
        t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(get_logger(), "Could not get transform");
    }

    pose.position.x = t.transform.translation.x;
    pose.position.y = t.transform.translation.y;
    pose.position.z = t.transform.translation.z;
    pose.orientation = t.transform.rotation;

    return pose;
}

double FloorRobotCommander::GetYaw(geometry_msgs::msg::Pose pose)
{
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

geometry_msgs::msg::Quaternion FloorRobotCommander::QuaternionFromRPY(double r, double p, double y)
{
    tf2::Quaternion q;
    geometry_msgs::msg::Quaternion q_msg;

    q.setRPY(r, p, y);

    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();

    return q_msg;
}

void FloorRobotCommander::AddModelToPlanningScene(
    std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
{
    moveit_msgs::msg::CollisionObject collision;

    collision.id = name;
    collision.header.frame_id = "world";

    shape_msgs::msg::Mesh mesh;
    shapes::ShapeMsg mesh_msg;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("test_competitor");
    std::stringstream path;
    path << "file://" << package_share_directory << "/meshes/" << mesh_file;
    std::string model_path = path.str();

    shapes::Mesh *m = shapes::createMeshFromResource(model_path);
    shapes::constructMsgFromShape(m, mesh_msg);

    mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    collision.meshes.push_back(mesh);
    collision.mesh_poses.push_back(model_pose);

    collision.operation = collision.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision);

    planning_scene_.addCollisionObjects(collision_objects);
}

void FloorRobotCommander::AddModelsToPlanningScene()
{
    // Add bins
    std::map<std::string, std::pair<double, double>> bin_positions = {
        {"bin1", std::pair<double, double>(-1.9, 3.375)},
        {"bin2", std::pair<double, double>(-1.9, 2.625)},
        {"bin3", std::pair<double, double>(-2.65, 2.625)},
        {"bin4", std::pair<double, double>(-2.65, 3.375)},
        {"bin5", std::pair<double, double>(-1.9, -3.375)},
        {"bin6", std::pair<double, double>(-1.9, -2.625)},
        {"bin7", std::pair<double, double>(-2.65, -2.625)},
        {"bin8", std::pair<double, double>(-2.65, -3.375)}};

    geometry_msgs::msg::Pose bin_pose;
    for (auto const &bin : bin_positions)
    {
        bin_pose.position.x = bin.second.first;
        bin_pose.position.y = bin.second.second;
        bin_pose.position.z = 0;
        bin_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

        AddModelToPlanningScene(bin.first, "bin.stl", bin_pose);
    }

    // Add assembly stations
    std::map<std::string, std::pair<double, double>> assembly_station_positions = {
        {"as1", std::pair<double, double>(-7.3, 3)},
        {"as2", std::pair<double, double>(-12.3, 3)},
        {"as3", std::pair<double, double>(-7.3, -3)},
        {"as4", std::pair<double, double>(-12.3, -3)},
    };

    geometry_msgs::msg::Pose assembly_station_pose;
    for (auto const &station : assembly_station_positions)
    {
        assembly_station_pose.position.x = station.second.first;
        assembly_station_pose.position.y = station.second.second;
        assembly_station_pose.position.z = 0;
        assembly_station_pose.orientation = QuaternionFromRPY(0, 0, 0);

        AddModelToPlanningScene(station.first, "assembly_station.stl", assembly_station_pose);
    }

    // Add assembly briefcases
    std::map<std::string, std::pair<double, double>> assembly_insert_positions = {
        {"as1_insert", std::pair<double, double>(-7.7, 3)},
        {"as2_insert", std::pair<double, double>(-12.7, 3)},
        {"as3_insert", std::pair<double, double>(-7.7, -3)},
        {"as4_insert", std::pair<double, double>(-12.7, -3)},
    };

    geometry_msgs::msg::Pose assembly_insert_pose;
    for (auto const &insert : assembly_insert_positions)
    {
        assembly_insert_pose.position.x = insert.second.first;
        assembly_insert_pose.position.y = insert.second.second;
        assembly_insert_pose.position.z = 1.011;
        assembly_insert_pose.orientation = QuaternionFromRPY(0, 0, 0);

        AddModelToPlanningScene(insert.first, "assembly_insert.stl", assembly_insert_pose);
    }

    geometry_msgs::msg::Pose conveyor_pose;
    conveyor_pose.position.x = -0.6;
    conveyor_pose.position.y = 0;
    conveyor_pose.position.z = 0;
    conveyor_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene("conveyor", "conveyor.stl", conveyor_pose);

    geometry_msgs::msg::Pose kts1_table_pose;
    kts1_table_pose.position.x = -1.3;
    kts1_table_pose.position.y = -5.84;
    kts1_table_pose.position.z = 0;
    kts1_table_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

    AddModelToPlanningScene("kts1_table", "kit_tray_table.stl", kts1_table_pose);

    geometry_msgs::msg::Pose kts2_table_pose;
    kts2_table_pose.position.x = -1.3;
    kts2_table_pose.position.y = 5.84;
    kts2_table_pose.position.z = 0;
    kts2_table_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene("kts2_table", "kit_tray_table.stl", kts2_table_pose);
}

geometry_msgs::msg::Quaternion FloorRobotCommander::SetRobotOrientation(double rotation)
{
    tf2::Quaternion tf_q;
    tf_q.setRPY(0, 3.14159, rotation);

    geometry_msgs::msg::Quaternion q;

    q.x = tf_q.x();
    q.y = tf_q.y();
    q.z = tf_q.z();
    q.w = tf_q.w();

    return q;
}

bool FloorRobotCommander::FloorRobotMovetoTarget()
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(floor_robot_->plan(plan));

    if (success)
    {
        return static_cast<bool>(floor_robot_->execute(plan));
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate plan");
        return false;
    }
}

bool FloorRobotCommander::FloorRobotMoveCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf)
{
    moveit_msgs::msg::RobotTrajectory trajectory;

    double path_fraction = floor_robot_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (path_fraction < 0.9)
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
        return false;
    }

    // Retime trajectory
    // floor_robot_->startStateMonitor();
    robot_trajectory::RobotTrajectory rt(floor_robot_->getCurrentState()->getRobotModel(), "floor_robot");
    rt.setRobotTrajectoryMsg(*floor_robot_->getCurrentState(), trajectory);
    totg_.computeTimeStamps(rt, vsf, asf);
    rt.getRobotTrajectoryMsg(trajectory);

    return static_cast<bool>(floor_robot_->execute(trajectory));
}

void FloorRobotCommander::FloorRobotWaitForAttach(double timeout)
{
    // Wait for part to be attached
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = floor_robot_->getCurrentPose().pose;

    while (!floor_gripper_state_.attached)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

        waypoints.clear();
        starting_pose.position.z -= 0.0005;
        waypoints.push_back(starting_pose);

        FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

        usleep(600);

        if (now() - start > rclcpp::Duration::from_seconds(timeout))
        {
            RCLCPP_ERROR(get_logger(), "Unable to pick up object");
            return;
        }
    }
}

void FloorRobotCommander::FloorRobotSendHome()
{
    // Move floor robot to home joint state
    floor_robot_->setNamedTarget("home");
    FloorRobotMovetoTarget();
}

bool FloorRobotCommander::FloorRobotSetGripperState(bool enable)
{
    if (floor_gripper_state_.enabled == enable)
    {
        if (floor_gripper_state_.enabled)
            RCLCPP_INFO(get_logger(), "Already enabled");
        else
            RCLCPP_INFO(get_logger(), "Already disabled");

        return false;
    }

    // Call enable service
    auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
    request->enable = enable;

    auto result = floor_robot_gripper_enable_->async_send_request(request);
    result.wait();

    if (!result.get()->success)
    {
        RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
        return false;
    }

    return true;
}


//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
bool FloorRobotCommander::FloorRobotEnterToolChanger(std::string station, std::string gripper_type)
{
    // Move floor robot to the corresponding kit tray table
    if (station == "kts1")
    {
        floor_robot_->setJointValueTarget(floor_kts1_js_);
    }
    else if (station == "kts2")
    {
        floor_robot_->setJointValueTarget(floor_kts2_js_);
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Invalid station name");
        return false;
    }

    FloorRobotMovetoTarget();

    // Move gripper into tool changer
    auto tc_pose = FrameWorldPose(station + "_tool_changer_" + gripper_type + "_frame");

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z, SetRobotOrientation(0.0)));

    if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1))
        return false;

    return true;
}

//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
bool FloorRobotCommander::FloorRobotExitToolChanger(std::string station, std::string gripper_type)
{
    // Move gripper into tool changer
    auto tc_pose = FrameWorldPose(station + "_tool_changer_" + gripper_type + "_frame");

    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

    if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1))
        return false;

    return true;
}

//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
bool FloorRobotCommander::FloorRobotMovePartToAGV(geometry_msgs::msg::Pose part_pose_,  std::string agv_)
{
    // move up after the part is attached
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(part_pose_.position.x, part_pose_.position.y,
                                  part_pose_.position.z + 0.3, SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    // Move to agv
    floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_[agv_]);
    floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    return true;
}

//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
bool FloorRobotCommander::FloorRobotRetractFromAGV(std::string agv)
{

    auto agv_tray_pose = FrameWorldPose(agv + "_tray");
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + 0.3, SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);
    return true;
}

//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
bool FloorRobotCommander::FloorRobotPlaceTrayOnAGV(std::string agv_num_, int tray_id_)
{
    auto agv_tray_pose = FrameWorldPose(agv_num_ + "_tray");
    std::string tray_name = "kit_tray_" + std::to_string(tray_id_);
    auto agv_rotation = GetYaw(agv_tray_pose);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));

    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_, SetRobotOrientation(agv_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);
    floor_robot_->detachObject(tray_name);

    // FloorRobotSetGripperState(false);

    return true;
}

bool FloorRobotCommander::FloorRobotMoveTrayToAGV(geometry_msgs::msg::Pose tray_pose_, std::string agv_num_)
{
    double tray_rotation = GetYaw(tray_pose_);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(tray_pose_.position.x, tray_pose_.position.y,
                                  tray_pose_.position.z + 0.3, SetRobotOrientation(tray_rotation)));

    // floor_robot_->getCurrentPose();
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_[agv_num_]);
    floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);

    // usleep(500);
    FloorRobotMovetoTarget();
    return true;
}

bool FloorRobotCommander::FloorRobotPickupTray(int tray_id_, geometry_msgs::msg::Pose tray_pose_, std::string station_)
{
    double tray_rotation = GetYaw(tray_pose_);

    // Move floor robot to the corresponding kit tray table
    if (station_ == "kts1")
    {
        floor_robot_->setJointValueTarget(floor_kts1_js_);
    }
    else
    {
        floor_robot_->setJointValueTarget(floor_kts2_js_);
    }
    FloorRobotMovetoTarget();

    // Move to tray
    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(BuildPose(tray_pose_.position.x, tray_pose_.position.y,
                                  tray_pose_.position.z + 0.2, SetRobotOrientation(tray_rotation)));
    waypoints.push_back(BuildPose(tray_pose_.position.x, tray_pose_.position.y,
                                  tray_pose_.position.z + pick_offset_, SetRobotOrientation(tray_rotation)));
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
    FloorRobotWaitForAttach(3.0);

    // Add kit tray to planning scene
    std::string tray_name = "kit_tray_" + std::to_string(tray_id_);
    AddModelToPlanningScene(tray_name, "kit_tray.stl", tray_pose_);
    floor_robot_->attachObject(tray_name);

    return true;
}


bool FloorRobotCommander::FloorRobotPickupPart(geometry_msgs::msg::Pose part_pose_, int part_type_, int part_color_, std::string bin_side_)
{
    double part_rotation = GetYaw(part_pose_);
    floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_[bin_side_]);
    floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(part_pose_.position.x, part_pose_.position.y,
                                  part_pose_.position.z + 0.5, SetRobotOrientation(part_rotation)));

    waypoints.push_back(BuildPose(part_pose_.position.x, part_pose_.position.y,
                                  part_pose_.position.z + part_heights_[part_type_] + pick_offset_, SetRobotOrientation(part_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
    FloorRobotWaitForAttach(3.0);

    // Add part to planning scene
    std::string part_name = part_colors_[part_color_] + "_" + part_types_[part_type_];
    AddModelToPlanningScene(part_name, part_types_[part_type_] + ".stl", part_pose_);
    floor_robot_->attachObject(part_name);

    auto part_to_pick = ariac_msgs::msg::Part();
    part_to_pick.type = part_type_;
    part_to_pick.color = part_color_;
    floor_robot_attached_part_ = part_to_pick;
    RCLCPP_INFO_STREAM(get_logger(), "Adding to the planning scene");

    return true;
}


bool FloorRobotCommander::FloorRobotPlacePartInTray(std::string agv_, int quadrant_)
{
    if (!floor_gripper_state_.attached)
    {
        RCLCPP_ERROR(get_logger(), "No part attached");
        return false;
    }

    // Move to agv
    floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_[agv_]);
    floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    // Determine target pose for part based on agv_tray pose
    auto agv_tray_pose = FrameWorldPose(agv_ + "_tray");

    auto part_drop_offset = BuildPose(quad_offsets_[quadrant_].first, quad_offsets_[quadrant_].second, 0.0,
                                      geometry_msgs::msg::Quaternion());

    auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));

    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] + drop_height_,
                                  SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    std::string part_name = part_colors_[floor_robot_attached_part_.color] +
                            "_" + part_types_[floor_robot_attached_part_.type];
    floor_robot_->detachObject(part_name);

    return true;
}

bool FloorRobotCommander::FloorRobotPlacePartOnKitTray(int agv_num, int quadrant)
{

    if (!floor_gripper_state_.attached)
    {
        RCLCPP_ERROR(get_logger(), "No part attached");
        return false;
    }

    // Move to agv
    floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    // Determine target pose for part based on agv_tray pose
    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");

    auto part_drop_offset = BuildPose(quad_offsets_[quadrant].first, quad_offsets_[quadrant].second, 0.0,
                                      geometry_msgs::msg::Quaternion());

    auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));

    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] + drop_height_,
                                  SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    // Drop part in quadrant
    FloorRobotSetGripperState(false);

    std::string part_name = part_colors_[floor_robot_attached_part_.color] +
                            "_" + part_types_[floor_robot_attached_part_.type];
    floor_robot_->detachObject(part_name);

    waypoints.clear();
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3,
                                  SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

    return true;
}


// ================================
int main(int argc, char *argv[])
{

    // start of testings
    rclcpp::init(argc, argv);
    auto move_group_node = std::make_shared<FloorRobotCommander>();
    rclcpp::spin(move_group_node);

    rclcpp::shutdown();
}
