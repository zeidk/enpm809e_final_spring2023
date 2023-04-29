#pragma once

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <unistd.h>

#include <cmath>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>

#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>
#include <ariac_msgs/msg/competition_state.hpp>

#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <ariac_msgs/srv/perform_quality_check.hpp>

#include <std_msgs/msg/bool.hpp>

#include <competitor_interfaces/msg/floor_robot_task.hpp>
#include <competitor_interfaces/msg/completed_order.hpp>
#include <competitor_interfaces/msg/robots.hpp>
// Custom Services
#include <competitor_interfaces/srv/change_gripper.hpp>
#include <competitor_interfaces/srv/exit_tool_changer.hpp>
#include <competitor_interfaces/srv/enter_tool_changer.hpp>
#include <competitor_interfaces/srv/pickup_part.hpp>
#include <competitor_interfaces/srv/pickup_tray.hpp>
#include <competitor_interfaces/srv/move_tray_to_agv.hpp>
#include <competitor_interfaces/srv/move_part_to_agv.hpp>
#include <competitor_interfaces/srv/place_tray_on_agv.hpp>
#include <competitor_interfaces/srv/place_part_in_tray.hpp>
#include <competitor_interfaces/srv/retract_from_agv.hpp>
// #include <competitor_interfaces/srv/move_part_to_agv.hpp>
#include <geometry_msgs/msg/pose.hpp>

class FloorRobotCommander : public rclcpp::Node
{
public:
    /// Constructor
    FloorRobotCommander();
    // Destructor
    ~FloorRobotCommander();

    // Floor Robot Public Functions
    void FloorRobotSendHome();
    bool FloorRobotSetGripperState(bool enable);
    bool FloorRobotEnterToolChanger(std::string station, std::string gripper_type);
    bool FloorRobotPlacePartOnKitTray(int agv_num, int quadrant);
    bool FloorRobotExitToolChanger(std::string station, std::string gripper_type);
    bool FloorRobotRetractFromAGV(std::string agv);
    bool FloorRobotPlacePartInTray(std::string agv_, int quadrant_);
    bool FloorRobotPickupTray(int tray_id_, geometry_msgs::msg::Pose tray_pose_, std::string station_);
    bool FloorRobotMoveTrayToAGV(geometry_msgs::msg::Pose tray_pose_, std::string agv_num_);
    bool FloorRobotMovePartToAGV(geometry_msgs::msg::Pose part_pose_,  std::string agv_num_);
    bool FloorRobotPickupPart(geometry_msgs::msg::Pose part_pose_, int part_type_, int part_color_, std::string bin_side_);
    bool FloorRobotPlaceTrayOnAGV(std::string agv, int tray_id);

private:
    // Testing
    rclcpp::Node::SharedPtr node_;
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;
    std::string node_namespace_;

    // MoveIt
    // moveit::planning_interface::MoveGroupInterface floor_robot_;
    moveit::planning_interface::MoveGroupInterfacePtr floor_robot_;

    // Robot Move Functions
    bool FloorRobotMovetoTarget();
    bool FloorRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf);
    void FloorRobotWaitForAttach(double timeout);

    geometry_msgs::msg::Quaternion SetRobotOrientation(double rotation);

    void LogPose(geometry_msgs::msg::Pose p);
    geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);
    geometry_msgs::msg::Pose BuildPose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation);
    geometry_msgs::msg::Pose FrameWorldPose(std::string frame_id);
    double GetYaw(geometry_msgs::msg::Pose pose);
    geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p, double y);

    void AddModelToPlanningScene(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose);
    void AddModelsToPlanningScene();

    moveit::planning_interface::PlanningSceneInterface planning_scene_;

    trajectory_processing::TimeOptimalTrajectoryGeneration totg_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Subscriptions
    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr floor_gripper_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_sub_;

    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts1_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts2_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr left_bins_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr right_bins_camera_sub_;

    // Orders List
    competitor_interfaces::msg::FloorRobotTask current_order_;
    std::vector<competitor_interfaces::msg::FloorRobotTask> orders_;
    unsigned int competition_state_;

    // Gripper State
    ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
    ariac_msgs::msg::Part floor_robot_attached_part_;
    ariac_msgs::msg::VacuumGripperState ceiling_gripper_state_;
    ariac_msgs::msg::Part ceiling_robot_attached_part_;

    // Sensor poses
    geometry_msgs::msg::Pose kts1_camera_pose_;
    geometry_msgs::msg::Pose kts2_camera_pose_;
    geometry_msgs::msg::Pose left_bins_camera_pose_;
    geometry_msgs::msg::Pose right_bins_camera_pose_;

    // Trays
    std::vector<ariac_msgs::msg::KitTrayPose> kts1_trays_;
    std::vector<ariac_msgs::msg::KitTrayPose> kts2_trays_;

    // Bins
    std::vector<ariac_msgs::msg::PartPose> left_bins_parts_;
    std::vector<ariac_msgs::msg::PartPose> right_bins_parts_;

    // Callback Groups
    rclcpp::CallbackGroup::SharedPtr topic_cb_group_;

    // Sensor Callbacks
    bool kts1_camera_received_data = false;
    bool kts2_camera_received_data = false;
    bool left_bins_camera_received_data = false;
    bool right_bins_camera_received_data = false;
    bool floor_robot_task_received_data_ = false;

    void kts1_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void kts2_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void left_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void right_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    // Competition state callback
    void competition_state_cb(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
    // Gripper State Callback
    void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);

    // ARIAC Services
    rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_checker_;
    rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr floor_robot_tool_changer_;
    rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr floor_robot_gripper_enable_;

    // ROS Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr floor_robot_move_home_srv_;
    rclcpp::Service<competitor_interfaces::srv::ChangeGripper>::SharedPtr floor_robot_change_gripper_srv_;
    rclcpp::Service<competitor_interfaces::srv::EnterToolChanger>::SharedPtr floor_robot_enter_tool_changer_srv_;
    rclcpp::Service<competitor_interfaces::srv::ExitToolChanger>::SharedPtr floor_robot_exit_gripper_slot_srv_;
    rclcpp::Service<competitor_interfaces::srv::PickupTray>::SharedPtr floor_robot_pickup_tray_srv_;
    rclcpp::Service<competitor_interfaces::srv::MoveTrayToAGV>::SharedPtr floor_robot_move_tray_to_agv_srv_;
    rclcpp::Service<competitor_interfaces::srv::PlaceTrayOnAGV>::SharedPtr floor_robot_place_tray_on_agv_srv_;
    rclcpp::Service<competitor_interfaces::srv::RetractFromAGV>::SharedPtr floor_robot_retract_from_agv_;
    rclcpp::Service<competitor_interfaces::srv::PickupPart>::SharedPtr floor_robot_pickup_part_srv_;
    rclcpp::Service<competitor_interfaces::srv::MovePartToAGV>::SharedPtr floor_robot_move_part_to_agv_srv_;
    rclcpp::Service<competitor_interfaces::srv::PlacePartInTray>::SharedPtr floor_robot_place_part_in_tray_srv_;

    /**
     * @brief Callback function for for the service /competitor/floor_robot/place_part_in_tray
     *
     * @param req_ Shared pointer to competitor_interfaces::srv::PlacePartInTray::Request
     * @param res_ Shared pointer to competitor_interfaces::srv::PlacePartInTray::Response
     */
    void
    FloorRobotPlacePartInTrayServiceCallback(
        competitor_interfaces::srv::PlacePartInTray::Request::SharedPtr req_,
        competitor_interfaces::srv::PlacePartInTray::Response::SharedPtr res_);

    /**
     * @brief Callback function for for the service /competitor/floor_robot/move_part_to_agv
     *
     * @param req_ Shared pointer to competitor_interfaces::srv::MovePartToAGV::Request
     * @param res_ Shared pointer to competitor_interfaces::srv::MovePartToAGV::Response
     */
    void
    FloorRobotMovePartToAGVServiceCallback(
        competitor_interfaces::srv::MovePartToAGV::Request::SharedPtr req_,
        competitor_interfaces::srv::MovePartToAGV::Response::SharedPtr res_);

    /**
     * @brief Callback function for for the service /competitor/floor_robot/pickup_part
     *
     * @param req_ Shared pointer to competitor_interfaces::srv::PickupPart::Request
     * @param res_ Shared pointer to competitor_interfaces::srv::PickupPart::Response
     */
    void FloorRobotPickupPartServiceCallback(
        competitor_interfaces::srv::PickupPart::Request::SharedPtr req_,
        competitor_interfaces::srv::PickupPart::Response::SharedPtr res_);

    /**
     * @brief Callback function for for the service /competitor/floor_robot/retract_from_agv
     *
     * @param req_ Shared pointer to competitor_interfaces::srv::RetractFromAGV::Request
     * @param res_ Shared pointer to competitor_interfaces::srv::RetractFromAGV::Response
     */
    void
    FloorRobotRetractFromAGVServiceCallback(
        competitor_interfaces::srv::RetractFromAGV::Request::SharedPtr req_,
        competitor_interfaces::srv::RetractFromAGV::Response::SharedPtr res_);

    /**
     * @brief Callback function for for the service /competitor/floor_robot/place_tray_on_agv
     *
     * @param req_ Shared pointer to competitor_interfaces::srv::PlaceTrayOnAGV::Request
     * @param res_ Shared pointer to competitor_interfaces::srv::PlaceTrayOnAGV::Response
     */
    void
    FloorRobotPlaceTrayOnAGVServiceCallback(
        competitor_interfaces::srv::PlaceTrayOnAGV::Request::SharedPtr req_,
        competitor_interfaces::srv::PlaceTrayOnAGV::Response::SharedPtr res_);
    /**
     * @brief Callback function for for the service /competitor/floor_robot/move_tray_to_agv
     *
     * @param req_ Shared pointer to competitor_interfaces::srv::MoveTrayToAGV::Request
     * @param res_ Shared pointer to competitor_interfaces::srv::MoveTrayToAGV::Response
     */
    void MoveTrayToAGVServiceCallback(
        competitor_interfaces::srv::MoveTrayToAGV::Request::SharedPtr req_,
        competitor_interfaces::srv::MoveTrayToAGV::Response::SharedPtr res_);
    /**
     * @brief Callback function for for the service /competitor/floor_robot/pickup_tray
     *
     * @param req_ Shared pointer to competitor_interfaces::srv::PickupTray::Request
     * @param res_ Shared pointer to competitor_interfaces::srv::PickupTray::Response
     */
    void
    FloorRobotPickupTrayServiceCallback(
        competitor_interfaces::srv::PickupTray::Request::SharedPtr req_,
        competitor_interfaces::srv::PickupTray::Response::SharedPtr res_);

    /**
     * @brief Callback function for for the service /competitor/floor_robot/go_home
     *
     * @param req_ Shared pointer to std_srvs::srv::Trigger::Request
     * @param res_ Shared pointer to std_srvs::srv::Trigger::Response
     */
    void
    FloorRobotMoveHomeServiceCallback(
        std_srvs::srv::Trigger::Request::SharedPtr req_,
        std_srvs::srv::Trigger::Response::SharedPtr res_);

    /**
     * @brief Callback function for for the service /competitor/floor_robot/exit_gripper_slot
     *
     * @param req_ Shared pointer to competitor_interfaces::srv::ExitToolChanger::Request
     * @param res_ Shared pointer to competitor_interfaces::srv::ExitToolChanger::Response
     */
    void FloorRobotExitToolChangerServiceCallback(
        competitor_interfaces::srv::ExitToolChanger::Request::SharedPtr req_,
        competitor_interfaces::srv::ExitToolChanger::Response::SharedPtr res_);

    /**
     * @brief Callback function for for the service /competitor/floor_robot/enter_tool_changer
     *
     * @param req_ Shared pointer to competitor_interfaces::srv::EnterToolChanger::Request
     * @param res_ Shared pointer to competitor_interfaces::srv::EnterToolChanger::Response
     */
    void
    FloorRobotEnterToolChangerServiceCallback(
        competitor_interfaces::srv::EnterToolChanger::Request::SharedPtr req_,
        competitor_interfaces::srv::EnterToolChanger::Response::SharedPtr res_);

    // Constants
    double kit_tray_thickness_ = 0.01;
    double drop_height_ = 0.002;
    double pick_offset_ = 0.003;
    double battery_grip_offset_ = -0.05;

    std::map<int, std::string> part_types_ = {
        {ariac_msgs::msg::Part::BATTERY, "battery"},
        {ariac_msgs::msg::Part::PUMP, "pump"},
        {ariac_msgs::msg::Part::REGULATOR, "regulator"},
        {ariac_msgs::msg::Part::SENSOR, "sensor"}};

    std::map<int, std::string> part_colors_ = {
        {ariac_msgs::msg::Part::RED, "red"},
        {ariac_msgs::msg::Part::BLUE, "blue"},
        {ariac_msgs::msg::Part::GREEN, "green"},
        {ariac_msgs::msg::Part::ORANGE, "orange"},
        {ariac_msgs::msg::Part::PURPLE, "purple"},
    };

    // Part heights
    std::map<int, double> part_heights_ = {
        {ariac_msgs::msg::Part::BATTERY, 0.04},
        {ariac_msgs::msg::Part::PUMP, 0.12},
        {ariac_msgs::msg::Part::REGULATOR, 0.07},
        {ariac_msgs::msg::Part::SENSOR, 0.07}};

    // Quadrant Offsets
    std::map<int, std::pair<double, double>> quad_offsets_ = {
        {ariac_msgs::msg::KittingPart::QUADRANT1, std::pair<double, double>(-0.08, 0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT2, std::pair<double, double>(0.08, 0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT3, std::pair<double, double>(-0.08, -0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT4, std::pair<double, double>(0.08, -0.12)},
    };

    std::map<std::string, double> rail_positions_ = {
        {"agv1", -4.5},
        {"agv2", -1.2},
        {"agv3", 1.2},
        {"agv4", 4.5},
        {"left_bins", 3},
        {"right_bins", -3}};

    // Joint value targets for kitting stations
    std::map<std::string, double> floor_kts1_js_ = {
        {"linear_actuator_joint", 4.0},
        {"floor_shoulder_pan_joint", 1.57},
        {"floor_shoulder_lift_joint", -1.57},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.57},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}};

    std::map<std::string, double> floor_kts2_js_ = {
        {"linear_actuator_joint", -4.0},
        {"floor_shoulder_pan_joint", -1.57},
        {"floor_shoulder_lift_joint", -1.57},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.57},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}};
};
