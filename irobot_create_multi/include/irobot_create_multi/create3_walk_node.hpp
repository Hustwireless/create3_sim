#pragma once

#include <mutex>

#include "create3_walk_msgs/action/walk.hpp"
#include "irobot_create_msgs/action/dock.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "irobot_create_msgs/action/navigate_to_position.hpp"
#include "irobot_create_msgs/msg/dock_status.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "irobot_create_msgs/msg/ir_opcode.hpp"
#include "irobot_create_msgs/msg/kidnap_status.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace create3_walk {
class Create3WalkNode : public rclcpp::Node{
public:
    Create3WalkNode();
    ~Create3WalkNode() = default;

private:
    using WalkAction = create3_walk_msgs::action::Walk;
    using GoalHandleWalk = rclcpp_action::ServerGoalHandle<WalkAction>;

    using DockAction = irobot_create_msgs::action::Dock;
    using UndockAction = irobot_create_msgs::action::Undock;
    using NavAction = irobot_create_msgs::action::NavigateToPosition;
    using DockMsg = irobot_create_msgs::msg::DockStatus;
    using HazardMsg = irobot_create_msgs::msg::HazardDetectionVector;
    using KidnapMsg = irobot_create_msgs::msg::KidnapStatus;
    using OdometryMsg = nav_msgs::msg::Odometry;
    using OpCodeMsg = irobot_create_msgs::msg::IrOpcode;
    using TwistMsg = geometry_msgs::msg::Twist;


    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const WalkAction::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleWalk> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleWalk> goal_handle);

    void execute(const std::shared_ptr<GoalHandleWalk> goal_handle);

    void dock_callback(DockMsg::ConstSharedPtr msg);
    void hazards_callback(HazardMsg::ConstSharedPtr msg);
    void ir_opcode_callback(OpCodeMsg::ConstSharedPtr msg);
    void kidnap_callback(KidnapMsg::ConstSharedPtr msg);
    void odom_callback(OdometryMsg::ConstSharedPtr msg);

    bool ready_to_start();
    bool reflexes_setup();

    int32_t m_last_behavior;

    double m_rate_hz;
    int m_opcodes_buffer_ms;

    std::atomic<bool> m_is_running;
    std::mutex m_mutex;

    std::atomic<bool> m_dock_msgs_received;
    DockMsg m_last_dock;
    HazardMsg m_last_hazards;
    KidnapMsg m_last_kidnap;
    OdometryMsg m_last_odom;
    std::vector<OpCodeMsg> m_last_opcodes;
    rclcpp::Time m_last_opcodes_cleared_time;

    rclcpp_action::Server<WalkAction>::SharedPtr m_walk_action_server;

    rclcpp_action::Client<DockAction>::SharedPtr m_dock_action_client;
    rclcpp_action::Client<UndockAction>::SharedPtr m_undock_action_client;
    rclcpp_action::Client<NavAction>::SharedPtr m_nav_action_client;

    rclcpp::Publisher<TwistMsg>::SharedPtr m_cmd_vel_publisher;

    rclcpp::AsyncParametersClient::SharedPtr m_reflexes_param_client;

    rclcpp::Subscription<DockMsg>::SharedPtr m_dock_subscription;
    rclcpp::Subscription<HazardMsg>::SharedPtr m_hazards_subscription;
    rclcpp::Subscription<KidnapMsg>::SharedPtr m_kidnap_subscription;
    rclcpp::Subscription<OdometryMsg>::SharedPtr m_odom_subscription;
    rclcpp::Subscription<OpCodeMsg>::SharedPtr m_ir_opcode_subscription;
};
}  // namespace create3_walk
