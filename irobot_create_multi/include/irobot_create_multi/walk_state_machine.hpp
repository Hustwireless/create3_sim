#pragma once

#include "irobot_create_multi/state.hpp"
#include "create3_walk_msgs/action/walk.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_msgs/action/dock.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "irobot_create_msgs/action/navigate_to_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "irobot_create_multi/behaviors/behavior.hpp"
#include "irobot_create_multi/behaviors/drive-straight-behavior.hpp"
#include "irobot_create_multi/behaviors/rotate-behavior.hpp"
#include "irobot_create_multi/behaviors/dock-behavior.hpp"
#include "irobot_create_multi/behaviors/undock-behavior.hpp"
#include "irobot_create_multi/behaviors/nav-behavior.hpp"

namespace create3_walk {

class WalkStateMachine {
public:
    using DockAction = irobot_create_msgs::action::Dock;
    using UndockAction = irobot_create_msgs::action::Undock;
    using NavAction = irobot_create_msgs::action::NavigateToPosition;
    using TwistMsg = geometry_msgs::msg::Twist;

    struct WalkOutput {
        int32_t current_behavior;
        State state;
    };

    WalkStateMachine(
        create3_walk_msgs::action::Walk::Goal goal,
        rclcpp::Clock::SharedPtr clock,
        rclcpp::Logger logger,
        rclcpp_action::Client<DockAction>::SharedPtr dock_action_client,
        rclcpp_action::Client<UndockAction>::SharedPtr undock_action_client,
        rclcpp_action::Client<NavAction>::SharedPtr nav_action_client,
        rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
        bool has_reflexes);
    ~WalkStateMachine();

    WalkOutput execute(const Behavior::Data & data);

    void cancel();

private:
    using FeedbackMsg = create3_walk_msgs::action::Walk::Feedback;

    void select_start_behavior(const Behavior::Data& data);

    void select_next_behavior(const Behavior::Data& data);

    void goto_dock();
    void goto_drive_straight(const DriveStraightBehavior::Config& config = DriveStraightBehavior::Config());
    void goto_rotate(const RotateBehavior::Config& config = RotateBehavior::Config());
    void goto_undock();
    void goto_nav();

    double compute_evade_rotation(const geometry_msgs::msg::Pose& pose, double resolution);

    std::shared_ptr<Behavior> m_current_behavior;
    State m_behavior_state;

    bool m_undocking;
    rclcpp::Time m_last_spiral_time;
    bool m_preparing_spiral;
    std::vector<double> m_evade_attempts;

    WalkOutput m_walk_output;
    create3_walk_msgs::action::Walk::Goal m_goal;
    rclcpp::Time m_start_time;
    bool m_has_reflexes;

    rclcpp_action::Client<DockAction>::SharedPtr m_dock_action_client;
    rclcpp_action::Client<UndockAction>::SharedPtr m_undock_action_client;
    rclcpp_action::Client<NavAction>::SharedPtr m_nav_action_client;
    rclcpp::Publisher<TwistMsg>::SharedPtr m_cmd_vel_publisher;
    rclcpp::Logger m_logger;
    rclcpp::Clock::SharedPtr m_clock;
};


} // namespace create3_walk