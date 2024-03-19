// Copyright 2021 iRobot Corporation. All Rights Reserved.

#pragma once

#include "irobot_create_multi/behaviors/behavior.hpp"
#include "create3_walk_msgs/action/walk.hpp"
#include "irobot_create_msgs/action/navigate_to_position.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace create3_walk {

class NavBehavior : public Behavior
{
public:
    using NavAction = irobot_create_msgs::action::NavigateToPosition;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavAction>;

    NavBehavior(
        rclcpp_action::Client<NavAction>::SharedPtr nav_action_client,
        rclcpp::Logger logger);

    ~NavBehavior() = default;

    State execute(const Data & data) override;

    int32_t get_id() const override { return create3_walk_msgs::action::Walk::Feedback::NAV; }

    void cleanup() override;

private:
    bool m_nav_action_sent {false};
    GoalHandleNav::SharedPtr m_nav_goal_handle;
    bool m_nav_goal_handle_ready {false};
    GoalHandleNav::WrappedResult m_nav_result;
    bool m_nav_result_ready {false};

    rclcpp_action::Client<NavAction>::SharedPtr m_nav_action_client;
    rclcpp::Logger m_logger;
};

} // namespace create3_walk
