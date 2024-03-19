// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include "irobot_create_multi/behaviors/nav-behavior.hpp"

namespace create3_walk {

NavBehavior::NavBehavior(
    rclcpp_action::Client<NavAction>::SharedPtr nav_action_client,
    rclcpp::Logger logger)
: m_nav_action_client(nav_action_client), m_logger(logger)
{

}

State NavBehavior::execute(const Data & data)
{
    // We can't undock until we discover the nav action server
    if (!m_nav_action_client->action_server_is_ready()) {
        RCLCPP_DEBUG(m_logger, "Waiting for nav action server");
        return State::RUNNING;
    }

    // Send nav command if not already sent and if we are not waiting for result
    if (!m_nav_action_sent) {
        RCLCPP_INFO(m_logger, "Sending nav goal!");
        auto goal_msg = NavAction::Goal();

        auto goal_pose = geometry_msgs::msg::PoseStamped();
        // random number between +- 2.5
        goal_pose.pose.position.x = 5.0 * (rand() % 2 ? 1 : -1);
        goal_pose.pose.position.y = 5.0 * (rand() % 2 ? 1 : -1);
        goal_pose.pose.position.z = 0.0;

        // print nav goal
        RCLCPP_INFO(m_logger, "Nav goal: x: %f, y: %f", goal_pose.pose.position.x, goal_pose.pose.position.y);
        
        goal_msg.goal_pose = goal_pose;

        auto send_goal_options = rclcpp_action::Client<NavAction>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](const GoalHandleNav::SharedPtr & goal_handle){
            m_nav_goal_handle_ready = true;
            m_nav_goal_handle = goal_handle;
        };
        send_goal_options.result_callback = [this](const GoalHandleNav::WrappedResult & result){
            m_nav_result_ready = true;
            m_nav_result = result;
        };

        m_nav_action_client->async_send_goal(goal_msg, send_goal_options);
        m_nav_action_sent = true;

        return State::RUNNING;
    }

    if (m_nav_goal_handle_ready && !m_nav_goal_handle) {
        RCLCPP_ERROR(m_logger, "Nav goal was rejected by server");
        return State::FAILURE;
    }

    if (m_nav_result_ready) {
        if (m_nav_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(m_logger, "Nav succeeded!");
            return State::SUCCESS;
        } else {
            RCLCPP_ERROR(m_logger, "Nav failed!");
            return State::FAILURE;
        }
    }

    return State::RUNNING;
}

void NavBehavior::cleanup()
{
    // This behavior is being cancelled, so send a cancel request to dock action server if it's running
    if (!m_nav_result_ready && m_nav_goal_handle_ready && m_nav_goal_handle) {
        m_nav_action_client->async_cancel_goal(m_nav_goal_handle);
    }
}

} // namespace create3_walk
