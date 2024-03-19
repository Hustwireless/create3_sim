#pragma once

#include "rclcpp/rclcpp.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "irobot_create_msgs/msg/dock_status.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <chrono>
#include <mutex>

using namespace std::chrono_literals;

class SimpleUndock : public rclcpp::Node {

public:
    using UndockAction = irobot_create_msgs::action::Undock;
    using ClientGoalHandleUndock = rclcpp_action::ClientGoalHandle<UndockAction>;

    using DockMsg = irobot_create_msgs::msg::DockStatus;

    SimpleUndock();
    ~SimpleUndock() = default;

    void execute();
    void dock_callback(DockMsg::ConstSharedPtr msg);
    bool m_undock_result_ready {false};

private:
    bool m_undock_action_sent {false};
    ClientGoalHandleUndock::SharedPtr m_undock_goal_handle;
    bool m_undock_goal_handle_ready {false};
    ClientGoalHandleUndock::WrappedResult m_undock_result;

    rclcpp_action::Client<UndockAction>::SharedPtr m_undock_action_client;
    
    std::mutex m_mutex;

    DockMsg m_last_dock;
    rclcpp::Subscription<DockMsg>::SharedPtr m_dock_subscription;

    rclcpp::Logger m_logger;
};
