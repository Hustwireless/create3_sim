// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include "irobot_create_multi/behaviors/estop-behavior.hpp"
#include "utils.hpp"

namespace create3_walk {

EstopBehavior::EstopBehavior(
    Config config,
    rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock)
: m_cmd_vel_publisher(cmd_vel_publisher), m_logger(logger)
{
    m_clock = clock;
    m_config = config;

    m_first_run = false;
    m_start_time = m_clock->now();

    RCLCPP_INFO(m_logger, "EstopBehavior created");
}

State EstopBehavior::execute(const Data & data)
{
    if (!m_first_run) {
        m_first_run = true;
        m_initial_position = data.pose.position;
    }

    bool timeout = m_clock->now() - m_start_time > m_config.clear_hazard_time;
    bool moved_enough = get_distance(data.pose.position, m_initial_position) > m_config.backup_distance;

    if (timeout || moved_enough) {
        if (is_front_hazard_active(data.hazards)) {
            RCLCPP_INFO(m_logger, "Estop failed: was not able to clear hazard (timeout %d distance %d)",
                timeout, moved_enough);
            return State::FAILURE;
        } else {
            RCLCPP_INFO(m_logger, "Estop successfully prevented hazard");
            return State::SUCCESS;
        }
    }

    // Command a negative velocity to backup from hazard
    auto twist_msg = std::make_unique<TwistMsg>();
    twist_msg->linear.x = - m_config.linear_vel;
    m_cmd_vel_publisher->publish(std::move(twist_msg));

    return State::RUNNING;
}

} // namespace create3_walk
