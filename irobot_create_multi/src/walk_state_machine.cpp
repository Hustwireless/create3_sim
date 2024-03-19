#include <chrono>
#include <math.h>

#include "irobot_create_multi/walk_state_machine.hpp"
#include "tf2/utils.h"

namespace create3_walk {

WalkStateMachine::WalkStateMachine(
    create3_walk_msgs::action::Walk::Goal goal,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Logger logger,
    rclcpp_action::Client<DockAction>::SharedPtr dock_action_client,
    rclcpp_action::Client<UndockAction>::SharedPtr undock_action_client,
    rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
    bool has_reflexes)
    :  m_logger(logger) 
{
    m_goal = goal;

    m_clock = clock;
    m_start_time = m_clock->now();
    m_has_reflexes = has_reflexes;

    m_dock_action_client = dock_action_client;
    m_undock_action_client = undock_action_client;
    m_cmd_vel_publisher = cmd_vel_publisher;

    m_undocking = false;
    m_preparing_spiral = false;
}     

WalkStateMachine::~WalkStateMachine() {
    this->cancel();
}

void WalkStateMachine::cancel() {
    if (m_current_behavior) {
        m_current_behavior->cleanup();
        m_current_behavior.reset();
    }
}

WalkStateMachine::WalkOutput WalkStateMachine::execute(const Behavior::Data& data) {
    if (!m_current_behavior) {
        this->select_start_behavior(data);
    } else {
        this->select_next_behavior(data);
    }

    // meaning behavior state failure or success
    if (m_walk_output.state != State::RUNNING) {
        return m_walk_output;
    }

    m_behavior_state = m_current_behavior->execute(data);
    m_walk_output.current_behavior = m_current_behavior->get_id();

    return m_walk_output;
}


void WalkStateMachine::select_start_behavior(const Behavior::Data& data) {
    if (data.dock.is_docked) {
           this->goto_undock();
    } else {
        this->goto_drive_straight();
    }
}

void WalkStateMachine::select_next_behavior(const Behavior::Data& data) {
    if (m_behavior_state == State::RUNNING) {
        m_walk_output.state = State::RUNNING;
        return;
    }

    bool explore_duration_elapsed = m_clock->now() - m_start_time > m_goal.explore_duration;
    bool max_runtime_elapsed = m_clock->now() - m_start_time > m_goal.max_runtime;
    if (max_runtime_elapsed) {
        m_walk_output.state = State::SUCCESS;
        return;
    }
    if (m_current_behavior->get_id() != FeedbackMsg::DOCK && 
        explore_duration_elapsed && data.dock.dock_visible) {
            this->goto_dock();
        return;
    }

    switch (m_current_behavior->get_id()) {
        case FeedbackMsg::DOCK: {
            if (m_behavior_state == State::FAILURE || !data.dock.is_docked) {
                m_walk_output.state = State::FAILURE;
                break;
            }
            m_walk_output.state = State::SUCCESS;
            return;
        }
        case FeedbackMsg::DRIVE_STRAIGHT: {
            auto rotate_config = RotateBehavior::Config();
            if (m_behavior_state == State::FAILURE) {
                if (m_evade_attempts.size() > 20) {
                    m_walk_output.state = State::FAILURE;
                    break;
                }

                constexpr double evade_resolution = 0.175433;
                rotate_config.target_rotation = compute_evade_rotation(data.pose, evade_resolution);
            } else {
                m_evade_attempts.clear();
            }
            rotate_config.robot_has_reflexes = m_has_reflexes;
            this->goto_rotate(rotate_config);
            break;
        }
        case FeedbackMsg::ROTATE: {
            if (m_behavior_state == State::FAILURE) {
                m_walk_output.state = State::FAILURE;
                break;
            }
            auto drive_config = DriveStraightBehavior::Config();
            this->goto_drive_straight(drive_config);
            break;
        }
        case FeedbackMsg::UNDOCK: {
            if (m_behavior_state == State::FAILURE || data.dock.is_docked) {
                m_walk_output.state = State::FAILURE;
                break;
            }

            auto drive_config = DriveStraightBehavior::Config();
            drive_config.max_distance = 0.25;
            drive_config.min_distance = 0.25;
            this->goto_drive_straight(drive_config);
            break;
        }
    }
}

double WalkStateMachine::compute_evade_rotation(const geometry_msgs::msg::Pose& pose, double resolution)
{
    tf2::Quaternion current_orientation;
    tf2::convert(pose.orientation, current_orientation);

    // Add current orientation to the list of failed attempts
    double current_yaw = tf2::getYaw(current_orientation);
    m_evade_attempts.push_back(current_yaw);

    tf2::Quaternion target_orientation;
    size_t i = 0;
    // We don't want this loop to search forever.
    // Eventually, if we failed too many times, return an orientation regardless of how different it is
    // from previous attempts.
    while (i < 100) {
        // Generate a new, random, target orientation
        double random_num = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        double random_angle = random_num * 2 * M_PI - M_PI;
        target_orientation.setRPY(0.0, 0.0, random_angle);

        // Check if the random orientation is different enough from past evade attempts
        bool valid_target = true;
        for (double angle : m_evade_attempts) {
            tf2::Quaternion attempt_orientation;
            attempt_orientation.setRPY(0.0, 0.0, angle);

            tf2::Quaternion relative_orientation = target_orientation * attempt_orientation.inverse();
            double relative_yaw = tf2::getYaw(relative_orientation);
            if (std::abs(relative_yaw) < std::abs(resolution)) {
                valid_target = false;
                break;
            }
        }

        // Exit as soon as we find a valid target orientation
        if (valid_target) {
            break;
        }
        i++;
    }

    tf2::Quaternion relative_orientation = target_orientation * current_orientation.inverse();
    double relative_yaw_rotation = tf2::getYaw(relative_orientation);
    return relative_yaw_rotation;
}

void WalkStateMachine::goto_dock()
{
    m_current_behavior = std::make_unique<DockBehavior>(m_dock_action_client, m_logger);
    m_walk_output.state = State::RUNNING;
}

void WalkStateMachine::goto_drive_straight(const DriveStraightBehavior::Config& config)
{
    m_current_behavior = std::make_shared<DriveStraightBehavior>(config, m_cmd_vel_publisher, m_logger, m_clock);
    m_walk_output.state = State::RUNNING;
}

void WalkStateMachine::goto_rotate(const RotateBehavior::Config& config)
{
    m_current_behavior = std::make_shared<RotateBehavior>(config, m_cmd_vel_publisher, m_logger, m_clock);
    m_walk_output.state = State::RUNNING;
}

void WalkStateMachine::goto_undock()
{
    m_undocking = true;
    m_current_behavior = std::make_unique<UndockBehavior>(m_undock_action_client, m_logger);
    m_walk_output.state = State::RUNNING;
}


} // namespace create3_walk