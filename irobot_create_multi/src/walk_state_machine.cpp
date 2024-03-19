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
    rclcpp_action::Client<NavAction>::SharedPtr nav_action_client,
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
    m_nav_action_client = nav_action_client;
    m_cmd_vel_publisher = cmd_vel_publisher;

    m_undocking = false;
    m_preparing_spiral = false;
    m_detected_obstacle = false;
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
        this->goto_nav();
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
        case FeedbackMsg::ROTATE: {
            if (m_behavior_state == State::FAILURE) {
                m_walk_output.state = State::FAILURE;
                break;
            }
            this->goto_nav();
            break;
        }
        case FeedbackMsg::ESTOP: {
            auto rotate_config = RotateBehavior::Config();
            rotate_config.robot_has_reflexes = m_has_reflexes;
            if (m_behavior_state == State::FAILURE) {
                m_walk_output.state = State::FAILURE;
                break;
            }
            if (!m_detected_obstacle) {
                this->goto_rotate(rotate_config);
                break;
            }
            m_detected_obstacle = false;
            if (detected_obstacle(data.ir_intensity)) {
                double rot_angle = decide_rotation_angle(data.ir_intensity);
                rotate_config.target_rotation = rot_angle;
                this->goto_rotate(rotate_config);
            } else {
                this->goto_nav();
            }
            break;
        }
        case FeedbackMsg::NAV: {
            auto rotate_config = RotateBehavior::Config();
            if (detected_obstacle(data.ir_intensity)) {
                RCLCPP_INFO(m_logger, "Detected obstacle, estop && backward to avoid it.");
                m_detected_obstacle = true;
                auto estop_config = EstopBehavior::Config();
                estop_config.backup_distance = 0.1;
                estop_config.linear_vel = 0.1;
                this->goto_estop(estop_config);
                break;
            }
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
            rotate_config.target_rotation = M_PI / 4.0;
            this->goto_rotate(rotate_config);
            break;
        }
        case FeedbackMsg::UNDOCK: {
            if (m_behavior_state == State::FAILURE || data.dock.is_docked) {
                m_walk_output.state = State::FAILURE;
                break;
            }
            auto drive_config = DriveStraightBehavior::Config();
            this->goto_nav();
            break;
        }
    }
}

bool WalkStateMachine::frameContains(const std::string& frame_id, const std::string& keyword) {
    return frame_id.find(keyword) != std::string::npos;
}

double WalkStateMachine::decide_rotation_angle(const irobot_create_msgs::msg::IrIntensityVector& ir_intensity) {
    RCLCPP_INFO(m_logger, "Deciding rotation angle");
    for (int i=0; i<7; i++) {
        int intensity = ir_intensity.readings[i].value;
        std::string frame_id = ir_intensity.readings[i].header.frame_id;
        if (intensity> 30) {
            if (frameContains(frame_id, "front")) {
                return M_PI / 2.0;
            } 
            if (frameContains(frame_id, "left")) {
                return M_PI / 4.0;
            } else if (frameContains(frame_id, "right")) {
                return -M_PI / 4.0;
            }
        }
    }
    return 0.0;
}

bool WalkStateMachine::detected_obstacle(const irobot_create_msgs::msg::IrIntensityVector& ir_intensity) {
    RCLCPP_INFO(m_logger, "Detecting obstacle");
    for (int i=0; i<7; i++) {
        int intensity = ir_intensity.readings[i].value;
        std::string frame_id = ir_intensity.readings[i].header.frame_id;
        RCLCPP_INFO(m_logger, "IR intensity %s: %d", frame_id.c_str(), intensity);
        if (intensity > 20) {
            return true;
        }
    }
    return false;
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

void WalkStateMachine::goto_nav()
{
    m_current_behavior = std::make_unique<NavBehavior>(m_nav_action_client, m_logger);
    m_walk_output.state = State::RUNNING;
}

void WalkStateMachine::goto_estop(const EstopBehavior::Config& config)
{
    m_current_behavior = std::make_unique<EstopBehavior>(config, m_cmd_vel_publisher, m_logger, m_clock);
    m_walk_output.state = State::RUNNING;
}


} // namespace create3_walk