#include "rclcpp/rclcpp.hpp"
#include "irobot_create_multi/undock_behavior.hpp"

SimpleUndock::SimpleUndock(): Node("simple_undock"), m_logger(get_logger()) {
    m_undock_action_client = rclcpp_action::create_client<UndockAction>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "undock");

    m_dock_subscription = this->create_subscription<DockMsg>(
        "dock_status",
        rclcpp::SensorDataQoS(),
        std::bind(&SimpleUndock::dock_callback, this, std::placeholders::_1));
}

void SimpleUndock::execute() {
    bool is_docked = false;
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        is_docked = m_last_dock.is_docked;
    }

    if (!m_undock_action_sent && !is_docked) {
        RCLCPP_ERROR(m_logger, "Robot is already undocked!");
        return;
    } 

    if (!m_undock_action_client->action_server_is_ready()) {
        RCLCPP_ERROR(m_logger, "Waiting for undock action server...");
        return;
    }

    // Send undock command if not already sent and if we are not waiting for result
    if (!m_undock_action_sent) {
        RCLCPP_INFO(m_logger, "Sending undocking goal!");
        auto goal_msg = UndockAction::Goal();

        auto send_goal_options = rclcpp_action::Client<UndockAction>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](const ClientGoalHandleUndock::SharedPtr & goal_handle){
            m_undock_goal_handle_ready = true;
            m_undock_goal_handle = goal_handle;
        };
        send_goal_options.result_callback = [this](const ClientGoalHandleUndock::WrappedResult & result){
            m_undock_result_ready = true;
            m_undock_result = result;
        };

        m_undock_action_client->async_send_goal(goal_msg, send_goal_options);
        m_undock_action_sent = true;

        return;
    }

    if (m_undock_goal_handle_ready && !m_undock_goal_handle) {
        RCLCPP_ERROR(m_logger, "Undock goal was rejected by server");
        return;
    }

    if (m_undock_result_ready) {
        if (m_undock_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(m_logger, "Undocking succeeded!");
            return;
        } else {
            RCLCPP_ERROR(m_logger, "Undocking failed!");
            return;
        }
    }
}

void SimpleUndock::dock_callback(DockMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_last_dock = *msg;
    RCLCPP_INFO(m_logger, "Dock status received: %s", m_last_dock.is_docked ? "docked" : "undocked");
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<SimpleUndock>();
    rclcpp::Rate loop_rate(10);

    while (!action_client->m_undock_result_ready && rclcpp::ok()) {
        action_client->execute();
        rclcpp::spin_some(action_client);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}