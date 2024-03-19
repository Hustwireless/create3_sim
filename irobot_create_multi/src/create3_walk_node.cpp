#include "irobot_create_multi/create3_walk_node.hpp"
#include "irobot_create_multi/walk_state_machine.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace create3_walk {

Create3WalkNode::Create3WalkNode() : rclcpp::Node("create3_walk") {

    m_walk_action_server = rclcpp_action::create_server<WalkAction>(
        this,
        "walk",
        std::bind(&Create3WalkNode::handle_goal, this, _1, _2),
        std::bind(&Create3WalkNode::handle_cancel, this, _1),
        std::bind(&Create3WalkNode::handle_accepted, this, _1));

    m_dock_action_client = rclcpp_action::create_client<DockAction>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "dock");
    
    m_undock_action_client = rclcpp_action::create_client<UndockAction>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "undock");

    m_nav_action_client = rclcpp_action::create_client<NavAction>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "navigate_to_position");

    m_cmd_vel_publisher = this->create_publisher<TwistMsg>("cmd_vel", 10);

    m_reflexes_param_client = std::make_shared<rclcpp::AsyncParametersClient>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_graph_interface(),
        this->get_node_services_interface(),
        "motion_control",
        rmw_qos_profile_parameters);

    m_ir_intensity_subscription = this->create_subscription<IrIntensityMsg>(
        "ir_intensity", rclcpp::SensorDataQoS(),
        std::bind(&Create3WalkNode::ir_intensity_callback, this, _1));

    m_dock_subscription = this->create_subscription<DockMsg>(
        "dock_status", rclcpp::SensorDataQoS(),
        std::bind(&Create3WalkNode::dock_callback, this, _1));

    m_hazards_subscription = this->create_subscription<HazardMsg>(
        "hazard_detection", rclcpp::SensorDataQoS(),
        std::bind(&Create3WalkNode::hazards_callback, this, _1));

    m_ir_opcode_subscription = this->create_subscription<OpCodeMsg>(
        "ir_opcode", rclcpp::SensorDataQoS(),
        std::bind(&Create3WalkNode::ir_opcode_callback, this, _1));

    m_odom_subscription = this->create_subscription<OdometryMsg>(
        "odom", rclcpp::SensorDataQoS(),
        std::bind(&Create3WalkNode::odom_callback, this, _1));

    m_kidnap_subscription = this->create_subscription<KidnapMsg>(
        "kidnap_status", rclcpp::SensorDataQoS(),
        std::bind(&Create3WalkNode::kidnap_callback, this, _1));


    m_rate_hz = this->declare_parameter<double>("rate_hz", 30.0);
    m_opcodes_buffer_ms = this->declare_parameter<int>("opcodes_buffer_ms", 200);

    m_dock_msgs_received = false;
    m_is_running = false;
    m_last_behavior = -1;
    m_last_opcodes_cleared_time = this->now();
    RCLCPP_INFO(this->get_logger(), "Create3WalkNode initialized");
}

rclcpp_action::GoalResponse Create3WalkNode::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const WalkAction::Goal> goal) {
    (void)uuid; (void)goal;  // suppress unused parameter warning

    RCLCPP_INFO(this->get_logger(), "Received goal request");

    if (!this->ready_to_start()) {
        RCLCPP_WARN(this->get_logger(), "Not ready to start the behavior");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Robot is ready to start the behavior");

    bool is_kidnapped = false;
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        is_kidnapped = m_last_kidnap.is_kidnapped;
    }
    if (is_kidnapped) {
        RCLCPP_WARN(this->get_logger(), "Robot is kidnapped, rejecting goal");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (m_is_running.exchange(true)) {
        RCLCPP_WARN(this->get_logger(), "Behavior is already running, rejecting goal");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Accepted goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Create3WalkNode::handle_cancel(
    const std::shared_ptr<GoalHandleWalk> goal_handle) {
    (void) goal_handle;  // suppress unused parameter warning

    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Create3WalkNode::handle_accepted(const std::shared_ptr<GoalHandleWalk> goal_handle) {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&Create3WalkNode::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void Create3WalkNode::execute(const std::shared_ptr<GoalHandleWalk> goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Rate loop_rate(m_rate_hz);
    const auto goal = goal_handle->get_goal();
    auto start_time = this->now();

    bool robot_has_reflexes = this->reflexes_setup();

    auto state_machine = std::make_unique<WalkStateMachine>(
        *goal,
        this->get_clock(),
        this->get_logger(),
        m_dock_action_client,
        m_undock_action_client,
        m_nav_action_client,
        m_cmd_vel_publisher,
        robot_has_reflexes);

    WalkStateMachine::WalkOutput output;
    output.state = State::RUNNING;
    bool is_docked = false;
    bool is_kidnapped = false;
    do {

        Behavior::Data data;
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            
            data.hazards = m_last_hazards;
            data.dock = m_last_dock;
            data.opcodes = m_last_opcodes;
            data.pose = m_last_odom.pose.pose;
            data.ir_intensity = m_last_ir_intensity;
            
            if (this->now() - m_last_opcodes_cleared_time 
                >= rclcpp::Duration(std::chrono::milliseconds(m_opcodes_buffer_ms))){
                    m_last_opcodes_cleared_time = this->now();
                    m_last_opcodes.clear();
            }

            is_kidnapped = m_last_kidnap.is_kidnapped;
            is_docked = m_last_dock.is_docked;
        }

        if (goal_handle->is_canceling()) {
            m_is_running = false;
            state_machine->cancel();
            auto result = std::make_shared<WalkAction::Result>();
            result->success = false;
            result->is_docked = is_docked;
            result->duration = this->now() - start_time;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal was canceled");
            return;
        }

        if (is_kidnapped) {
            m_is_running = false;
            state_machine->cancel();
            auto result = std::make_shared<WalkAction::Result>();
            result->success = false;
            result->is_docked = is_docked;
            result->duration = this->now() - start_time;
            goal_handle->abort(result);
            RCLCPP_INFO(this->get_logger(), "Robot was kidnapped");
            return;
        }

        output = state_machine->execute(data);
        if (m_last_behavior != output.current_behavior) {
            auto feedback = std::make_shared<WalkAction::Feedback>();
            feedback->current_behavior = output.current_behavior;
            goal_handle->publish_feedback(feedback);
            m_last_behavior = output.current_behavior;
        }

        loop_rate.sleep();
    } while (output.state == State::RUNNING && rclcpp::ok());

    RCLCPP_INFO(this->get_logger(), "Goal finished! Walk action terminated");

    if (rclcpp::ok()) {
        m_is_running = false;
        auto result = std::make_shared<WalkAction::Result>();
        result->success = (output.state == State::SUCCESS);
        result->is_docked = is_docked;
        result->duration = this->now() - start_time;
        if (result->success) {
            goal_handle->succeed(result);
        } else {
            goal_handle->abort(result);
        }
    }

}

bool Create3WalkNode::reflexes_setup()
{
    bool robot_has_reflexes = true;

    const std::vector<std::string> param_names = {
        "reflexes.REFLEX_BUMP",
        "reflexes.REFLEX_CLIFF",
        "reflexes.REFLEX_WHEEL_DROP",
        "reflexes_enabled"
    };

    // Check if reflexes are active
    auto get_params_future = m_reflexes_param_client->get_parameters(param_names);
    auto parameters = get_params_future.get();
    bool all_enabled = true;
    bool all_disabled = true;
    for (const rclcpp::Parameter& param : parameters) {
        all_enabled = all_enabled && param.as_bool();
        all_disabled = all_disabled && !param.as_bool();
    }

    if (all_enabled) {
        robot_has_reflexes = true;
        RCLCPP_INFO(this->get_logger(), "Reflexes are enabled on the robot!");
    } else if (all_disabled) {
        robot_has_reflexes = false;
        RCLCPP_INFO(this->get_logger(), "Reflexes are disabled on the robot!");
    } else {
        // If some reflexes are enabled and some are disabled, activate them all.
        RCLCPP_WARN(this->get_logger(), "Some reflexes were disabled, activating all of them");
        std::vector<rclcpp::Parameter> parameters;
        for (const std::string & name : param_names) {
            parameters.push_back(rclcpp::Parameter(name, true));
        }
        auto set_params_future = m_reflexes_param_client->set_parameters(parameters);
        auto results = set_params_future.get();
        bool success = true;
        for (const rcl_interfaces::msg::SetParametersResult& res : results) {
            success = success && res.successful;
        }

        if (!success) {
            throw std::runtime_error{"Unable to activate required parameters"};
        }
        robot_has_reflexes = true;
    }

    return robot_has_reflexes;
}

bool Create3WalkNode::ready_to_start() {
    RCLCPP_INFO(this->get_logger(), "Checking if the robot is ready to start the behavior");
    if (m_dock_subscription->get_publisher_count() == 0 ||
        m_hazards_subscription->get_publisher_count() == 0 ||
        m_ir_opcode_subscription->get_publisher_count() == 0 ||
        m_odom_subscription->get_publisher_count() == 0 ||
        m_kidnap_subscription->get_publisher_count() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "Some subscriptions haven't discovered their publishers yet");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Check publishers passed");

    if (m_cmd_vel_publisher->get_subscription_count() == 0) {
        RCLCPP_WARN(this->get_logger(), "Some publishers haven't discovered their subscriptions yet");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Check subscriptions passed");

    if (!m_reflexes_param_client->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Some parameters servers are not ready yet");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Check parameters servers passed");

    if (!m_dock_action_client->action_server_is_ready() ||
        !m_undock_action_client->action_server_is_ready() ||
        !m_nav_action_client->action_server_is_ready())
    {
        RCLCPP_WARN(this->get_logger(), "Some actions servers are not ready yet");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Check action servers passed");

    // We must know if the robot is docked or not before starting the behavior
    if (!m_dock_msgs_received) {
        RCLCPP_WARN(this->get_logger(), "Didn't receive a dock message yet");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Check dock messages passed");

    return true;
}

void Create3WalkNode::dock_callback(DockMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_dock_msgs_received = true;
    m_last_dock = *msg;
}

void Create3WalkNode::hazards_callback(HazardMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_last_hazards = *msg;
}

void Create3WalkNode::ir_opcode_callback(OpCodeMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_last_opcodes.push_back(*msg);
}

void Create3WalkNode::kidnap_callback(KidnapMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_last_kidnap = *msg;
}

void Create3WalkNode::odom_callback(OdometryMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_last_odom = *msg;
}

void Create3WalkNode::ir_intensity_callback(IrIntensityMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_last_ir_intensity = *msg;
}


}  // namespace create3_walk