#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

using namespace std::chrono_literals;

class SimpleRobotMove : public rclcpp::Node {
public:
  SimpleRobotMove()
  : Node("simple_robot_move")
  {
    // Create a publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Create a timer
    this->wait_time_ = 1.0;
    timer_ = this->create_wall_timer(
      500ms, std::bind(&SimpleRobotMove::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.5;
    message.angular.z = 0.5;

    RCLCPP_INFO(this->get_logger(), "Publishing: linear.x: %f, angular.z: %f",
      message.linear.x, message.angular.z);
    publisher_->publish(message);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  float wait_time_;

};