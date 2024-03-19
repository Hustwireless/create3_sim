#include "irobot_create_multi/simple_robot_move.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleRobotMove>());
  rclcpp::shutdown();
  return 0;
}