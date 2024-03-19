// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include <memory>

#include "irobot_create_multi/create3_walk_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<create3_walk::Create3WalkNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
