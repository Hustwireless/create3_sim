cmake_minimum_required(VERSION 3.8)
project(irobot_create_multi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(create3_walk_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


set(executable_name create3_walk)
set(library_name ${executable_name}_core)

set(dependencies
    geometry_msgs
    irobot_create_msgs
    nav_msgs
    rclcpp
    rclcpp_action
    tf2_geometry_msgs
    create3_walk_msgs
)

add_library(${library_name} SHARED
    src/create3_walk_node.cpp
    src/behaviors/dock-behavior.cpp
    src/behaviors/drive-straight-behavior.cpp
    src/behaviors/rotate-behavior.cpp
    src/behaviors/reflex-behavior.cpp
    src/behaviors/undock-behavior.cpp
    src/behaviors/utils.cpp
    src/walk_state_machine.cpp
    src/behaviors/nav-behavior.cpp
    src/behaviors/estop-behavior.cpp
)
target_include_directories(${library_name} PUBLIC include)
ament_target_dependencies(${library_name} ${dependencies})

include_directories(include)

# add_executable(simple_robot_move src/simple_robot_move.cpp src/simple_robot_move_node.cpp)
# ament_target_dependencies(simple_robot_move rclcpp geometry_msgs)

# add_executable(simple_undock_node src/undock_behavior.cpp)
# ament_target_dependencies(simple_undock_node ${dependencies})

add_executable(${executable_name} src/main.cpp)
ament_target_dependencies(${executable_name} ${dependencies})
target_link_libraries(${executable_name} ${library_name})

install(TARGETS ${library_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install(TARGETS
#   simple_robot_move
#   simple_undock_node
#   DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY include/
    DESTINATION include/
)

install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
