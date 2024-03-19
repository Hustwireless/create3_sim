#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 in Gazebo and optionally also in RViz.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_gazebo_gui', default_value='true',
                          choices=['true', 'false'],
                          description='Set "false" to run gazebo headless.'),
    DeclareLaunchArgument('spawn_dock', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn the standard dock model.'),
    DeclareLaunchArgument('world_path', default_value='',
                          description='Set world path, by default is empty.world'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('visualize_rays', default_value='false',
                          choices=['true', 'false'],
                          description='Enable/disable ray visualization')
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    # Directories
    create3_multi_bringup = get_package_share_directory('irobot_create_multi')

    # Paths
    gazebo_launch = PathJoinSubstitution(
        [create3_multi_bringup, 'launch', 'gazebo.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [create3_multi_bringup, 'launch', 'create3_spawn.launch.py'])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('world_path', LaunchConfiguration('world_path')),
            ('use_gazebo_gui', LaunchConfiguration('use_gazebo_gui'))
        ]
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('use_rviz', LaunchConfiguration('use_rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
            ('visualize_rays', LaunchConfiguration('visualize_rays'))])

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Gazebo
    ld.add_action(gazebo)
    # Robot spawn
    ld.add_action(robot_spawn)
    return ld
