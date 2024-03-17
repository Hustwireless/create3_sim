#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 nodes

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap

ARGUMENTS = [
    DeclareLaunchArgument('gazebo', default_value='classic',
                          choices=['classic', 'ignition'],
                          description='Which gazebo simulator to use'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace')
]


def generate_launch_description():

    # Directories
    pkg_create3_common_bringup = get_package_share_directory('irobot_create_common_bringup')
    pkg_create3_control = get_package_share_directory('irobot_create_control')
    namespace = LaunchConfiguration('namespace')
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Paths
    control_launch_file = PathJoinSubstitution(
        [pkg_create3_control, 'launch', 'include', 'control.py'])
    hazards_params_yaml_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'config', namespace, 'hazard_vector_params.yaml'])
    ir_intensity_params_yaml_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'config', namespace, 'ir_intensity_vector_params.yaml'])
    wheel_status_params_yaml_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'config', namespace, 'wheel_status_params.yaml'])
    mock_params_yaml_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'config', namespace, 'mock_params.yaml'])
    robot_state_yaml_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'config', namespace, 'robot_state_params.yaml'])
    kidnap_estimator_yaml_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'config', namespace, 'kidnap_estimator_params.yaml'])
    ui_mgr_params_yaml_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'config', namespace, 'ui_mgr_params.yaml'])

    # Includes
    diffdrive_controller = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([control_launch_file]),
                launch_arguments=[('namespace', LaunchConfiguration('namespace'))]
            ),
        ]
    )
    

    # Publish hazards vector
    hazards_vector_node = Node(
        package='irobot_create_nodes',
        name='hazards_vector_publisher',
        namespace=namespace,
        executable='hazards_vector_publisher',
        parameters=[hazards_params_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
        remappings=remappings,
    )

    # Publish IR intensity vector
    ir_intensity_vector_node = Node(
        package='irobot_create_nodes',
        name='ir_intensity_vector_publisher',
        namespace=namespace,
        executable='ir_intensity_vector_publisher',
        parameters=[ir_intensity_params_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
        remappings=remappings,
    )

    # Motion Control
    motion_control_node = Node(
        package='irobot_create_nodes',
        name='motion_control',
        namespace=namespace,
        executable='motion_control',
        parameters=[{'use_sim_time': True}],
        output='screen',
        remappings=remappings,
    )

    # Publish wheel status
    wheel_status_node = Node(
        package='irobot_create_nodes',
        name='wheel_status_publisher',
        namespace=namespace,
        executable='wheel_status_publisher',
        parameters=[wheel_status_params_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
        remappings=remappings,
    )

    # Publish mock topics
    mock_topics_node = Node(
        package='irobot_create_nodes',
        name='mock_publisher',
        namespace=namespace,
        executable='mock_publisher',
        parameters=[mock_params_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
        remappings=remappings,
    )

    # Publish robot state
    robot_state_node = Node(
        package='irobot_create_nodes',
        name='robot_state',
        namespace=namespace,
        executable='robot_state',
        parameters=[robot_state_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
        remappings=remappings,
    )

    # Publish kidnap estimator
    kidnap_estimator_node = Node(
        package='irobot_create_nodes',
        name='kidnap_estimator_publisher',
        namespace=namespace,
        executable='kidnap_estimator_publisher',
        parameters=[kidnap_estimator_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
        remappings=remappings,
    )

    # UI topics / actions
    ui_mgr_node = Node(
        package='irobot_create_nodes',
        name='ui_mgr',
        namespace=namespace,
        executable='ui_mgr',
        parameters=[ui_mgr_params_yaml_file,
                    {'use_sim_time': True},
                    {'gazebo': LaunchConfiguration('gazebo')}],
        output='screen',
        remappings=remappings,
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Include robot description
    ld.add_action(diffdrive_controller)
    # Add nodes to LaunchDescription
    ld.add_action(hazards_vector_node)
    ld.add_action(ir_intensity_vector_node)
    ld.add_action(motion_control_node)
    ld.add_action(wheel_status_node)
    ld.add_action(mock_topics_node)
    ld.add_action(robot_state_node)
    ld.add_action(kidnap_estimator_node)
    ld.add_action(ui_mgr_node)

    return ld
