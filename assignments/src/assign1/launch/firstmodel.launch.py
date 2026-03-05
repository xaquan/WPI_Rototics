#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    pkg_name = "assign1"
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([FindPackageShare(pkg_name), 'urdf', 'myfirst.urdf.xacro']),
        description='Absolute path to URDF/Xacro model file',
    )

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Start joint_state_publisher_gui to provide /joint_states',
    )

    # rvizconfig_arg = DeclareLaunchArgument(
    #     name='rvizconfig',
    #     default_value=PathJoinSubstitution([FindPackageShare(pkg_name), 'rviz', 'display.rviz']),
    # )

    gazebo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare(pkg_name), 'launch', 'gazebo.launch.py']),
        launch_arguments={
            'model': LaunchConfiguration('model')
        }.items(),
    )

    # Publish joint states so robot_state_publisher can publish TF frames for RViz.
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     output='both',
    #     condition=IfCondition(LaunchConfiguration('use_gui')),
    # )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     output='screen',
    #     arguments=['-d', LaunchConfiguration('rvizconfig')],
    # )

    # Spawn controllers so joint1_controller subscribes to command topic.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='both',
    )

    joint1_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint1_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='both',
    )

    load_joint1_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint1_controller_spawner],
        )
    )

    return LaunchDescription([
        model_arg,
        use_gui_arg,
        # rvizconfig_arg,
        gazebo_launch,
        joint_state_publisher_gui_node,
        # rviz_node,
        joint_state_broadcaster_spawner,
        load_joint1_after_jsb,
    ])