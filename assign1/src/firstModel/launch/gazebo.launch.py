#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = "firstModel"
    pkg_share = get_package_share_directory(pkg_name)

    default_model_path = os.path.join(pkg_share, "urdf", "myfirst.urdf")
    default_world_path = os.path.join(pkg_share, "worlds", "empty.world")

    declare_model = DeclareLaunchArgument(
        "model",
        default_value=default_model_path,
        description="Absolute path to robot URDF/Xacro file",
    )

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=default_world_path,
        description="Absolute path to world file",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_use_gui = DeclareLaunchArgument(
        "use_gui",
        default_value="false",
        description="Start joint_state_publisher_gui (usually false for Gazebo)",
    )

    # Convert xacro -> URDF for robot_state_publisher and spawn_entity
    robot_description = Command(["xacro ", LaunchConfiguration("model")])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gui")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # Gazebo Classic launch file from gazebo_ros
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    # Spawn robot into Gazebo from robot_description topic
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "myfirst_robot",
        ],
    )

    return LaunchDescription(
        [
            declare_model,
            declare_world,
            declare_use_sim_time,
            declare_use_gui,
            gazebo,
            robot_state_publisher,
            joint_state_publisher_gui,
            spawn_entity,
        ]
    )