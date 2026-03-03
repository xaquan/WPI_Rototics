#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- Your package + files ----
    # Example: firstModel/urdf/myfirst.urdf.xacro
    urdf_package_arg = DeclareLaunchArgument(
        "urdf_package",
        default_value="firstModel",
        description="Package that contains the URDF/Xacro",
    )

    urdf_package_path_arg = DeclareLaunchArgument(
        "urdf_package_path",
        default_value="urdf/myfirst.urdf.xacro",
        description="Path to the URDF/Xacro inside the package",
    )

    world_package_arg = DeclareLaunchArgument(
        "world_package",
        default_value="firstModel",
        description="Package that contains the world file",
    )

    world_package_path_arg = DeclareLaunchArgument(
        "world_package_path",
        default_value="worlds/empty.world",
        description="Path to the world file inside the package",
    )

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start Gazebo GUI",
    )

    # ---- Build absolute paths using FindPackageShare ----
    model_path = PathJoinSubstitution([
        FindPackageShare(LaunchConfiguration("urdf_package")),
        LaunchConfiguration("urdf_package_path"),
    ])

    world_path = PathJoinSubstitution([
        FindPackageShare(LaunchConfiguration("world_package")),
        LaunchConfiguration("world_package_path"),
    ])

    # ---- robot_description from xacro ----
    robot_description = Command(["xacro ", model_path])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )

    # ---- Gazebo ----
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "world": world_path,     # comment this line if you want Gazebo default empty world
            "pause": "false",
        }.items(),
    )

    # ---- Spawn robot in Gazebo from robot_description topic ----
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "myfirst_robot",
            "-topic", "robot_description",
            "-x", "0", "-y", "0", "-z", "0.5",
        ],
    )

    return LaunchDescription([
        gui_arg,
        urdf_package_arg,
        urdf_package_path_arg,
        world_package_arg,
        world_package_path_arg,
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
    ])