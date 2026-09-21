import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="robot1")
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0")

    xacro_path = os.path.join(
        get_package_share_directory("combat_sim"), "urdf", "simple_bot.urdf.xacro")

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_path,
            " robot_name:=", LaunchConfiguration("robot_name"),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", [LaunchConfiguration("robot_name"), "/robot_description"],
            "-entity", LaunchConfiguration("robot_name"),
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-Y", LaunchConfiguration("yaw"),
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_name_arg, x_arg, y_arg, yaw_arg,
        robot_state_publisher,
        spawn_entity,
    ])
