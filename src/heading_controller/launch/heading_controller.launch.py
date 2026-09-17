import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="",
        description="Namespace prefix for multi-robot setups."
    )

    params_file = os.path.join(
        get_package_share_directory("heading_controller"),
        "config",
        "controller_params.yaml"
    )

    heading_controller_node = Node(
        package="heading_controller",
        executable="heading_controller_node",
        name="heading_controller_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription([
        robot_name_arg,
        heading_controller_node,
    ])