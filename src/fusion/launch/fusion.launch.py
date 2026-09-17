import os
import ast

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
    sensor_topics_arg = DeclareLaunchArgument(
        "sensor_topics",
        default_value="[]",
        description="Python-list-literal string of sensor topics to fuse. "
                     "If empty, falls back to fusion_params.yaml's default list."
    )

    params_file = os.path.join(
        get_package_share_directory("fusion"),
        "config",
        "fusion_params.yaml"
    )

    def make_node(context):
        raw = LaunchConfiguration("sensor_topics").perform(context)
        topics = ast.literal_eval(raw) if raw not in ("", "[]") else None

        parameters = [params_file]
        if topics:
            parameters.append({"sensor_topics": topics})

        return [Node(
            package="fusion",
            executable="fusion_node",
            name="fusion_node",
            namespace=LaunchConfiguration("robot_name"),
            parameters=parameters,
            output="screen",
        )]

    from launch.actions import OpaqueFunction
    return LaunchDescription([
        robot_name_arg,
        sensor_topics_arg,
        OpaqueFunction(function=make_node),
    ])