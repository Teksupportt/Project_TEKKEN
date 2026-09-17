import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def build_sensor_topics(context):
    """Reads the same sensors_config YAML perception.launch.py uses, and
    derives the list of output topics fusion should subscribe to — keeps
    a single source of truth instead of maintaining two separate lists."""
    perception_share = get_package_share_directory("perception")
    sensors_config_path = LaunchConfiguration("sensors_config").perform(context)

    if not os.path.isabs(sensors_config_path):
        sensors_config_path = os.path.join(perception_share, "config", sensors_config_path)

    with open(sensors_config_path, "r") as f:
        sensors_cfg = yaml.safe_load(f)

    return [sensor["output_topic"] for sensor in sensors_cfg.get("sensors", [])]


def launch_setup(context, *args, **kwargs):
    sensor_topics = build_sensor_topics(context)

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("perception"),
                "launch", "perception.launch.py")
        ),
        launch_arguments={
            "robot_name": LaunchConfiguration("robot_name"),
            "sensors_config": LaunchConfiguration("sensors_config"),
        }.items(),
    )

    fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fusion"),
                "launch", "fusion.launch.py")
        ),
        launch_arguments={
            "robot_name": LaunchConfiguration("robot_name"),
            "sensor_topics": str(sensor_topics),  # passed through as a launch arg
        }.items(),
    )

    heading_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("heading_controller"),
                "launch", "heading_controller.launch.py")
        ),
        launch_arguments={
            "robot_name": LaunchConfiguration("robot_name"),
        }.items(),
    )

    return [perception_launch, fusion_launch, heading_controller_launch]


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="",
        description="Namespace prefix applied to every node, for multi-robot setups."
    )
    sensors_config_arg = DeclareLaunchArgument(
        "sensors_config",
        default_value="sensors_example.yaml",
        description="Sensor instance list YAML — single source of truth for both "
                     "perception's launched nodes and fusion's sensor_topics."
    )

    return LaunchDescription([
        robot_name_arg,
        sensors_config_arg,
        OpaqueFunction(function=launch_setup),
    ])