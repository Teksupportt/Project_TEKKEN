# Edit robot and sensor yaml paths at the end of this file

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


TYPE_TO_EXECUTABLE = {
    "lidar_2d": "lidar_2d_tracker",
    "lidar_3d": "lidar_3d_tracker",
    "camera_cv": "camera_cv_tracker",
    "camera_ml": "camera_ml_tracker",
}

TYPE_TO_DEFAULT_PARAMS_FILE = {
    "lidar_2d": "lidar_2d_tracker.yaml",
    "lidar_3d": "lidar_3d_tracker.yaml",
    "camera_cv": "camera_cv_tracker.yaml",
    "camera_ml": "camera_ml_tracker.yaml",
}


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("perception")
    sensors_config_path = LaunchConfiguration("sensors_config").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)

    if not os.path.isabs(sensors_config_path):
        sensors_config_path = os.path.join(pkg_share, "config", sensors_config_path)

    with open(sensors_config_path, "r") as f:
        sensors_cfg = yaml.safe_load(f)

    nodes = []
    for i, sensor in enumerate(sensors_cfg.get("sensors", [])):
        sensor_type = sensor["type"]
        if sensor_type not in TYPE_TO_EXECUTABLE:
            raise ValueError(f"Unknown sensor type '{sensor_type}' in sensors_config")

        executable = TYPE_TO_EXECUTABLE[sensor_type]
        default_params_file = os.path.join(
            pkg_share, "config", TYPE_TO_DEFAULT_PARAMS_FILE[sensor_type])

        # Unique node name per instance, required since ROS2 forbids
        # duplicate node names even across different input topics.
        node_name = f"{executable}_{i}"

        remappings = [("detection", sensor["output_topic"])]
        if sensor_type in ("lidar_2d",):
            remappings.append(("scan", sensor["input_topic"]))
        elif sensor_type in ("lidar_3d",):
            remappings.append(("points", sensor["input_topic"]))
        elif sensor_type in ("camera_cv", "camera_ml"):
            remappings.append(("image_raw", sensor["input_topic"]))
            if "camera_info_topic" in sensor:
                remappings.append(("camera_info", sensor["camera_info_topic"]))

        param_overrides = {"sensor_id": sensor["sensor_id"]}
        if "model_path" in sensor:
            param_overrides["model_path"] = sensor["model_path"]

        nodes.append(Node(
            package="perception",
            executable=executable,
            name=node_name,
            namespace=robot_name,
            parameters=[default_params_file, param_overrides],
            remappings=remappings,
            output="screen",
        ))

    return nodes


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="robot_1",
        description="Namespace prefix for multi-robot setups."
    )
    sensors_config_arg = DeclareLaunchArgument(
        "sensors_config",
        default_value="sensors_example.yaml",
        description="Path to a sensor instance list YAML (absolute, or a filename under perception/config)."
    )

    return LaunchDescription([
        robot_name_arg,
        sensors_config_arg,
        OpaqueFunction(function=launch_setup),
    ])
