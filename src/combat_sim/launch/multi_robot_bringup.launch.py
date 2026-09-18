import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    combat_sim_share = get_package_share_directory("combat_sim")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    bringup_share = get_package_share_directory("bringup")

    world_path = os.path.join(combat_sim_share, "worlds", "combat_arena.world")
    sensors_sim_path = os.path.join(combat_sim_share, "config", "sensors_sim.yaml")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_path}.items(),
    )

    def spawn(robot_name, x, y, yaw):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(combat_sim_share, "launch", "spawn_robot.launch.py")
            ),
            launch_arguments={
                "robot_name": robot_name, "x": str(x), "y": str(y), "yaw": str(yaw),
            }.items(),
        )

    def combat_stack(robot_name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "combat_framework.launch.py")
            ),
            launch_arguments={
                "robot_name": robot_name,
                "sensors_config": sensors_sim_path,
            }.items(),
        )

    # robot1 and robot2 spawned facing each other, 3m apart.
    spawn_robot1 = spawn("robot1", x=-1.5, y=0.0, yaw=0.0)
    spawn_robot2 = spawn("robot2", x=1.5, y=0.0, yaw=3.14159)

    stack_robot1 = combat_stack("robot1")
    stack_robot2 = combat_stack("robot2")

    return LaunchDescription([
        gazebo,
        spawn_robot1,
        spawn_robot2,
        stack_robot1,
        stack_robot2,
    ])
