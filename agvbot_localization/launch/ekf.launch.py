#/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name="ekf_config",
        default_value= os.path.join(get_package_share_directory("agvbot_localization"),"config","ekf.yaml"),
        description="Path to ekf file"
    )
    ekf_config = LaunchConfiguration("ekf_config")
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[ekf_config]
    )





    return LaunchDescription([
        model_arg,
        robot_localization
    ])