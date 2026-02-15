#!/usr/bin env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    agv_description_dir = get_package_share_directory("agvbot_description")
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value= os.path.join(agv_description_dir,"urdf","Agv_bot.xacro"),
        description="Path to robot urdf file"
    )
    robot_description =ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),value_type=str)

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )
    rviz_node = Node(

        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=("-d",os.path.join(agv_description_dir,"rviz", "display.rviz"))
    )

    return LaunchDescription([
        model_arg,
        robot_state_pub_node,
        joint_state_publisher_gui,
        rviz_node,
        
    ])