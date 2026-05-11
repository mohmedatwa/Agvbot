#/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    
    Agvbot= Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_controller", 
                   "--controller-manager", 
                   "/controller_manager",
         ],

    )
    odom_tf= Node(
        package='topic_tools',
        executable='relay',
        name='tf_odometry_relay',
        output='screen',
        arguments=['/mecanum_controller/tf_odometry', 'tf'],
    )


    return LaunchDescription(
        [         
            joint_state_broadcaster_spawner,         
            Agvbot,
            odom_tf,
        ]
    )