from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecanum_control',
            executable='mecanum_controller',
            name='mecanum_controller',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
    ])

