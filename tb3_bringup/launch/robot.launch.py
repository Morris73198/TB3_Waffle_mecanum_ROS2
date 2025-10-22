from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tb3_opencr_control',
            executable='opencr_controller',
            name='tb3_opencr_controller',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
    ])
