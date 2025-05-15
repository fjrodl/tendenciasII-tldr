from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='optical_flow_ros2',
            executable='optical_flow_node',
            name='optical_flow_node',
            parameters=[
                {'video_source': 0},
                {'show_debug': False}
            ],
            output='screen'
        )
    ])
