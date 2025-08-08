from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='janus_publisher',
            executable='streamer',
            name='streamer_node',
            output='screen',
            parameters=[{
                'device': '/dev/video0',
                'host': '127.0.0.1',
                'rtp_port': 10000,
                'rtcp_port': 10001,
                'pt': 126,
                'width': 1280,
                'height': 720,
                'fps': 30,
                'bitrate': 2000,
                'keyint': 30,
                'use_vaapi': False,
            }]
        )
    ])
