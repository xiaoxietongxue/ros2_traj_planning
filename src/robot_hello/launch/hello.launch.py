from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_hello', executable='talker', name='talker', output='screen'),
        Node(package='robot_hello', executable='listener', name='listener', output='screen'),
    ])
