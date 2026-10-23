"""只显示 UR5e 模型，不启动 MoveIt。复用官方 ur_description/view_ur.launch.xml。"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_description'),
                    'launch',
                    'view_ur.launch.xml',
                ])
            ]),
            launch_arguments={'ur_type': 'ur5e'}.items(),
        )
    ])
