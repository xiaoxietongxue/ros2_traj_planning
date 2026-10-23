"""用 mock hardware 启动 UR5e，并发送一条五次多项式关节轨迹。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare('ur_description')
    urdf = PathJoinSubstitution([description_share, 'urdf', 'ur.urdf.xacro'])
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ', urdf,
                ' ur_type:=ur5e',
                ' name:=ur',
                ' use_fake_hardware:=true',
                ' tf_prefix:=',
            ]),
            value_type=str)
    }
    controllers = PathJoinSubstitution([
        FindPackageShare('robot_trajectory_execution'),
        'config',
        'controllers.yaml',
    ])
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'joint_trajectory_controller',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )
    execute = Node(
        package='robot_trajectory_execution',
        executable='execute_trajectory',
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='true'),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controllers],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen',
        ),
        spawner,
        RegisterEventHandler(
            OnProcessExit(target_action=spawner, on_exit=[execute])
        ),
    ])
