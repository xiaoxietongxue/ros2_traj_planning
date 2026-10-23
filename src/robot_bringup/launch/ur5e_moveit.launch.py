"""启动 UR5e 的 TF / 关节状态，并复用官方 ur_moveit.launch.py。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    launch_rviz = LaunchConfiguration('launch_rviz')
    run_demo = LaunchConfiguration('run_demo')

    description_share = FindPackageShare('ur_description')
    urdf_file = PathJoinSubstitution([description_share, 'urdf', 'ur.urdf.xacro'])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', urdf_file,
        ' ur_type:=ur5e',
        ' name:=ur',
        ' safety_limits:=true',
        ' safety_pos_margin:=0.15',
        ' safety_k_position:=20',
        ' tf_prefix:=',
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    initial_zeros = {
        'shoulder_pan_joint': 0.0,
        'shoulder_lift_joint': -1.5707,
        'elbow_joint': 0.0,
        'wrist_1_joint': 0.0,
        'wrist_2_joint': 0.0,
        'wrist_3_joint': 0.0,
    }

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[
                robot_description,
                {'rate': 30, 'zeros': initial_zeros},
            ],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_moveit_config'),
                    'launch',
                    'ur_moveit.launch.py',
                ])
            ]),
            launch_arguments={
                'ur_type': 'ur5e',
                'launch_rviz': launch_rviz,
                'launch_servo': 'false',
            }.items(),
        ),
        Node(
            package='robot_motion_planning',
            executable='planning_demo',
            name='planning_demo',
            output='screen',
            parameters=[robot_description],
            condition=IfCondition(run_demo),
        ),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='true',
                              description='是否启动 RViz 2'),
        DeclareLaunchArgument('run_demo', default_value='false',
                              description='是否自动运行 planning_demo'),
        OpaqueFunction(function=launch_setup),
    ])
