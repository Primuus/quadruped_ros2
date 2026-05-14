from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path

pkg_path = get_package_share_path('robot_gazebo')
default_rviz_config_path = pkg_path / 'rviz/urdf.rviz'

world_path = str(pkg_path / 'worlds' / 'rc26.world')

config_path = PathJoinSubstitution(
        [FindPackageShare("robot_gazebo"), "config", "go1_ros_control.yaml"]
    )

robot_description = ParameterValue(Command(['xacro ', PathJoinSubstitution([str(pkg_path), 'urdf/go1/go1.urdf.xacro'])]))

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=str(pkg_path / 'models')
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        ExecuteProcess(
            cmd=['gzserver', world_path, '-slibgazebo_ros_init.so', '-slibgazebo_ros_factory.so', '-slibgazebo_ros_force_system.so', '--verbose'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description', '-entity', 'go1', '-x', '5.5','-y', '5.5','-z', '0.26', '-R', '0.0', '-P', '0.0', '-Y', '0.0',],
                    output='screen'
                ),
            ]
        ),

        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "joint_states_controller",
                        "--controller-manager",
                        "/controller_manager"
                    ]
                ),

                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "joint_effort_controller",
                        "--controller-manager",
                        "/controller_manager"
                    ]
                ),
            ]
        ),
    ])
