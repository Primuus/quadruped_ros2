from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_serial_arg = DeclareLaunchArgument(
        'start_serial',
        default_value='true',
        description='Start the QLP serial remote controller input node.',
    )
    start_kbd_arg = DeclareLaunchArgument(
        'start_kbd',
        default_value='false',
        description='Start the keyboard backup input node.',
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial device used by the QLP remote controller.',
    )
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate used by the QLP remote controller.',
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_gazebo'),
                'launch',
                'go1_gazebo.launch.py',
            ])
        )
    )

    quadruped_node = Node(
        package='quadruped',
        executable='quadruped_node',
        name='quadruped_control',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'output_mode': 0.0,
            'lock_output_mode': True,
            'require_joint_state': True,
        }],
    )

    serial_controller_node = Node(
        package='serial_controller',
        executable='serial_controller_node',
        name='serial_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_serial')),
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
        }],
    )

    kbd_node = Node(
        package='kbd',
        executable='kbd',
        name='keyboard_control',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('start_kbd')),
    )

    return LaunchDescription([
        start_serial_arg,
        start_kbd_arg,
        serial_port_arg,
        baud_rate_arg,
        gazebo_launch,
        quadruped_node,
        serial_controller_node,
        kbd_node,
    ])
