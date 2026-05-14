from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():
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
    mcu_port_arg = DeclareLaunchArgument(
        'mcu_port',
        default_value='/dev/ttyUSB1',
        description='Serial device used by the MCU micro-ROS client.',
    )
    mcu_baud_rate_arg = DeclareLaunchArgument(
        'mcu_baud_rate',
        default_value='115200',
        description='Serial baud rate used by the MCU micro-ROS client.',
    )
    micro_ros_verbosity_arg = DeclareLaunchArgument(
        'micro_ros_verbosity',
        default_value='6',
        description='micro-ROS Agent verbosity level.',
    )
    terminal_arg = DeclareLaunchArgument(
        'terminal',
        default_value='auto',
        description='Terminal emulator executable. Use auto to detect one.',
    )

    launcher = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([
                FindPackagePrefix('quadruped'),
                'lib',
                'quadruped',
                'real_robot_terminals',
            ]),
            '--serial-port',
            LaunchConfiguration('serial_port'),
            '--baud-rate',
            LaunchConfiguration('baud_rate'),
            '--mcu-port',
            LaunchConfiguration('mcu_port'),
            '--mcu-baud-rate',
            LaunchConfiguration('mcu_baud_rate'),
            '--micro-ros-verbosity',
            LaunchConfiguration('micro_ros_verbosity'),
            '--terminal',
            LaunchConfiguration('terminal'),
        ],
        output='screen',
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        mcu_port_arg,
        mcu_baud_rate_arg,
        micro_ros_verbosity_arg,
        terminal_arg,
        launcher,
    ])
