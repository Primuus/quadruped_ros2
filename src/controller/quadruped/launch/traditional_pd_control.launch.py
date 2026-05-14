from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time for the quadruped controller.',
    )
    output_mode_arg = DeclareLaunchArgument(
        'output_mode',
        default_value='1.0',
        description='0.0 publishes to simulation effort controller, 1.0 publishes to real MCU topic.',
    )
    lock_output_mode_arg = DeclareLaunchArgument(
        'lock_output_mode',
        default_value='false',
        description='Ignore /input/mode and keep the configured output_mode.',
    )
    require_joint_state_arg = DeclareLaunchArgument(
        'require_joint_state',
        default_value='false',
        description='Wait for /joint_states before entering the FSM.',
    )
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
    start_micro_ros_agent_arg = DeclareLaunchArgument(
        'start_micro_ros_agent',
        default_value='false',
        description='Start the micro-ROS Agent used by the MCU.',
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

    quadruped_node = Node(
        package='quadruped',
        executable='quadruped_node',
        name='quadruped_control',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'output_mode': ParameterValue(LaunchConfiguration('output_mode'), value_type=float),
            'lock_output_mode': ParameterValue(LaunchConfiguration('lock_output_mode'), value_type=bool),
            'require_joint_state': ParameterValue(LaunchConfiguration('require_joint_state'), value_type=bool),
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

    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_micro_ros_agent')),
        arguments=[
            'serial',
            '--dev',
            LaunchConfiguration('mcu_port'),
            '-b',
            LaunchConfiguration('mcu_baud_rate'),
            PythonExpression(["'-v' + '", LaunchConfiguration('micro_ros_verbosity'), "'"]),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        output_mode_arg,
        lock_output_mode_arg,
        require_joint_state_arg,
        start_serial_arg,
        start_kbd_arg,
        serial_port_arg,
        baud_rate_arg,
        start_micro_ros_agent_arg,
        mcu_port_arg,
        mcu_baud_rate_arg,
        micro_ros_verbosity_arg,
        quadruped_node,
        serial_controller_node,
        kbd_node,
        micro_ros_agent_node,
    ])
