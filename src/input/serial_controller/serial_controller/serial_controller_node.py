import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from quadruped.msg import JoystickCommand
from serial import Serial, SerialException
import threading


from .data_processor import QlpJoystickProcessor
from .qlp_protocol import FC_JOYSTICK_REPORT, QlpFrame, QlpStreamParser


class SerialControllerNode(Node):
    def __init__(self):
        super().__init__('serial_controller')

        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        # 获取参数
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        self.mode_publisher = self.create_publisher(Float64MultiArray, '/input/mode', 10)
        self.key_publisher = self.create_publisher(String, '/kbd_input', 10)
        self.joystick_publisher = self.create_publisher(JoystickCommand, '/joystick/command', 10)

        self.last_mode_active = False
        self.last_logged_mode_active = None
        self.last_logged_key = ''

        # 数据处理器（使用 QLP 遥控器摇杆处理器）
        self.processor = QlpJoystickProcessor(self)
        self.parser = QlpStreamParser()

        # 初始化
        try:
            self.serial = Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f'串口已打开: {serial_port} @ {baud_rate} baud')
        except SerialException as e:
            self.get_logger().error(f'无法打开串口 {serial_port}: {e}')
            raise
        
        self.running = True
        self.serial_thread = threading.Thread(target=self.read_serial_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        self.get_logger().info('Serial Controller 已启动，发布传统PD输入话题')

    def read_serial_loop(self):
        """后台线程读取串口数据并按 QLP 协议解帧。"""
        while self.running and rclpy.ok():
            try:
                new_data = self.serial.read(max(1, self.serial.in_waiting))
                if not new_data:
                    continue

                for frame in self.parser.parse_bytes(new_data):
                    self.handle_frame(frame)
            except SerialException as e:
                self.get_logger().error(f'串口读取错误: {e}')
            except Exception as e:
                self.get_logger().warn(f'处理数据时出错: {e}')

    def handle_frame(self, frame: QlpFrame):
        """处理一帧完整的 QLP 协议数据。"""
        if frame.fc != FC_JOYSTICK_REPORT:
            return

        try:
            msg_mode, key, joystick_data = self.processor.process(frame)
            if msg_mode:
                self.mode_publisher.publish(msg_mode)

            mode_active = bool(msg_mode.data and msg_mode.data[0] != 0.0)
            if mode_active != self.last_logged_mode_active:
                self.get_logger().info(f'遥控器模式: {1.0 if mode_active else 0.0}')
                self.last_logged_mode_active = mode_active
            if key and key != self.last_logged_key:
                self.get_logger().info(f'遥控器按键: {key}')
            self.last_logged_key = key if key else ''

            if self.last_mode_active and not mode_active:
                msg_key = String()
                msg_key.data = 'i'
                self.key_publisher.publish(msg_key)
            elif not mode_active and key:
                msg_key = String()
                msg_key.data = key
                self.key_publisher.publish(msg_key)
            self.last_mode_active = mode_active

            if joystick_data:
                joystick_msg = JoystickCommand()
                joystick_msg.header.stamp = self.get_clock().now().to_msg()
                joystick_msg.header.frame_id = 'joystick'
                joystick_msg.x1 = joystick_data.get('x1', 0.0)
                joystick_msg.y1 = joystick_data.get('y1', 0.0)
                joystick_msg.x2 = joystick_data.get('x2', 0.0)
                joystick_msg.y2 = joystick_data.get('y2', 0.0)
                joystick_msg.key = key if key else ''
                self.joystick_publisher.publish(joystick_msg)
        except Exception as e:
            self.get_logger().warn(f'Frame processing failed: {e}')

    def destroy_node(self):
        """关闭节点时清理资源"""
        self.running = False
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
            self.get_logger().info('串口已关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
