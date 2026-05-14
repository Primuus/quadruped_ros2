"""
串口数据处理接口模块
用户可以继承 SerialDataProcessor 类并重写 process() 方法来实现自定义处理逻辑
"""

from abc import ABC, abstractmethod
from typing import Any

from std_msgs.msg import Float64MultiArray

from .qlp_protocol import QlpFrame, unpack_joystick_report


class SerialDataProcessor(ABC):
    """串口数据处理抽象基类"""

    def __init__(self, node):
        """
        初始化处理器
        :param node: ROS2节点实例，用于获取时钟和发布消息
        """
        self.node = node

    @abstractmethod
    def process(self, data: Any):
        """
        处理协议层输出的数据并返回 (mode_msg, key, joystick_data) 元组

        :param data: 协议层输出的数据对象
        :return: 处理后的 (模式消息, 按键字符, 摇杆数据) 元组
        """
        pass


class QlpJoystickProcessor(SerialDataProcessor):
    """处理 QLP 协议的遥控器摇杆上报帧。"""

    def process(self, data: QlpFrame):
        if not isinstance(data, QlpFrame):
            raise ValueError(f"期望 QlpFrame，实际收到 {type(data).__name__}")

        report = unpack_joystick_report(data)

        msg_mode = Float64MultiArray()
        msg_mode.data = [1.0 if self._is_mode_active(report.mode) else 0.0]

        joystick_data = {
            'x1': report.joy_lx,
            'y1': report.joy_ly,
            'x2': report.joy_rx,
            'y2': report.joy_ry,
        }

        return msg_mode, report.key_char, joystick_data

    @staticmethod
    def _is_mode_active(mode_value: int) -> bool:
        if mode_value == 0:
            return False
        if mode_value == ord('0'):
            return False
        return True


Uint8JoystickProcessor = QlpJoystickProcessor
