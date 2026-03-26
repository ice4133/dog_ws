#!/usr/bin/env python3.8
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class UpperCommsNode(Node):
    def __init__(self):
        super().__init__('upper_comms_node')
        self.get_logger().info('Upper comms node initialized')
        
        # 订阅上位机数据topic
        self.subscription = self.create_subscription(
            String,
            'upper_data',
            self.upper_data_callback,
            10
        )
        
        # 存储最新的上位机数据
        self.latest_data = {
            'imu': {},
            'label': '',
            'distance': 0.0,
            'x_offset': 0.0,
            'is_qr': False,
            'qr_content': ''
        }
        
    def upper_data_callback(self, msg):
        """
        处理上位机传来的数据
        数据格式示例:
        {
            "imu": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "label": "obstacle",
            "distance": 1.5,
            "x_offset": 0.1,
            "is_qr": false,
            "qr_content": ""
        }
        """
        try:
            data = json.loads(msg.data)
            self.latest_data = {
                'imu': data.get('imu', {}),
                'label': data.get('label', ''),
                'distance': data.get('distance', 0.0),
                'x_offset': data.get('x_offset', 0.0),
                'is_qr': data.get('is_qr', False),
                'qr_content': data.get('qr_content', '')
            }
            self.get_logger().debug(f'Received upper data: {self.latest_data}')
        except Exception as e:
            self.get_logger().error(f'Failed to parse upper data: {e}')
    
    def get_latest_data(self):
        """
        获取最新的上位机数据
        """
        return self.latest_data

class UpperData:
    """
    上位机数据结构
    """
    def __init__(self, data_dict):
        self.imu = data_dict.get('imu', {})
        self.label = data_dict.get('label', '')
        self.distance = data_dict.get('distance', 0.0)
        self.x_offset = data_dict.get('x_offset', 0.0)
        self.is_qr = data_dict.get('is_qr', False)
        self.qr_content = data_dict.get('qr_content', '')

if __name__ == '__main__':
    rclpy.init()
    upper_comms_node = UpperCommsNode()
    rclpy.spin(upper_comms_node)
    upper_comms_node.destroy_node()
    rclpy.shutdown()
