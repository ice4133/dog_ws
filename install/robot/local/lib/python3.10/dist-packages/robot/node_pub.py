#!/usr/bin/env python3
"""
电机命令发布器节点
用于同时向前6个电机发送控制命令
支持位置随时间线性增长
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import signal
import sys

class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_publisher')
        
        # 前6个电机的话题名称
        self.motor_topics = [
            "/real_robot/motor_cmd_FR_hip_joint",    # 电机0
            "/real_robot/motor_cmd_FR_thigh_joint",  # 电机1
            "/real_robot/motor_cmd_FR_calf_joint",   # 电机2
            "/real_robot/motor_cmd_FL_hip_joint",    # 电机3
            "/real_robot/motor_cmd_FL_thigh_joint",  # 电机4
            "/real_robot/motor_cmd_FL_calf_joint",   # 电机5
        ]
        # 使用不同的变量名，避免与可能的只读属性冲突
        self._motor_publishers = []  # 使用下划线前缀
        for i, topic in enumerate(self.motor_topics):
            publisher = self.create_publisher(Float32MultiArray, topic, 100)
            self._motor_publishers.append(publisher)
            self.get_logger().info(f"创建发布器: {topic}")
        

        # 定时器 - 每0.001秒发布一次命令
        self.timer = self.create_timer(0.001, self.publish_all_commands)  # 1000Hz
        
        # 电机命令参数
        self._kp = 0.0    # 刚度
        self._kd = 0.03      # 阻尼
        self._pos = 0.0    # 目标位置 (rad)
        self._speed = 36.0  # 目标速度 (rad/s)
        self._tau = 0.0     # 目标力矩 (Nm)
        
        # 默认mode值 (需要根据电机实际情况设置)
        # mode字节格式: 高4位是ID，低4位中前3位是001，最后1位是0
        # 所以低4位固定为 0b0010 = 2
        # mode = (ID << 4) | 0b0010
        self._motor_modes = [(i << 4) | 0b0010 for i in range(6)]
        
        # 时间相关变量
        self._start_time = self.get_clock().now()  # 记录节点启动时间
        self._last_update_time = self._start_time  # 记录上次更新时间
        self._pos_increment_per_second = 0  # 每秒增加3.14弧度
        
        # 初始位置
        self._initial_pos = 0.0
        
        self.get_logger().info(f"电机命令发布器已启动，将向{len(self._motor_publishers)}个电机发送命令")
        self.get_logger().info(f"参数: kp={self._kp}, kd={self._kd}, 初始pos={self._pos}, speed={self._speed}, tau={self._tau}")
        self.get_logger().info(f"位置将以每秒{self._pos_increment_per_second}弧度增加")
        self.get_logger().info(f"电机模式(十进制): {self._motor_modes}")
        
        # 发布计数器
        self._publish_count = 0
        
        # 注册Ctrl+C信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
    
    @property
    def kp(self):
        return self._kp
    
    @property
    def kd(self):
        return self._kd
    
    @property
    def pos(self):
        return self._pos
    
    @property
    def speed(self):
        return self._speed
    
    @property
    def tau(self):
        return self._tau
    
    @property
    def motor_modes(self):
        return self._motor_modes
    
    @property
    def motor_publishers(self):
        return self._motor_publishers
    
    @property
    def publish_count(self):
        return self._publish_count
    
    @publish_count.setter
    def publish_count(self, value):
        self._publish_count = value
    
    def update_position_based_on_time(self):
        """根据运行时间更新位置"""
        current_time = self.get_clock().now()
        
        # 计算从开始到现在的时间差（秒）
        time_diff = (current_time - self._start_time).nanoseconds / 1e9
        
        # 计算新的位置：每秒增加3.14弧度
        new_pos = self._initial_pos + time_diff * self._pos_increment_per_second
        
        # 更新位置
        self._pos = new_pos
        
        # 每0.1秒记录一次位置信息
        if (current_time - self._last_update_time).nanoseconds / 1e9 >= 0.1:
            self.get_logger().info(
                f"运行时间: {time_diff:.2f}s, 当前位置: {self._pos:.3f}rad, "
                f"每秒增加: {self._pos_increment_per_second}rad"
            )
            self._last_update_time = current_time
    
    def create_motor_command_msg(self, motor_id):
        """创建电机命令消息"""
        msg = Float32MultiArray()
        
        # 数据格式: [mode, tau, q, dq, kp, kd]
        # mode: 控制模式字节
        # tau:  目标力矩 (Nm)
        # q:    目标位置 (rad)
        # dq:   目标速度 (rad/s)
        # kp:   位置刚度
        # kd:   速度阻尼
        
        msg.data = [
            float(self._motor_modes[motor_id]),  # mode (转换为float)
            self._tau,                           # tau
            self._pos,                           # q (位置)
            self._speed,                         # dq (速度)
            self._kp,                            # kp
            self._kd                             # kd
        ]
        
        return msg
    
    def publish_all_commands(self):
        """同时发布所有电机命令"""
        try:
            # 根据时间更新位置
            self.update_position_based_on_time()
            
            # 发布所有电机命令
            for i, publisher in enumerate(self._motor_publishers):
                msg = self.create_motor_command_msg(i)
                publisher.publish(msg)
            
            self._publish_count += 1
            
            # 每发布100次打印一次日志（避免过于频繁）
            if self._publish_count % 100 == 0:
                self.get_logger().info(
                    f"已发布第{self._publish_count}批命令: "
                    f"电机0-5, pos={self._pos:.3f}"
                )
        except Exception as e:
            self.get_logger().error(f"发布命令时出错: {e}")
    
    def set_motor_parameters(self, kp=None, kd=None, pos=None, speed=None, tau=None):
        """设置电机参数"""
        try:
            if kp is not None:
                self._kp = float(kp)
            if kd is not None:
                self._kd = float(kd)
            if pos is not None:
                self._pos = float(pos)
                self._initial_pos = float(pos)  # 更新初始位置
            if speed is not None:
                self._speed = float(speed)
            if tau is not None:
                self._tau = float(tau)
            
            self.get_logger().info(f"更新参数: kp={self._kp}, kd={self._kd}, pos={self._pos}, speed={self._speed}, tau={self._tau}")
        except Exception as e:
            self.get_logger().error(f"设置参数时出错: {e}")
    
    def set_position_increment_rate(self, increment_per_second):
        """设置位置增加速率（弧度/秒）"""
        self._pos_increment_per_second = float(increment_per_second)
        self.get_logger().info(f"位置增加速率设置为: {self._pos_increment_per_second} 弧度/秒")
    
    def reset_position_timer(self):
        """重置位置计时器"""
        self._start_time = self.get_clock().now()
        self._initial_pos = self._pos
        self.get_logger().info(f"位置计时器已重置，当前位置: {self._pos}")
    
    def set_motor_mode(self, motor_id, mode):
        """设置特定电机的模式"""
        try:
            if 0 <= motor_id < len(self._motor_modes):
                self._motor_modes[motor_id] = int(mode)
                self.get_logger().info(f"设置电机{motor_id}模式为: {mode} (0x{mode:02X})")
            else:
                self.get_logger().warn(f"电机ID{motor_id}超出范围")
        except Exception as e:
            self.get_logger().error(f"设置电机模式时出错: {e}")
    
    def signal_handler(self, sig, frame):
        """处理Ctrl+C信号"""
        self.get_logger().info("接收到中断信号，正在停止发布...")
        self.stop_and_exit()
    
    def stop_and_exit(self):
        """停止发布并退出"""
        try:
            # 发送一次停止命令（所有参数设为0）
            self.get_logger().info("发送停止命令...")
            
            original_params = (self._kp, self._kd, self._pos, self._speed, self._tau)
            
            # 临时设置为0
            self.set_motor_parameters(kp=0.0, kd=0.0, pos=0.0, speed=0.0, tau=0.0)
            
            # 发送一次停止命令
            for i, publisher in enumerate(self._motor_publishers):
                msg = self.create_motor_command_msg(i)
                publisher.publish(msg)
            
            # 恢复原始参数
            self._kp, self._kd, self._pos, self._speed, self._tau = original_params
            
            self.get_logger().info("已发送停止命令，正在关闭节点...")
        except Exception as e:
            self.get_logger().error(f"停止过程中出错: {e}")
        finally:
            # 销毁定时器
            if hasattr(self, 'timer') and self.timer:
                self.timer.cancel()
            
            # 销毁节点
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    
    # 初始化node变量为None，确保在异常处理中可以安全访问
    node = None
    
    try:
        # 创建节点
        node = MotorCommandPublisher()
        
        # 启动节点
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("用户中断，正在清理...")
        else:
            print("用户中断，节点未完全初始化")
    except Exception as e:
        # 安全地记录异常，检查node是否已创建
        if node:
            node.get_logger().error(f"节点运行异常: {e}")
        else:
            print(f"节点初始化异常: {e}")
    finally:
        # 确保在节点存在时才调用stop_and_exit
        if node:
            node.stop_and_exit()
        else:
            # 如果节点未创建成功，直接关闭rclpy
            rclpy.shutdown()
            print("节点未创建成功，直接退出")

if __name__ == '__main__':
    main()