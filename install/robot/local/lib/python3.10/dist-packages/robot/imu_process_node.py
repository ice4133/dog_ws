#!/usr/bin/env python3
"""IMU数据预处理节点：强化偏航稳定+抑制横滚/俯仰波动+数据滤波+降低打印频率"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math

class LowPassFilter:
    """一阶低通滤波器：平滑角速度/加速度数据，抑制高频噪声"""
    def __init__(self, alpha=0.1):
        self.alpha = alpha  # 滤波系数（越小越平滑，0.05-0.2为宜）
        self.x = None
        self.y = None
        self.z = None

    def update(self, x, y, z):
        if self.x is None:  # 初始化
            self.x = x
            self.y = y
            self.z = z
        else:
            # 一阶低通滤波公式：输出 = 系数*当前值 + (1-系数)*历史值
            self.x = self.alpha * x + (1 - self.alpha) * self.x
            self.y = self.alpha * y + (1 - self.alpha) * self.y
            self.z = self.alpha * z + (1 - self.alpha) * self.z
        return self.x, self.y, self.z

class ImuProcessNode(Node):
    def __init__(self):
        super().__init__("imu_process_node")
        # 1. 订阅/发布话题：保持不变
        self.imu_sub = self.create_subscription(Imu, "/livox/imu", self.imu_callback, 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data_processed", 10)

        # 2. 预处理参数（新增打印频率参数）
        self.declare_parameter("alpha", 0.98)          # 互补滤波系数
        self.declare_parameter("gravity", 9.81)        # 重力加速度
        self.declare_parameter("calibrate_max", 400)   # 初始零偏校准次数
        self.declare_parameter("static_threshold", 0.01)# 静态判断阈值（rad/s）
        self.declare_parameter("yaw_lock_gain", 0.005) # 偏航锁定增益
        self.declare_parameter("roll_pitch_lock_gain", 0.003) # 横滚/俯仰锁定增益（更小，更稳）
        self.declare_parameter("gyro_bias_update_rate", 50) # 零偏更新频率
        self.declare_parameter("filter_alpha", 0.1)    # 低通滤波系数
        self.declare_parameter("data_print_interval", 50) # 新增：IMU数据打印间隔（每10帧打印一次）
        self.declare_parameter("separator_print_interval", 20) # 新增：分隔线打印间隔
        
        # 读取参数
        self.alpha = self.get_parameter("alpha").value
        self.gravity = self.get_parameter("gravity").value
        self.calibrate_max = self.get_parameter("calibrate_max").value
        self.static_threshold = self.get_parameter("static_threshold").value
        self.yaw_lock_gain = self.get_parameter("yaw_lock_gain").value
        self.roll_pitch_lock_gain = self.get_parameter("roll_pitch_lock_gain").value
        self.gyro_bias_update_rate = self.get_parameter("gyro_bias_update_rate").value
        self.filter_alpha = self.get_parameter("filter_alpha").value
        self.data_print_interval = self.get_parameter("data_print_interval").value
        self.separator_print_interval = self.get_parameter("separator_print_interval").value

        # 3. 原有姿态解算缓存：保持不变
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # 4. 零偏校准相关变量：保持不变
        self.calibrate_count = 0
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.gyro_bias_buffer = [0.0, 0.0, 0.0]
        self.bias_update_count = 0
        self.is_calibrated = False

        # 5. 偏航稳定+新增横滚/俯仰锁定变量
        self.yaw_static_lock = 0.0
        self.roll_static_lock = 0.0  # 横滚锁定基准
        self.pitch_static_lock = 0.0 # 俯仰锁定基准
        self.is_static = False
        self.data_count = 0  # 新增：数据帧计数器（控制打印频率）

        # 6. 新增：低通滤波器实例（角速度+加速度）
        self.gyro_filter = LowPassFilter(self.filter_alpha)
        self.accel_filter = LowPassFilter(self.filter_alpha)

        # 日志打印（仅初始化日志，频率不变）
        self.get_logger().info("IMU数据预处理节点已启动（增强版：滤波+全姿态锁定+低打印频率）")
        self.get_logger().info(f"核心参数：alpha={self.alpha} | 静态阈值={self.static_threshold} rad/s | 滤波系数={self.filter_alpha}")
        self.get_logger().info(f"锁定增益：偏航={self.yaw_lock_gain} | 横滚/俯仰={self.roll_pitch_lock_gain}")
        self.get_logger().info(f"打印频率：每{self.data_print_interval}帧打印一次数据 | 每{self.separator_print_interval}帧打印分隔线")
        self.get_logger().info("======================================")
        self.get_logger().info("请保持IMU静止，正在进行初始零偏校准...")

    def imu_callback(self, msg: Imu):
        """增强版回调：滤波+横滚/俯仰锁定+降低打印频率"""
        # ========== 步骤0：初始零偏校准（保持不变，关键日志保留） ==========
        if not self.is_calibrated:
            self.gyro_bias[0] += msg.angular_velocity.x
            self.gyro_bias[1] += msg.angular_velocity.y
            self.gyro_bias[2] += msg.angular_velocity.z
            self.calibrate_count += 1

            # 校准进度日志：每100帧打印一次（不变，避免频繁）
            if self.calibrate_count % 100 == 0:
                self.get_logger().info(f"初始校准进度：{self.calibrate_count}/{self.calibrate_max}")

            if self.calibrate_count >= self.calibrate_max:
                self.gyro_bias = [b / self.calibrate_count for b in self.gyro_bias]
                self.is_calibrated = True
                # 初始化所有姿态锁定基准
                self.yaw_static_lock = self.yaw
                self.roll_static_lock = self.roll
                self.pitch_static_lock = self.pitch
                self.last_time = self.get_clock().now()
                self.get_logger().info("======================================")
                self.get_logger().info(f"初始零偏校准完成！零偏值：x={self.gyro_bias[0]:.6f} | y={self.gyro_bias[1]:.6f} | z={self.gyro_bias[2]:.6f} rad/s")
                self.get_logger().info("开始处理IMU数据（已启用滤波+全姿态锁定）...")
                self.get_logger().info("======================================")
            return

        # ========== 步骤1：数据预处理（新增滤波） ==========
        # 1.1 去除初始零偏
        raw_gx = msg.angular_velocity.x - self.gyro_bias[0]
        raw_gy = msg.angular_velocity.y - self.gyro_bias[1]
        raw_gz = msg.angular_velocity.z - self.gyro_bias[2]
        
        # 1.2 新增：角速度低通滤波（核心！抑制高频噪声）
        filtered_gx, filtered_gy, filtered_gz = self.gyro_filter.update(raw_gx, raw_gy, raw_gz)
        angular_vel = [filtered_gx, filtered_gy, filtered_gz]

        # 1.3 静止判断（基于滤波后的角速度，更准确）
        gyro_mag = math.hypot(filtered_gx, filtered_gy, filtered_gz)
        self.is_static = (gyro_mag < self.static_threshold)

        # 1.4 零偏实时更新（保持不变）
        if self.is_static:
            self.gyro_bias_buffer[0] += filtered_gx
            self.gyro_bias_buffer[1] += filtered_gy
            self.gyro_bias_buffer[2] += filtered_gz
            self.bias_update_count += 1

            if self.bias_update_count >= self.gyro_bias_update_rate:
                self.gyro_bias[0] += self.gyro_bias_buffer[0] / self.gyro_bias_update_rate * 0.1
                self.gyro_bias[1] += self.gyro_bias_buffer[1] / self.gyro_bias_update_rate * 0.1
                self.gyro_bias[2] += self.gyro_bias_buffer[2] / self.gyro_bias_update_rate * 0.1
                self.gyro_bias_buffer = [0.0, 0.0, 0.0]
                self.bias_update_count = 0

                # 更新所有姿态锁定基准（缓慢校准，抵消温漂）
                self.yaw_static_lock = self.yaw * 0.99 + self.yaw_static_lock * 0.01
                self.roll_static_lock = self.roll * 0.99 + self.roll_static_lock * 0.01
                self.pitch_static_lock = self.pitch * 0.99 + self.pitch_static_lock * 0.01

        # 1.5 线加速度归一化 + 新增低通滤波
        raw_acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        acc_mag = math.sqrt(raw_acc[0]**2 + raw_acc[1]**2 + raw_acc[2]**2)
        if acc_mag > 0:
            acc_normalized = [a * self.gravity / acc_mag for a in raw_acc]
        else:
            acc_normalized = raw_acc
        # 加速度滤波（核心！抑制线加速度波动）
        linear_acc = list(self.accel_filter.update(*acc_normalized))

        # ========== 步骤2：互补滤波姿态解算（新增横滚/俯仰锁定） ==========
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt <= 0:
            dt = 1/200

        # 2.1 角速度积分更新姿态（预测）
        self.roll += angular_vel[0] * dt
        self.pitch += angular_vel[1] * dt
        
        # 2.2 全姿态锁定：静止时拉回基准值，运动时正常积分
        if self.is_static:
            # 横滚锁定（增益更小，更稳定）
            self.roll = self.roll * (1 - self.roll_pitch_lock_gain) + self.roll_static_lock * self.roll_pitch_lock_gain
            # 俯仰锁定
            self.pitch = self.pitch * (1 - self.roll_pitch_lock_gain) + self.pitch_static_lock * self.roll_pitch_lock_gain
            # 偏航锁定（原有逻辑）
            self.yaw = self.yaw * (1 - self.yaw_lock_gain) + self.yaw_static_lock * self.yaw_lock_gain
        else:
            # 运动时正常积分
            self.yaw += angular_vel[2] * dt

        # 2.3 加速度计校正+互补滤波（保持不变）
        roll_acc = math.atan2(linear_acc[1], linear_acc[2])
        pitch_acc = math.atan2(-linear_acc[0], math.sqrt(linear_acc[1]**2 + linear_acc[2]**2))
        self.roll = self.alpha * self.roll + (1 - self.alpha) * roll_acc
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * pitch_acc

        # ========== 步骤3：构造消息+发布（保持不变） ==========
        q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        processed_imu = Imu()
        processed_imu.header = msg.header
        processed_imu.header.frame_id = "livox_frame"
        processed_imu.orientation.x = q[0]
        processed_imu.orientation.y = q[1]
        processed_imu.orientation.z = q[2]
        processed_imu.orientation.w = q[3]
        processed_imu.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        processed_imu.angular_velocity.x = angular_vel[0]
        processed_imu.angular_velocity.y = angular_vel[1]
        processed_imu.angular_velocity.z = angular_vel[2]
        processed_imu.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        processed_imu.linear_acceleration.x = linear_acc[0]
        processed_imu.linear_acceleration.y = linear_acc[1]
        processed_imu.linear_acceleration.z = linear_acc[2]
        processed_imu.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        # ========== 关键修改：控制打印频率 ==========
        self.data_count += 1
        # 仅当数据帧计数达到间隔时，才打印IMU数据
        if self.data_count % self.data_print_interval == 0:
            self.print_processed_data(processed_imu)
        
        self.imu_pub.publish(processed_imu)

    def print_processed_data(self, imu_msg: Imu):
        """降低打印频率后的打印函数"""
        q = [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w]
        roll_deg = math.degrees(euler_from_quaternion(q)[0])
        pitch_deg = math.degrees(euler_from_quaternion(q)[1])
        yaw_deg = math.degrees(euler_from_quaternion(q)[2])

        angular_x = imu_msg.angular_velocity.x
        angular_y = imu_msg.angular_velocity.y
        angular_z = imu_msg.angular_velocity.z
        linear_x = imu_msg.linear_acceleration.x
        linear_y = imu_msg.linear_acceleration.y
        linear_z = imu_msg.linear_acceleration.z

        # 静止状态提示
        static_str = "【静止锁定】" if self.is_static else "【运动模式】"
        self.get_logger().info(
            f"{static_str} 时间戳：{imu_msg.header.stamp.sec}.{int(imu_msg.header.stamp.nanosec//1e6):03d}\n"
            f"姿态（角度）：横滚={roll_deg:.2f}° | 俯仰={pitch_deg:.2f}° | 偏航={yaw_deg:.2f}°\n"
            f"角速度：x={angular_x:.4f} | y={angular_y:.4f} | z={angular_z:.4f} rad/s\n"
            f"线加速度：x={linear_x:.4f} | y={linear_y:.4f} | z={linear_z:.4f} m/s²"
        )
        # 分隔线打印：仅当达到分隔线间隔时打印
        if self.data_count % self.separator_print_interval == 0:
            self.get_logger().info("-----------------------------------------\n")

def main(args=None):
    rclpy.init(args=args)
    node = ImuProcessNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()