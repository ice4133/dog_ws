#!/usr/bin/env python3.8
import math
import time

# 条件导入UpperData
try:
    from upper_comms_node import UpperData
except ImportError:
    # 本地定义UpperData类用于测试
    class UpperData:
        def __init__(self, data_dict):
            self.imu = data_dict.get('imu', {})
            self.label = data_dict.get('label', '')
            self.distance = data_dict.get('distance', 0.0)
            self.x_offset = data_dict.get('x_offset', 0.0)
            self.is_qr = data_dict.get('is_qr', False)
            self.qr_content = data_dict.get('qr_content', '')

class PIDController:
    """
    PID控制器类，用于平滑处理偏差
    """
    def __init__(self, kp=1.0, ki=0.0, kd=0.1):
        self.kp = kp  # 比例系数
        self.ki = ki  # 积分系数
        self.kd = kd  # 微分系数
        self.last_error = 0.0  # 上一次误差
        self.integral = 0.0    # 积分值
        self.integral_limit = 10.0  # 积分限幅
        self.error_threshold = 0.1  # 积分分离阈值
    
    def compute(self, error, dt=0.01):
        """
        计算控制量
        error: 当前误差
        dt: 时间步长
        """
        # 积分分离：当误差较大时，不使用积分项
        if abs(error) < self.error_threshold:
            # 只有当误差较小时，才累积积分
            self.integral += error * dt
            # 积分限幅：限制积分项的最大值
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        else:
            # 当误差较大时，重置积分项
            self.integral = 0.0
        
        # 积分重置：当误差符号改变时，重置积分项
        if error * self.last_error < 0:
            self.integral = 0.0
        
        derivative = (error - self.last_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return output

class ControlCommand:
    """
    控制命令结构
    """
    def __init__(self):
        self.left_speed = 0.0    # 左侧轮子速度
        self.right_speed = 0.0   # 右侧轮子速度
        self.mode = "wheel"      # 运动方式：wheel(纯轮式), gait(足式步态)

class DecisionControl:
    """
    决策控制模块
    根据上位机数据生成控制命令
    """
    def __init__(self):
        # 控制参数
        self.max_speed = 50.0      # 最大速度
        self.min_speed = 10.0      # 最小速度
        self.speed_threshold = 0.5  # 距离阈值，低于此值减速
        self.align_threshold = 0.05  # 对正偏差阈值
        self.turn_angle = 90.0      # 二维码转向角度
        self.obstacle_distance_threshold = 0.5  # 障碍物接近阈值
        self.obstacle_stop_threshold = 0.3     # 障碍物停止阈值
        
        # PID控制器
        self.pid_controller = PIDController(kp=5.0, ki=0.0, kd=1.0)
        
        # 状态变量
        self.current_state = "idle"
        self.obstacle_state = "none"  # 障碍物处理状态：none, approaching, processing, completed
        self.obstacle_type = ""       # 障碍物类型
        self.action_start_time = 0     # 动作开始时间
        self.qr_state = "none"        # 二维码处理状态：none, processing, completed
        self.last_qr_content = ""     # 上一个二维码内容
    
    def process_upper_data(self, upper_data):
        """
        处理上位机数据并生成控制命令
        """
        cmd = ControlCommand()
        
        # 处理二维码
        if upper_data.is_qr:
            cmd = self.process_qr_code(upper_data)
        else:
            # 处理普通目标
            cmd = self.process_target(upper_data)
            # 重置二维码状态
            self.qr_state = "none"
        
        return cmd
    
    def process_qr_code(self, upper_data):
        """
        处理二维码数据
        逻辑：
        1. 解析二维码内容
        2. 根据内容生成控制命令
        3. 设置适当的左右轮速度
        4. 记录二维码处理状态
        """
        cmd = ControlCommand()
        
        # 获取二维码内容并转换为小写
        qr_content = upper_data.qr_content.lower()
        
        # 根据二维码内容生成控制命令
        if qr_content == "left":
            # 左转：左侧减速，右侧加速
            cmd.left_speed = 15.0  # 左侧较低速度
            cmd.right_speed = 45.0  # 右侧较高速度
            cmd.mode = "gait"      # 使用足式步态
            self.current_state = "turning_left"
            self.qr_state = "processing"
            self.last_qr_content = "left"
        elif qr_content == "right":
            # 右转：右侧减速，左侧加速
            cmd.left_speed = 45.0  # 左侧较高速度
            cmd.right_speed = 15.0  # 右侧较低速度
            cmd.mode = "gait"      # 使用足式步态
            self.current_state = "turning_right"
            self.qr_state = "processing"
            self.last_qr_content = "right"
        else:
            # 未知二维码内容，停止
            cmd.left_speed = 0.0
            cmd.right_speed = 0.0
            cmd.mode = "wheel"
            self.current_state = "idle"
            self.qr_state = "none"
            self.last_qr_content = ""
        
        return cmd
    
    def execute_obstacle_action(self, obstacle_type):
        """
        执行障碍物动作
        返回值：动作是否完成
        """
        try:
            import manual
            if obstacle_type == "step":
                # 处理台阶
                print("Executing step action")
                # 假设manual.some_step_function()返回动作是否完成
                # return manual.some_step_function()  # 替换为实际的函数名
                # 暂时返回False，需要根据实际函数返回值修改
                return False
            elif obstacle_type == "stair":
                # 处理楼梯
                print("Executing stair action")
                # 假设manual.some_stair_function()返回动作是否完成
                # return manual.some_stair_function()  # 替换为实际的函数名
                # 暂时返回False，需要根据实际函数返回值修改
                return False
            elif obstacle_type == "obstacle":
                # 处理普通障碍物
                print("Executing obstacle action")
                # 假设manual.some_obstacle_function()返回动作是否完成
                # return manual.some_obstacle_function()  # 替换为实际的函数名
                # 暂时返回False，需要根据实际函数返回值修改
                return False
        except Exception as e:
            print(f"Error executing obstacle action: {e}")
        return False
    
    def check_action_completed(self):
        """
        检查动作是否完成
        
        三种检测方式：
        1. 使用动作函数的返回值（推荐）
        2. 检查manual模块中的动作完成标志
        3. 根据时间判断
        """
        # 1. 使用动作函数的返回值（推荐）
        # 注意：这里需要根据实际的obstacle_type调用相应的动作函数
        if self.obstacle_type:
            action_completed = self.execute_obstacle_action(self.obstacle_type)
            if action_completed:
                return True
        
        # 2. 检查manual模块中的动作完成标志
        try:
            import manual
            # 示例：检查跳断桥动作是否完成
            if hasattr(manual, 'duanqiao_flag') and manual.duanqiao_flag == 1:
                return True
            # 其他动作的检查...
        except Exception as e:
            pass
        
        # 3. 或者根据时间判断，假设动作需要3秒完成
        if time.time() - self.action_start_time > 3.0:
            return True
        
        return False
    
    def process_target(self, upper_data):
        """
        处理普通目标数据
        逻辑流程：
        1. 无障碍物情况：label为"0"且非二维码时
        2. 障碍物处理：检测到障碍物时
        3. 普通导航：其他情况
        """
        cmd = ControlCommand()
        
        # 获取数据
        distance = upper_data.distance
        x_offset = upper_data.x_offset
        label = upper_data.label
        
        # 1. 无障碍物情况：当label为"0"且非二维码时
        if str(label) == "0" and not upper_data.is_qr:
            # 重置障碍物状态
            self.obstacle_state = "none"
            self.obstacle_type = ""
            
            # 无障碍物，设置默认速度
            base_speed = self.min_speed
            
            # 根据x_offset控制行进方向
            if abs(x_offset) > self.align_threshold:
                # 使用PID控制器处理偏差，计算左右轮速度差
                error = -x_offset  # 负号表示偏差方向
                speed_diff = self.pid_controller.compute(error)
                
                # 计算左右轮速度
                cmd.left_speed = base_speed - speed_diff
                cmd.right_speed = base_speed + speed_diff
                
                # 限制速度范围
                cmd.left_speed = max(-self.max_speed, min(self.max_speed, cmd.left_speed))
                cmd.right_speed = max(-self.max_speed, min(self.max_speed, cmd.right_speed))
            else:
                # 偏差较小，保持直线行驶
                cmd.left_speed = base_speed
                cmd.right_speed = base_speed
            
            # 设置模式和状态
            cmd.mode = "wheel"
            self.current_state = "moving"
            return cmd
        
        # 2. 障碍物处理：当检测到障碍物时
        if isinstance(label, str):
            label_lower = label.lower()
            if label_lower in ["step", "stair", "obstacle"]:
                self.obstacle_type = label_lower
                
                # 检测是否接近障碍物
                if distance < self.obstacle_distance_threshold and self.obstacle_state == "none":
                    # 接近障碍物，准备执行动作
                    self.obstacle_state = "approaching"
                    self.current_state = "approaching_obstacle"
                    
                    # 接近障碍物时，设置一个较低的速度
                    base_speed = self.min_speed * 0.5
                    
                    # 根据x_offset控制行进方向
                    if abs(x_offset) > self.align_threshold:
                        # 使用PID控制器处理偏差，计算左右轮速度差
                        error = -x_offset  # 负号表示偏差方向
                        speed_diff = self.pid_controller.compute(error)
                        
                        # 计算左右轮速度
                        cmd.left_speed = base_speed - speed_diff
                        cmd.right_speed = base_speed + speed_diff
                        
                        # 限制速度范围
                        cmd.left_speed = max(-self.max_speed, min(self.max_speed, cmd.left_speed))
                        cmd.right_speed = max(-self.max_speed, min(self.max_speed, cmd.right_speed))
                    else:
                        # 偏差较小，保持直线行驶
                        cmd.left_speed = base_speed
                        cmd.right_speed = base_speed
                    
                    # 设置模式
                    cmd.mode = "gait"
                    return cmd
                
                elif distance < self.obstacle_stop_threshold and self.obstacle_state == "approaching":
                    # 到达障碍物旁边，开始执行动作
                    self.obstacle_state = "processing"
                    self.action_start_time = time.time()
                    self.execute_obstacle_action(label_lower)
                    
                    # 立即返回，避免继续执行下面的代码
                    cmd.left_speed = 0.0
                    cmd.right_speed = 0.0
                    cmd.mode = "gait"
                    self.current_state = "processing_obstacle"
                    return cmd
                
                elif self.obstacle_state == "processing":
                    # 正在执行动作，保持停止状态
                    cmd.left_speed = 0.0
                    cmd.right_speed = 0.0
                    cmd.mode = "gait"
                    self.current_state = "processing_obstacle"
                    
                    # 检查动作是否完成
                    if self.check_action_completed():
                        self.obstacle_state = "completed"
                        # 动作完成，重置状态
                        self.obstacle_state = "none"
                        self.obstacle_type = ""
                        # 继续原来的导航流程
                        self.current_state = "moving"
                        # 动作完成后，设置一个默认速度继续前进
                        base_speed = self.min_speed
                        cmd.left_speed = base_speed
                        cmd.right_speed = base_speed
                        cmd.mode = "gait"
                    return cmd
        
        # 3. 普通导航：当检测到非障碍物目标时
        # 根据距离计算基础速度
        if distance > 3.0:
            base_speed = self.max_speed
        elif distance > self.speed_threshold:
            base_speed = self.min_speed + (distance - self.speed_threshold) * (self.max_speed - self.min_speed) / (3.0 - self.speed_threshold)
        else:
            base_speed = 0.0
        
        # 根据x_offset控制行进方向
        if abs(x_offset) > self.align_threshold:
            # 使用PID控制器处理偏差，计算左右轮速度差
            error = -x_offset  # 负号表示偏差方向
            speed_diff = self.pid_controller.compute(error)
            
            # 计算左右轮速度
            cmd.left_speed = base_speed - speed_diff
            cmd.right_speed = base_speed + speed_diff
            
            # 限制速度范围
            cmd.left_speed = max(-self.max_speed, min(self.max_speed, cmd.left_speed))
            cmd.right_speed = max(-self.max_speed, min(self.max_speed, cmd.right_speed))
        else:
            # 偏差较小，保持直线行驶
            cmd.left_speed = base_speed
            cmd.right_speed = base_speed
        
        # 根据目标类型选择运动模式
        if isinstance(label, str):
            label_lower = label.lower()
            if label_lower in ["step", "stair", "obstacle"]:
                cmd.mode = "gait"
            elif label_lower in ["flat", "ground"]:
                cmd.mode = "wheel"
            else:
                # 默认使用轮式
                cmd.mode = "wheel"
        else:
            # 默认使用轮式
            cmd.mode = "wheel"
        
        # 根据距离判断是否停止
        if distance < self.obstacle_stop_threshold and self.obstacle_state == "none":
            cmd.left_speed = 0.0
            cmd.right_speed = 0.0
            self.current_state = "approached"
        else:
            self.current_state = "moving"
        
        return cmd
    
    def get_state(self):
        """
        获取当前状态
        """
        return self.current_state
    
    def reset(self):
        """
        重置状态
        """
        self.current_state = "idle"
        self.qr_state = "none"
        self.last_qr_content = ""
        self.obstacle_state = "none"
        self.obstacle_type = ""
        self.action_start_time = 0

if __name__ == '__main__':
    # 测试决策控制模块
    decision_control = DecisionControl()
    
    # 测试普通目标
    test_data_1 = {
        "imu": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "label": "obstacle",
        "distance": 2.0,
        "x_offset": 0.1,
        "is_qr": False,
        "qr_content": ""
    }
    upper_data_1 = UpperData(test_data_1)
    cmd_1 = decision_control.process_upper_data(upper_data_1)
    print(f"Test 1 - Left Speed: {cmd_1.left_speed:.2f}, Right Speed: {cmd_1.right_speed:.2f}, Mode: {cmd_1.mode}, State: {decision_control.current_state}")
    
    # 测试二维码
    test_data_2 = {
        "imu": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "label": "qr_code",
        "distance": 1.0,
        "x_offset": 0.0,
        "is_qr": True,
        "qr_content": "left"
    }
    upper_data_2 = UpperData(test_data_2)
    cmd_2 = decision_control.process_upper_data(upper_data_2)
    print(f"Test 2 - Left Speed: {cmd_2.left_speed:.2f}, Right Speed: {cmd_2.right_speed:.2f}, Mode: {cmd_2.mode}, State: {decision_control.current_state}")
    
    # 测试无障碍物情况
    test_data_3 = {
        "imu": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "label": "0",
        "distance": 5.0,
        "x_offset": 0.0,
        "is_qr": False,
        "qr_content": ""
    }
    upper_data_3 = UpperData(test_data_3)
    cmd_3 = decision_control.process_upper_data(upper_data_3)
    print(f"Test 3 - Left Speed: {cmd_3.left_speed:.2f}, Right Speed: {cmd_3.right_speed:.2f}, Mode: {cmd_3.mode}, State: {decision_control.current_state}")
    
    # 测试障碍物处理逻辑
    print("\n=== Testing obstacle processing ===")
    
    # 测试接近障碍物
    test_data_4 = {
        "imu": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "label": "step",
        "distance": 0.4,
        "x_offset": 0.0,
        "is_qr": False,
        "qr_content": ""
    }
    upper_data_4 = UpperData(test_data_4)
    cmd_4 = decision_control.process_upper_data(upper_data_4)
    print(f"Test 4 - Left Speed: {cmd_4.left_speed:.2f}, Right Speed: {cmd_4.right_speed:.2f}, Mode: {cmd_4.mode}, State: {decision_control.current_state}, Obstacle State: {decision_control.obstacle_state}")
    
    # 测试到达障碍物
    test_data_5 = {
        "imu": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "label": "step",
        "distance": 0.2,
        "x_offset": 0.0,
        "is_qr": False,
        "qr_content": ""
    }
    upper_data_5 = UpperData(test_data_5)
    cmd_5 = decision_control.process_upper_data(upper_data_5)
    print(f"Test 5 - Left Speed: {cmd_5.left_speed:.2f}, Right Speed: {cmd_5.right_speed:.2f}, Mode: {cmd_5.mode}, State: {decision_control.current_state}, Obstacle State: {decision_control.obstacle_state}")
    
    # 测试动作执行中
    test_data_6 = {
        "imu": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "label": "step",
        "distance": 0.2,
        "x_offset": 0.0,
        "is_qr": False,
        "qr_content": ""
    }
    upper_data_6 = UpperData(test_data_6)
    # 等待一段时间，模拟动作执行
    time.sleep(0.5)
    cmd_6 = decision_control.process_upper_data(upper_data_6)
    print(f"Test 6 - Left Speed: {cmd_6.left_speed:.2f}, Right Speed: {cmd_6.right_speed:.2f}, Mode: {cmd_6.mode}, State: {decision_control.current_state}, Obstacle State: {decision_control.obstacle_state}")
    
    # 测试动作完成
    test_data_7 = {
        "imu": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "label": "step",
        "distance": 0.2,
        "x_offset": 0.0,
        "is_qr": False,
        "qr_content": ""
    }
    upper_data_7 = UpperData(test_data_7)
    # 等待更长时间，模拟动作完成
    time.sleep(3.0)
    cmd_7 = decision_control.process_upper_data(upper_data_7)
    print(f"Test 7 - Left Speed: {cmd_7.left_speed:.2f}, Right Speed: {cmd_7.right_speed:.2f}, Mode: {cmd_7.mode}, State: {decision_control.current_state}, Obstacle State: {decision_control.obstacle_state}")
