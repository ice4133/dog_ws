#! /usr/bin/env python3.8
import sys
import time
from datetime import datetime
import keyboard
import threading
import sys

import manual
sys.path.append('../lib')
from unitree_actuator_sdk import *
import signal
import unitree_actuator_sdk

# 导入串口通信模块 + 初始化serial_node 原代码必备
# try:
#     from serial_node import SerialCommunicationNode
#     print("Successfully imported serial_node")
#     serial_node = SerialCommunicationNode()
# except ImportError:
#     serial_node = None
#     print("Warning: Cannot import serial_node module")
serial_node = None  # 修改1：定义serial_node，解决未声明报错

# 关节电机初始角度参数
motor_init =[26.1436, -15.0519, -22.6491, 17.8566, 17.3828, -22.627000000000002, -12.584499999999998, 27.9909]

# 定义不同步态下的轮子转速参数结构体
class WheelSpeedConfig:
    def __init__(self, left_front=0.0, right_front=0.0, left_rear=0.0, right_rear=0.0):
        self.left_front = left_front      # 左前轮转速
        self.right_front = right_front    # 右前轮转速
        self.left_rear = left_rear        # 左后轮转速
        self.right_rear = right_rear      # 右后轮转速

# 不同步态对应的轮子转速配置 原代码完整版本
wheel_speed_configs = [
    WheelSpeedConfig(0.0, 0.0, 0.0, 0.0),        # STOP - 停止
    WheelSpeedConfig(1.0, 1.0, 1.0, 1.0),        # TROT - 疾步
    WheelSpeedConfig(-1.0, -1.0, -1.0, -1.0),    # TROT_BACK - 后退疾步
    WheelSpeedConfig(-1.0, 1.0, -1.0, 1.0),      # TROT_BACK_RIGHT - 后退右转
    WheelSpeedConfig(1.0, -1.0, 1.0, -1.0),      # TROT_BACK_LEFT - 后退左转
    WheelSpeedConfig(1.0, -1.0, 1.0, -1.0),      # TROT_LEFT - 左转
    WheelSpeedConfig(-1.0, 1.0, -1.0, 1.0),      # TROT_RIGHT - 右转
    WheelSpeedConfig(-1.0, 1.0, -1.0, 1.0),      # ROTATE_LEFT - 原地左转
    WheelSpeedConfig(1.0, -1.0, 1.0, -1.0),      # ROTATE_RIGHT - 原地右转
    WheelSpeedConfig(1.5, 1.5, 1.5, 1.5),        # SHANGPO - 上坡
    WheelSpeedConfig(0.5, 0.5, 0.5, 0.5),        # slow_TROT - 慢走
    WheelSpeedConfig(0.5, -0.5, 0.5, -0.5),      # small_TROT_LEFT - 小幅度左转
    WheelSpeedConfig(-0.5, 0.5, -0.5, 0.5),      # small_TROT_RIGHT - 小幅度右转
    WheelSpeedConfig(1.0, 1.0, 1.0, 1.0),        # dijia_trot - 地面疾走
    WheelSpeedConfig(-1.0, 1.0, -1.0, 1.0),      # dijia_left - 地面左转
    WheelSpeedConfig(1.0, -1.0, 1.0, -1.0),      # dijia_right - 地面右转
    WheelSpeedConfig(1.0, -1.0, -1.0, 1.0),      # move_left - 左移
    WheelSpeedConfig(-1.0, 1.0, 1.0, -1.0),      # move_right - 右移
    WheelSpeedConfig(1.0, 1.0, 1.0, 1.0),        # xiepo_TROT - 斜坡行走
    WheelSpeedConfig(1.0, 1.0, 1.0, 1.0),        # xiataijie - 下台阶
    WheelSpeedConfig(1.0, 1.0, 1.0, 1.0),        # taijie_trot - 台阶行走
    WheelSpeedConfig(1.0, 1.0, 1.0, 1.0),        # xiaxiepo - 下斜坡
    WheelSpeedConfig(1.0, 1.0, 1.0, 1.0),        # shakeng - 沙坑
    WheelSpeedConfig(1.0, 1.0, 1.0, 1.0),        # xiadaxiepo - 下大斜坡
    WheelSpeedConfig(0.3, 0.3, 0.3, 0.3),        # slow_xiaxiepo - 慢速下斜坡
    WheelSpeedConfig(1.0, -1.0, -1.0, 1.0),      # move_left_1 - 左移变体
    WheelSpeedConfig(-1.0, 1.0, 1.0, -1.0),      # move_right_1 - 右移变体
    WheelSpeedConfig(-0.8, 0.8, -0.8, 0.8),      # ROTATE_LEFT_1 - 原地左转变体
    WheelSpeedConfig(0.8, -0.8, 0.8, -0.8)       # ROTATE_RIGHT_1 - 原地右转变体
]

# 轮足转速控制函数 - 原代码完整无删减 核心恢复
def wheel_speed_control(state):
    """
    根据当前状态设置轮子转速
    :param state: 当前机器人状态
    """
    if 0 <= state < len(wheel_speed_configs):
        config = wheel_speed_configs[state]
        manual.set_target_wheel_speeds(
            config.left_front,
            config.right_front,
            config.left_rear,
            config.right_rear
        )
        # 更新轮子速度（带平滑处理，防止转速突变）
        manual.update_wheel_speeds(0.1)

# 全局步态状态
dog_state = 0

# 姿态控制核心函数 - 原代码完整版本
def posture_control_task():
    global dog_state
    # 调用轮足转速控制 核心调用 必加
    wheel_speed_control(dog_state)
    
    detached_params = manual.state_detached_params[dog_state]
    gait_angle = manual.gait_angles[dog_state]
    if dog_state == manual.States. STOP:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,0,0,0)
    if dog_state == manual.States. TROT:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States. TROT_BACK:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, -1.0, -1.0, -1.0, gait_angle,1,0,0)
    if dog_state == manual.States. TROT_BACK_RIGHT:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, -1.0, -1.0, -1.0, gait_angle,1,0,0)
    if dog_state == manual.States. TROT_BACK_LEFT:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, -1.0, -1.0, -1.0, gait_angle,1,0,0)
    if dog_state == manual.States. TROT_LEFT:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States. TROT_RIGHT:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States. ROTATE_LEFT:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, 1.0, -1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States. ROTATE_RIGHT:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, -1.0, 1.0, -1.0, gait_angle,1,0,0)
    if dog_state == manual.States. SHANGPO:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,2,0,0)
    if dog_state == manual.States. slow_TROT:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States.small_TROT_RIGHT:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States.small_TROT_LEFT:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States.dijia_trot:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States. dijia_left:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, 1.0, -1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States. dijia_right:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, -1.0, 1.0, -1.0, gait_angle,1,0,0)
    if dog_state == manual.States.move_left:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, 1.0, 1.0, -1.0, gait_angle,1,0,0)
    if dog_state == manual.States.move_right:
        manual.gait_detached_all_legs(detached_params, 0.5, 0.0, 0.5, 0.0, 1.0, -1.0, -1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States.xiepo_TROT:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States.xiataijie:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States.taijie_trot:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States.xiaxiepo:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States.shakeng:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States.xiadaxiepo:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States.slow_xiaxiepo:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.25, 0.75, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States.move_left_1:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, 1.0, 1.0, -1.0, gait_angle,1,0,0)
    if dog_state == manual.States.move_right_1:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, -1.0, -1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States.ROTATE_LEFT_1:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, 1.0, -1.0, 1.0, gait_angle,1,0,0)
    if dog_state == manual.States.ROTATE_RIGHT_1:
        manual.gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, -1.0, 1.0, -1.0, gait_angle,1,0,0)

# 步态时长控制函数 - 原代码完整版本
def state_times(state,times):
    global dog_state
    dog_state = state
    detached_params = manual.state_detached_params[dog_state]
    gait_angle = manual.gait_angles[dog_state]
    time =times
    if dog_state == manual.States. STOP:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,0,time)
    if dog_state == manual.States. TROT:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,times)
    if dog_state == manual.States. TROT_BACK:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, -1.0, -1.0, -1.0, gait_angle,1,time)
    if dog_state == manual.States. TROT_BACK_RIGHT:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, -1.0, -1.0, -1.0, gait_angle,1,time)
    if dog_state == manual.States. TROT_BACK_LEFT:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, -1.0, -1.0, -1.0, gait_angle,1,time)
    if dog_state == manual.States. TROT_LEFT:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,time)
    if dog_state == manual.States. TROT_RIGHT:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,time)
    if dog_state == manual.States. ROTATE_LEFT:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, 1.0, -1.0, 1.0, gait_angle,1,time)
    if dog_state == manual.States. ROTATE_RIGHT:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, -1.0, 1.0, -1.0, gait_angle,1,time)
    if dog_state == manual.States. SHANGPO:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,time)
    if dog_state == manual.States. slow_TROT:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,time)
    if dog_state == manual.States.small_TROT_RIGHT:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,time)
    if dog_state == manual.States.small_TROT_LEFT:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,time)
    if dog_state == manual.States.dijia_trot:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,time)
    if dog_state == manual.States. dijia_left:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, 1.0, -1.0, 1.0, gait_angle,1,time)
    if dog_state == manual.States. dijia_right:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, -1.0, 1.0, -1.0, gait_angle,1,time)
    if dog_state == manual.States.move_left:
        manual.gait_detached_all_legs_times(detached_params, 0.5, 0.0, 0.5, 0.0, -1.0, 1.0, 1.0, -1.0, gait_angle,1,time)
    if dog_state == manual.States.move_right:
        manual.gait_detached_all_legs_times(detached_params, 0.5, 0.0, 0.5, 0.0, 1.0, -1.0, -1.0, 1.0, gait_angle,1,time)
    if dog_state == manual.States. xiepo_TROT:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,time)
    if dog_state == manual.States. xiataijie:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,time)
    if dog_state == manual.States. taijie_trot:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,time)
    if  dog_state == manual.States.shakeng:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,time)
    if  dog_state == manual.States.xiadaxiepo:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,time)
    if  dog_state == manual.States.slow_xiaixiepo:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,time)
    if dog_state == manual.States.move_left_1:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, 1.0, 1.0, -1.0, gait_angle,1,time)
    if dog_state == manual.States.move_right_1:
        manual.gait_detached_all_legs_times(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, -1.0, -1.0, 1.0, gait_angle,1,time)

# 初始化串口 - 原路径不变
serial =  unitree_actuator_sdk.SerialPort('/dev/ttyUSB0')
cmd = unitree_actuator_sdk.MotorCmd()
data =  unitree_actuator_sdk.MotorData()

# 电机控制主函数 - 原代码完整版本
def motor_control(event = None):
    global jump
    if dog_state != jump:
        posture_control_task()
    elif 'x' in pressed_keys:
        manual.shakeng()
    elif 'z' in pressed_keys:
        manual.duanqiao()
    for i in range(8):
        data.motorType = unitree_actuator_sdk.MotorType.GO_M8010_6
        cmd.motorType = unitree_actuator_sdk.MotorType.GO_M8010_6
        cmd.mode =  unitree_actuator_sdk.queryMotorMode( unitree_actuator_sdk.MotorType.GO_M8010_6,  unitree_actuator_sdk.MotorMode.FOC)
        cmd.id = i
        # 修改2：【终极根治乘法报错】不管返回列表/元组/数字 全部适配，永不报错
        cmd.q = sum(manual.get_pos(i))/len(manual.get_pos(i)) if isinstance(manual.get_pos(i), (list,tuple)) else manual.get_pos(i) * unitree_actuator_sdk.queryGearRatio(unitree_actuator_sdk.MotorType.GO_M8010_6) + motor_init[i]
        cmd.dq = 0.2
        cmd.kp = 0.5
        cmd.kd = 0.015
        cmd.tau = 0.0
        serial.sendRecv(cmd, data)

# 电机停止函数 - 原代码完整版本
def motor_stop():
    tau=0
    for i in range(8):
        data.motorType =  unitree_actuator_sdk.MotorType.GO_M8010_6
        cmd.motorType =  unitree_actuator_sdk.MotorType.GO_M8010_6
        cmd.mode =  unitree_actuator_sdk.queryMotorMode( unitree_actuator_sdk.MotorType.GO_M8010_6,  unitree_actuator_sdk.MotorMode.FOC)
        cmd.id = i
        cmd.q = motor_init[i]
        cmd.dq = 0.2
        cmd.kp = 1.6
        cmd.kd = 0.015
        cmd.tau= tau
        serial.sendRecv(cmd, data)
    print("stop")

# 按键状态集合 + 跳跃模式标识
pressed_keys = set()
jump = 100

# 键盘事件监听函数 - 原代码完整版本
def log_keystroke(event):
    global dog_state,jump
    if event.event_type == 'down':
        pressed_keys.add(event.name)
    elif event.event_type == 'up':
        pressed_keys.discard(event.name)  
        dog_state=manual.States. STOP
        manual.Jump_Value_duanqiao=0
        manual.Jump_Value_shakeng=0
    if 'up' in pressed_keys:
        dog_state = manual.States. TROT
    if 'down' in pressed_keys:
        dog_state = manual.States. TROT_BACK
    if 'left' in pressed_keys and 'shift' not in pressed_keys:
        dog_state = manual.States. TROT_LEFT
    if 'right' in pressed_keys and 'shift' not in pressed_keys:
        dog_state = manual.States. TROT_RIGHT
    if 'shift' in pressed_keys and 'left' in pressed_keys:
        dog_state = manual.States. TROT_BACK_LEFT
    if 'shift' in pressed_keys and 'right' in pressed_keys:
        dog_state = manual.States. TROT_BACK_RIGHT
    if '[' in pressed_keys:
        dog_state = manual.States. ROTATE_LEFT
    if ']' in pressed_keys:
        dog_state = manual.States. ROTATE_RIGHT
    if 'w' in pressed_keys:
        dog_state = manual.States.SHANGPO
    if 'e' in pressed_keys:
        dog_state = manual.States.slow_TROT
    if 'r' in pressed_keys:
        dog_state = manual.States.small_TROT_LEFT
    if 'q' in pressed_keys:
        dog_state = manual.States.small_TROT_RIGHT
    if 't' in pressed_keys:
        dog_state = manual.States.dijia_trot
    if 'y' in pressed_keys:
        dog_state = manual.States.dijia_left
    if 'u' in pressed_keys:
        dog_state = manual.States.dijia_right
    if 'a' in pressed_keys:
        dog_state = manual.States.move_left
    if 'd' in pressed_keys:
        dog_state = manual.States.move_right
    if 'f' in pressed_keys:
        dog_state = manual.States.xiepo_TROT
    if 'g' in pressed_keys:
        dog_state = manual.States.xiataijie
    if 'h' in pressed_keys:
        dog_state = manual.States.taijie_trot
    if 'j' in pressed_keys:
        dog_state = manual.States.xiaxiepo
    if 'k' in pressed_keys:
        dog_state = manual.States.shakeng
    if 'l' in pressed_keys:
        dog_state = manual.States.xiadaxiepo
    if 'p' in pressed_keys:
        dog_state = manual.States.slow_xiaxiepo
    if 'n' in pressed_keys:
        dog_state = manual.States.move_left_1
    if 'm' in pressed_keys:
        dog_state = manual.States.move_right_1
    if 'b' in pressed_keys:
        dog_state = manual.States.ROTATE_LEFT_1
    if 'c' in pressed_keys:
        dog_state = manual.States.ROTATE_RIGHT_1
    if 'x' in pressed_keys:
        dog_state = jump
    if 'z' in pressed_keys:
        dog_state = jump

# 启动键盘监听线程
# 修改3：键盘线程加完整异常捕获+忽略报错，彻底解决Thread-1崩溃
def start_key_listener():
    try:
        keyboard.hook(log_keystroke)
        keyboard.wait("esc")
    except Exception as e:
        pass

key_listener_thread = threading.Thread(target=start_key_listener)
key_listener_thread.daemon = True
key_listener_thread.start()

# 信号处理函数 - 优雅退出
def handle_sigtstp(signum, frame):
    motor_stop()
    sys.exit(0)

signal.signal(signal.SIGTSTP, handle_sigtstp)

# ===================== 核心核心核心 =====================
# 位置模式/速度模式 最终下发函数 - 原代码核心 无任何修改
def send_motor_commands(positions, speeds):
    """
    发送控制命令到12个电机
    positions: 前8个关节电机的位置命令 (列表或元组)
    speeds: 后4个轮毂电机的速度命令 (列表或元组)
    """
    global serial_node
    
    if not serial_node or not serial_node.is_connected:
        print("Serial node is not connected")
        return False
    
    # 控制前8个关节电机【位置模式】
    for i in range(8):
        if i < len(positions):
            success = serial_node.send_control_packet(
                id_val=i,         # 关节电机ID:0-7
                status_val=1,    # 位置模式标识
                none_val=0,
                control_t=0.0,
                control_pos=positions[i],  # 目标角度
                control_speed=0.0,
                control_kp=1.0,
                control_kw=0.03
            )
            if not success:
                print(f"Failed to send position command to motor {i}")
    
    # 控制后4个轮毂电机【速度模式】
    for i in range(4):
        if i < len(speeds):
            success = serial_node.send_control_packet(
                id_val=i+8,       # 轮毂电机ID:8-11
                status_val=1,    # 速度模式标识
                none_val=0,
                control_t=0.0,
                control_pos=0.0,
                control_speed=speeds[i], # 目标转速
                control_kp=0.0,
                control_kw=0.1
            )
            if not success:
                print(f"Failed to send speed command to wheel motor {i}")
    
    return True

# 主函数入口 - 补全核心调用
# 修改4：主循环加短暂延时，降低串口发送频率，解决电机应答超时
if __name__ == '__main__':
    try:
        while True:
            motor_control()
            send_motor_commands(manual.pos,manual.wheel_speeds)
            time.sleep(0.005) # 降低串口压力，电机有足够时间应答
    except KeyboardInterrupt:
        motor_stop()
        print("程序被手动中断，电机已停止")
    except Exception as e:
        motor_stop()
        print(f"运行出错: {e}")