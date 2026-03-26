#! /usr/bin/env python
import math
import time

ReductionAndAngleRatio=1#减速和角度比   
x=[0]*4  #足端x坐标
y=[0]*4  #足端y坐标
L1 =19.96	#L1 大腿长  cm
L2=20 #L2 小腿长  cm
theta1=[0]*4   #步态解算的角度
theta2=[0]*4   #步态解算的角度
pos=[0]*8      #电机旋转圈数
prev_t =0     #上次足端轨迹规划的时间
Jump_Value=0  #记录跳跃时间

Gait_Value=0   #记录每次足端轨迹规划的时间

#腿部激活标志位  1：允许运动 0：不允许运动
_leg_active = [1, 1, 1, 1]

# 定义 pos 数据
pos = [0] * 8  # 电机角度
# 定义一个函数来获取 pos 的全部数据（未启用）
def get_pos_1():
    return pos

# 定义一个函数来获取 pos 的某个元素,用在步态发布函数中
def get_pos(index):
    if 0 <= index < len(pos):
        return pos[index]
    else:
        raise IndexError("索引超出范围")
# 轮子速度相关变量
target_wheel_speeds = [0.0, 0.0, 0.0, 0.0]  # 目标轮子速度 [左前, 右前, 左后, 右后]
wheel_speeds = [0.0, 0.0, 0.0, 0.0]  # 当前轮子速度

def set_target_wheel_speeds(left_front, right_front, left_rear, right_rear):
    """
    设置目标轮子速度
    :param left_front: 左前轮速度
    :param right_front: 右前轮速度
    :param left_rear: 左后轮速度
    :param right_rear: 右后轮速度
    """
    target_wheel_speeds[0] = left_front
    target_wheel_speeds[1] = right_front
    target_wheel_speeds[2] = left_rear
    target_wheel_speeds[3] = right_rear

def update_wheel_speeds(smooth_factor):
    """
    更新轮子速度，应用平滑因子使速度变化更加平稳
    :param smooth_factor: 平滑因子，值越小变化越平缓
    """
    for i in range(4):
        # 使用线性插值实现平滑的速度过渡
        wheel_speeds[i] = wheel_speeds[i] + (target_wheel_speeds[i] - wheel_speeds[i]) * smooth_factor





# 定义一个类来存储机器人的步态角度信息
class GaitAngle:
    def __init__(self, gait_angle_leg1_0: float, gait_angle_leg1_1: float,gait_angle_leg2_2: float,gait_angle_leg2_3: float, gait_angle_leg3_4: float,gait_angle_leg3_5: float, gait_angle_leg4_6: float,gait_angle_leg4_7: float):
        self.gait_angle_leg1_0 = gait_angle_leg1_0   # 第一条腿的步态角度   
        self.gait_angle_leg1_1 = gait_angle_leg1_1 
        self.gait_angle_leg2_2 = gait_angle_leg2_2  # 第二条腿的步态角度
        self.gait_angle_leg2_3 = gait_angle_leg2_3
        self.gait_angle_leg3_4 = gait_angle_leg3_4  # 第三条腿的步态角度
        self.gait_angle_leg3_5 = gait_angle_leg3_5 
        self.gait_angle_leg4_6 = gait_angle_leg4_6  # 第四条腿的步态角度
        self.gait_angle_leg4_7 = gait_angle_leg4_7 

# 这里可以耦合八条腿各自的角度
gait_angles = [
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),   #stop
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  # TROT 
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  # TROT_BACK 
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  # TROT_BACK_RIGHT 
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  # TROT_BACK_LEFT
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  # TROT_LEFT
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),   # TROT_RIGHT
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),   # ROTATE_LEFT
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),   # ROTATE_RIGHTs
    GaitAngle(0, 0.0, 0, 0.0,0, 0.0, 0, 0.0),#shangpo
    #GaitAngle(0.0, 0.0, 0.0, 0.0,0.0,0.0, 0.0, 0.0),  # shangpo
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  # slow_TROT 
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  # small_TROT_LEFT
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  # small_TROT_RIGHT
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  #dijia_trot
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  #dijia_left
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  #dijia_right
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  #move_left
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  #move_right
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  #xiepo_trotwwwwwww
    GaitAngle(15.0,-15.0, 15.0, -15.0,-15.0, 15.0, -15.0, 15.0),  #xiataijie
    GaitAngle(30.0,30.0, 30.0, 30.0, -30.0, -30.0, -30.0,-30.0),  #taijie_trot
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  #xiaxiepo
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0,0.0, 0.0, 0.0),#shakeng
    GaitAngle(15.0,-15.0, 15.0, -15.0,-15.0, 15.0, -15.0, 15.0),#xiadaxiepo
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  #xiaxiepo
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  #move_left_1
    GaitAngle(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0),  #move_right_1
   GaitAngle(15.0,-15.0, 15.0, -15.0,-15.0, 15.0, -15.0, 15.0),   # ROTATE_LEFT_1
   GaitAngle(15.0,-15.0, 15.0, -15.0,-15.0, 15.0, -15.0, 15.0),   # ROTATE_RIGHT_1
]


class States:      #状态枚举（用于步态参数的调用）
    STOP = 0                 # 停止
    TROT = 1                  # 疾步
    TROT_BACK = 2              # 后退疾步
    TROT_BACK_RIGHT = 3       # 后退右转
    TROT_BACK_LEFT = 4      # 后退左转
    TROT_LEFT = 5           # 左转
    TROT_RIGHT = 6           # 右转
    ROTATE_LEFT = 7          # 原地左转
    ROTATE_RIGHT = 8         # 原地右转
    TROT_WHEEL = 9
    slow_TROT = 10
    small_TROT_LEFT = 11
    small_TROT_RIGHT = 12
    dijia_trot = 13
    dijia_left = 14
    dijia_right = 15
    move_left = 16
    move_right = 17
    xiepo_TROT  = 18
    xiataijie = 19
    taijie_trot = 20
    xiaxiepo = 21
    shakeng = 22
    xiadaxiepo = 23
    slow_xiaxiepo = 24
    move_left_1 = 25
    move_right_1 = 26
    ROTATE_LEFT_1= 27
    ROTATE_RIGHT_1= 28
   

class jump_state:
    jump_taijie_1 = 0
    jump_taijie_2 = 1
    jump_gaoqinag = 2


class JumpTime:
    def __init__(self, prep_time, launch_time, forward_time, fall_time, land_time):
        self.prep_time = prep_time
        self.launch_time = launch_time
        self.forward_time = forward_time
        self.fall_time = fall_time
        self.land_time = land_time

class JumpLength:
    def __init__(self, stance_height, jump_extension, forward_extension, fall_extension):
        self.stance_height = stance_height
        self.jump_extension = jump_extension
        self.forward_extension = forward_extension
        self.fall_extension = fall_extension

class JumpAngle:
    def __init__(self, prep_angle, jump_angle, forward_angle):
        self.prep_angle = prep_angle
        self.jump_angle = jump_angle
        self.forward_angle = forward_angle

class JumpAngleOffsetPeriod:
    def __init__(self, step1, step2, step3, step4):
        self.step1 = step1
        self.step2 = step2
        self.step3 = step3
        self.step4 = step4
jump_times = [             #prep_time, launch_time, forward_time, fall_time, land_time
    JumpTime(0.1, 0.025, 0.1, 0.15, 0.10),   #jump_taijie_1
    JumpTime(0.1, 0.025, 0.05, 0.15, 0.050),   #jump_taijie_2
    JumpTime(0.25, 0.025, 0.15, 0.45, 0.050),    #jump_xiataijie_1
    JumpTime(0.15, 0.025, 0.15, 0.45, 0.050),    #jump_xiataijie_2
    JumpTime(0.25, 0.025, 0.15, 0.25, 0.050),    #duanqiao_1
    JumpTime(0.25, 0.025, 0.15, 0.25, 0.050),    #duanqiao_2
    JumpTime(0.25, 0.025, 0.2, 0.25, 0.050),    #xiepo
    JumpTime(0.25, 0.025, 0.15, 0.25, 0.050),   #shangxiepo_1
    JumpTime(0.25, 0.025, 0.15, 0.25, 0.050),   #shangxiepo_2
    JumpTime(0.35, 0.025, 0.15, 0.25, 0.020),   #xiaxiepo
    JumpTime(0.15, 0.025, 0.15, 0.1 ,0.015),    #shakeng
    #JumpTime(0.25, 0.025, 0.15, 0.25, 0.050)    #原地跳跃
]

jump_lengths = [     
            #stance_height, jump_extension, forward_extension, fall_extension
    JumpLength(21.6, 40.0, 20.0, 20.0),     #jump_taijie_1
    JumpLength(21.6, 40.0, 15.0, 21.6),     #jump_taijie_2
    JumpLength(21.6, 35.0, 15.0, 21.6),     #jump_xiataijie_1
    JumpLength(21.6, 30.0, 15.0, 21.6),     #jump_xiataijie_2
    JumpLength(21.6, 45.0, 15.0, 21.6),     #duanqiao_1
    JumpLength(21.6, 45.0, 15.0, 21.6),     #duanqiao_2
    JumpLength(21.6, 45.0, 20.0, 20.0),     #xiepo
    JumpLength(21.6, 45.0, 20.0, 20.0),     #shangxiepo_1
    JumpLength(21.6, 45.0, 20.0, 21.6),     #shangxiepo_2
    JumpLength(21.6, 45.0, 20.0, 21.6),     #xiaxiepo
    JumpLength(21.6, 60.0, 15.0, 21.6)      #shakeng
    #JumpLength(21.6, 40.0, 21.6, 21.6),    #原地跳跃
]


jump_angles = [            #prep_angle, jump_angle, forward_angle
    JumpAngle(20.0, 50.0, 15.0),       #jump_taijie_1 
    JumpAngle(27.0, 40.0, 15.0),       #jump_taijie_2
    JumpAngle(20.0, 55.0, 10.0),       #jump_xiataijie_1
    JumpAngle(20.0, 45.0, 10.0),       #jump_xiataijie_2
    JumpAngle(20.0, 45.0, 10.0),       #duanqiao_1
    JumpAngle(20.0, 45.0, 10.0),       #duanqiao_2
    JumpAngle(20.0, 50.0, 15.0),       #xiepo
    JumpAngle(35.0, 60.0, -10.0),      #shangxiepo_1
    JumpAngle(35.0, 60.0, 30.0),       #shangxiepo_2
    JumpAngle(30.0, 30.0, 25.0),       #xiaxiepo
    JumpAngle(30.0, 30.0, 10.0)        #shakeng
    #JumpAngle(0.0, 0.0, 0.0),    #原地跳跃
]
state_jumpangle_offset_period = [
    JumpAngleOffsetPeriod([35,35,35,35],                    #jump_taijie_1
                          [0.0, 0.0, -20.0, -20.0],
                          [30.0, 30.0, 30.0, 30.0],
                          [30.0, 30.0, 0.0, 0.0]), 
    JumpAngleOffsetPeriod([75,75,35,35], 
                          [-40.0, -40.0,-20.0, -20.0],
                          [0.0, 0.0,0.0, 0.0],
                          [0.0, 0.0, 0.0, 0.0]),
    JumpAngleOffsetPeriod([23.0,23.0,23.0,23.0],            #jump_xiataijie_1
                          [-10.0,-10.0, 5.0, 5.0],
                          [-45.0, -45.0, 30.0, 30.0],
                          [-15.0, -15.0, 40.0, 40.0]),
    JumpAngleOffsetPeriod([25.0,25.0,70.0,70.0], 
                          [-20.0, -20.0, 5.0, 5.0],
                          [0.0, 0.0, 0.0, 0.0],
                          [0.0, 0.0, 0.0, 0.0]),
    JumpAngleOffsetPeriod([23.0,23.0,23.0,23.0],                 #duanqiao_1
                          [-10.0,-10.0, 10.0, 10.0],
                          [30.0, 30.0, -35.0, -35.0],
                          [30.0, 30.0, -45.0, -45.0]),
    JumpAngleOffsetPeriod([65.0,65.0,0.0,0.0], 
                          [10.0, 10.0, -10.0, -10.0],
                          [0.0, 0.0, 0.0, 0.0],
                          [0.0, 0.0, 0.0, 0.0]),
    JumpAngleOffsetPeriod([50,50,50,50],            #xiepo        
                          [0.0, 0.0, -20.0, -20.0],
                          [0.0, 0.0, 0.0, 0.0],
                          [0.0, 0.0, 0.0, 0.0]), 
    JumpAngleOffsetPeriod([62,62,62,62],                    #shangxiepo_1
                          [0.0, 0.0, -20.0, -20.0],
                          [30.0, 30.0, -10.0, -10.0],
                          [40.0, 40.0, 0.0, 0.0]),  
    JumpAngleOffsetPeriod([75,75,30,30], 
                          [0.0, 0.0,-10.0, -10.0],
                          [30.0, 30.0, 30.0, 30.0],
                          [0.0, 0.0, 0.0, 0.0]),
    JumpAngleOffsetPeriod([0,0,0,0],                #xiaxiepo
                          [-10.0, 60.0,-10.0,60.0],
                          [30.0, 30.0, 30.0, 30.0],
                          [0.0, 0.0, 0.0, 0.0]),
    JumpAngleOffsetPeriod([30.0,30.0,30.0,30.0],                 #shakeng
                          [-10.0,-10.0, 10.0, 10.0],
                          [-35.0, -35.0, -35.0, -35.0],
                          [30.0, 30.0, -45.0, -45.0]),
    # JumpAngleOffsetPeriod([35,35,35,35],     #原地跳跃
    #                       [40.0, 40.0,40.0, 40.0],
    #                       [0.0, 0.0, 0.0, 0.0],
    #                       [0.0, 0.0, 0.0, 0.0]),
]

def command_all_legs_v_jump(jump_angle_offset_period_s):
    leg1_angle = jump_angle_offset_period_s[0]
    leg2_angle = jump_angle_offset_period_s[1]
    leg3_angle = jump_angle_offset_period_s[2]
    leg4_angle = jump_angle_offset_period_s[3]
    
    set_all_legs_coupled_position(0, leg1_angle, leg1_angle,leg2_angle, leg2_angle,leg3_angle,leg3_angle, leg4_angle,leg4_angle)
    set_all_legs_coupled_position(1, leg1_angle, leg1_angle,leg2_angle, leg2_angle,leg3_angle,leg3_angle, leg4_angle,leg4_angle)
    set_all_legs_coupled_position(2, leg1_angle, leg1_angle,leg2_angle, leg2_angle,leg3_angle,leg3_angle, leg4_angle,leg4_angle)
    set_all_legs_coupled_position(3, leg1_angle, leg1_angle,leg2_angle, leg2_angle,leg3_angle,leg3_angle, leg4_angle,leg4_angle)
def execute_jump(jump_time_s, jump_length_s, jump_angle_s, jump_angle_offset_s):
    global dog_state
    global Jump_Value
    global jump_count
    prep_time = jump_time_s.prep_time#准备时间
    launch_time = jump_time_s.launch_time#起飞时间
    forward_time = jump_time_s.forward_time#前进时间
    fall_time = jump_time_s.fall_time#降落时间
    land_time = jump_time_s.land_time#着地时间

    stance_height = jump_length_s.stance_height#机身高度
    jump_extension = jump_length_s.jump_extension#跳跃伸长度
    forward_extension = jump_length_s.forward_extension#前进伸长度

    prep_angle = jump_angle_s.prep_angle#准备角
    jump_angle = jump_angle_s.jump_angle#跳跃角
    forward_angle = jump_angle_s.forward_angle#前进角

    t = Jump_Value / 1000.0
    Jump_Value += 1
    if t<=prep_time:
        # print("prep")
        for i in range(4):
            x[i] = -stance_height * math.sin(prep_angle * math.pi / 180)
            y[i] = stance_height * math.cos(prep_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0) 
        command_all_legs_v_jump(jump_angle_offset_s.step1)
        if jump_count == 0:
            slow_pre()
        if jump_count == 1:
            pass
    elif prep_time < t < prep_time + launch_time:
        # print("launch")
        for i in range(4):
            x[i] = -jump_extension * math.sin(jump_angle * math.pi / 180)
            y[i] =  jump_extension * math.cos(jump_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(jump_angle_offset_s.step2)
    elif prep_time + launch_time <= t < prep_time + launch_time + forward_time:
        # print("forward")
        for i in range(4):
            x[i] = forward_extension * math.sin(forward_angle * math.pi / 180)
            y[i] = forward_extension * math.cos(forward_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(jump_angle_offset_s.step3)
        slow_text()
    elif prep_time + launch_time + forward_time <= t < prep_time + launch_time + forward_time +fall_time:  # jump_up_adjust_time is 0.0
        #print("fall")
        if jump_count == 0:
            for i in range(4):
                x[i] = forward_extension * math.sin(forward_angle * math.pi / 180)
                y[i] = forward_extension * math.cos(forward_angle * math.pi / 180)
            for i in range(4):
                cartesian_to_theta(i, 1.0)
            command_all_legs_v_jump(jump_angle_offset_s.step4)
            slow_pre()
        else:
            slow_standup_1()
    else:
        Jump_Value = 0
        jump_count += 1

Jump_Value_shakeng = 0 
jump_shankeng_over = 0
def execute_jump_shakeng(jump_time_s, jump_length_s, jump_angle_s, jump_angle_offset_s):
    global dog_state
    global Jump_Value_shakeng
    global jump_shankeng_over
    prep_time = jump_time_s.prep_time#准备时间
    launch_time = jump_time_s.launch_time#起飞时间
    forward_time = jump_time_s.forward_time#前进时间
    fall_time = jump_time_s.fall_time#降落时间
    land_time = jump_time_s.land_time#着地时间

    stance_height = jump_length_s.stance_height#机身高度
    jump_extension = jump_length_s.jump_extension#跳跃伸长度
    forward_extension = jump_length_s.forward_extension#前进伸长度

    prep_angle = jump_angle_s.prep_angle#准备角
    jump_angle = jump_angle_s.jump_angle#跳跃角
    forward_angle = jump_angle_s.forward_angle#前进角

    t = Jump_Value_shakeng / 1000.0
    Jump_Value_shakeng += 1
    if t<=prep_time:
        #print("jump_prep",t)
        for i in range(4):
            x[i] = -stance_height * math.sin(prep_angle * math.pi / 180)
            y[i] = stance_height * math.cos(prep_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0) 
        command_all_legs_v_jump(jump_angle_offset_s.step1)
        #slow_pre()
    elif prep_time < t < prep_time + launch_time:
        #print("jump_launch",t)
        for i in range(4):
            x[i] = -jump_extension * math.sin(jump_angle * math.pi / 180)
            y[i] =  jump_extension * math.cos(jump_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(jump_angle_offset_s.step2)
    elif prep_time + launch_time <= t < prep_time + launch_time + forward_time:
        #print("jump_forward",t)
        for i in range(4):
            x[i] = forward_extension * math.sin(forward_angle * math.pi / 180)
            y[i] = forward_extension * math.cos(forward_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(jump_angle_offset_s.step3)
        slow_text()
    elif prep_time + launch_time + forward_time <= t < prep_time + launch_time + forward_time +fall_time:  # jump_up_adjust_time is 0.0
        #print("jump_fall",t)
        slow_standup_1()
    else:
        #print("jump_over",t)
        Jump_Value_shakeng = 0
        jump_shankeng_over += 1

Jump_Value_daxiepo = 0
jump_count_daxiepo = 0
def execute_jump_daxiepo(jump_time_s, jump_length_s, jump_angle_s, jump_angle_offset_s):
    global dog_state
    global Jump_Value_daxiepo
    global jump_count_daxiepo,daxiepo_over
    prep_time = jump_time_s.prep_time#准备时间
    launch_time = jump_time_s.launch_time#起飞时间
    forward_time = jump_time_s.forward_time#前进时间
    fall_time = jump_time_s.fall_time#降落时间
    land_time = jump_time_s.land_time#着地时间

    stance_height = jump_length_s.stance_height#机身高度
    jump_extension = jump_length_s.jump_extension#跳跃伸长度
    forward_extension = jump_length_s.forward_extension#前进伸长度

    prep_angle = jump_angle_s.prep_angle#准备角
    jump_angle = jump_angle_s.jump_angle#跳跃角
    forward_angle = jump_angle_s.forward_angle#前进角

    t = Jump_Value_daxiepo / 1000.0
    Jump_Value_daxiepo += 1
    if t<=prep_time:
        # print("prep")
        for i in range(4):
            x[i] = -stance_height * math.sin(prep_angle * math.pi / 180)
            y[i] = stance_height * math.cos(prep_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0) 
        command_all_legs_v_jump(jump_angle_offset_s.step1)
        if jump_count_daxiepo == 0:
            slow_pre()
        if jump_count_daxiepo == 2:
            pass
    elif prep_time < t < prep_time + launch_time:
        # print("launch")
        for i in range(4):
            x[i] = -jump_extension * math.sin(jump_angle * math.pi / 180)
            y[i] =  jump_extension * math.cos(jump_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(jump_angle_offset_s.step2)
    elif prep_time + launch_time <= t < prep_time + launch_time + forward_time:
        # print("forward")
        for i in range(4):
            x[i] = forward_extension * math.sin(forward_angle * math.pi / 180)
            y[i] = forward_extension * math.cos(forward_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(jump_angle_offset_s.step3)
        slow_text()
    elif prep_time + launch_time + forward_time <= t < prep_time + launch_time + forward_time +fall_time:  # jump_up_adjust_time is 0.0
        #print("fall")
        if jump_count_daxiepo == 0:
            for i in range(4):
                x[i] = forward_extension * math.sin(forward_angle * math.pi / 180)
                y[i] = forward_extension * math.cos(forward_angle * math.pi / 180)
            for i in range(4):
                cartesian_to_theta(i, 1.0)
            command_all_legs_v_jump(jump_angle_offset_s.step4)
            slow_pre()
        else:
            slow_standup_1()
    else:
        if jump_count_daxiepo == 0:
            jump_count_daxiepo += 1
        elif jump_count_daxiepo == 2:
            daxiepo_over +=  1

def execute_jump_taijie(jump_time_s, jump_length_s, jump_angle_s, jump_angle_offset_s):
    global jump_count_taijie
    global dog_state
    global Jump_Value
    prep_time = jump_time_s.prep_time#准备时间
    launch_time = jump_time_s.launch_time#起飞时间
    forward_time = jump_time_s.forward_time#前进时间
    fall_time = jump_time_s.fall_time#降落时间
    land_time = jump_time_s.land_time#着地时间

    stance_height = jump_length_s.stance_height#机身高度
    jump_extension = jump_length_s.jump_extension#跳跃伸长度
    forward_extension = jump_length_s.forward_extension#前进伸长度

    prep_angle = jump_angle_s.prep_angle#准备角
    jump_angle = jump_angle_s.jump_angle#跳跃角
    forward_angle = jump_angle_s.forward_angle#前进角
    offest_trot=[30,30,-30,-30]
    t = Jump_Value / 1000.0
    Jump_Value += 1
    if t<=prep_time:
        # print("prep")
        for i in range(4):
            x[i] = -stance_height * math.sin(prep_angle * math.pi / 180)
            y[i] = stance_height * math.cos(prep_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0) 
        command_all_legs_v_jump(jump_angle_offset_s.step1)
        if jump_count_taijie == 0:
            pass
        elif jump_count_taijie ==1:
            #slow_pre()
            pass
    elif prep_time < t < prep_time + launch_time:
        # print("launch")
        for i in range(4):
            x[i] = -jump_extension * math.sin(jump_angle * math.pi / 180)
            y[i] =  jump_extension * math.cos(jump_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(jump_angle_offset_s.step2)
    elif prep_time + launch_time <= t < prep_time + launch_time + forward_time:
        # print("forward")
        for i in range(4):
            x[i] = forward_extension * math.sin(forward_angle * math.pi / 180)
            y[i] = forward_extension * math.cos(forward_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(jump_angle_offset_s.step3)
        slow_text()
    elif prep_time + launch_time + forward_time <= t < prep_time + launch_time + forward_time +fall_time:  # jump_up_adjust_time is 0.0
        #print("fall")
        if jump_count_taijie == 0:
            for i in range(4):
                x[i] =0
                y[i] =stance_height
            for i in range(4):
                cartesian_to_theta(i, 1.0)
            command_all_legs_v_jump(offest_trot)
            slow_pre()
        else:
            slow_standup_1()
    else:
        Jump_Value = 0
        jump_count_taijie +=1

Jump_Value_xiaoxiepo = 0
def execute_jump_xiaxiepo(jump_time_s, jump_length_s, jump_angle_s, jump_angle_offset_s):
    global dog_state
    global Jump_Value_xiaoxiepo
    global jump_count_xiaxiepo
    prep_time = jump_time_s.prep_time#准备时间
    launch_time = jump_time_s.launch_time#起飞时间
    forward_time = jump_time_s.forward_time#前进时间
    fall_time = jump_time_s.fall_time#降落时间
    land_time = jump_time_s.land_time#着地时间

    stance_height = jump_length_s.stance_height#机身高度
    jump_extension = jump_length_s.jump_extension#跳跃伸长度
    forward_extension = jump_length_s.forward_extension#前进伸长度

    prep_angle = jump_angle_s.prep_angle#准备角
    jump_angle = jump_angle_s.jump_angle#跳跃角
    forward_angle = jump_angle_s.forward_angle#前进角
    xiaxiepo_1_offest= [0,0,0,0]

    t = Jump_Value_xiaoxiepo / 1000.0
    Jump_Value_xiaoxiepo += 1
    #print("1111",t)
    if t <=prep_time:
        y_roll=math.tan(10)*33.3/4/2
        y[0] =21.8+ (y_roll)-2
        y[1] =21.8+ (y_roll)
        y[2] =21.8-(y_roll-2)+4
        y[3] =21.8-(y_roll-2+4)+1
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(xiaxiepo_1_offest)
        slow_pre()
    elif prep_time <=t<prep_time+launch_time:
        #print("launch")
        for i in range(4):
            x[i] = -jump_extension * math.sin(jump_angle * math.pi / 180)
            y[i] =  jump_extension * math.cos(jump_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(jump_angle_offset_s.step2)
    elif prep_time + launch_time <= t < prep_time + launch_time + forward_time:
        #print("forward")
        for i in range(4):
            x[i] = forward_extension * math.sin(forward_angle * math.pi / 180)
            y[i] = forward_extension * math.cos(forward_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(jump_angle_offset_s.step3)
        slow_text()
    elif prep_time + launch_time + forward_time <= t < prep_time + launch_time + forward_time +fall_time:  # jump_up_adjust_time is 0.0
        #print("fall")
        slow_standup_1()
    else:
        jump_count_xiaxiepo =1
    # else:
    #     Jump_Value_xiaoxiepo = 0
        



last_pos = [0]*8
def slow_pre():
    global pos, last_pos
    up = 0.025#0.01
    for i in range(8):
        if abs(pos[i] - last_pos[i]) >= up:
            if pos[i] < last_pos[i]:
                pos[i] = last_pos[i]
                last_pos[i] -= up
            elif pos[i] > last_pos[i]:
                pos[i] = last_pos[i]
                last_pos[i] += up
            else:
                pos[i] = last_pos[i]
        else:
            pass
def slow_pre_back():
    global pos, last_pos
    up = 0.01
    for i in range(4,8):
        if abs(pos[i] - last_pos[i]) >= up:
            if pos[i] < last_pos[i]:
                pos[i] = last_pos[i]
                last_pos[i] -= up
            elif pos[i] > last_pos[i]:
                pos[i] = last_pos[i]
                last_pos[i] += up
            else:
                pos[i] = last_pos[i]
        else:
            pass
def slow_text():
    global pos
    for i in range(8):
        last_pos[i] = pos[i]


def slow_standup_1():
    global pos
    fall = 0.015
    for  i in range(8):
        if pos[i] < 0 and abs(pos[i]) >=fall :
            pos[i] += fall
        if pos[i] > 0 and abs(pos[i]) >=fall :
            pos[i] -= fall
        elif abs(pos[i]) <=fall:
            pos[i] =0


stand_up_over = 0
def slow_standup():
    global pos,stand_up_over
    fall = 0.003#0.005
    for  i in range(8):
        #print("1111111111111111i:%f,pos:%f"%(i,pos[i]))
        if pos[i] < 0 and abs(pos[i]) >=fall :
            pos[i] += fall
            stand_up_over  = 1
        elif pos[i] > 0 and abs(pos[i]) >=fall :
            pos[i] -= fall
            stand_up_over = 1
        elif abs(pos[i]) <fall:
            pos[i] =0
    if pos[1]==0 and pos[2]==0 and pos[3]==0 and pos[4] ==0 and pos[5]==0 and pos[6]==0 and pos[7]==0 and pos[0] ==0:
            stand_up_over = 2


def execute_jump_1(jump_time_s, jump_length_s, jump_angle_s, jump_angle_offset_s):
    global dog_state
    global Jump_Value
    global jump_count
    prep_time = jump_time_s.prep_time#准备时间
    launch_time = jump_time_s.launch_time#起飞时间
    forward_time = jump_time_s.forward_time#前进时间
    fall_time = jump_time_s.fall_time#降落时间
    land_time = jump_time_s.land_time#着地时间

    stance_height = jump_length_s.stance_height#机身高度
    jump_extension = jump_length_s.jump_extension#跳跃伸长度
    forward_extension = jump_length_s.forward_extension#前进伸长度

    prep_angle = jump_angle_s.prep_angle#准备角
    jump_angle = jump_angle_s.jump_angle#跳跃角
    forward_angle = jump_angle_s.forward_angle#前进角

    t = Jump_Value / 1000.0
    Jump_Value += 1
    if t < prep_time / 2:
        #print("prep1")
        for i in range(2, 4):#后腿
            x[i] = 0
            y[i] = stance_height
        for i in range(2):#前腿
            x[i] = -stance_height * math.sin(prep_angle * math.pi / 180)
            y[i] = stance_height * math.cos(prep_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)#坐标转为角度
        command_all_legs_v_jump(jump_angle_offset_s.step1)
    elif prep_time / 2 <= t <= prep_time:
        #print("prep2")
        for i in range(2, 4):
            x[i] = -stance_height * math.sin(prep_angle * math.pi / 180)
            y[i] = stance_height * math.cos(prep_angle * math.pi / 180)
        for i in range(2):
            x[i] = -stance_height * math.sin(prep_angle * math.pi / 180)
            y[i] = stance_height * math.cos(prep_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(jump_angle_offset_s.step1)
    elif prep_time < t < prep_time + launch_time:
        #print("launch")
        for i in range(4):
            x[i] = -jump_extension * math.sin(jump_angle * math.pi / 180)
            y[i] =  jump_extension * math.cos(jump_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(jump_angle_offset_s.step2)
    elif prep_time + launch_time <= t < prep_time + launch_time + forward_time:
        #print("forward")
        for i in range(4):
            x[i] = 0
            y[i] = 21.6
        for i in range(4):
            cartesian_to_theta_1(i, 1.0)
        command_all_legs_v_jump(jump_angle_offset_s.step3)
    elif prep_time + launch_time + forward_time <= t < prep_time + launch_time + forward_time +fall_time:  # jump_up_adjust_time is 0.0
        #print("fall")
        for i in range(4):
            x[i] = 0
            y[i] = 21.6
        for i in range(4):
            cartesian_to_theta_1(i, 1.0)
        command_all_legs_v_jump(jump_angle_offset_s.step4)
    else:
        Jump_Value = 0


#666666666666666666666666666666666666666666

def command_all_legs_v_qianyue(jump_angle_offset_period_s,angle):
    leg1_angle = jump_angle_offset_period_s[0]
    leg2_angle = jump_angle_offset_period_s[1]
    leg3_angle = jump_angle_offset_period_s[2]
    leg4_angle = jump_angle_offset_period_s[3]
    
    set_all_legs_coupled_position_qianyue(0, leg1_angle, leg2_angle, leg3_angle, leg4_angle,angle)
    set_all_legs_coupled_position_qianyue(1, leg1_angle, leg2_angle, leg3_angle, leg4_angle,angle)
    set_all_legs_coupled_position_qianyue(2, leg1_angle, leg2_angle, leg3_angle, leg4_angle,angle)
    set_all_legs_coupled_position_qianyue(3, leg1_angle, leg2_angle, leg3_angle, leg4_angle,angle)

def set_all_legs_coupled_position_qianyue(leg_id, leg1_angle, leg2_angle, leg3_angle, leg4_angle,angle):
    """
    设置所有腿的耦合位置

    :param leg_id: 腿的标识符，取值为0（左前腿）、1（右前腿）、2（左后腿）、3（右后腿）
    :param leg1_angle: 第1条腿的目标角度
    :param leg2_angle: 第2条腿的目标角度
    :param leg3_angle: 第3条腿的目标角度
    :param leg4_angle: 第4条腿的目标角度
    :param theta1: 第一组角度数组
    :param theta2: 第二组角度数组
    :param ReductionAndAngleRatio: 角度转换比例
    """

    # 根据腿的ID设置对应腿的目标角度
    if leg_id == 0:  # 左前腿
        temp_pid.ref_agle[0] = theta1[0] * ReductionAndAngleRatio
        temp_pid.ref_agle[1] = theta2[0] * ReductionAndAngleRatio
    elif leg_id == 2:  # 左后腿
        temp_pid.ref_agle[4] = theta1[2] * ReductionAndAngleRatio
        temp_pid.ref_agle[5] = theta2[2] * ReductionAndAngleRatio
    elif leg_id == 1:  # 右前腿
        temp_pid.ref_agle[2] = -theta1[1] * ReductionAndAngleRatio
        temp_pid.ref_agle[3] = -theta2[1] * ReductionAndAngleRatio
    elif leg_id == 3:  # 右后腿
        temp_pid.ref_agle[6] = -theta1[3] * ReductionAndAngleRatio
        temp_pid.ref_agle[7] = -theta2[3] * ReductionAndAngleRatio
    
    #计算每个电机的最终旋转圈数
    pos[0] = deg_2_cir(temp_pid.ref_agle[0])  - deg_2_cir(leg1_angle)#-0.25
    pos[1] = deg_2_cir(temp_pid.ref_agle[1]) - deg_2_cir(leg1_angle)#-0.25
    pos[2] = deg_2_cir(temp_pid.ref_agle[2])  + deg_2_cir(leg2_angle)#+0.25
    pos[3] = deg_2_cir(temp_pid.ref_agle[3])  + deg_2_cir(leg2_angle)#+0.25
    pos[4] = -deg_2_cir(temp_pid.ref_agle[4])  + deg_2_cir(leg3_angle)-deg_2_cir(angle)#+0.25
    pos[5] = -deg_2_cir(temp_pid.ref_agle[5])  + deg_2_cir(leg3_angle)+deg_2_cir(angle)#+0.25
    pos[6] = -deg_2_cir(temp_pid.ref_agle[6])  - deg_2_cir(leg4_angle)+deg_2_cir(angle)#-0.25
    pos[7] = -deg_2_cir(temp_pid.ref_agle[7])  - deg_2_cir(leg4_angle)-deg_2_cir(angle)#-0.25


"""
前跃动作
准备阶段：四条腿收起降低身体，要有一定的角度        jump_count += 1
飞行阶段：前腿伸直，后退旋转到一定角度后甚伸直
前伸阶段：四条腿收起，防止碰到高墙
落地阶段
"""
duanqiao_flag = 0
Jump_Value_duanqiao = 0
sum_backangle_duanqiao = 0
jump_count = 0
def duanqiao():
    global sum_backangle_duanqiao 
    global Jump_Value_duanqiao
    global duanqiao_flag,jump_count
    #time
    prep_time_front = 0.15#0.25
    prep_time_back = 0.15
    prep_time = prep_time_front+prep_time_back
    launch_time = 0.04#这个时间段内可能达不到45度，需要调时间
    forward_time = 0.12
    fall_time = 0.05
    land_time = 0.05
    #stance_height
    stance_height = 21.6
    #prep
    prep_front_angle = 20
    prep_back_angle  = 35
    prep_offset_pront = [20,20,0,0]
    prep_offset_back  = [20,20,70,70]
    #jump
    jump_angle = 20
    jump_extension_front = 30
    jump_extension_back = 90
    
    #froward
    forward_angle = 10
    forward_extension = 15
    forward_offset = [0,0,0,0]#前两个参数是前腿前伸的耦合度，后两个参数是后腿伸直的耦合度
    #跳跃角度
    jump_back_angle = 50

    t = Jump_Value_duanqiao / 1000.0
    Jump_Value_duanqiao += 1
    if t<prep_time_front:
        for i in range(2):#前腿
            x[i] = -stance_height * math.sin(prep_front_angle * math.pi / 180)
            y[i] = stance_height * math.cos(prep_front_angle * math.pi / 180)
        for i in range(2,4):
            #  x[i] = -stance_height * math.sin(prep_back_angle * math.pi / 180)
            #  y[i] = stance_height * math.cos(prep_back_angle * math.pi / 180)
             x[i] = 0
             y[i] = stance_height 
        for i in range(4):
            cartesian_to_theta(i, 1.0)#坐标转为角度
        command_all_legs_v_jump(prep_offset_back)
        slow_pre()
    elif prep_time_front<= t<prep_time_back+prep_time_front:
        for i in range(2):#前腿
            x[i] = -stance_height * math.sin(prep_front_angle * math.pi / 180)
            y[i] = stance_height * math.cos(prep_front_angle * math.pi / 180)
        for i in range(2,4):
             x[i] = -stance_height * math.sin(prep_back_angle * math.pi / 180)
             y[i] = stance_height * math.cos(prep_back_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)#坐标转为角度
        command_all_legs_v_jump(prep_offset_back)
        slow_pre()
    elif prep_time <= t < prep_time + launch_time and sum_backangle_duanqiao<jump_back_angle:
        # print("launch")
        for i in range(2):
            x[i] = -jump_extension_front * math.sin(jump_angle * math.pi / 180)
            y[i] =  jump_extension_front * math.cos(jump_angle * math.pi / 180)
        for i in range(2, 4):
            x[i] = -stance_height * math.sin(prep_back_angle * math.pi / 180)
            y[i] =  stance_height * math.cos(prep_back_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        sum_backangle_duanqiao += 2
        # print(sum_backangle)
        command_all_legs_v_qianyue(prep_offset_back,sum_backangle_duanqiao)
        """  
        这里有个函数可以控制八条腿的角度，sum_backangle最为累加值控制旋转
        加到一定角度之后，后腿伸直
        """
    elif prep_time < t < prep_time + launch_time and sum_backangle_duanqiao>=jump_back_angle:
        # print("shenzhi")
        for i in range(2):
            x[i] = forward_extension * math.sin(forward_angle * math.pi / 180)
            y[i] = forward_extension * math.cos(forward_angle * math.pi / 180)
        for i in range(2, 4):
            x[i] = -jump_extension_back * math.sin((jump_angle+prep_back_angle+jump_back_angle) * math.pi / 180)
            y[i] =  jump_extension_back * math.cos((jump_angle+prep_back_angle+jump_back_angle) * math.pi / 180)
        for i in range(2,4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(forward_offset)
    elif prep_time + launch_time <= t < prep_time + launch_time + forward_time:
        #print("forward")
        print(jump_count)
        if jump_count <15:
            jump_count+= 0#0.20
        for i in range(4):
            x[i] = (forward_extension-jump_count) * math.sin(forward_angle * math.pi / 180)
            y[i] = (forward_extension-jump_count) * math.cos(forward_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(forward_offset)
    elif prep_time + launch_time + forward_time <= t < prep_time + launch_time +forward_time+ fall_time:  # jump_up_adjust_time is 0.0
        slow_standup_1()
    elif t >= prep_time + launch_time +forward_time+ fall_time:#断桥跳完了
        duanqiao_flag = 1
    
    
    
Jump_Value_gaoqiang = 0   
sum_backangle_gaoqiang=0   
gaoqiang_over = 0 
def gaoqiang():
    global sum_backangle_gaoqiang
    global Jump_Value_gaoqiang
    global gaoqiang_over
    #time
    prep_time_front = 0.125#0.25
    prep_time_back = 0.125
    prep_time = prep_time_front+prep_time_back
    launch_time = 0.05#这个时间段内可能达不到45度，需要调时间
    forward_time =0.1
    fall_time = 0.25
    land_time = 0.05
    #stance_height
    stance_height = 21.6
    #prep
    prep_front_angle = 20
    prep_back_angle  = 35
    prep_offset_pront = [0,0,70,70]#20,20,70,70
    prep_offset_back  = [20,20,70,70]
    #jump
    jump_angle = 20
    jump_extension_front = 30
    jump_extension_back = 90
    
    #froward
    forward_angle = -30
    forward_extension= 13.5
    forward_offset = [0,0,0,0]#前两个参数是前腿前伸的耦合度，后两个参数是后腿伸直的耦合度
    forward_offset = [10,10,10,10]
    #跳跃角度
    jump_back_angle = 60  #60
    t = Jump_Value_gaoqiang/ 1000.0
    Jump_Value_gaoqiang+= 1
    if t<prep_time_front:
        # print("prep1")
        for i in range(2, 4):
            x[i] = -stance_height * math.sin(prep_back_angle * math.pi / 180)
            y[i] = stance_height * math.cos(prep_back_angle * math.pi / 180)
        for i in range(2):
            x[i] = 0
            y[i] = stance_height 
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(prep_offset_pront)
        slow_pre()
    if  prep_time_front<t <= prep_time_front+prep_time_back:
        # print("prep2")
        for i in range(2, 4):
            x[i] = -stance_height * math.sin(prep_back_angle * math.pi / 180)
            y[i] = stance_height * math.cos(prep_back_angle * math.pi / 180)
        for i in range(2):
            x[i] = -stance_height * math.sin(prep_front_angle * math.pi / 180)
            y[i] = stance_height * math.cos(prep_back_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(prep_offset_back)
        slow_pre()
    elif prep_time < t < prep_time + launch_time and sum_backangle_gaoqiang<jump_back_angle:
        #print("launch",t)
        for i in range(2):
            x[i] = -jump_extension_front * math.sin(jump_angle * math.pi / 180)
            y[i] =  jump_extension_front * math.cos(jump_angle * math.pi / 180)
        for i in range(2, 4):
            x[i] = -stance_height * math.sin(prep_back_angle * math.pi / 180)
            y[i] =  stance_height * math.cos(prep_back_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        sum_backangle_gaoqiang += 2
        #print(sum_backangle)
        command_all_legs_v_qianyue(prep_offset_back,sum_backangle_gaoqiang)
        """  
        这里有个函数可以控制八条腿的角度，sum_backangle最为累加值控制旋转
        加到一定角度之后，后腿伸直
        """
    elif prep_time < t < prep_time + launch_time and sum_backangle_gaoqiang>=jump_back_angle:
        #print("shenzhi",t)
        for i in range(2):
            x[i] = 0
            y[i] = 13
        for i in range(2, 4):
            x[i] = -jump_extension_back * math.sin((jump_angle+prep_back_angle+jump_back_angle) * math.pi / 180)
            y[i] =  jump_extension_back * math.cos((jump_angle+prep_back_angle+jump_back_angle) * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(forward_offset)
    elif prep_time + launch_time <= t < prep_time + launch_time + forward_time:
        # print("forward")
        for i in range(4):
            x[i] = forward_extension * math.sin(-forward_angle * math.pi / 180)
            y[i] = forward_extension * math.cos(-forward_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(forward_offset)
    elif prep_time + launch_time + forward_time <= t < prep_time + launch_time + forward_time+fall_time:  # jump_up_adjust_time is 0.0
        # print("fall") 
        slow_standup_1() 
    elif t>=prep_time + launch_time + forward_time+fall_time:
        gaoqiang_over = 1      
    
xiaoxiepo_flag = 0
Jump_Value_xiaoxiepo1 = 0
sum_backangle_shangxiepo=0
def shangxiepo():#小斜坡
    global sum_backangle_shangxiepo
    global Jump_Value_xiaoxiepo1
    global xiaoxiepo_flag,dog_state
    #time
    prep_time_front = 0.15#0.25
    prep_time_back = 0.27
    prep_time = (prep_time_front+prep_time_back)/2
    launch_time = 0.05#这个时间段内可能达不到45度，需要调时间
    forward_time = 0.15
    fall_time = 0.25
    land_time = 0.05
    stand_time =0.01
    #stance_height
    stance_height = 21.6
    #prep
    prep_front_angle = 20
    prep_back_angle  = 35
    prep_offset_pront = [20,20,0,0]#20,20,70,70
    prep_offset_back  = [20,20,70,70]
    #jump
    jump_angle = 20
    jump_extension_front = 30
    jump_extension_back = 60#90
    #froward
    forward_angle = 10
    forward_extension = 15
    forward_offset = [0,0,0,0]#前两个参数是前腿前伸的耦合度，后两个参数是后腿伸直的耦合度
    #跳跃角度
    jump_back_angle = 50  #60
    t = Jump_Value_xiaoxiepo1 / 1000.0
    Jump_Value_xiaoxiepo1 += 1
    if t< stand_time:
        dog_state = States.dijia_trot
        posture_control_task() 
        #slow_text()
        #slow_standup_1()
    # if stand_time <=t< prep_time_front+stand_time:
    #     for i in range(2, 4):#后腿
    #         x[i] = 0
    #         y[i] = stance_height
    #     for i in range(2):#前腿
    #         x[i] = -stance_height * math.sin(prep_front_angle * math.pi / 180)
    #         y[i] = stance_height * math.cos(prep_front_angle * math.pi / 180)
    #     for i in range(4):
    #         cartesian_to_theta(i, 1.0)#坐标转为角度
    #     command_all_legs_v_jump(prep_offset_pront)
    #     slow_pre()
    # elif prep_time_front+stand_time<= t <= prep_time_front+prep_time_back+stand_time:
    #     for i in range(2, 4):
    #         #print("prep_2")
    #         x[i] = -stance_height * math.sin(prep_back_angle * math.pi / 180)
    #         y[i] = stance_height * math.cos(prep_back_angle * math.pi / 180)
    #     for i in range(2):
    #         x[i] = -stance_height * math.sin(prep_front_angle * math.pi / 180)
    #         y[i] = stance_height * math.cos(prep_back_angle * math.pi / 180)
    #     for i in range(4):
    #         cartesian_to_theta(i, 1.0)
    #     command_all_legs_v_jump(prep_offset_back)
    #     slow_pre()
    elif stand_time<= t <= prep_time+stand_time:
        for i in range(2, 4):
            #print("prep_2")
            x[i] = -stance_height * math.sin(prep_back_angle * math.pi / 180)
            y[i] = stance_height * math.cos(prep_back_angle * math.pi / 180)
        for i in range(2):
            x[i] = -stance_height * math.sin(prep_front_angle * math.pi / 180)
            y[i] = stance_height * math.cos(prep_back_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(prep_offset_back)
        #slow_pre()
    elif prep_time +stand_time< t < prep_time + launch_time+stand_time and sum_backangle_shangxiepo<jump_back_angle:
        for i in range(2):
            x[i] = -jump_extension_front * math.sin(jump_angle * math.pi / 180)
            y[i] =  jump_extension_front * math.cos(jump_angle * math.pi / 180)
        for i in range(2, 4):
            x[i] = -stance_height * math.sin(prep_back_angle * math.pi / 180)
            y[i] =  stance_height * math.cos(prep_back_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        sum_backangle_shangxiepo += 2
        command_all_legs_v_qianyue(prep_offset_back,sum_backangle_shangxiepo)
        """  
        这里有个函数可以控制八条腿的角度，sum_backangle最为累加值控制旋转
        加到一定角度之后，后腿伸直
        """
    elif prep_time +stand_time< t < prep_time + launch_time +stand_time and sum_backangle_shangxiepo>=jump_back_angle:
        for i in range(2):
            x[i] = forward_extension * math.sin(forward_angle * math.pi / 180)
            y[i] = forward_extension * math.cos(forward_angle * math.pi / 180)
        for i in range(2, 4):
            x[i] = -jump_extension_back * math.sin((jump_angle+prep_back_angle+jump_back_angle) * math.pi / 180)
            y[i] =  jump_extension_back * math.cos((jump_angle+prep_back_angle+jump_back_angle) * math.pi / 180)
        for i in range(2,4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(forward_offset)
    elif prep_time + launch_time +stand_time<= t < prep_time + launch_time + forward_time/2+stand_time:
        for i in range(4):
            x[i] = 0
            y[i] = stance_height
        for i in range(4):
            cartesian_to_theta_1(i, 1.0)
        command_all_legs_v_jump(forward_offset)
    elif prep_time + launch_time + forward_time/2+stand_time <= t < prep_time + launch_time + forward_time+stand_time:
        for i in range(4):
            x[i] = 0
            y[i] = stance_height
        for i in range(4):
            cartesian_to_theta_1(i, 1.0)
        command_all_legs_v_jump(forward_offset)
        slow_text()
    elif prep_time + launch_time + forward_time+stand_time<t<prep_time + launch_time + forward_time+fall_time+stand_time:
        for i in range(4):
            x[i] = 0
            y[i] = stance_height
        for i in range(4):
            cartesian_to_theta_2(i, 1.0)
        command_all_legs_v_jump(forward_offset)
        slow_pre()
    elif prep_time + launch_time + forward_time+fall_time +stand_time< t < prep_time + launch_time + forward_time+fall_time+land_time+stand_time:
        xiaoxiepo_flag = 1


Value_xiaxiepo_1 = 0
xiaxiepo_1_over = 0
def xiaxiepo_1():#跳下小斜坡
    global Value_xiaxiepo_1,dog_state,xiaoxiepo_flag,xiaxiepo_1_over
    xiaxiepo_1_offest = [0,0,0,0]
    xiaxiepo_2_offest=[35,35,55,42]
    prep_time = 0.15
    jump_time = 0.15
    land_time = 0.02
    fall_time = 0.2
    stand_time = 0.1
    t = Value_xiaxiepo_1/ 1000.0
    Value_xiaxiepo_1 += 1
    jump_extension = 38
    jump_angle= -60
    jump_offset= [0,0,0,0]
    forward_extension = 25
    forward_angle= 15
    forward_offset= [0,0,0,0]
    y_roll=math.tan(10)*33.3/4
    if t < prep_time:
        print("perp")
        x[0]= 5
        x[1]= 5
        y[0] =21.8+ (y_roll)+4
        y[1] =21.8+ (y_roll)+4
        y[2] =21.8-(y_roll-2)+8
        y[3] =21.8-(y_roll-2+4)+1
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(xiaxiepo_1_offest)
        slow_pre_back()
    if prep_time<=t<prep_time+jump_time:
        print("prep_2")
        # xiaoxiepo_flag = 2
        # dog_state =States.xiaxiepo
        # params = state_detached_params[dogslow_pre_backstate]
        # gait_angle = gait_angles[dog_state]
        # gait_detached_all_legs(params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
        # y[2] =21.8-(y_roll-2)+7
        # y[3] =21.8-(y_roll-2+4)+1
        # x[2] = 0
        # x[3] = 0
        x[0]= -5
        x[1]= -5
        
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(xiaxiepo_2_offest)
        slow_pre()
    if prep_time+jump_time <= t< prep_time+jump_time+land_time:
        print("land")
        for i in range(2):
            x[i] = jump_extension * math.sin(jump_angle * math.pi / 180)
            y[i] = jump_extension * math.cos(jump_angle * math.pi / 180)
        for i in range(2, 4):
            xiaoxiepo_flag =0
            x[i] = forward_extension * math.sin(forward_angle * math.pi / 180)
            y[i] = forward_extension * math.cos(forward_angle * math.pi / 180) 
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(jump_offset)
    if prep_time+jump_time+land_time<=t<prep_time+jump_time+land_time+fall_time:
        for i in range(2):
            x[i] = forward_extension * math.sin(forward_angle * math.pi / 180)
            y[i] = forward_extension * math.cos(forward_angle * math.pi / 180)
        for i in range(2, 4):
            xiaoxiepo_flag =0
            x[i] = forward_extension * math.sin(forward_angle * math.pi / 180)
            y[i] = forward_extension * math.cos(forward_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(forward_offset)
    if prep_time+jump_time+land_time+fall_time<=t<prep_time+jump_time+land_time+fall_time+stand_time:
        slow_standup_1()
    if prep_time+jump_time+land_time+fall_time+stand_time<= t:
        xiaxiepo_1_over= 1

Value_xiaxiepo_2 = 0
xiaoxiepo_flag_2 = 0
def xiaxiepo_2():#走下小斜坡
    global Value_xiaxiepo_2,dog_state,xiaoxiepo_flag_2
    xiaxiepo_1_offest = [0,0,0,0]
    prep_time = 0.15
    t = Value_xiaxiepo_2/ 1000.0
    Value_xiaxiepo_2 += 1
    if t < prep_time:
        y[0] =21.8
        y[1] =21.8
        y[2] =21.8
        y[3] =21.8
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(xiaxiepo_1_offest)
        slow_pre()
    if t>=prep_time:
        xiaoxiepo_flag_2 = 1
        

Jump_Value_xiaxiepo = 0
sum_backangle_xiaxiepo =0
def xiaxiepo():#前跃式下小斜坡
    global Jump_Value_xiaxiepo
    global sum_backangle_xiaxiepo ,xiaxiepo_1_over
    prep_time_1 = 0.35#0.25
    prep_time_2 = 0.35
    prep_time  =prep_time_1
    launch_time = 0.05#这个时间段内可能达不到45度，需要调时间
    forward_time = 0.15
    fall_time = 0.25
    land_time = 0.05
    #stance_height
    stance_height = 21.6
    #prep
    prep_front_angle = 20
    prep_back_angle  =35
    prep_offset_pront = [40,40,0,0]#20,20,70,70
    prep_offset_back  = [20,20,70,70]
    xiaxiepo_1_offest=[50,50,0,0]
    xiaxiepo_2_offest=[0,0,0,0]
    #jump
    jump_angle =5
    jump_extension_front = 75
    jump_extension_back = 60#90
    #froward
    forward_angle = 25
    forward_extension_front = 26
    forward_extension_back  = 23
    jump_offest = [0,0,0,0]
    forward_offset = [15,15,0,0]#前两个参数是前腿前伸的耦合度，后两个参数是后腿伸直的耦合度
    #跳跃角度
    jump_back_angle =30 #60
    t = Jump_Value_xiaxiepo/ 1000.0
    Jump_Value_xiaxiepo += 1
    if t<prep_time/2:
        y_roll=math.tan(10)*33.3/4
        x[0]= 5
        x[1]= 5
        y[0] =21.8+ (y_roll)+4
        y[1] =21.8+ (y_roll)+4
        y[2] =21.8-(y_roll-2)+8
        y[3] =21.8-(y_roll-2+4)+1
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(xiaxiepo_2_offest)
        slow_pre_back()
    if prep_time/2<=t < prep_time:
        y_roll=math.tan(10)*33.3/4
        x[0]= 5
        x[1]= 5
        y[0] =21.8+ (y_roll)-5#-2
        y[1] =21.8+ (y_roll)-3
        y[2] =21.8-(y_roll-2)+4
        y[3] =21.8-(y_roll-2+4)+1
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(xiaxiepo_1_offest)
        slow_pre()
    elif prep_time < t < prep_time + launch_time and sum_backangle_xiaxiepo<jump_back_angle:
        for i in range(2):
            x[i] = -jump_extension_front * math.sin(jump_angle * math.pi / 180)
            y[i] =  jump_extension_front * math.cos(jump_angle * math.pi / 180)
        for i in range(2, 4):
            y_roll=math.tan(10)*33.3/4
            if i == 2:
                x[i] = -(stance_height) * math.sin(prep_back_angle * math.pi / 180)
                y[i] =  (stance_height +7)* math.cos(prep_back_angle * math.pi / 180)
            if i ==3:
                x[i] = -stance_height * math.sin(prep_back_angle * math.pi / 180)
                y[i] =  stance_height * math.cos(prep_back_angle * math.pi / 180)
            # if i == 2:
            #     x[i] = -(stance_height) * math.sin(prep_back_angle * math.pi / 180)
            #     y[i] =  (21.8-(y_roll-2)+4 +8)* math.cos(prep_back_angle * math.pi / 180)
            # if i ==3:
            #     x[i] = -stance_height * math.sin(prep_back_angle * math.pi / 180)
            #     y[i] =  (21.8-(y_roll-2+4)+1) * math.cos(prep_back_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        sum_backangle_xiaxiepo += 2
        command_all_legs_v_qianyue(prep_offset_back,sum_backangle_xiaxiepo)
        """  
        这里有个函数可以控制八条腿的角度，sum_backangle最为累加值控制旋转
        加到一定角度之后，后腿伸直
        """
    elif prep_time < t < prep_time + launch_time and sum_backangle_xiaxiepo>=jump_back_angle:
        for i in range(2):
            x[i] = 0
            y[i] = 15
        for i in range(2, 4):
            x[i] = -jump_extension_back * math.sin((jump_back_angle+prep_back_angle+jump_angle) * math.pi / 180)
            if i ==2:
                y[i] =  jump_extension_back+7 * math.cos((jump_back_angle+prep_back_angle+jump_angle) * math.pi / 180)
            if i ==3:
                y[i] =  jump_extension_back * math.cos((jump_back_angle+prep_back_angle+jump_angle) * math.pi / 180)    
        # for i in range(2, 4):
        #     y_roll=math.tan(10)*33.3/4
        #     x[i] = -jump_extension_back * math.sin((jump_back_angle+prep_back_angle+jump_angle) * math.pi / 180)
        #     if i ==2:
        #         y[i] =  (21.8-(y_roll-2)+4 +8) * math.cos((jump_back_angle+prep_back_angle+jump_angle) * math.pi / 180)
        #     elif i ==3:
        #         y[i] = (21.8-(y_roll-2+4)+1) * math.cos((jump_back_angle+prep_back_angle+jump_angle) * math.pi / 180)
        for i in range(2,4):
            cartesian_to_theta_2(i, 1.0)
        command_all_legs_v_jump(jump_offest)
    elif prep_time + launch_time <= t < prep_time + launch_time + forward_time:
        for i in range(2):
            x[i] = forward_extension_front * math.sin(forward_angle * math.pi / 180)
            y[i] = forward_extension_front * math.cos(forward_angle * math.pi / 180)
        for i in range(2,4):
            x[i] = (forward_extension_back-10) * math.sin(-forward_angle * math.pi / 180)
            y[i] = (forward_extension_back-10) * math.cos(-forward_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(forward_offset)
    elif prep_time + launch_time + forward_time<t<prep_time + launch_time + forward_time+fall_time:
        xiaxiepo_1_over=1
        slow_standup_1()


class GaitParams:
    def __init__(self, stance_height=0.0, step_length=0.0, up_amp=0.0, down_amp=0.0, flight_percent=0.0, freq=0.0):
        self.stance_height = stance_height  # 机身离地高度 (cm)
        self.step_length = step_length      # 步长 (cm)
        self.up_amp = up_amp                # 上抬振幅 (cm)
        self.down_amp = down_amp            # 下蹬振幅 (cm)
        self.flight_percent = flight_percent  # 占空比
        self.freq = freq                    # 频率 (Hz)

class DetachedParam:
    def __init__(self, params1, params2, params3, params4):
        self.detached_params1 = params1
        self.detached_params2 = params2
        self.detached_params3 = params3 
        self.detached_params4 = params4
    def __getitem__(self, index):
        if index == 0:
            return self.detached_params1
        elif index == 1:
            return self.detached_params2
        elif index == 2:
            return self.detached_params3
        elif index == 3:
            return self.detached_params4
        else:
            raise IndexError("Index out of range")


# 初始化步态参数
# stance_height: 机身离地高度 (cm)
# step_length: 步长 (cm)
# up_amp: 上抬振幅 (cm)
# down_amp: 下蹬振幅 (cm)
# flight_percent: 占空比
# freq: 频率 (Hz)

# # 分离型步态参数
# detached_params = DetachedParam()

# 状态参数数组
state_detached_params = [
    #STOP 0
    DetachedParam(
        GaitParams(20.0, 0.0, 0, 0.00, 0.5, 0.0),
        GaitParams(20.0, 0.0, 0, 0.00, 0.5, 0.0),
        GaitParams(20.0, 0.0, 0, 0.00, 0.5, 0.0),
        GaitParams(20.0, 0.0, 0, 0.00, 0.5, 0.0),
        # GaitParams(16.8, 5.0, 3.50, 0.00, 0.35, 10.0),
        # GaitParams(16.8, 5.0, 3.50, 0.00, 0.35, 10.0),
        # GaitParams(16.8, 5.0, 3.50, 0.00, 0.35, 10.0),
        # GaitParams(16.8, 5.0, 3.50, 0.00, 0.35, 10.0),
    ),
    # TROT 1
    DetachedParam(   
        GaitParams(20.0, 12, 4.5, 0.0, 0.5, 10.0),#21.8,15,4,0.5,0.35,2.0
        GaitParams(20.0, 12, 4.5, 0.0, 0.5, 10.0),
        GaitParams(20.0, 12, 4.5, 0.0, 0.5, 10.0),
        GaitParams(20.0, 12, 4.5, 0.0, 0.5, 10.0), 
        
        # GaitParams(16.8, 0.30, 4, 0.00, 0.5, 20.0),
        # GaitParams(24.8, -035, 2, 0.00, 2, 20.0),
        # GaitParams(16.8, 0.0, 4, 0.00, 0.5, 20.0),
        # GaitParams(24.8, -0.5, 2, 0.00, 2, 20.0),  
    ),
    # TROT_BACK 2
    DetachedParam(
        GaitParams(20.0, 12.0, 4.5, 0.00, 0.5, 10.0),
        GaitParams(20.0, 12.0, 4.5, 0.00, 0.5, 10.0),
        GaitParams(20.0, 12.0, 4.5, 0.00, 0.5, 10.0),
        GaitParams(20.0, 12.0, 4.5, 0.00, 0.5, 10.0),
    ),
    #TROT_BACK_RIGHT 3    
    DetachedParam(
        GaitParams(20.8, 18.0, 4.50, 0.00, 0.5, 20.0),
        GaitParams(20.8, 10.0, 4.50, 0.00, 0.5, 20.0),
        GaitParams(20.8, 18.0, 4.50, 0.00, 0.5, 20.0),
        GaitParams(20.8, 10.0, 4.50, 0.00, 0.5, 20.0),
    ),   
    #TROT_BACK_LEFT 4  
    DetachedParam(
        GaitParams(20.8, 10.0, 4.50, 0.00, 0.5, 20.0),
        GaitParams(20.8, 18.0, 4.50, 0.00, 0.5, 20.0),
        GaitParams(20.8, 10.0, 4.50, 0.00, 0.5, 20.0),
        GaitParams(20.8, 18.0, 4.50, 0.00, 0.5, 20.0),
    ),  
    #TROT_LEFT = 5 
    DetachedParam(
        GaitParams(20.8, 5.0, 4.50, 0.00, 0.5, 12.0),
        GaitParams(20.8, 16.0, 4.50, 0.00, 0.5,12.0),
        GaitParams(20.8, 5.0, 4.50, 0.00, 0.5, 12.0),
        GaitParams(20.8, 16.0, 4.50, 0.00, 0.5, 12.0),
    ),        
    #TROT_RIGHT = 6  
    DetachedParam(
        GaitParams(20.8, 16.0, 4.50, 0.00, 0.5, 12.0),
        GaitParams(20.8, 5.0,  4.50, 0.00, 0.5, 12.0),
        GaitParams(20.8, 16.0, 4.50, 0.00, 0.5, 12.0),
        GaitParams(20.8, 5.0,  4.50, 0.00, 0.5, 12.0),
    ),
    #ROTATE_LEFT = 7
    DetachedParam(
        GaitParams(20.8, 7.0, 3.50, 0.00, 0.5, 17.0),
        GaitParams(20.8, 7.0, 3.50, 0.00, 0.5, 17.0),
        GaitParams(20.8, 7.0, 3.50, 0.00, 0.5, 17.0),
        GaitParams(20.8, 7.0, 3.50, 0.00, 0.5, 17.0),
    ),
    #ROTATE_RIGHT = 8
    DetachedParam(
        GaitParams(20.8, 7.0, 3.50, 0.00, 0.5, 17.0),
        GaitParams(20.8, 7.0, 3.50, 0.00, 0.5, 17.0),
        GaitParams(20.8, 7.0, 3.50, 0.00, 0.5, 17.0),
        GaitParams(20.8, 7.0, 3.50, 0.00, 0.5, 17.0),
    ),
    #TROT_WHEEL = 9
    DetachedParam(
        # GaitParams(21.8, 7.0, 6.0, 0.00, 0.35, 12),
        # GaitParams(21.8, 7.0, 6.0, 0.00, 0.35, 12),
        # GaitParams(21.8, 7.0, 8.0, 0.00, 0.35, 12),
        # GaitParams(21.8, 7.0, 8.0, 0.00, 0.35, 12),
        GaitParams(20.8, 0, 0.0, 0.00, 0.5, 8),
        GaitParams(20.8, 0, 0.0, 0.00, 0.5, 8),
        GaitParams(20.8, 0, 0.0, 0.00, 0.5, 8),
        GaitParams(20.8, 0, 0.0, 0.00, 0.5, 8),
    ),
    #slow_TORT = 10
    DetachedParam(   
        GaitParams(21.8, 6.0, 4.0, 0.00, 0.35, 12.0),#23/5
        GaitParams(21.8, 6.0, 4.0, 0.00, 0.35, 12.0),
        GaitParams(21.8, 6.0, 4.0, 0.00, 0.35, 12.0),
        GaitParams(21.8, 6.0, 4.0, 0.00, 0.35, 12.0),   
    ),
    #small_TROT_LEFT = 11
    # DetachedParam(   
    #     GaitParams(21.8, 7,  4.0, 0.50, 0.25, 25.0),#15/5.2#length+=1,qrep+=5
    #     GaitParams(21.8, 19, 4.0, 0.50, 0.25, 25.0),
    #     GaitParams(21.8, 7,  4.0, 0.50, 0.25, 25.0),
    #     GaitParams(21.8, 19, 4.0, 0.50, 0.25, 25.0),   
    # ),
    DetachedParam(   
        GaitParams(21.8, 6.2,  4.0, 0.50, 0.25, 25.0),#15/5.2#length+=1,qrep+=5
        GaitParams(21.8, 19, 4.0, 0.50, 0.25, 25.0),
        GaitParams(21.8, 6.2,  4.0, 0.50, 0.25, 25.0),
        GaitParams(21.8, 19, 4.0, 0.50, 0.25, 25.0),   
    ),
    #small_TROT_RIGHT = 12
    # DetachedParam(   
    #     GaitParams(21.8, 20, 4.0, 0.50, 0.25, 25.0),
    #     GaitParams(21.8, 7,  4.0, 0.50, 0.25, 25.0),
    #     GaitParams(21.8, 20, 4.0, 0.50, 0.25, 25.0),
    #     GaitParams(21.8, 7,  4.0, 0.50, 0.25, 25.0),   
    # )
    DetachedParam(   
        GaitParams(21.8, 19, 4.0, 0.50, 0.25, 25.0),
        GaitParams(21.8, 6.2,  4.0, 0.50, 0.25, 25.0),
        GaitParams(21.8, 19, 4.0, 0.50, 0.25, 25.0),
        GaitParams(21.8, 6.2,  4.0, 0.50, 0.25, 25.0),   
    ),
    #dijia_trot = 13
     DetachedParam(   
        GaitParams(16.8, 10.0, 3.0, 0.00, 0.35, 18.0),#23/5
        GaitParams(16.8, 10.0, 3.0, 0.00, 0.35, 18.0),
        GaitParams(16.8, 10.0, 3.0, 0.00, 0.35, 18.0),
        GaitParams(16.8, 10.0, 3.0, 0.00, 0.35, 18.0),   
    ),
    #dijia_LEFT = 14
    DetachedParam(
        GaitParams(16.8, 5.0, 3.50, 0.00, 0.35, 18.0),
        GaitParams(16.8, 5.0, 3.50, 0.00, 0.35, 18.0),
        GaitParams(16.8, 5.0, 3.50, 0.00, 0.35, 18.0),
        GaitParams(16.8, 5.0, 3.50, 0.00, 0.35, 18.0),
    ),
    #dijai_RIGHT = 15
    DetachedParam(
        GaitParams(16.8, 5.0, 3.50, 0.00, 0.35, 18.0),
        GaitParams(16.8, 5.0, 3.50, 0.00, 0.35, 18.0),
        GaitParams(16.8, 5.0, 3.50, 0.00, 0.35, 18.0),
        GaitParams(16.8, 5.0, 3.50, 0.00, 0.35, 18.0),
    ),
    #move_left = 16
    DetachedParam(
        GaitParams(23.8, 0.0, 5, 0.00, 0.5, 25.0),
        GaitParams(18.8, -0.0, 2, 2.00, 0.7, 25.0),
        GaitParams(23.8, 0.0, 5, 0.00, 0.5, 25.0),
        GaitParams(18.8, -0.0, 2, 2.00, 0.7, 25.0), 
    ),
    #move_right= 17
    DetachedParam(
         GaitParams(18.8, -0.0, 2, 2.00, 0.7, 25.0),
         GaitParams(23.8, 0.0, 5, 0.00, 0.5, 25.0),
         GaitParams(18.8, -0.0, 2, 2.00, 0.7, 25.0), 
         GaitParams(23.8, 0.0, 2, 0.00, 0.5, 25.0),
    ),
    #xiepo_trot = 18
    DetachedParam(   
        GaitParams(21.8, 12.5, 3, 0.25, 0.35, 18.0),#21.8,15,4,0.5,0.35,2.0
        GaitParams(21.8, 12.0, 3, 0.25, 0.35, 18.0),
        GaitParams(21.8, 12.5, 3, 0.25, 0.35, 18.0),
        GaitParams(21.8, 12.0, 3, 0.25, 0.35, 18.0),   
    ),
    #xiataijie = 19
    DetachedParam(   
        GaitParams(21.8, 5, 3, 0.5, 0.35, 18.0),#21.8,15,4,0.5,0.35,2.0
        GaitParams(21.8, 5, 3, 0.5, 0.35, 18.0),
        GaitParams(21.8, 5, 3, 0.5, 0.35, 18.0),
        GaitParams(21.8, 5, 3, 0.5, 0.35, 18.0),   
    ),
    #taijie_trot = 20
    DetachedParam(   
        GaitParams(21.8, 3, 3, 0.5, 0.35, 10.0),#21.8,15,4,0.5,0.35,2.0
        GaitParams(21.8, 3, 3, 0.5, 0.35, 10.0),
        GaitParams(21.8, 3, 3, 0.5, 0.35, 10.0),
        GaitParams(21.8, 3, 3, 0.5, 0.35, 10.0),   
    ),
    #xiaxiepo = 21
    DetachedParam(   
        GaitParams(21.8, 12, 4, 1.5, 0.35, 6.0),#21.8,15,4,0.5,0.35,2.0
        GaitParams(21.8, 12, 4, 1.5, 0.35, 6.0),
        GaitParams(21.8, 12, 4, 1.5, 0.35, 6.0),
        GaitParams(21.8, 12, 4, 1.5, 0.35, 6.0),   
    ),
    #shakeng = 22
    DetachedParam(
        # GaitParams(26.8, 16, 14, 0.0, 0.25, 7.0),#21.8,15,4,0.5,0.35,2.0
        # GaitParams(26.8, 16, 14, 0.0, 0.25, 7.0),
        # GaitParams(26.8, 16, 14, 0.0, 0.25, 7.0),
        # GaitParams(26.8, 16, 14, 0.0, 0.25, 7.0), 
        # GaitParams(30.8, 16, 18, 0.0, 0.25, 7.0),#21.8,15,4,0.5,0.35,2.0
        # GaitParams(30.8, 16, 18, 0.0, 0.25, 7.0),
        # GaitParams(30.8, 16, 18, 0.0, 0.25, 7.0),
        # GaitParams(30.8, 16, 18, 0.0, 0.25, 7.0), 
        
        GaitParams(30.8, 16, 17, 0.0, 0.25, 7.0),#21.8,15,4,0.5,0.35,2.0
        GaitParams(30.8, 16, 17, 0.0, 0.25, 7.0),
        GaitParams(30.8, 16, 17, 0.0, 0.25, 7.0),
        GaitParams(30.8, 16, 17, 0.0, 0.25, 7.0), 
    ),
    #xiadaxiepo = 23
    DetachedParam(
        # GaitParams(26.8, 16, 14, 0.0, 0.25, 7.0),#21.8,15,4,0.5,0.35,2.0
        # GaitParams(26.8, 16, 14, 0.0, 0.25, 7.0),
        # GaitParams(26.8, 16, 14, 0.0, 0.25, 7.0),
        # GaitParams(26.8, 16, 14, 0.0, 0.25, 7.0), 
        GaitParams(21.8, 12, 6, 0.0, 0.25, 16.0),#21.8,15,4,0.5,0.35,2.0
        GaitParams(21.8, 12, 6, 0.0, 0.25, 16.0),
        GaitParams(21.8, 12, 6, 0.0, 0.25, 16.0),
        GaitParams(21.8, 12, 6, 0.0, 0.25, 16.0), 
    ),
    #slow_xiaxiepo = 24
    DetachedParam(   
        GaitParams(21.8, 3.5, 3, 0.5, 0.35, 12.0),#21.8,15,4,0.5,0.35,2.0
        GaitParams(21.8, 3.0, 3, 0.5, 0.35, 12.0),
        GaitParams(21.8, 3.5, 3, 0.5, 0.35, 12.0),
        GaitParams(21.8, 3.0, 3, 0.5, 0.35, 12.0),   
    ),
    #move_left_1 = 25
    DetachedParam(   
        GaitParams(21.8, 7.5, 4.0, 0.00, 0.5, 12.0),
        GaitParams(21.8, 9, 4.0, 0.50, 0.5, 12.0),
        GaitParams(21.8, 7.5, 4.0, 0.50, 0.5, 12.0),
        GaitParams(21.8, 9, 4.0, 0.00, 0.5, 12.0), 
    ),
    #move_right_1 = 26
    DetachedParam(   
        GaitParams(21.8, 9, 4.0, 0.50, 0.5, 12.0),
        GaitParams(21.8, 7.5, 4.0, 0.00, 0.5, 12.0),
        GaitParams(21.8, 9, 4.0, 0.00, 0.5, 12.0),
        GaitParams(21.8, 7.5, 4.0, 0.50, 0.5, 12.0), 
    ),
    #ROTATE_1 = 27
    DetachedParam(   
        GaitParams(21.8, 5, 3, 0.5, 0.35, 18.0),#21.8,15,4,0.5,0.35,2.0
        GaitParams(21.8, 5, 3, 0.5, 0.35, 18.0),
        GaitParams(21.8, 5, 3, 0.5, 0.35, 18.0),
        GaitParams(21.8, 5, 3, 0.5, 0.35, 18.0),   
    ),
    #ROTATE_RIGHT_1 = 28
    DetachedParam(   
        GaitParams(21.8, 5, 3, 0.5, 0.35, 18.0),#21.8,15,4,0.5,0.35,2.0
        GaitParams(21.8, 5, 3, 0.5, 0.35, 18.0),
        GaitParams(21.8, 5, 3, 0.5, 0.35, 18.0),
        GaitParams(21.8, 5, 3, 0.5, 0.35, 18.0),   
    ),
]

sin_count =0
judge = 0
trot_over_flag = 0
t_task = 0.25
def sin_trajectory(t, params, gait_offset,judge_flag,times):
    global prev_t
    global sin_count
    global judge
    global trot_over_flag
    """
    生成四足机器人的正弦轨迹

    该函数根据机器人的步态参数和当前时间计算机器人的脚位移。它使用正弦函数来模拟脚的运动，
    包括站立和摆动相。函数通过更新静态变量count来循环计算每条腿的位置。

    :param t: 当前时间，用于计算轨迹
    :param params: 步态参数，包括步高、步幅、步频等
    :param gait_offset: 步态偏移，用于调整步态的起始点
    :param times: 轨迹规划次数,times==0时次数为无穷,程序正常运行,times!=0时轨迹规划times次后停止
    """
    # 静态变量p用于累加每个腿的相位，gp用于存储调整后的相位
    if not hasattr(sin_trajectory, "p"):
        sin_trajectory.p = [0, 0, 0, 0]
    p = sin_trajectory.p
    
    if not hasattr(sin_trajectory, "count"):
        sin_trajectory.count = 0
    count = sin_trajectory.count
    gp = [0] * 4
    percent_back = [0] * 4

    # 从参数中提取步态变量
    stance_height = params.stance_height
    down_amp = params.down_amp
    up_amp = params.up_amp
    flight_percent = params.flight_percent
    step_length = params.step_length
    freq = params.freq
    x_bias = 0.0#（可能会使狗运动更加自然，暂未使用）

    # # 根据机器人的状态调整x轴偏移量
    # if dog_state == 'TROT':
    #     x_bias = 0.0
    # # elif dog_state == 'TROT' and (count == 2 or count == 3):
    # #     x_bias = 3.0
    # else:
    #     x_bias = 0.0


    # 根据相位计算脚的位置
    if times == 0:
        if judge_flag == 1:
            p[count] += freq * (t - prev_t)
            gp[count] = (p[count] + gait_offset) % 1.0
            if gp[count] <= flight_percent:
                # 在空中摆动相
                x[count] = (gp[count] / flight_percent) * step_length - step_length / 2.0 + x_bias
                y[count] = -up_amp * math.sin(math.pi * gp[count] / flight_percent) + stance_height
            else:
                # 在地面上的站立相
                percent_back[count] = (gp[count] - flight_percent) / (1.0 - flight_percent)
                x[count] = -percent_back[count] * step_length + step_length / 2.0 + x_bias
                y[count] = down_amp * math.sin(math.pi * percent_back[count]) + stance_height
        elif judge_flag == 0:
            p[count] = 0
            gp[count] = (p[count] + gait_offset) % 1.0
            x[count] = 0.0
            y[count] = stance_height
        elif judge_flag == 2:
            p[count] += freq * (t - prev_t)
            gp[count] = (p[count] + gait_offset) % 1.0
            if gp[count] <= flight_percent:
                if gp[count]<= t_task:
                    # 在空中摆动相
                    x[count] = (gp[count] / flight_percent) * step_length - step_length / 2.0 + x_bias
                    y[count] = -up_amp * math.sin(math.pi * gp[count] / flight_percent) + stance_height
                elif gp[count] > t_task and gp[count] <= 0.35:
                    x[count] = (t_task / flight_percent) * step_length - step_length / 2.0 + x_bias
                    y[count] = -up_amp * math.sin(math.pi * t_task / flight_percent) + stance_height
            else:
                # 在地面上的站立相
                x_start = (t_task / flight_percent) * step_length - step_length / 2.0 + x_bias
                x_end = -1 * step_length + step_length / 2.0 + x_bias
                y_start = -up_amp * math.sin(math.pi * t_task / flight_percent) + stance_height
                y_end = down_amp * math.sin(math.pi * 1) + stance_height
                percent_back[count] = (gp[count] - flight_percent) / (1.0 - flight_percent)
                x[count] = x_start + (x_end - x_start) * percent_back[count]
                #y[count] = down_amp * math.sin(math.pi * percent_back[count]) + stance_height
                y[count] = y_start + (y_end - y_start) * percent_back[count]
        # 更新计数器，以循环计算每条腿的位置
        if count < 3:
            count += 1
        else:
            prev_t = t
            count = 0     
    if times != 0 and sin_count<times:
        if judge_flag == 1:
            p[count] += freq * (t - prev_t)
            # gp[count] = (p[count] + gait_offset) % 1.0
            gp[count] = ((int((p[count] + gait_offset) % 1.0 * 1000)) / 1000)
            if gp[0] > 0.9 and judge ==0:
                judge = 1
            if gp[0] != 0 and gp[0] <0.1 and judge ==1:
                judge = 0
                sin_count += 1
                #print("gp",gp[0])
            if gp[count] <= flight_percent:
                # 在空中摆动相
                x[count] = (gp[count] / flight_percent) * step_length - step_length / 2.0 + x_bias
                y[count] = -up_amp * math.sin(math.pi * gp[count] / flight_percent) + stance_height
            else:
                # 在地面上的站立相
                percent_back[count] = (gp[count] - flight_percent) / (1.0 - flight_percent)
                x[count] = -percent_back[count] * step_length + step_length / 2.0 + x_bias
                y[count] = down_amp * math.sin(math.pi * percent_back[count]) + stance_height
        elif judge_flag == 0:
            p[count] = 0
            gp[count] = (p[count] + gait_offset) % 1.0
            x[count] = 0.0
            y[count] = stance_height 
        # 更新计数器，以循环计算每条腿的位置
        if count < 3:
            count += 1
        else:
            prev_t = t
            count = 0
    if times != 0 and sin_count==times:
        sin_count=0
        trot_over_flag =1
    # if times != 0 and sin_count==times:
    #     if judge_flag == 1:
    #         p[count] += freq * (t - prev_t)
    #         # gp[count] = (p[count] + gait_offset) % 1.0
    #         gp[count] = ((int((p[count] + gait_offset) % 1.0 * 1000)) / 1000)
    #         if gp[0] > 0.99 and judge ==0:
    #             judge = 1
    #         if gp[0] != 0 and gp[0] <0.01 and judge ==1:
    #             judge = 0
    #             sin_count += 1
    #         if gp[count] <= flight_percent:
    #             # 在空中摆动相
    #             x[count] = (gp[count] / flight_percent) * step_length - step_length / 2.0 + x_bias
    #             y[count] = -up_amp * math.sin(math.pi * gp[count] / flight_percent) + stance_height
    #         else:
    #             # 在地面上的站立相
    #             percent_back[count] = (gp[count] - flight_percent) / (1.0 - flight_percent)
    #             x[count] = -percent_back[count] * step_length + step_length / 2.0 + x_bias
    #             y[count] = down_amp * math.sin(math.pi * percent_back[count]) + stance_height
    #     elif judge_flag == 0:
    #         p[count] = 0
    #         gp[count] = (p[count] + gait_offset) % 1.0
    #         x[count] = 0.0
    #         y[count] = stance_height 
    #     # 更新计数器，以循环计算每条腿的位置
    #     if count < 3:
    #         count += 1
    #     else:
    #         prev_t = t
    #         count = 0
    # # 更新静态变量
    sin_trajectory.p = p
    sin_trajectory.count = count

def angle_from_T_deg(T_deg: float) -> float:
    """
    Compute the internal angle theta (in radians) between sides a=1.725 and b=20,
    given T (in radians), where the internal angle between sides b=20 and c=2.5 
    is (168.59 degrees - T).

    Returns:
        theta in radians, or float('nan') if the quadrilateral cannot be formed.
    
    Notes:
        - Valid T range: approx [22.2°, 113.7°] → [0.387, 1.984] radians.
        - No exceptions raised; NaN indicates invalid configuration.
        - Optimized for real-time robotics: minimal ops, no dynamic allocation.
    """
    # Precomputed constants (hard-coded for speed)
    T_rad = T_deg*math.pi/180
    cos_phi = math.cos(math.radians(168.59) - T_rad)
    e_sq = 406.25 - 100.0 * cos_phi

    # Feasibility: e must satisfy |20.4 - 1.725| <= e <= 20.4 + 1.725
    if e_sq < 348.755625 or e_sq > 489.515625 or e_sq < 0.0:
        return float('nan')

    e = math.sqrt(e_sq)
    numerator = 2.975625 + e_sq - 416.16      # a^2 + e^2 - d^2
    denominator = 3.45 * e                      # 2 * a * e

    if denominator == 0.0:
        return float('nan')

    cos_theta = numerator / denominator

    # Clamp to [-1, 1] to avoid acos domain error due to floating-point noise
    if cos_theta < -1.0:
        cos_theta = -1.0
    elif cos_theta > 1.0:
        cos_theta = 1.0
    
    theta = math.acos(cos_theta)*180/math.pi
    
    return theta

def cartesian_to_theta(leg_id, leg_direction):
    """
    将笛卡尔坐标转换为角度

    :param leg_id: 腿部ID，用于识别不同的腿
    :param leg_direction: 腿部方向，1.0表示前侧，-1.0表示后侧

    本函数根据腿部的笛卡尔坐标位置计算腿部关节的角度
    它通过几何关系和三角函数来确定腿部两个主要关节的角度
    """
    # 静态变量初始化
    L = [0] * 4
    cos_param_M = [0] * 4
    cos_param_T = [0] * 4
    M = [0] * 4
    N = [0] * 4
    T = [0] * 4
    A1 = [0] * 4
    A2 = [0] * 4
    
    x_final = [0] * 4
    y_final = [0] * 4

    x_final[leg_id] = x[leg_id]
    y_final[leg_id] = y[leg_id]

    # y_roll=math.tan(10)*33.3/4
    # y_final[1] -= y_roll-2.5
    # y_final[3] -= y_roll-2
    # y_final[0] += y_roll+2
    # y_final[2] += y_roll+3


    L[leg_id] = math.sqrt(x_final[leg_id]**2 + y_final[leg_id]**2)
    
    N[leg_id] = math.asin(x_final[leg_id] / L[leg_id]) * 180.0 / math.pi
    
    cos_param_M[leg_id] = (L1**2 + L[leg_id]**2 - L2**2  ) / (2.0 * L1 * L[leg_id])
    cos_param_T[leg_id] = (L1**2 + L2**2 - L[leg_id]**2 ) / (2.0 * L1 * L2)
    if cos_param_M[leg_id] < -1.0:
        M[leg_id] = math.pi
    elif cos_param_M[leg_id] > 1.0:
        M[leg_id] = 0
    else:
        M[leg_id] = math.acos(cos_param_M[leg_id])

    if cos_param_T[leg_id] < -1.0:
        T[leg_id] = math.pi
    elif cos_param_T[leg_id] > 1.0:
        T[leg_id] = 0
    else:
        T[leg_id] = math.acos(cos_param_T[leg_id])

        
    
    M[leg_id] = M[leg_id] * 180.0 / math.pi #转角度制
    T[leg_id] = T[leg_id] * 180.0 / math.pi
    
    A1[leg_id] = M[leg_id] - N[leg_id] #- 36.1#大腿（左）右反
    A2[leg_id] = -angle_from_T_deg(T[leg_id]) + 21.86#小腿(右)  
    
    # A1[leg_id] = M[leg_id] - N[leg_id]
    # A2[leg_id] = M[leg_id] + N[leg_id]

    if leg_direction == 1.0 and -88 < A2[leg_id] < 1:
        theta1[leg_id] = A2[leg_id]  #小腿（右
        theta2[leg_id] = A1[leg_id] -36.1 #大腿（左
        #print("theta2:%f",theta2[0])
    elif leg_direction == -1.0 and -88 < A2[leg_id] < 1:
        theta1[leg_id] = A2[leg_id]
        theta2[leg_id] = M[leg_id] + N[leg_id] -36.1
    else :
        print("T generation false")

def cartesian_to_theta_imu(leg_id, leg_direction,pitch,roll):
    """
    将笛卡尔坐标转换为角度

    :param leg_id: 腿部ID，用于识别不同的腿
    :param leg_direction: 腿部方向，1.0表示前侧，-1.0表示后侧

    本函数根据腿部的笛卡尔坐标位置计算腿部关节的角度
    它通过几何关系和三角函数来确定腿部两个主要关节的角度
    """
    # 静态变量初始化
    L = [0] * 4
    cos_param = [0] * 4
    M = [0] * 4
    N = [0] * 4
    A1 = [0] * 4
    A2 = [0] * 4
    
    x_final = [0] * 4
    y_final = [0] * 4

    x_final[leg_id] = x[leg_id]
    y_final[leg_id] = y[leg_id]

    roll = roll * math.pi / 180
    pitch = pitch * math.pi / 180
    y_roll=math.tan(roll)*33.3/2
    y_pitch=math.tan(pitch)*32.5/2
    y_final[1] -= -y_roll+y_pitch
    y_final[3] -= -y_roll-y_pitch
    y_final[0] += -y_roll-y_pitch
    y_final[2] += -y_roll+y_pitch
    #print("y1:%f,y2:%f,y3:%f,y4:%f"%(y_final[0],y_final[1],y_final[2],y_final[3]))
    
    


    L[leg_id] = math.sqrt(x_final[leg_id]**2 + y_final[leg_id]**2)
    
    N[leg_id] = math.asin(x_final[leg_id] / L[leg_id]) * 180.0 / math.pi
    
    cos_param[leg_id] = (L1**2 + L[leg_id]**2 - L2**2) / (2.0 * L1 * L[leg_id])
    if cos_param[leg_id] < -1.0:
        M[leg_id] = math.pi
    elif cos_param[leg_id] > 1.0:
        M[leg_id] = 0
    else:
        M[leg_id] = math.acos(cos_param[leg_id])
    
    M[leg_id] = M[leg_id] * 180.0 / math.pi
    
    A1[leg_id] = 90-(M[leg_id] - N[leg_id])
    A2[leg_id] = 90-(M[leg_id] + N[leg_id])
    
    # A1[leg_id] = M[leg_id] - N[leg_id]
    # A2[leg_id] = M[leg_id] + N[leg_id]

    if leg_direction == 1.0:
        theta1[leg_id] = A2[leg_id]  # 前腿
        theta2[leg_id] = A1[leg_id]  # 后腿
    elif leg_direction == -1.0:
        theta1[leg_id] = A1[leg_id]
        theta2[leg_id] = A2[leg_id]

def cartesian_to_theta_1(leg_id, leg_direction):
    """
    将笛卡尔坐标转换为角度

    :param leg_id: 腿部ID，用于识别不同的腿
    :param leg_direction: 腿部方向，1.0表示前侧，-1.0表示后侧

    本函数根据腿部的笛卡尔坐标位置计算腿部关节的角度
    它通过几何关系和三角函数来确定腿部两个主要关节的角度
    """
    # 静态变量初始化
    L = [0] * 4
    cos_param = [0] * 4
    M = [0] * 4
    N = [0] * 4
    A1 = [0] * 4
    A2 = [0] * 4
    
    x_final = [0] * 4
    y_final = [0] * 4

    x_final[leg_id] = x[leg_id]
    y_final[leg_id] = y[leg_id]

    y_roll=math.tan(10)*33.3/4
    # y_final[1] -= y_roll-2.5
    # y_final[3] -= y_roll-2
    # y_final[0] += y_roll+2
    # y_final[2] += y_roll+3
    y_final[1] -= y_roll-2-2
    y_final[3] -= y_roll-2+5
    y_final[0] += y_roll+2+2
    y_final[2] += y_roll+3-5
    

    L[leg_id] = math.sqrt(x_final[leg_id]**2 + y_final[leg_id]**2)
    
    N[leg_id] = math.asin(x_final[leg_id] / L[leg_id]) * 180.0 / math.pi
    
    cos_param[leg_id] = (L1**2 + L[leg_id]**2 - L2**2) / (2.0 * L1 * L[leg_id])
    if cos_param[leg_id] < -1.0:
        M[leg_id] = math.pi
    elif cos_param[leg_id] > 1.0:
        M[leg_id] = 0
    else:
        M[leg_id] = math.acos(cos_param[leg_id])
    
    M[leg_id] = M[leg_id] * 180.0 / math.pi
    
    A1[leg_id] = 90-(M[leg_id] - N[leg_id])
    A2[leg_id] = 90-(M[leg_id] + N[leg_id])
    
    # A1[leg_id] = M[leg_id] - N[leg_id]
    # A2[leg_id] = M[leg_id] + N[leg_id]

    if leg_direction == 1.0:
        theta1[leg_id] = A2[leg_id]  # 前腿
        theta2[leg_id] = A1[leg_id]  # 后腿
    elif leg_direction == -1.0:
        theta1[leg_id] = A1[leg_id]
        theta2[leg_id] = A2[leg_id]

def cartesian_to_theta_2(leg_id, leg_direction):
    """
    将笛卡尔坐标转换为角度

    :param leg_id: 腿部ID，用于识别不同的腿
    :param leg_direction: 腿部方向，1.0表示前侧，-1.0表示后侧

    本函数根据腿部的笛卡尔坐标位置计算腿部关节的角度
    它通过几何关系和三角函数来确定腿部两个主要关节的角度
    """
    # 静态变量初始化
    L = [0] * 4
    cos_param = [0] * 4
    M = [0] * 4
    N = [0] * 4
    A1 = [0] * 4
    A2 = [0] * 4
    
    x_final = [0] * 4
    y_final = [0] * 4

    x_final[leg_id] = x[leg_id]
    y_final[leg_id] = y[leg_id]

    y_roll=math.tan(10)*33.3/4
    y_final[1] -= y_roll-2.5
    y_final[3] -= y_roll-2.5
    y_final[0] += y_roll+3.2
    y_final[2] += y_roll+3
    # y_final[1] -= y_roll-2
    # y_final[3] -= y_roll-2
    # y_final[0] += y_roll+2
    # y_final[2] += y_roll+3
    

    L[leg_id] = math.sqrt(x_final[leg_id]**2 + y_final[leg_id]**2)
    
    N[leg_id] = math.asin(x_final[leg_id] / L[leg_id]) * 180.0 / math.pi
    
    cos_param[leg_id] = (L1**2 + L[leg_id]**2 - L2**2) / (2.0 * L1 * L[leg_id])
    if cos_param[leg_id] < -1.0:
        M[leg_id] = math.pi
    elif cos_param[leg_id] > 1.0:
        M[leg_id] = 0
    else:
        M[leg_id] = math.acos(cos_param[leg_id])
    
    M[leg_id] = M[leg_id] * 180.0 / math.pi
    
    A1[leg_id] = 90-(M[leg_id] - N[leg_id])
    A2[leg_id] = 90-(M[leg_id] + N[leg_id])
    
    # A1[leg_id] = M[leg_id] - N[leg_id]
    # A2[leg_id] = M[leg_id] + N[leg_id]

    if leg_direction == 1.0:
        theta1[leg_id] = A2[leg_id]  # 前腿
        theta2[leg_id] = A1[leg_id]  # 后腿
    elif leg_direction == -1.0:
        theta1[leg_id] = A1[leg_id]
        theta2[leg_id] = A2[leg_id]
def cartesian_to_theta_3(leg_id, leg_direction):
    """
    将笛卡尔坐标转换为角度

    :param leg_id: 腿部ID，用于识别不同的腿
    :param leg_direction: 腿部方向，1.0表示前侧，-1.0表示后侧

    本函数根据腿部的笛卡尔坐标位置计算腿部关节的角度
    它通过几何关系和三角函数来确定腿部两个主要关节的角度
    """
    # 静态变量初始化
    L = [0] * 4
    cos_param = [0] * 4
    M = [0] * 4
    N = [0] * 4
    A1 = [0] * 4
    A2 = [0] * 4
    
    x_final = [0] * 4
    y_final = [0] * 4

    x_final[leg_id] = x[leg_id]
    y_final[leg_id] = y[leg_id]

    y_roll=math.tan(10)*33.3/4/2
    y_final[0] += (y_roll)+4
    y_final[1] += (y_roll)+4
    y_final[2] -=(y_roll-2)-5+3
    y_final[3] -=(y_roll-2+4)#+3


    L[leg_id] = math.sqrt(x_final[leg_id]**2 + y_final[leg_id]**2)
    
    N[leg_id] = math.asin(x_final[leg_id] / L[leg_id]) * 180.0 / math.pi
    
    cos_param[leg_id] = (L1**2 + L[leg_id]**2 - L2**2) / (2.0 * L1 * L[leg_id])
    if cos_param[leg_id] < -1.0:
        M[leg_id] = math.pi
    elif cos_param[leg_id] > 1.0:
        M[leg_id] = 0
    else:
        M[leg_id] = math.acos(cos_param[leg_id])
    
    M[leg_id] = M[leg_id] * 180.0 / math.pi
    
    A1[leg_id] = 90-(M[leg_id] - N[leg_id])
    A2[leg_id] = 90-(M[leg_id] + N[leg_id])
    # A1[leg_id] = M[leg_id] - N[leg_id]
    # A2[leg_id] = M[leg_id] + N[leg_id]

    if leg_direction == 1.0:
        theta1[leg_id] = A2[leg_id]  # 前腿
        theta2[leg_id] = A1[leg_id]  # 后腿
    elif leg_direction == -1.0:
        theta1[leg_id] = A1[leg_id]
        theta2[leg_id] = A2[leg_id]

def deg_2_cir(angle):
    """
    将角度转换为圆周率表示
    :param angle: 角度值
    :return: 圆周率表示的角度
    """
    return angle * (math.pi / 180.0)
# def deg_2_cir(angle):
#     return angle /360
class TempData:
    def __init__(self):
        self.ref_agle = [0.0] * 8  # 存储目标角度
        self.out = [0.0] * 8       # 其他输出数据

temp_pid = TempData()  # 创建一个TempData实例

def set_all_legs_coupled_position(leg_id, leg1_angle_0, leg1_angle_1,leg2_angle_2, leg2_angle_3,leg3_angle_4, leg3_angle_5,leg4_angle_6,leg4_angle_7):
    """
    设置所有腿的耦合位置

    :param leg_id: 腿的标识符，取值为0（左前腿）、1（右前腿）、2（左后腿）、3（右后腿）
    :param leg1_angle: 第1条腿的目标角度
    :param leg2_angle: 第2条腿的目标角度
    :param leg3_angle: 第3条腿的目标角度
    :param leg4_angle: 第4条腿的目标角度
    :param theta1: 第一组角度数组
    :param theta2: 第二组角度数组
    :param ReductionAndAngleRatio: 角度转换比例
    """
    global pos
    # 根据腿的ID设置对应腿的目标角度
    if leg_id == 0:  # 左前腿
        temp_pid.ref_agle[0] = theta2[0] * ReductionAndAngleRatio
        temp_pid.ref_agle[1] = -theta1[0] * ReductionAndAngleRatio
    elif leg_id == 2:  # 左后腿
        temp_pid.ref_agle[4] = theta2[2] * ReductionAndAngleRatio
        temp_pid.ref_agle[5] = -theta1[2] * ReductionAndAngleRatio
    elif leg_id == 1:  # 右前腿
        temp_pid.ref_agle[2] = -theta2[1] * ReductionAndAngleRatio
        temp_pid.ref_agle[3] = theta1[1] * ReductionAndAngleRatio
    elif leg_id == 3:  # 右后腿
        temp_pid.ref_agle[6] = -theta2[3] * ReductionAndAngleRatio
        temp_pid.ref_agle[7] = theta1[3] * ReductionAndAngleRatio
    
    #计算每个电机的最终旋转圈数(弧度)
    pos[1] = deg_2_cir(temp_pid.ref_agle[0])  #- deg_2_cir(leg1_angle_0)#-0.25
    pos[0] = deg_2_cir(temp_pid.ref_agle[1]) #- deg_2_cir(leg1_angle_1)#-0.25
    pos[3] = deg_2_cir(temp_pid.ref_agle[2])  #+ deg_2_cir(leg2_angle_2)#+0.25
    pos[2] = deg_2_cir(temp_pid.ref_agle[3])  #+ deg_2_cir(leg2_angle_3)#+0.25
    pos[5] = deg_2_cir(temp_pid.ref_agle[4])  #+ deg_2_cir(leg3_angle_4)#+0.25
    pos[4] = deg_2_cir(temp_pid.ref_agle[5])  #+ deg_2_cir(leg3_angle_5)#+0.25
    pos[7] = deg_2_cir(temp_pid.ref_agle[6])  #- deg_2_cir(leg4_angle_6)#-0.25
    pos[6] = deg_2_cir(temp_pid.ref_agle[7])  #- deg_2_cir(leg4_angle_7)#-0.25
        
    


def gait_detached_all_legs(d_params,  # 步态分离参数
                           leg0_offset, leg1_offset, leg2_offset, leg3_offset,
                           leg0_direction, leg1_direction, leg2_direction, leg3_direction,
                           gait_angle_s,judge_flag,pitch,roll):
    """
    执行四足机器人的步态分离控制。

    该函数根据指定的步态参数和腿的偏移及方向，计算并设置每条腿的电机位置，以实现特定的步态。

    :param d_params: 步态分离参数，包含每条腿的步态细节。
    :param leg0_offset: 第一条腿的偏移量。
    :param leg1_offset: 第二条腿的偏移量。
    :param leg2_offset: 第三条腿的偏移量。
    :param leg3_offset: 第四条腿的偏移量。
    :param leg0_direction: 第一条腿的方向。
    :param leg1_direction: 第二条腿的方向。
    :param leg2_direction: 第三条腿的方向。
    :param leg3_direction: 第四条腿的方向。
    :param gait_angle_s: 包含每条腿的步态角度信息。
    """
    global Gait_Value  
    # 计算时间间隔，用于步态控制。
    t = Gait_Value / 1000.000000
    Gait_Value += 1
    # print(Gait_Value)
    # 提取每个腿的增益角度。
    leg1_angle_0 = gait_angle_s.gait_angle_leg1_0
    leg1_angle_1 = gait_angle_s.gait_angle_leg1_1
    leg2_angle_2 = gait_angle_s.gait_angle_leg2_2
    leg2_angle_3 = gait_angle_s.gait_angle_leg2_3
    leg3_angle_4 = gait_angle_s.gait_angle_leg3_4
    leg3_angle_5 = gait_angle_s.gait_angle_leg3_5
    leg4_angle_6 = gait_angle_s.gait_angle_leg4_6
    leg4_angle_7 = gait_angle_s.gait_angle_leg4_7
    # print("leg1_angle_0",leg1_angle_0)
# 初始化步态参数
# stance_height: 机身离地高度 (cm)
# step_length: 步长 (cm)
# up_amp: 上抬振幅 (cm)
# down_amp: 下蹬振幅 (cm)
# flight_percent: 占空比
# freq: 频率 (Hz)
    # 根据步态参数和偏移量计算每条腿的轨迹。
    if _leg_active[0]:
        a1 = sin_trajectory(t, d_params[0], leg0_offset,judge_flag,0)
    if _leg_active[1]:
        a2 = sin_trajectory(t, d_params[1], leg1_offset,judge_flag,0)
    if _leg_active[2]:
        a3= sin_trajectory(t, d_params[2], leg2_offset,judge_flag,0)
    if _leg_active[3]:
        a4=sin_trajectory(t, d_params[3], leg3_offset,judge_flag,0)

    cartesian_to_theta(0,leg0_direction)
    cartesian_to_theta(1,leg1_direction)
    cartesian_to_theta(2,leg2_direction)
    cartesian_to_theta(3,leg3_direction)

    # 根据计算结果设置每条腿的电机位置。
    set_all_legs_coupled_position(0, leg1_angle_0, leg1_angle_1,leg2_angle_2, leg2_angle_3,leg3_angle_4, leg3_angle_5,leg4_angle_6,leg4_angle_7)
    set_all_legs_coupled_position(1, leg1_angle_0, leg1_angle_1,leg2_angle_2, leg2_angle_3,leg3_angle_4, leg3_angle_5,leg4_angle_6,leg4_angle_7)
    set_all_legs_coupled_position(2, leg1_angle_0, leg1_angle_1,leg2_angle_2, leg2_angle_3,leg3_angle_4, leg3_angle_5,leg4_angle_6,leg4_angle_7)    
    set_all_legs_coupled_position(3, leg1_angle_0, leg1_angle_1,leg2_angle_2, leg2_angle_3,leg3_angle_4, leg3_angle_5,leg4_angle_6,leg4_angle_7)


def gait_detached_all_legs_times(d_params,  # 步态分离参数
                           leg0_offset, leg1_offset, leg2_offset, leg3_offset,
                           leg0_direction, leg1_direction, leg2_direction, leg3_direction,
                           gait_angle_s,judge_flag,times):
    global Gait_Value  
    # 计算时间间隔，用于步态控制。
    t = Gait_Value / 1000.000000
    Gait_Value += 1
    # print(Gait_Value)
    # 提取每个腿的步态角度。
    leg1_angle_0 = gait_angle_s.gait_angle_leg1_0
    leg1_angle_1 = gait_angle_s.gait_angle_leg1_1
    leg2_angle_2 = gait_angle_s.gait_angle_leg2_2
    leg2_angle_3 = gait_angle_s.gait_angle_leg2_3
    leg3_angle_4 = gait_angle_s.gait_angle_leg3_4
    leg3_angle_5 = gait_angle_s.gait_angle_leg3_5
    leg4_angle_6 = gait_angle_s.gait_angle_leg4_6
    leg4_angle_7 = gait_angle_s.gait_angle_leg4_7
# 初始化步态参数
# stance_height: 机身离地高度 (cm)
# step_length: 步长 (cm)
# up_amp: 上抬振幅 (cm)
# down_amp: 下蹬振幅 (cm)
# flight_percent: 占空比
# freq: 频率 (Hz)
    # 根据步态参数和偏移量计算每条腿的轨迹。
    if _leg_active[0]:
        A1 = sin_trajectory(t, d_params[0], leg0_offset,judge_flag,times)
    if _leg_active[1]:
        A2 = sin_trajectory(t, d_params[1], leg1_offset,judge_flag,times)
    if _leg_active[2]:
        A3 = sin_trajectory(t, d_params[2], leg2_offset,judge_flag,times)
    if _leg_active[3]:
        A4 = sin_trajectory(t, d_params[3], leg3_offset,judge_flag,times)

    # 计算每条腿的方向 theta。
    if xiaoxiepo_flag ==0:
        cartesian_to_theta(0, leg0_direction)
        cartesian_to_theta(1, leg1_direction)
        cartesian_to_theta(2, leg2_direction)
        cartesian_to_theta(3, leg3_direction)
    if xiaoxiepo_flag ==1:
        cartesian_to_theta_2(0,leg0_direction)
        cartesian_to_theta_2(1,leg1_direction)
        cartesian_to_theta_2(2,leg2_direction)
        cartesian_to_theta_2(3,leg3_direction)
    if xiaoxiepo_flag ==2:
        cartesian_to_theta_3(0,leg0_direction)
        cartesian_to_theta_3(1,leg1_direction)
        cartesian_to_theta_3(2,leg2_direction)
        cartesian_to_theta_3(3,leg3_direction)
    # 根据计算结果设置每条腿的电机位置。
    set_all_legs_coupled_position(0, leg1_angle_0, leg1_angle_1,leg2_angle_2, leg2_angle_3,leg3_angle_4, leg3_angle_5,leg4_angle_6,leg4_angle_7)
    set_all_legs_coupled_position(1, leg1_angle_0, leg1_angle_1,leg2_angle_2, leg2_angle_3,leg3_angle_4, leg3_angle_5,leg4_angle_6,leg4_angle_7)
    set_all_legs_coupled_position(2, leg1_angle_0, leg1_angle_1,leg2_angle_2, leg2_angle_3,leg3_angle_4, leg3_angle_5,leg4_angle_6,leg4_angle_7)    
    set_all_legs_coupled_position(3, leg1_angle_0, leg1_angle_1,leg2_angle_2, leg2_angle_3,leg3_angle_4, leg3_angle_5,leg4_angle_6,leg4_angle_7)


# jump_count = 0
# jump_state = 0
# def posture_control_task_taijie():
#     global jump_count
#     global jump_state
#     global jump_flag
#     if jump_count == 0:
#         jump_state = 0
#         # print("jump_state:",jump_state)
#         jumptime = jump_times[jump_state]
#         jumplength = jump_lengths[jump_state]
#         jumpangles = jump_angles[jump_state]
#         jump_angle_offset_period = state_jumpangle_offset_period[jump_state]
#         execute_jump_taijie(jumptime, jumplength, jumpangles, jump_angle_offset_period)
#     elif jump_count == 1:
#         time.sleep(0.15)
#         jump_count += 1
#     elif jump_count == 2:
#         jump_state = 1
#         # print("jump_state:",jump_state)
#         jumptime = jump_times[jump_state]
#         jumplength = jump_lengths[jump_state]
#         jumpangles = jump_angles[jump_state]
#         jump_angle_offset_period = state_jumpangle_offset_period[jump_state]
#         execute_jump_taijie(jumptime, jumplength, jumpangles, jump_angle_offset_period)
#     elif jump_count >= 3:
#         jump_state =2 
#         jump_count = 0

"""
上台阶要跳两次
"""
jump_count_taijie = 0
def posture_control_task_taijie_1():
    global jump_count_taijie
    jump_state = 0
    jumptime = jump_times[jump_state]
    jumplength = jump_lengths[jump_state]
    jumpangles = jump_angles[jump_state]
    jump_angle_offset_period = state_jumpangle_offset_period[jump_state]
    execute_jump_taijie(jumptime, jumplength, jumpangles, jump_angle_offset_period)

def posture_control_task_taijie_2():
    global jump_count_taijie
    jump_state = 1
    jumptime = jump_times[jump_state]
    jumplength = jump_lengths[jump_state]
    jumpangles = jump_angles[jump_state]
    jump_angle_offset_period = state_jumpangle_offset_period[jump_state]
    execute_jump_taijie(jumptime, jumplength, jumpangles, jump_angle_offset_period)


daxiepo_over = 0
def posture_control_task_shangdaxiepo():
    global jump_count_daxiepo,daxiepo_over
    if jump_count_daxiepo == 0:
        jump_state = 7
        #print("dog_state:",jump_state)
        jumptime = jump_times[jump_state]
        jumplength = jump_lengths[jump_state]
        jumpangles = jump_angles[jump_state]
        jump_angle_offset_period = state_jumpangle_offset_period[jump_state]
        execute_jump(jumptime, jumplength, jumpangles, jump_angle_offset_period)
    elif jump_count_daxiepo == 1:
        time.sleep(0.1)
        #pass
        jump_count_daxiepo += 1
    elif jump_count_daxiepo == 2:
        jump_state = 8
        #print("dog_state:",jump_state)
        jumptime = jump_times[jump_state]
        jumplength = jump_lengths[jump_state]
        jumpangles = jump_angles[jump_state]
        jump_angle_offset_period = state_jumpangle_offset_period[jump_state]
        execute_jump(jumptime, jumplength, jumpangles, jump_angle_offset_period)




def posture_control_task_xiataijie():
    global jump_count
    if jump_count == 0:
        jump_state = 2
        #print("dog_state:",jump_state)
        jumptime = jump_times[jump_state]
        jumplength = jump_lengths[jump_state]
        jumpangles = jump_angles[jump_state]
        jump_angle_offset_period = state_jumpangle_offset_period[jump_state]
        execute_jump(jumptime, jumplength, jumpangles, jump_angle_offset_period)
    elif jump_count == 1:
        time.sleep(2)
        #pass
        jump_count += 1
    elif jump_count == 2:
        jump_state = 3
        #print("dog_state:",jump_state)
        jumptime = jump_times[jump_state]
        jumplength = jump_lengths[jump_state]
        jumpangles = jump_angles[jump_state]
        jump_angle_offset_period = state_jumpangle_offset_period[jump_state]
        execute_jump(jumptime, jumplength, jumpangles, jump_angle_offset_period)


jump_xiaduanqiao_state =0
def posture_control_task_xiaduanqiao():#用的是下台阶的步态
    global jump_count
    global jump_xiaduanqiao_state
    if jump_count == 0:
        jump_state = 2
        #print("dog_state:",jump_state)
        jumptime = jump_times[jump_state]
        jumplength = jump_lengths[jump_state]
        jumpangles = jump_angles[jump_state]
        jump_angle_offset_period = state_jumpangle_offset_period[jump_state]
        execute_jump(jumptime, jumplength, jumpangles, jump_angle_offset_period)
    elif jump_count == 1:
        time.sleep(0.5)
        #pass
        jump_count += 1
    elif jump_count == 2:
        jump_state = 3
        #print("dog_state:",jump_state)
        jumptime = jump_times[jump_state]
        jumplength = jump_lengths[jump_state]
        jumpangles = jump_angles[jump_state]
        jump_angle_offset_period = state_jumpangle_offset_period[jump_state]
        execute_jump(jumptime, jumplength, jumpangles, jump_angle_offset_period)
    elif jump_count >=3:
        jump_xiaduanqiao_state = 1
        jump_count = 0



def posture_control_task_duanqiao():###没有用到S
    global jump_count
    if jump_count == 0:
        jump_state = 4
        #print("dog_state:",jump_state)
        jumptime = jump_times[jump_state]
        jumplength = jump_lengths[jump_state]
        jumpangles = jump_angles[jump_state]
        jump_angle_offset_period = state_jumpangle_offset_period[jump_state]
        execute_jump(jumptime, jumplength, jumpangles, jump_angle_offset_period)
    elif jump_count == 1:
        time.sleep(2)
        #pass
        jump_count += 1
    elif jump_count == 2:
        jump_state = 5
        #print("dog_state:",jump_state)
        jumptime = jump_times[jump_state]
        jumplength = jump_lengths[jump_state]
        jumpangles = jump_angles[jump_state]
        jump_angle_offset_period = state_jumpangle_offset_period[jump_state]
        execute_jump(jumptime, jumplength, jumpangles, jump_angle_offset_period)

def shakeng():
        jump_state = 10
        #print("dog_state:",jump_state)
        jumptime = jump_times[jump_state]
        jumplength = jump_lengths[jump_state]
        jumpangles = jump_angles[jump_state]
        jump_angle_offset_period = state_jumpangle_offset_period[jump_state]
        execute_jump_shakeng(jumptime, jumplength, jumpangles, jump_angle_offset_period)

jump_xiaxiepo_over =0
jump_count_xiaxiepo = 0
def posture_control_task_xiaxiepo():#下小斜坡
        global jump_count_xiaxiepo
        global jump_xiaxiepo_over
        if jump_count_xiaxiepo == 0:
            jump_state = 9
            # print("dog_state:",jump_state)
            jumptime = jump_times[jump_state]
            jumplength = jump_lengths[jump_state]
            jumpangles = jump_angles[jump_state]
            jump_angle_offset_period = state_jumpangle_offset_period[jump_state]
            execute_jump_xiaxiepo(jumptime, jumplength, jumpangles, jump_angle_offset_period)
        elif jump_count_xiaxiepo ==1:
            jump_xiaxiepo_over=+1


def command_all_legs_shangtaijie(jump_angle_offset_period_s,angle_front,angle_back):
    leg1_angle = jump_angle_offset_period_s[0]
    leg2_angle = jump_angle_offset_period_s[1]
    leg3_angle = jump_angle_offset_period_s[2]
    leg4_angle = jump_angle_offset_period_s[3]
    
    set_all_legs_coupled_position_taijie(0, leg1_angle, leg2_angle, leg3_angle, leg4_angle,angle_front,angle_back)
    set_all_legs_coupled_position_taijie(1, leg1_angle, leg2_angle, leg3_angle, leg4_angle,angle_front,angle_back)
    set_all_legs_coupled_position_taijie(2,leg1_angle, leg2_angle, leg3_angle, leg4_angle,angle_front,angle_back)
    set_all_legs_coupled_position_taijie(3, leg1_angle, leg2_angle, leg3_angle, leg4_angle,angle_front,angle_back)
def set_all_legs_coupled_position_taijie(leg_id,leg1_angle, leg2_angle, leg3_angle, leg4_angle,angle_front,angle_back):
    """
    设置所有腿的耦合位置

    :param leg_id: 腿的标识符，取值为0（左前腿）、1（右前腿）、2（左后腿）、3（右后腿）
    :param leg1_angle: 第1条腿的目标角度
    :param leg2_angle: 第2条腿的目标角度
    :param leg3_angle: 第3条腿的目标角度
    :param leg4_angle: 第4条腿的目标角度
    :param theta1: 第一组角度数组
    :param theta2: 第二组角度数组
    :param ReductionAndAngleRatio: 角度转换比例
    """

    # 根据腿的ID设置对应腿的目标角度
    if leg_id == 0:  # 左前腿
        temp_pid.ref_agle[0] = theta1[0] * ReductionAndAngleRatio
        temp_pid.ref_agle[1] = theta2[0] * ReductionAndAngleRatio
    elif leg_id == 2:  # 左后腿
        temp_pid.ref_agle[4] = theta1[2] * ReductionAndAngleRatio
        temp_pid.ref_agle[5] = theta2[2] * ReductionAndAngleRatio
    elif leg_id == 1:  # 右前腿
        temp_pid.ref_agle[2] = -theta1[1] * ReductionAndAngleRatio
        temp_pid.ref_agle[3] = -theta2[1] * ReductionAndAngleRatio
    elif leg_id == 3:  # 右后腿
        temp_pid.ref_agle[6] = -theta1[3] * ReductionAndAngleRatio
        temp_pid.ref_agle[7] = -theta2[3] * ReductionAndAngleRatio
    
    #计算每个电机的最终旋转圈数
    pos[0] = deg_2_cir(temp_pid.ref_agle[0])  - deg_2_cir(leg1_angle)-deg_2_cir(angle_front)#-0.25
    pos[1] = deg_2_cir(temp_pid.ref_agle[1]) - deg_2_cir(leg1_angle)+deg_2_cir(angle_front)#-0.25
    pos[2] = deg_2_cir(temp_pid.ref_agle[2])  + deg_2_cir(leg2_angle)+deg_2_cir(angle_front)#+0.25
    pos[3] = deg_2_cir(temp_pid.ref_agle[3])  + deg_2_cir(leg2_angle)-deg_2_cir(angle_front)#+0.25
    pos[4] = -deg_2_cir(temp_pid.ref_agle[4])  + deg_2_cir(leg3_angle)-deg_2_cir(angle_back)#+0.25
    pos[5] = -deg_2_cir(temp_pid.ref_agle[5])  + deg_2_cir(leg3_angle)+deg_2_cir(angle_back)#+0.25
    pos[6] = -deg_2_cir(temp_pid.ref_agle[6])  - deg_2_cir(leg4_angle)+deg_2_cir(angle_back)#-0.25
    pos[7] = -deg_2_cir(temp_pid.ref_agle[7])  - deg_2_cir(leg4_angle)-deg_2_cir(angle_back)#-0.25


"""
第一次翻转
"""
Jump_Value_shangtaijie =0
jump_count = 0
sum_angle_front = 0
sum_angle_back = 0
stance_offest = 0
taijie_over1 = 0
taijie_over2 = 0
def shangtaijie_1():
    #time
    prep_time_front = 0.125#0.25
    prep_time_back = 0.125
    prep_time = prep_time_front+prep_time_back
    jump_time = 0.4
    land_time = 0.25
     #stance_height
    stance_height = 21.8
    #prep，第一次翻转
    prep_front_angle = 0
    prep_back_angle  = 0
    prep_offset_pront = [0,0,40,40]#20,20,70,70
    prep_offset_back  = [40,40,40,40]
    #prep,第二次翻转
    prep_offset_back_2  = [80,80,40,40]
    #jump
    jump_length = 60
    jump_offset = [0,0,0,0]
    jump_angle = 0
    num_back = 50
    #num_front = 0
    global Jump_Value_shangtaijie,jump_count,sum_angle_front,sum_angle_back,stance_offest,taijie_over1
    t =Jump_Value_shangtaijie/ 1000.0
    Jump_Value_shangtaijie+= 1
    if t<=prep_time_front:
        # print("prep1")
        for i in range(2, 4):
            x[i] = -stance_height * math.sin(prep_back_angle * math.pi / 180)
            y[i] = stance_height * math.cos(prep_back_angle * math.pi / 180)
        for i in range(2):
            x[i] = 0
            y[i] = stance_height 
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(prep_offset_pront)
        slow_pre()
    if  prep_time_front<t <= prep_time_front+prep_time_back:
        # print("prep2")
        for i in range(2, 4):
            x[i] = -stance_height * math.sin(prep_back_angle * math.pi / 180)
            y[i] = stance_height * math.cos(prep_back_angle * math.pi / 180)
        for i in range(2):
            x[i] = -stance_height * math.sin(prep_back_angle * math.pi / 180)
            y[i] = stance_height * math.cos(prep_back_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_v_jump(prep_offset_back)
        slow_pre()
    if prep_time<t<=jump_time:
        for i in range (2,4):
            if jump_count<=num_back:
                x[i] = 0
                y[i] =  jump_length 
                jump_count +=1
            elif jump_count>num_back:
                x[i] = 0
                y[i] = stance_height-5
                if sum_angle_back<180:
                    sum_angle_back +=8
                else:
                    sum_angle_back =180
        for i in range(2):
            x[i] = 0
            y[i] = stance_height
            if sum_angle_front<180:
                sum_angle_front +=1.5
            else:
                sum_angle_front =180
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_shangtaijie(prep_offset_back,sum_angle_front,sum_angle_back)
        slow_text()
    if prep_time+jump_time<t<=prep_time+jump_time+land_time:
        for i in range(2):
            x[i] = 0
            y[i] = stance_height+12
        for i in range(2,4):
            x[i] = 0
            y[i] = stance_height -5
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_shangtaijie(prep_offset_back,sum_angle_front,sum_angle_back)
        slow_pre()
    elif t>prep_time+jump_time+land_time:
        taijie_over1 = 1
sum_angle_front_2= 180
sum_angle_back_2 = 180
jump_count_2 =0
Jump_Value_shangtaijie_2 =0
def shangtaijie_2():
    #time
    prep_1 = 0.05
    prep_2 = 0.25
    jump_time = 0.4
    land_time = 0.25
     #stance_height
    stance_height = 21.8
    #prep，第一次翻转
    prep_offset_pront = [0,0,40,40]#20,20,70,70
    prep_offset_back  = [40,40,40,40]
    #prep,第二次翻转
    prep_offset_back_2  = [60,60,50,50]
    #jump
    jump_length = 200
    jump_angle = 0
    num_back = 90
    #num_front = 0
    global Jump_Value_shangtaijie_2,jump_count_2,sum_angle_front_2,sum_angle_back_2,stance_offest,taijie_over2
    t =Jump_Value_shangtaijie_2/ 1000.0
    Jump_Value_shangtaijie_2+= 1
    if t<=prep_1:
        for i in range(2):
            x[i] = 0
            y[i] = stance_height+12
        for i in range(2,4):
            x[i] = 0
            y[i] = stance_height -5
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_shangtaijie(prep_offset_back,180,180)
        slow_pre()
    if prep_1<t<=prep_2+prep_1:
        #print("prep2")
        for i in range(2, 4):
            x[i] = -(stance_height-5) * math.sin(jump_angle * math.pi / 180)
            y[i] =  (stance_height-5) * math.cos(jump_angle * math.pi / 180)
        for i in range(2):
            x[i] = -stance_height * math.sin(jump_angle * math.pi / 180)
            y[i] =  stance_height * math.cos(jump_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_shangtaijie(prep_offset_back_2,180,180)
    if prep_2+prep_1<t<= prep_2+prep_1+jump_time:
        for i in range (2):
            if jump_count_2<=num_back:
                x[i] = -jump_length * math.sin(jump_angle * math.pi / 180)
                y[i] =  jump_length * math.cos(jump_angle * math.pi / 180)
                jump_count_2 +=1
                prep_offset_back=[0,0,40,40]
            elif jump_count_2>num_back:
                x[i] = 0
                y[i] = stance_height-5
                if sum_angle_front_2>0:
                    sum_angle_front_2 -=18
                else:
                    sum_angle_front_2 =0
        for i in range(2,4):
            #"""""""
            if jump_count<= num_back:
                x[i] = -(stance_height-5) * math.sin(jump_angle * math.pi / 180)
                y[i] =  (stance_height-5) * math.cos(jump_angle * math.pi / 180)
            elif jump_count > num_back:
                x[i] = 0
                y[i] = stance_height -5
            #""""""
            if sum_angle_back_2>0:
                sum_angle_back_2 -=0.75
            else:
                sum_angle_back_2 =0
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_shangtaijie(prep_offset_back,sum_angle_front_2,sum_angle_back_2)
        slow_text()
        #slow_pre()
    if prep_1+prep_2+jump_time<t<=prep_2+prep_1+jump_time+land_time:
        for i in range(2):
            x[i] = 0
            y[i] = stance_height-5
        for i in range(2,4):
            x[i] = 0
            y[i] = stance_height +12
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_shangtaijie(prep_offset_back,0,0)
        slow_pre()
    elif t>prep_2+prep_1+jump_time+land_time:
        taijie_over2 =1


sum_angle_front_3= 0
sum_angle_back_3= 0
jump_count_3 =0
Jump_Value_shangtaijie_3 =0
taijie_over3= 0
def shangtaijie_3():
    #time
    prep_1 = 0.25
    prep_2 = 0.25
    jump_time = 0.4
    land_time = 0.25
     #stance_height
    stance_height = 21.8
    #prep，第一次翻转
    prep_offset_pront = [0,0,40,40]#20,20,70,70
    prep_offset_back  = [40,40,40,40]
    #prep,第二次翻转
    prep_offset_back_2  = [70,70,50,50]
    #jump
    jump_length = 200
    jump_angle = 10
    num_back = 90
    num_front =  20
    global Jump_Value_shangtaijie_3,jump_count_3,sum_angle_front_3,sum_angle_back_3,stance_offest,taijie_over3
    t =Jump_Value_shangtaijie_3/ 1000.0
    Jump_Value_shangtaijie_3+= 1
    if t<=prep_1:
        for i in range(2):
            x[i] = 0
            y[i] = stance_height
        for i in range(2,4):
            x[i] = 0
            y[i] = stance_height +8
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_shangtaijie(prep_offset_back,0,0)
        slow_pre()
    if prep_1<t<=prep_2+prep_1:
        #print("prep2")
        for i in range(2):
            x[i] = -(stance_height-5) * math.sin(jump_angle * math.pi / 180)
            y[i] =  (stance_height-5) * math.cos(jump_angle * math.pi / 180)
        for i in range(2,4):
            x[i] = -stance_height * math.sin(jump_angle * math.pi / 180)
            y[i] =  stance_height * math.cos(jump_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_shangtaijie(prep_offset_back_2,0,0)
        #slow_pre()
    if prep_2+prep_1<t<= prep_2+prep_1+jump_time:
        for i in range (2,4):
            if jump_count_3<=num_back:
                x[i] = -jump_length * math.sin(jump_angle * math.pi / 180)
                y[i] =  jump_length * math.cos(jump_angle * math.pi / 180)
                jump_count_3 +=1
                prep_offset_back_2 = [70,70,0,0]
            elif jump_count_3>num_back:
                x[i] = 0
                y[i] = stance_height-5
                if sum_angle_back_3<180:
                    sum_angle_back_3 +=18
                else:
                    sum_angle_back_3 =180
        for i in range(2):
            #"""""""
            if jump_count_3<= num_front:
                x[i] = -(stance_height-5) * math.sin(jump_angle * math.pi / 180)
                y[i] =  (stance_height-5) * math.cos(jump_angle * math.pi / 180)
                sum_angle_front_3 = 0
            elif jump_count_3 > num_front:
                x[i] = 0
                y[i] = stance_height 
                if sum_angle_front_3<180:
                    sum_angle_front_3 +=2.5
                else:
                    sum_angle_front_3 =180
            #""""""
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_shangtaijie(prep_offset_back_2,sum_angle_front_3,sum_angle_back_3)
        slow_text()
        #slow_pre()
    if prep_1+prep_2+jump_time<t<=prep_2+prep_1+jump_time+land_time:
        for i in range(2):
            x[i] = 0
            y[i] = stance_height+12
        for i in range(2,4):
            x[i] = 0
            y[i] = stance_height -5
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_shangtaijie(prep_offset_back,180,180)
        slow_pre()
    elif t>prep_2+prep_1+jump_time+land_time:
        taijie_over3 =1
sum_angle_front_4= 180
sum_angle_back_4 = 180
jump_count_4 =0
Jump_Value_shangtaijie_4 =0
taijie_over4 = 0
def shangtaijie_4():
    #time
    prep_1 = 0.05
    prep_2 = 0.25
    jump_time = 0.4
    land_time = 0.25
     #stance_height
    stance_height = 21.8
    #prep，第一次翻转
    prep_offset_pront = [0,0,40,40]#20,20,70,70
    prep_offset_back  = [40,40,40,40]
    #prep,第二次翻转
    prep_offset_back_2  = [70,70,50,50]
    offset_last  =  [0,0,0,0]
    #jump
    jump_length = 200
    jump_angle = 15
    num_back = 90
    num_front  = 10
    global Jump_Value_shangtaijie_4,jump_count_4,sum_angle_front_4,sum_angle_back_4,stance_offest,taijie_over4
    t =Jump_Value_shangtaijie_4/ 1000.0
    Jump_Value_shangtaijie_4+= 1
    if t<=prep_1:
        for i in range(2):
            x[i] = 0
            y[i] = stance_height+8
        for i in range(2,4):
            x[i] = 0
            y[i] = stance_height 
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_shangtaijie(prep_offset_back,180,180)
        slow_pre()
    if prep_1<t<=prep_2+prep_1:
        #print("prep2")
        for i in range(2, 4):
            x[i] = -(stance_height-5) * math.sin(jump_angle * math.pi / 180)
            y[i] =  (stance_height-5) * math.cos(jump_angle * math.pi / 180)
        for i in range(2):
            x[i] = -stance_height * math.sin(jump_angle * math.pi / 180)
            y[i] =  stance_height * math.cos(jump_angle * math.pi / 180)
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_shangtaijie(prep_offset_back_2,180,180)
    if prep_2+prep_1<t<= prep_2+prep_1+jump_time:
        for i in range (2):
            if jump_count_4<=num_back:
                x[i] = -jump_length * math.sin(jump_angle * math.pi / 180)
                y[i] =  jump_length * math.cos(jump_angle * math.pi / 180)
                jump_count_4 +=1
                prep_offset_back=[0,0,40,40]
            elif jump_count_4>num_back:
                x[i] = 0
                y[i] = stance_height-5
                if sum_angle_front_4>0:
                    sum_angle_front_4 -=18
                else:
                    sum_angle_front_4 =0
        for i in range(2,4):
            if jump_count_4<= num_front:
                x[i] = -(stance_height-5) * math.sin(jump_angle * math.pi / 180)
                y[i] =  (stance_height-5) * math.cos(jump_angle * math.pi / 180)
                sum_angle_back_4 = 180
            elif jump_count_4 > num_front:
                x[i] = 0
                y[i] = stance_height 
                if sum_angle_back_4>0:
                    sum_angle_back_4 -=2.5
                else:
                    sum_angle_back_4 =0
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_shangtaijie(prep_offset_back,sum_angle_front_4,sum_angle_back_4)
        slow_text()
        #slow_pre()
    if prep_1+prep_2+jump_time<t<=prep_2+prep_1+jump_time+land_time:
        for i in range(2):
            x[i] = 0
            y[i] = stance_height
        for i in range(2,4):
            x[i] = 0
            y[i] = stance_height 
        for i in range(4):
            cartesian_to_theta(i, 1.0)
        command_all_legs_shangtaijie(offset_last,0,0)
        slow_pre()
    elif t>prep_2+prep_1+jump_time+land_time:
        taijie_over4 =1


jump_count_shangtaijie = 0
jump_state = 0
def shangtaijie():#先跳跃再翻滚
    global jump_count
    global jump_state
    global jump_flag,tai
    if jump_count == 0:
        jump_state = 0
        # print("jump_state:",jump_state)
        jumptime = jump_times[jump_state]
        jumplength = jump_lengths[jump_state]
        jumpangles = jump_angles[jump_state]
        jump_angle_offset_period = state_jumpangle_offset_period[jump_state]
        execute_jump_taijie(jumptime, jumplength, jumpangles, jump_angle_offset_period)
    elif jump_count == 1 and taijie_over3 == 0:
        print("00jump_count:%d,taijie_over3:%d"%(jump_count,taijie_over3))
        shangtaijie_3()
    elif jump_count == 1 and taijie_over3 == 1:
        print("11jump_count:%d,taijie_over3:%d"%(jump_count,taijie_over3))
        shangtaijie_4()
# def posture_control_task_shangxiaoxiepo():#没有使用
#     jump_state = 6
#     print("dog_state:",jump_state)
#     jumptime = jump_times[jump_state]
#     jumplength = jump_lengths[jump_state]
#     jumpangles = jump_angles[jump_state]
#     jump_angle_offset_period = state_jumpangle_offset_period[jump_state]
#     execute_jump_1(jumptime, jumplength, jumpangles, jump_angle_offset_period)

    


 #6666666666666666666666666666666

# if __name__ == "__main__":
#     balace_control()

 # 添加到 manual.py 文件末尾

import math
import numpy as np

# 足端轨迹规划相关变量
leg_positions = [[0, 0], [0, 0], [0, 0], [0, 0]]  # 每条腿的当前目标位置 [x, y]
leg_targets = [[0, 0], [0, 0], [0, 0], [0, 0]]    # 每条腿的目标位置 [x, y]
leg_moving = [False, False, False, False]          # 每条腿是否在移动
leg_move_start = [[0, 30], [0, 30], [0, 30], [0, 30]]  # 每条腿移动开始时的位置
leg_move_time = [0.0, 0.0, 0.0, 0.0]              # 每条腿当前移动的时间
leg_total_time = [1.0, 1.0, 1.0, 1.0]             # 每条腿移动的总时间
leg_trajectory_type = ["linear"] * 4               # 每条腿的轨迹类型

def plan_leg_trajectory(leg_id, target_x = 0, target_y = 20, duration=1.0, trajectory_type="linear"):
    """
    为指定腿规划轨迹到目标位置
    :param leg_id: 腿ID (0-3)
    :param target_x: 目标X坐标
    :param target_y: 目标Y坐标
    :param duration: 移动持续时间
    :param trajectory_type: 轨迹类型 ("linear", "arc", "circle")
    """
    global leg_positions, leg_targets, leg_moving, leg_move_start, leg_move_time, leg_total_time, leg_trajectory_type
    
    # 保存移动前的位置作为起始位置
      # 使用你现有的x, y数组
    
    # 设置目标位置
    leg_targets[leg_id] = [target_x, target_y]
    
    # 设置移动参数
    leg_moving[leg_id] = True
    
    leg_total_time[leg_id] = duration
    leg_trajectory_type[leg_id] = trajectory_type

def plan_custom_trajectory(leg_id, points_list, duration=1.0):
    """
    为指定腿规划自定义轨迹
    :param leg_id: 腿ID (0-3)
    :param points_list: 轨迹点列表 [(x1,y1), (x2,y2), ...]
    :param duration: 总移动时间
    """
    global leg_positions, leg_targets, leg_moving, leg_move_start, leg_move_time, leg_total_time, leg_trajectory_type
    
    # 对于自定义轨迹，我们将其分解为多个线性段
    if len(points_list) > 1:
        leg_move_start[leg_id] = [x[leg_id], y[leg_id]]
        leg_targets[leg_id] = points_list[-1]  # 最终目标是最后一个点
        leg_moving[leg_id] = True
        leg_move_time[leg_id] = 0.0
        leg_total_time[leg_id] = duration
        leg_trajectory_type[leg_id] = "custom"
        
        # 保存自定义轨迹点（需要额外变量存储）
        if not hasattr(plan_custom_trajectory, 'custom_paths'):
            plan_custom_trajectory.custom_paths = {}
        plan_custom_trajectory.custom_paths[leg_id] = points_list

def update_leg_trajectory(leg_id, dt):
    """
    更新指定腿的轨迹位置
    :param leg_id: 腿ID
    :param dt: 时间增量
    :return: 是否还在移动
    """
    global leg_positions, leg_targets, leg_moving, leg_move_start, leg_move_time
    
    if not leg_moving[leg_id]:
        return False
    
    leg_move_time[leg_id] += dt
    progress = min(leg_move_time[leg_id] / leg_total_time[leg_id], 1.0)
    
    if progress >= 1.0:
        # 移动完成，设置到目标位置
        x[leg_id] = leg_targets[leg_id][0]
        y[leg_id] = leg_targets[leg_id][1]
        leg_moving[leg_id] = False
        return False
    else:
        # 计算中间位置
        start_x, start_y = leg_move_start[leg_id]
        target_x, target_y = leg_targets[leg_id]
        
        if leg_trajectory_type[leg_id] == "linear":
            # 线性插值
            current_x = start_x + (target_x - start_x) * progress
            current_y = start_y + (target_y - start_y) * progress
        elif leg_trajectory_type[leg_id] == "arc":
            # 弧形轨迹（抬腿动作）
            current_x = start_x + (target_x - start_x) * progress
            current_y_linear = start_y + (target_y - start_y) * progress
            # 添加Z方向的弧形抬升
            z_lift = 4 * 3 * progress * (1 - progress)  # 抬升高度，可根据需要调整
            current_y = current_y_linear + z_lift
        else:
            # 默认线性插值
            current_x = start_x + (target_x - start_x) * progress
            current_y = start_y + (target_y - start_y) * progress
        
        # 更新全局x, y数组
        x[leg_id] = current_x
        y[leg_id] = current_y
        return True

def update_all_leg_trajectories(dt=0.01):
    """
    更新所有腿部的轨迹位置
    :param dt: 时间增量
    :return: 是否还有腿部在移动
    """
    still_moving = False
    
    for leg_id in range(4):
        if update_leg_trajectory(leg_id, dt):
            still_moving = True
    
    # 更新完位置后，执行运动学计算
    for i in range(4):
        cartesian_to_theta(i, 1.0)  # 使用你现有的运动学计算函数
    
    return still_moving

def move_leg_to_target(leg_id, target_x, target_y, duration=1.0, trajectory_type="linear"):
    """
    移动指定腿到目标位置
    :param leg_id: 腿ID (0-3)
    :param target_x: 目标X坐标
    :param target_y: 目标Y坐标
    :param duration: 移动时间
    :param trajectory_type: 轨迹类型
    """
    plan_leg_trajectory(leg_id, target_x, target_y, duration, trajectory_type)

def move_multiple_legs(leg_configs):
    """
    同时移动多条腿
    :param leg_configs: {leg_id: {'target': [x, y], 'duration': t, 'type': 'linear'}, ...}
    """
    for leg_id, config in leg_configs.items():
        if 0 <= leg_id <= 3:
            target_x, target_y = config['target']
            duration = config.get('duration', 1.0)
            trajectory_type = config.get('type', 'linear')
            move_leg_to_target(leg_id, target_x, target_y, duration, trajectory_type)

def is_any_leg_moving():
    """
    检查是否有腿部在移动
    :return: 是否有腿部在移动
    """
    return any(leg_moving)

def stop_all_leg_movements():
    """
    停止所有腿部移动
    """
    global leg_moving
    leg_moving = [False, False, False, False]

def integrate_trajectory_with_control():
    """
    将轨迹规划集成到你的控制循环中
    在你的主控制循环中调用此函数
    """
    # 更新轨迹（dt需要根据你的主循环时间间隔调整）
    still_moving = update_all_leg_trajectories(0.01)  # 请根据你的实际时间步长调整
    
    return still_moving

# 使用示例（在你的控制代码中使用）：
"""
# 1. 移动单条腿到指定位置
move_leg_to_target(0, 10, 5, duration=2.0, trajectory_type="arc")

# 2. 同时移动多条腿
configs = {
    0: {'target': [5, 3], 'duration': 2.0, 'type': 'linear'},
    1: {'target': [-5, 3], 'duration': 2.0, 'type': 'arc'},
    2: {'target': [4, -2], 'duration': 1.5, 'type': 'linear'},
    3: {'target': [-4, -2], 'duration': 1.5, 'type': 'arc'}
}
move_multiple_legs(configs)

# 3. 在你的主控制循环中集成
# 在你现有的主循环中添加：
if integrate_trajectory_with_control():
    # 还有腿部在移动，继续执行
    pass
else:
    # 所有移动完成
    pass
"""


def posture_control_task():
    global dog_state
    
    # print("dog_state:",dog_state)
    detached_params = state_detached_params[dog_state]  # 每条腿的步态参数w``
    gait_angle = gait_angles[dog_state]
    if dog_state ==  States. STOP:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,0,0,0)
    if dog_state ==  States. TROT:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States. TROT_BACK:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, -1.0, -1.0, -1.0, gait_angle,1,0,0)
    if dog_state ==  States. TROT_BACK_RIGHT:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, -1.0, -1.0, -1.0, gait_angle,1,0,0)
    if dog_state ==  States. TROT_BACK_LEFT:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, -1.0, -1.0, -1.0, gait_angle,1,0,0)
    if dog_state ==  States. TROT_LEFT:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States. TROT_RIGHT:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States. ROTATE_LEFT:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, 1.0, -1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States. ROTATE_RIGHT:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, -1.0, 1.0, -1.0, gait_angle,1,0,0)
    if dog_state ==  States. SHANGPO:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,2,0,0)
    if dog_state ==  States. slow_TROT:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States.small_TROT_RIGHT:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States.small_TROT_LEFT:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States.dijia_trot:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States. dijia_left:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, 1.0, -1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States. dijia_right:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, -1.0, 1.0, -1.0, gait_angle,1,0,0)
    if dog_state ==  States.move_left:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, 1.0, 1.0, -1.0, gait_angle,1,0,0)
    if dog_state ==  States.move_right:
         gait_detached_all_legs(detached_params, 0.5, 0.0, 0.5, 0.0, 1.0, -1.0, -1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States.xiepo_TROT:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States.xiataijie:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States.taijie_trot:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States.xiaxiepo:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States.shakeng:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States.xiadaxiepo:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States.slow_xiaxiepo:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.25, 0.75, 1.0, 1.0, 1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States.move_left_1:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, 1.0, 1.0, -1.0, gait_angle,1,0,0)
    if dog_state ==  States.move_right_1:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, -1.0, -1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States.ROTATE_LEFT_1:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, -1.0, 1.0, -1.0, 1.0, gait_angle,1,0,0)
    if dog_state ==  States.ROTATE_RIGHT_1:
         gait_detached_all_legs(detached_params, 0.0, 0.5, 0.5, 0.0, 1.0, -1.0, 1.0, -1.0, gait_angle,1,0,0)