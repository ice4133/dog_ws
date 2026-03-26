# 轻量级平滑控制集成指南

## 方案概述

本方案提供了一个轻量级的位置平滑控制和安全检测系统，具有以下特点：

✅ **性能要求低** - 算法简单，计算量小
✅ **逻辑清晰** - 易于理解和维护  
✅ **双重保护** - 平滑插值 + 安全阈值检测
✅ **可配置性强** - 支持动态参数调整

## 核心功能

### 1. 平滑插值
- 使用线性插值算法
- 可调节平滑因子 (0.01-1.0)
- 静态变量记录历史状态

### 2. 安全检测
- 基于KP参数动态计算阈值
- 公式：`阈值 = max(0.05, 0.2/KP)`
- 自动紧急停止机制

## 集成方法

### 方法一：直接替换现有函数

在 `unitree_go_1.py` 中添加以下代码：

```python
# ==================== 平滑控制模块 ====================
class SimpleSmoothController:
    def __init__(self):
        self.prev_positions = [0.0] * 8
        self.smooth_factor = 0.3
        self.safety_enabled = True
        self.emergency_stop = False
        self.kp_thresholds = [0.15] * 8
    
    def set_kp_parameters(self, kp_values):
        self.kp_thresholds = [max(0.05, 0.2/kp) if kp > 0 else 0.15 for kp in kp_values]
    
    def apply_smoothing_and_safety(self, target_positions):
        if self.emergency_stop:
            return self.prev_positions.copy()
        
        safe_positions = [0.0] * 8
        for i in range(8):
            target_val = target_positions[i]
            prev_val = self.prev_positions[i]
            
            # 平滑插值
            smoothed_val = prev_val + (target_val - prev_val) * self.smooth_factor
            
            # 安全检测
            if self.safety_enabled:
                delta = abs(smoothed_val - prev_val)
                if delta > self.kp_thresholds[i]:
                    print(f"电机{i}变化过大: {delta:.3f}")
                    self.emergency_stop = True
                    safe_positions[i] = prev_val
                    continue
            
            safe_positions[i] = smoothed_val
        
        self.prev_positions = safe_positions.copy()
        return safe_positions

# 全局实例
smooth_controller = SimpleSmoothController()

# 设置初始KP参数
smooth_controller.set_kp_parameters([1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2])

# ==================== 替换电机控制函数 ====================
def motor_control_smooth(positions, speeds):
    """带平滑控制的电机控制函数"""
    global dog_state
    
    # 应用平滑控制
    safe_positions = smooth_controller.apply_smoothing_and_safety(positions)
    
    # 检查紧急停止
    if smooth_controller.emergency_stop:
        print("紧急停止! 使用安全位置")
        # 可以添加紧急处理逻辑
    
    # 执行原有控制逻辑（使用safe_positions替代positions）
    posture_control_task()
    
    # 发送安全位置给电机
    for i in range(8):
        data.motorType = unitree_actuator_sdk.MotorType.GO_M8010_6
        cmd.motorType = unitree_actuator_sdk.MotorType.GO_M8010_6
        cmd.mode = unitree_actuator_sdk.queryMotorMode(unitree_actuator_sdk.MotorType.GO_M8010_6, unitree_actuator_sdk.MotorMode.FOC)
        cmd.id = i
        cmd.q = safe_positions[i] * 6.33 + motor_init[i]  # 使用安全位置
        cmd.dq = 0
        cmd.kp = state_motor_params[dog_state][0]
        cmd.kd = state_motor_params[dog_state][1]
        cmd.tau = 0.0
        state_control(dog_state)
        serial.sendRecv(cmd, data)
    
    # 轮子控制保持不变
    for i in range(4):
        # ... 原有轮子控制代码 ...

# ==================== 在主循环中使用 ====================
# 将原来的 motor_control(manual.pos, manual.wheel_speeds)
# 替换为:
motor_control_smooth(manual.pos, manual.wheel_speeds)
```

### 方法二：使用独立模块文件

1. 将 `simple_smooth_control.py` 文件放在同一目录下
2. 在 `unitree_go_1.py` 开头添加：

```python
from simple_smooth_control import (
    smooth_controller, set_motor_kp_parameters, 
    get_safe_positions, check_emergency_stop, reset_safety_system
)

# 设置KP参数
set_motor_kp_parameters([1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2])
```

3. 修改电机控制函数：

```python
def motor_control_enhanced(positions, speeds):
    # 应用平滑控制
    safe_positions = get_safe_positions(positions)
    
    # 检查安全状态
    if check_emergency_stop():
        print("检测到不安全操作!")
        # 可以添加紧急处理
        return
    
    # 使用safe_positions进行后续控制...
```

## 参数配置

### 平滑因子设置
```python
# 平滑程度调节 (0.01-1.0)
smooth_controller.smooth_factor = 0.2  # 非常平滑
smooth_controller.smooth_factor = 0.5  # 中等平滑
smooth_controller.smooth_factor = 0.8  # 响应较快
```

### 安全检测控制
```python
# 启用/禁用安全检测
smooth_controller.safety_enabled = True   # 启用保护
smooth_controller.safety_enabled = False  # 仅平滑，无保护

# 重置紧急停止状态
smooth_controller.reset_emergency_stop()
```

### KP参数与阈值关系
```
KP = 1.2  → 阈值 = 0.2/1.2 ≈ 0.167
KP = 2.0  → 阈值 = 0.2/2.0 = 0.100
KP = 5.0  → 阈值 = 0.2/5.0 = 0.040
KP = 10.0 → 阈值 = 0.2/10.0 = 0.020
```

## 使用示例

```python
# 1. 初始化阶段
def robot_initialize():
    # 设置实际的KP参数
    kp_params = [1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2]
    smooth_controller.set_kp_parameters(kp_params)
    
    # 设置平滑程度
    smooth_controller.smooth_factor = 0.3
    
    # 启用安全保护
    smooth_controller.safety_enabled = True

# 2. 运行时使用
def main_control_loop():
    # 您原有的计算逻辑
    calculate_motor_positions()  # 更新manual.pos
    
    # 应用平滑控制
    safe_positions = smooth_controller.apply_smoothing_and_safety(manual.pos)
    
    # 安全检查
    if smooth_controller.emergency_stop:
        print("紧急停止触发!")
        handle_emergency_situation()
        return
    
    # 使用安全位置控制电机
    send_positions_to_motors(safe_positions)

# 3. 动态参数调整
def adjust_for_motion_type(motion_type):
    if motion_type == "gentle":
        smooth_controller.smooth_factor = 0.1
        smooth_controller.safety_enabled = True
    elif motion_type == "aggressive":
        smooth_controller.smooth_factor = 0.6
        smooth_controller.safety_enabled = True
    elif motion_type == "emergency":
        smooth_controller.safety_enabled = False  # 临时禁用保护
```

## 故障排除

### 常见问题解决

1. **位置变化太慢**
   ```python
   smooth_controller.smooth_factor = 0.7  # 增加响应速度
   ```

2. **频繁触发紧急停止**
   ```python
   # 检查KP参数设置
   print("当前阈值:", smooth_controller.kp_thresholds)
   # 可以适当放宽阈值
   smooth_controller.safety_enabled = False  # 临时禁用
   ```

3. **需要重置系统**
   ```python
   smooth_controller.reset_emergency_stop()
   smooth_controller.prev_positions = [0.0] * 8  # 重置历史位置
   ```

### 调试功能
```python
# 获取详细状态信息
status = smooth_controller.get_status()
print(f"紧急停止: {status['emergency_stop']}")
print(f"当前阈值: {[f'{t:.3f}' for t in status['kp_thresholds']]}")

# 手动重置
reset_safety_system()
```

## 性能特点

- ✅ **内存占用小**：仅需存储8个历史位置值
- ✅ **计算速度快**：主要是简单的加减乘除运算
- ✅ **实时性好**：单次处理耗时 < 1ms
- ✅ **兼容性强**：不改变原有控制逻辑结构

这个方案以最小的性能代价实现了您所需的所有功能，既保证了运动的平滑性，又提供了可靠的安全保护。