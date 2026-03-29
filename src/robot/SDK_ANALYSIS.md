# Unitree 电机SDK .so文件分析

## 📋 一、.so文件概述

### 文件清单
位置：`resource/robot/sdk_lib/`

| 文件名 | 类型 | 平台 | 用途 |
|--------|------|------|------|
| **libUnitreeMotorSDK_Arm64.so** | C/C++库 | ARM64（树莓派、TX2等） | Unitree电机驱动底层库 |
| **libUnitreeMotorSDK_Linux64.so** | C/C++库 | Linux x86_64 | Unitree电机驱动底层库 |
| **unitree_actuator_sdk.cpython-38-x86_64-linux-gnu.so** | Python扩展 | Linux x86_64 Python3.8 | 电机SDK的Python绑定层 |

### 核心作用链
```
C/C++ 电机硬件协议
     ↓
libUnitreeMotorSDK_*.so (底层库)
     ↓
unitree_actuator_sdk.*.so (Python绑定)
     ↓
Python代码调用（from unitree_actuator_sdk import *）
```

---

## 🔌 二、Python使用方式分析

### 导入方式
```python
from unitree_actuator_sdk import *  # 导入所有类和常量
```

### 核心API（通过.so文件暴露）

#### 1. **SerialPort** - 串口通信类
```python
serial = unitree_actuator_sdk.SerialPort('/dev/ttyUSB0')
# 与电机进行双向通信
serial.sendRecv(cmd, data)
```

#### 2. **MotorCmd** - 电机命令结构体
```python
cmd = unitree_actuator_sdk.MotorCmd()
cmd.motorType = unitree_actuator_sdk.MotorType.GO_M8010_6
cmd.id = 0                  # 电机ID (0-11, GO_1有12个关节电机)
cmd.mode = query_mode       # 控制模式（FOC等）
cmd.q = angle               # 期望角度位置
cmd.dq = angular_velocity   # 期望角速度
cmd.kp = proportional_gain  # 比例增益
cmd.kd = derivative_gain    # 微分增益
cmd.tau = torque_feedforward # 前馈力矩
```

#### 3. **MotorData** - 电机反馈数据结构体
```python
data = unitree_actuator_sdk.MotorData()
# 通过serial.sendRecv()填充以下字段：
data.q          # 实际角度位置
data.dq         # 实际角速度
data.tau        # 实际输出力矩
data.motorType  # 电机类型
```

#### 4. **MotorType枚举** - 支持的电机型号
```python
unitree_actuator_sdk.MotorType.GO_M8010_6  # Unitree GO1的关节电机
```

#### 5. **MotorMode枚举** - 控制模式
```python
unitree_actuator_sdk.MotorMode.FOC  # 磁场定向控制（主要模式）
```

#### 6. **queryMotorMode()** - 模式查询函数
```python
cmd.mode = unitree_actuator_sdk.queryMotorMode(
    unitree_actuator_sdk.MotorType.GO_M8010_6,
    unitree_actuator_sdk.MotorMode.FOC
)
```

---

## 📊 三、使用代码示例分析

### unitree_go_1.py 中的使用方式
```python
# 初始化
serial = unitree_actuator_sdk.SerialPort('/dev/ttyUSB0')
cmd = unitree_actuator_sdk.MotorCmd()
data = unitree_actuator_sdk.MotorData()

# 控制循环
while True:
    # 设置12个关节电机
    for i in range(12):
        cmd.motorType = unitree_actuator_sdk.MotorType.GO_M8010_6
        cmd.id = i                                    # 电机ID
        cmd.mode = unitree_actuator_sdk.queryMotorMode(...)
        cmd.q = target_angle                         # 目标角度
        cmd.dq = 0.0                                # 目标角速度
        cmd.kp = 1.0                                # 比例增益
        cmd.kd = 0.03                               # 微分增益
        cmd.tau = 0.0                               # 前馈
        
        # 发送命令并接收反馈
        serial.sendRecv(cmd, data)
        
        # data 现在包含电机的实时状态
        actual_position = data.q
```

### motor_init.py 中的使用方式
```python
# 导入SDK
from unitree_actuator_sdk import *
from Robot_go import *

# 创建通信和数据对象
serial = SerialPort('/dev/ttyUSB0')  # 通过通配符导入
cmd = MotorCmd()
data = MotorData()

# 控制流程相同
```

---

## 🏗️ 四、CMakeLists.txt 当前配置

### 当前实现
```cmake
# 安装SDK的.so库到robot包的lib目录
install(DIRECTORY
  resource/${PROJECT_NAME}/sdk_lib/
  DESTINATION lib/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.so"
)
```

### 配置说明
- ✅ **已配置** - .so文件会被安装到 `/opt/ros/<distro>/lib/robot/sdk_lib/` 
- ✅ **已配置** - Python运行时可自动查找这些库（通过ROS2的环境变量）
- ✅ **setup.py配置** - 同时在setup.py中也配置了.so文件的安装路径

---

## 📌 五、工作流程

### 编译和安装流程
```
colcon build
  ↓
CMakeLists.txt 执行 install(DIRECTORY)
  ↓
.so文件复制到 build/robot/lib/robot/sdk_lib/
  ↓
colcon install
  ↓
.so文件复制到 install/robot/lib/python3.8/site-packages/robot/sdk_lib/
  ↓
或直接复制到系统Python路径
  ↓
Python代码: from unitree_actuator_sdk import *
  ↓
Python寻找sys.path中的unitree_actuator_sdk.*.so
  ↓
加载.so文件 → 暴露C++类和函数给Python
```

### 运行时依赖关系
```
unitree_actuator_sdk.cpython-38-x86_64-linux-gnu.so
  ↓ (依赖于)
libUnitreeMotorSDK_Linux64.so  (x86_64平台)
或
libUnitreeMotorSDK_Arm64.so    (ARM平台)
```

---

## ⚙️ 六、关键配置点

### 1. **串口设备路径**
```python
# 当前代码使用
serial = SerialPort('/dev/ttyUSB0')

# 需要确保：
# - /dev/ttyUSB0 存在（USB-UART适配器）
# - 用户有权限访问此设备
# - 波特率和协议配置正确
```

### 2. **Python版本匹配**
```
.so文件: cpython-38-x86_64-linux-gnu.so
        ↓
要求: Python 3.8
```

### 3. **平台匹配**
```
.so文件: x86_64-linux-gnu
        ↓
当前: Linux x86_64 (不是ARM64)
```

---

## 🎯 七、CMakeLists.txt 优化建议

### 当前配置（已经很好）
```cmake
install(DIRECTORY
  resource/${PROJECT_NAME}/sdk_lib/
  DESTINATION lib/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.so"
)
```

### 增强版本（可选）
```cmake
# 方案1：精确指定.so文件
install(FILES
  resource/${PROJECT_NAME}/sdk_lib/libUnitreeMotorSDK_Arm64.so
  resource/${PROJECT_NAME}/sdk_lib/libUnitreeMotorSDK_Linux64.so
  resource/${PROJECT_NAME}/sdk_lib/unitree_actuator_sdk.cpython-38-x86_64-linux-gnu.so
  DESTINATION lib/${PROJECT_NAME}/
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ 
              GROUP_EXECUTE GROUP_READ 
              WORLD_EXECUTE WORLD_READ
)

# 方案2：平台特定安装（根据架构选择）
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  install(FILES
    resource/${PROJECT_NAME}/sdk_lib/libUnitreeMotorSDK_Arm64.so
    DESTINATION lib/${PROJECT_NAME}/
  )
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  install(FILES
    resource/${PROJECT_NAME}/sdk_lib/libUnitreeMotorSDK_Linux64.so
    resource/${PROJECT_NAME}/sdk_lib/unitree_actuator_sdk.cpython-38-x86_64-linux-gnu.so
    DESTINATION lib/${PROJECT_NAME}/
  )
endif()
```

---

## 📚 八、使用流程总结

### 对用户（机器人开发者）
```
1. 把.py文件写好 (unitree_go_1.py, motor_init.py等)
   ↓
2. 执行 colcon build
   ↓
3. source install/setup.bash
   ↓
4. 运行 ros2 run robot motor_control
   ↓
5. Python导入: from unitree_actuator_sdk import *
   ↓
6. 使用SerialPort、MotorCmd等进行电机控制
```

### 文件依赖关系
```
robot/
├── resource/robot/sdk_lib/
│   ├── libUnitreeMotorSDK_Arm64.so ────────┐
│   ├── libUnitreeMotorSDK_Linux64.so ──┐   │
│   └── unitree_actuator_sdk.*.so ──────┼───┤
│                                        │   │
├── robot/                              │   │
│   ├── unitree_go_1.py ─────────┐      │   │
│   ├── motor_init.py ──────────┬┘      │   │
│   ├── motor_init2.py ─────────┤      │   │
│   ├── manual.py ──────────────┤      │   │
│   └── auto_obstacle.py ───────┤      │   │
│                               ↓      ↓   ↓
│                    [ from unitree_actuator_sdk import * ]
│
├── CMakeLists.txt ─→ 安装.so文件到lib/
└── setup.py ──────→ 安装.so文件到site-packages/
```

---

## ✅ 九、验证和调试

### 验证.so文件是否正确安装
```bash
# 列出已安装的.so文件
find $INSTALL_DIR -name "*.so" | grep robot

# 检查.so文件依赖
ldd install/lib/robot/unitree_actuator_sdk.cpython-38-x86_64-linux-gnu.so
```

### 测试Python导入
```python
import sys
sys.path.insert(0, '/path/to/installed/lib/robot/')
from unitree_actuator_sdk import SerialPort, MotorCmd, MotorData
print("导入成功！")
```

### 查看.so文件信息
```bash
file resource/robot/sdk_lib/unitree_actuator_sdk.cpython-38-x86_64-linux-gnu.so
# 应输出: ELF 64-bit LSB shared object, x86-64 ...
```

---

## 🎓 总结

| 项目 | 说明 |
|-----|------|
| **文件性质** | 预编译的二进制库（无需编译） |
| **主要用途** | 与Unitree GO-1机器人的12个关节电机进行串口通信 |
| **API方式** | Python C扩展模块，暴露SerialPort、MotorCmd等类 |
| **当前CMake配置** | ✅ 已正确配置安装 |
| **是否需要修改** | 否（现有配置已完整） |
| **可选优化** | 添加平台检测、权限设置、依赖检查 |

---

