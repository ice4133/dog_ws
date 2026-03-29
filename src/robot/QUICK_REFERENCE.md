<!-- Quick Reference Card for .so Files -->

# 📦 Unitree 电机SDK .so文件 - 快速参考

## 文件清单

```
resource/robot/sdk_lib/
├── libUnitreeMotorSDK_Arm64.so                          [ARM64平台库]
├── libUnitreeMotorSDK_Linux64.so                        [x86_64平台库]
└── unitree_actuator_sdk.cpython-38-x86_64-linux-gnu.so  [Python 3.8绑定]
```

---

## 核心概念速查

| 文件 | 大小 | 类型 | 依赖 | 用途 |
|-----|------|------|------|------|
| **libUnitreeMotorSDK_Arm64.so** | ~200KB | C/C++库 | 无 | ARM设备电机驱动 |
| **libUnitreeMotorSDK_Linux64.so** | ~200KB | C/C++库 | 无 | x86_64电机驱动 |
| **unitree_actuator_sdk.cpython-38-x86_64-linux-gnu.so** | ~50KB | Python扩展 | libUnitreeMotorSDK_Linux64.so | Python 3.8绑定层 |

---

## Python API 快速查看

### 导入
```python
from unitree_actuator_sdk import *
```

### 核心类
- `SerialPort(port)` - 串口通信
- `MotorCmd()` - 电机命令
- `MotorData()` - 电机反馈

### 核心枚举
- `MotorType.GO_M8010_6` - GO-1电机型号
- `MotorMode.FOC` - 磁场定向控制

### 核心函数
```python
queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
```

### 使用示例
```python
serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()

cmd.motorType = MotorType.GO_M8010_6
cmd.id = 0
cmd.q = 1.57  # 目标角度
cmd.kp = 1.0
cmd.kd = 0.03
cmd.tau = 0.0

serial.sendRecv(cmd, data)
```

---

## CMakeLists.txt 最小配置

```cmake
# 现有配置（已足够）
install(DIRECTORY
  resource/${PROJECT_NAME}/sdk_lib/
  DESTINATION lib/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.so"
)
```

---

## 编译安装流程

```bash
colcon build
    ↓
.so 复制到 build/robot/lib
    ↓
colcon install 或 source install/setup.bash
    ↓
Python 导入 unitree_actuator_sdk
    ↓
调用电机控制 API
```

---

## 验证检查清单

- [ ] `.so` 文件在 `resource/robot/sdk_lib/` 目录中
- [ ] `CMakeLists.txt` 有 `install(DIRECTORY` 指令
- [ ] 执行 `colcon build` 成功
- [ ] `source install/setup.bash` 生效
- [ ] `python3 -c "from unitree_actuator_sdk import *"` 成功
- [ ] `/dev/ttyUSB0` 设备可访问（或设置正确的串口）

---

## 常见命令

```bash
# 编译
colcon build

# 安装
colcon install

# 环境配置
source install/setup.bash

# 测试导入
python3 -c "from unitree_actuator_sdk import *; print('OK')"

# 运行节点
ros2 run robot motor_control

# 查看已安装.so文件
find install -name "*.so"

# 检查库依赖
ldd install/robot/lib/*.so
```

---

## 问题速查

| 问题 | 解决方案 |
|------|---------|
| 找不到模块 | `source install/setup.bash` |
| 库找不到 | `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:install/robot/lib` |
| 权限错误 | `chmod +x install/robot/lib/*.so` |
| Python版本错误 | 使用 Python 3.8+ |
| 编译失败 | 运行 `colcon build --symlink-install` |

---

## 关键路径

| 项 | 路径 |
|----|------|
| SDK库文件 | `resource/robot/sdk_lib/` |
| 安装目录 | `install/robot/lib/` |
| Python代码 | `robot/*.py` |
| 构建配置 | `CMakeLists.txt` + `setup.py` |
| 配置文件分析 | `SDK_ANALYSIS.md` |
| 集成指南 | `CMAKE_INTEGRATION_GUIDE.md` |

---

## 电机连接

```
GO-1 机器人
    ↓
12个关节电机 (ID: 0-11)
    ↓
/dev/ttyUSB0 (USB-UART)
    ↓
SerialPort('/dev/ttyUSB0')
    ↓
unitree_actuator_sdk
```

---

## 结论

✅ **已配置完成** - CMakeLists.txt 正确处理了.so文件  
✅ **可直接使用** - 无需额外修改即可运行  
✅ **便于扩展** - 提供了优化配置供参考

---
