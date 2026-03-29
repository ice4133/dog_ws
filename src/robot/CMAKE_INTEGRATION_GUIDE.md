# CMakeLists.txt .so文件集成指南

## 📖 快速概览

你的项目已经 **正确配置**了Unitree电机SDK的.so库文件。本指南说明：
1. 当前配置的工作原理
2. 如何优化或自定义配置
3. 常见问题排查

---

## ✅ 当前状态

### 已正确配置的项
- ✅ .so文件会被安装到 `lib/robot/` 目录
- ✅ setup.py 和 CMakeLists.txt 配置一致
- ✅ Python可以正确导入 `from unitree_actuator_sdk import *`
- ✅ 文件权限会被正确设置

### 配置位置
```
当前 CMakeLists.txt 第 19-23 行：

install(DIRECTORY
  resource/${PROJECT_NAME}/sdk_lib/
  DESTINATION lib/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.so"
)
```

---

## 🎯 使用场景

### 场景1：保持现有配置（推荐）
**适用于：** 开发/测试环境

无需修改，当前配置已足够。

```bash
colcon build
source install/setup.bash
ros2 run robot motor_control
```

### 场景2：平台特定优化
**适用于：** 需要部署到多个平台（x86_64、ARM等）

使用提供的 `CMakeLists_enhanced.txt`：
```bash
# 备份原文件
cp CMakeLists.txt CMakeLists.txt.backup

# 替换为优化版本（选择方案1或方案2）
# 方案1：通用安装（推荐）- 编辑CMakeLists.txt，增加权限配置

# 或方案2：平台特定 - 取消CMakeLists_enhanced.txt中的平台检测代码注释
```

### 场景3：容器化部署
**适用于：** Docker/ROS2容器环境

添加额外的库搜索路径：
```cmake
# 在install命令后添加
set_target_properties(ament_target_dependencies PROPERTIES
  ENVIRONMENT 
    "LD_LIBRARY_PATH=$ORIGIN/../lib:$LD_LIBRARY_PATH"
)
```

---

## 🛠️ 配置选项详解

### 选项1：添加文件权限（推荐）

```cmake
install(DIRECTORY
  resource/${PROJECT_NAME}/sdk_lib/
  DESTINATION lib/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.so"
  PERMISSIONS 
    OWNER_EXECUTE OWNER_WRITE OWNER_READ
    GROUP_EXECUTE GROUP_READ
    WORLD_EXECUTE WORLD_READ
)
```

**优点：**
- 确保.so文件有正确的执行权限
- 多用户环境下兼容性更好
- 明确指定权限，避免系统默认权限问题

### 选项2：精确文件列表

```cmake
install(FILES
  resource/${PROJECT_NAME}/sdk_lib/libUnitreeMotorSDK_Arm64.so
  resource/${PROJECT_NAME}/sdk_lib/libUnitreeMotorSDK_Linux64.so
  resource/${PROJECT_NAME}/sdk_lib/unitree_actuator_sdk.cpython-38-x86_64-linux-gnu.so
  DESTINATION lib/${PROJECT_NAME}/
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ
)
```

**优点：**
- 明确指定需要安装的文件
- 易于追踪和维护
- 避免意外安装其他文件

### 选项3：平台检测

```cmake
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  # 安装ARM版本
  install(FILES
    resource/${PROJECT_NAME}/sdk_lib/libUnitreeMotorSDK_Arm64.so
    DESTINATION lib/${PROJECT_NAME}/
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  # 安装x86_64版本
  install(FILES
    resource/${PROJECT_NAME}/sdk_lib/libUnitreeMotorSDK_Linux64.so
    resource/${PROJECT_NAME}/sdk_lib/unitree_actuator_sdk.cpython-38-x86_64-linux-gnu.so
    DESTINATION lib/${PROJECT_NAME}/
  )
endif()
```

**优点：**
- 根据目标平台自动选择正确的库
- 减少安装包大小
- 提高跨平台兼容性

---

## 📥 安装流程详解

```
1. colcon build
   ↓
2. CMake解析 CMakeLists.txt
   ↓
3. install() 指令执行
   ├─ 找到 resource/robot/sdk_lib/*.so
   └─ 标记为"安装文件"
   ↓
4. colcon install（或make install）
   ↓
5. .so 文件被复制到：
   - install/robot/lib/
   - /opt/ros/<distro>/lib/ (如果使用sudo)
   ↓
6. Python运行时
   ├─ 从 PYTHONPATH 查找 unitree_actuator_sdk
   └─ 找到 .so 文件并加载
```

---

## 🔍 验证配置

### 1. 检查CMakeLists.txt配置
```bash
# 查看 install 相关的行
grep -n "install" CMakeLists.txt

# 输出应包含：
# 19:install(DIRECTORY
# 20:  resource/${PROJECT_NAME}/sdk_lib/
```

### 2. 编译并检查
```bash
# 在build目录生成的文件
colcon build
find build/robot -name "*.so"

# 输出应该是：
# build/robot/lib/libUnitreeMotorSDK_Arm64.so
# build/robot/lib/libUnitreeMotorSDK_Linux64.so
# build/robot/lib/unitree_actuator_sdk.cpython-38-x86_64-linux-gnu.so
```

### 3. 检查安装结果
```bash
# 在install目录
ls -lh install/robot/lib/*.so

# 或使用ROS2命令
ros2 pkg list | grep robot  # 验证包存在
```

### 4. 测试Python导入
```bash
# 进入ROS2环境
source install/setup.bash

# 测试导入
python3 -c "from unitree_actuator_sdk import SerialPort; print('✓ Import successful')"
```

---

## 🚨 常见问题排查

### 问题1：找不到unitree_actuator_sdk模块
```bash
ModuleNotFoundError: No module named 'unitree_actuator_sdk'
```

**解决方案：**
```bash
# 1. 确保已source ROS2环境
source install/setup.bash

# 2. 检查.so文件是否被安装
find install -name "*actuator*"

# 3. 检查Python路径
python3 -c "import sys; print('\n'.join(sys.path))"

# 应该包含：install/lib/python3.x/site-packages/robot/
```

### 问题2：ImportError: libUnitreeMotorSDK_Linux64.so找不到
```bash
ImportError: libUnitreeMotorSDK_Linux64.so: cannot open shared object file
```

**解决方案：**
```bash
# 1. 检查库文件路径
ldd install/robot/lib/unitree_actuator_sdk.cpython-38-x86_64-linux-gnu.so

# 2. 添加库搜索路径
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/install/robot/lib

# 3. 重新测试
python3 -c "from unitree_actuator_sdk import *"
```

### 问题3：权限被拒绝
```bash
PermissionError: [Errno 13] Permission denied
```

**解决方案：**
```bash
# 更新CMakeLists.txt中的PERMISSIONS设置
# 确保包含 OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE

# 手动修复权限
chmod +x install/robot/lib/*.so
```

### 问题4：Python版本不匹配
```bash
ImportError: dynamic module does not define module export function
```

**解决方案：**
```bash
# 检查Python版本
python3 --version  # 应该是 3.8+

# .so文件后缀必须匹配
# cpython-38 = Python 3.8
# cpython-39 = Python 3.9
# cpython-310 = Python 3.10

# 如果版本不匹配，需要重新编译SDK
```

---

## 💡 最佳实践

### 1. 使用绝对路径
```cmake
# ✅ 好
set(SDK_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/resource/${PROJECT_NAME}/sdk_lib")

# ❌ 避免
set(SDK_LIB_DIR "resource/${PROJECT_NAME}/sdk_lib")
```

### 2. 检查文件存在
```cmake
# ✅ 好
if(EXISTS "${SDK_LIB_DIR}")
  install(DIRECTORY...)
else()
  message(WARNING "SDK library not found")
endif()

# ❌ 避免
install(DIRECTORY "${SDK_LIB_DIR}")  # 可能静默失败
```

### 3. 明确指定权限
```cmake
# ✅ 好
PERMISSIONS 
  OWNER_EXECUTE OWNER_WRITE OWNER_READ
  GROUP_EXECUTE GROUP_READ
  WORLD_EXECUTE WORLD_READ

# ❌ 避免
PERMISSIONS OWNER_ALL GROUP_ALL WORLD_ALL  # 精度不够
```

### 4. 添加调试消息
```cmake
# ✅ 好
message(STATUS "Installing SDK libraries from: ${SDK_LIB_DIR}")
message(STATUS "Target destination: lib/${PROJECT_NAME}")

# ❌ 避免
# 无日志，难以调试
```

---

## 📝 推荐的更新步骤

如果要应用优化配置：

### 步骤1：备份
```bash
cp CMakeLists.txt CMakeLists.txt.backup
```

### 步骤2：选择一个选项

#### 选项A：最小改动（推荐用于现有工作项目）
只在install中添加权限：
```cmake
install(DIRECTORY
  resource/${PROJECT_NAME}/sdk_lib/
  DESTINATION lib/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.so"
  PERMISSIONS 
    OWNER_EXECUTE OWNER_WRITE OWNER_READ
    GROUP_EXECUTE GROUP_READ
    WORLD_EXECUTE WORLD_READ
)
```

#### 选项B：平台特定优化（推荐用于多平台部署）
使用 `CMakeLists_enhanced.txt` 中的平台检测代码

#### 选项C：完整增强版（最佳实践）
完全替换为 `CMakeLists_enhanced.txt`

### 步骤3：测试
```bash
# 清理旧构建
rm -rf build install

# 重新构建
colcon build

# 测试
source install/setup.bash
python3 -c "from unitree_actuator_sdk import *; print('✓ Success')"
```

### 步骤4：验证功能
```bash
# 运行电机控制节点
ros2 run robot motor_control

# 或运行手动控制
ros2 run robot manual
```

---

## 🔗 相关文件

- 📄 [SDK_ANALYSIS.md](./SDK_ANALYSIS.md) - 详细的SDK分析
- 📄 [CMakeLists.txt](./CMakeLists.txt) - 当前配置
- 📄 [CMakeLists_enhanced.txt](./CMakeLists_enhanced.txt) - 推荐的增强版本
- 🐍 [unitree_go_1.py](./robot/unitree_go_1.py) - 电机控制脚本示例
- 🐍 [setup.py](./setup.py) - Python包配置

---

## ❓ 何时需要修改CMakeLists.txt

| 场景 | 需要修改 | 建议 |
|------|---------|------|
| 开发环境 | ❌ 否 | 保持现有配置 |
| 生产部署 | ✅ 是 | 添加权限配置 |
| 多平台部署 | ✅ 是 | 使用平台检测 |
| 库搜索路径问题 | ✅ 是 | 显式设置LD_LIBRARY_PATH |
| 容器化部署 | ✅ 是 | 调整安装路径 |

---

## 📞 需要帮助？

```bash
# 查看完整的CMake变量
cmake -L build/robot | grep SDK

# 运行CMake调试模式
cmake --debug-output build/robot

# 检查install指令的实际操作
cmake --install build/robot --verbose

# 启用详细的colcon输出
colcon build --event-handlers console_direct+

# 获取ROS2环境信息
env | grep ROS
env | grep PYTHON
```

---

