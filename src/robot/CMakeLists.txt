cmake_minimum_required(VERSION 3.8)
project(robot)

# 编译选项（可选，增强代码规范性）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找ROS2核心依赖（Python节点必需）
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# 安装Python模块（ROS2 Humble兼容，无DIRECTORY参数）
ament_python_install_package(${PROJECT_NAME})

# ========== 新增：安装SDK的so库到robot包的运行目录 ==========
# 用途: Unitree 电机 SDK 的网络库文件
install(DIRECTORY
  resource/${PROJECT_NAME}/sdk_lib/
  DESTINATION lib/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.so"
)

# 配置节点可执行文件（与setup.py的entry_points完全对应）
install(PROGRAMS
  # ${PROJECT_NAME}/serial_node.py
  ${PROJECT_NAME}/imu_process_node.py
  ${PROJECT_NAME}/unitree_go_1.py  # 新增：电机初始化脚本
  ${PROJECT_NAME}/manual.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件（如果存在launch目录则保留）
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# 测试相关（可选）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 声明功能包类型
ament_package()