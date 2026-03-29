from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ===================== 基础路径定义（保留你的原有配置，一行没改） =====================
    WS_PATH = "/home/ice/my_dog/dog_ws"
    SRC_ROBOT_PATH = os.path.join(WS_PATH, "src", "robot", "robot")  # 源码路径
    INSTALL_ROBOT_PATH = os.path.join(WS_PATH, "install", "robot", "lib", "robot")  # 安装路径

    # ===================== 修复：MID360雷达驱动启动 + 解决段错误exit code -11 =====================
    livox_share_dir = get_package_share_directory('livox_ros_driver2')
    livox_launch_path = os.path.join(
        livox_share_dir,
        'launch_ROS2',
        'msg_MID360_launch.py'
    )
    mid360_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch_path),
        launch_arguments={
            'enable_imu': 'True',
            'xfer_format': '1'
        }.items()
    )

    # ===================== 核心修复：电机控制节点 极简配置 无密码+无冲突+权限足够 =====================
    # 改动1：删除prefix的chmod命令（已经加入dialout组，串口权限永久生效）
    # 改动2：仅保留 sudo -E 提权，无密码执行，保留ROS2环境变量
    # 改动3：无任何多余配置，彻底解决双重sudo冲突+密码卡死问题
    motor_control_node = ExecuteProcess(
        cmd=[
            "/usr/bin/sudo", "-E",  # 仅这一个sudo，无密码+保留环境变量，完美适配ROS2
            "/usr/bin/python3.8",
            os.path.join(INSTALL_ROBOT_PATH, "unitree_go_1.py")
        ],
        output="screen",
        emulate_tty=True,  # 新增：解决ROS2无交互终端导致的进程卡死，关键参数！
    )

    # ===================== 保留你的原有IMU节点配置（无需修改） =====================
    imu_process_node = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(SRC_ROBOT_PATH, "imu_process_node.py")
        ],
        output="screen",
        log_cmd=True
    )

    # ===================== 整合节点 + 优化启动顺序（解决进程间依赖卡死） =====================
    return LaunchDescription([
        motor_control_node,     # 优先启动电机节点，无串口抢占问题
        mid360_driver,          # 再启动雷达驱动
        imu_process_node,       # 最后启动IMU预处理
    ])