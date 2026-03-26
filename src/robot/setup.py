from setuptools import setup
import os

package_name = 'robot'

# 容错处理SDK库文件
sdk_lib_dir = os.path.join('resource', package_name, 'sdk_lib')
sdk_lib_files = []
if os.path.exists(sdk_lib_dir):
    sdk_lib_files = [
        os.path.join(sdk_lib_dir, f) 
        for f in os.listdir(sdk_lib_dir) 
        if f.endswith('.so') and os.path.isfile(os.path.join(sdk_lib_dir, f))
    ]

# ========== 新增：收集launch文件 ==========
launch_files = []
launch_dir = os.path.join('launch')
if os.path.exists(launch_dir):
    launch_files = [
        os.path.join(launch_dir, f) 
        for f in os.listdir(launch_dir) 
        if f.endswith('.py') and os.path.isfile(os.path.join(launch_dir, f))
    ]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ========== 核心：安装launch文件到share目录 ==========
        ('share/' + package_name + '/launch', launch_files),
        # SDK库文件
        (os.path.join('lib', package_name, 'sdk_lib'), sdk_lib_files),
    ],
    install_requires=['setuptools', 'pyserial>=3.5', 'numpy==1.24.4'],
    zip_safe=False,
    maintainer='i-mini900',
    maintainer_email='your_email@example.com',
    description='ROS2 robot package with Unitree Actuator SDK',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "imu_process = robot.imu_process_node:main",
            "motor_control = robot.unitree_go_1:main",
            "manual = robot.manual:main",

        ],
    },
)