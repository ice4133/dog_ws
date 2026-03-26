from setuptools import find_packages
from setuptools import setup

setup(
    name='unitree_legged_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('unitree_legged_msgs', 'unitree_legged_msgs.*')),
)
