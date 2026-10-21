# ROS 2 Python 包安装配置：注册包索引、共享资源和可执行入口，供 colcon 构建使用。
from glob import glob
from setuptools import setup

setup(
    name='marvin_sim', version='0.1.0', packages=['marvin_sim'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/marvin_sim']),
        ('share/marvin_sim', ['package.xml']),
        ('share/marvin_sim/launch', glob('launch/*.launch.py')),
        ('share/marvin_sim/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Workspace maintainer', maintainer_email='maintainer@example.com',
    description='Native SDK local controller and Marvin kinematic simulator',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'controller = marvin_sim.node:main',
        'panel = marvin_sim.panel:main',
        'sdk_demo = marvin_sim.demo:main',
    ]},
)
