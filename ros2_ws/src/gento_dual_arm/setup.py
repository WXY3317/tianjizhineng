# ROS 2 Python 包安装配置：注册包索引、共享资源和可执行入口，供 colcon 构建使用。
from glob import glob
from setuptools import setup
setup(
    name='gento_dual_arm', version='0.1.0', packages=['gento_dual_arm'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/gento_dual_arm']),
                ('share/gento_dual_arm', ['package.xml']),
                ('share/gento_dual_arm/launch', glob('launch/*.launch.py'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Workspace maintainer', maintainer_email='maintainer@example.com',
    description='Gento SDK dual-arm control and feedback', license='UNLICENSED',
    entry_points={'console_scripts': ['bridge = gento_dual_arm.node:main', 'send_target = gento_dual_arm.send_target:main']},
)
