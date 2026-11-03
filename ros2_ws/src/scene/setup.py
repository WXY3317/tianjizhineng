from setuptools import setup

setup(
    name='scene', version='0.1.0', packages=['scene'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/scene']),
                ('share/scene', ['package.xml']),
                ('share/scene/launch', ['launch/scene.launch.py'])],
    install_requires=['setuptools', 'paho-mqtt>=1.5,<2'], zip_safe=True,
    maintainer='Workspace maintainer', maintainer_email='maintainer@example.com',
    description='MQTT vision targets in the robot body frame', license='UNLICENSED',
    entry_points={'console_scripts': ['vision_targets = scene.node:main']},
)
