from setuptools import setup

setup(
    name='gento_control', version='0.3.0', packages=['gento_control'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/gento_control']), ('share/gento_control', ['package.xml'])], install_requires=['setuptools'], zip_safe=True,
    maintainer='Workspace maintainer', maintainer_email='maintainer@example.com',
    description='Motion task management and Cartesian/joint planning through driver RPC', license='UNLICENSED',
    entry_points={'console_scripts': ['send_target = gento_control.send_target:main', 'control = gento_control.node:main']},
)
