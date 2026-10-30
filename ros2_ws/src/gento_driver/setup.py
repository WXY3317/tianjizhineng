from setuptools import setup

setup(
    name='gento_driver', version='0.3.0', packages=['gento_driver'],
    package_data={'gento_driver': ['data/*.Cord']},
    data_files=[('share/ament_index/resource_index/packages', ['resource/gento_driver']), ('share/gento_driver', ['package.xml', 'BODY_KINEMATICS.md']), ('share/gento_driver/launch', ['launch/dual_arm.launch.py'])], install_requires=['setuptools', 'numpy>=1.21', 'scipy>=1.8'], zip_safe=False,
    maintainer='Workspace maintainer', maintainer_email='maintainer@example.com',
    description='Single SDK owner with validated trajectory action and independent safety', license='UNLICENSED',
    entry_points={'console_scripts': ['bridge = gento_driver.node:main', 'body_ik_example = gento_driver.body_kinematics_example:main']},
)
