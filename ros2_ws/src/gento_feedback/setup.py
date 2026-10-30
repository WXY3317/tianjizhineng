from setuptools import setup

setup(
    name='gento_feedback', version='0.3.0', packages=['gento_feedback'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/gento_feedback']), ('share/gento_feedback', ['package.xml'])], install_requires=['setuptools'], zip_safe=True,
    maintainer='Workspace maintainer', maintainer_email='maintainer@example.com',
    description='Raw-state aggregation, diagnostics and trajectory recording', license='UNLICENSED',
    entry_points={'console_scripts': ['feedback = gento_feedback.node:main']},
)
