# ROS 2 Python 包安装配置：注册包索引、共享资源和可执行入口，供 colcon 构建使用。
from pathlib import Path
from setuptools import setup

data = [('share/ament_index/resource_index/packages', ['resource/marvin_mesh_sim']),
        ('share/marvin_mesh_sim', ['package.xml', 'README.md'])]
for directory in ('launch', 'urdf', 'config', 'meshes', 'rviz', 'native'):
    for folder in sorted({p.parent for p in Path(directory).rglob('*') if p.is_file()}):
        data.append((f'share/marvin_mesh_sim/{folder}',
                     [str(p) for p in sorted(folder.iterdir()) if p.is_file()]))

setup(name='marvin_mesh_sim', version='0.1.0', packages=['marvin_mesh_sim'],
      data_files=data, install_requires=['setuptools'], zip_safe=True,
      maintainer='Workspace maintainer', maintainer_email='maintainer@example.com',
      description='STL arm and dexterous-hand native SDK simulation', license='Apache-2.0',
      entry_points={'console_scripts': [
          'controller = marvin_mesh_sim.node:main',
          'panel = marvin_mesh_sim.panel:main',
          'sg100_demo = marvin_mesh_sim.sg100_demo:main',
          'gento_demo = marvin_mesh_sim.gento_demo:main',
          'sdk_demo = marvin_mesh_sim.demo:main',
          'generate_model = marvin_mesh_sim.generate:main',
      ]})
