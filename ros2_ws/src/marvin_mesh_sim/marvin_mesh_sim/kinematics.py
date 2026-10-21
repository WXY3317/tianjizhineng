# 提供运动学计算与输入校验；长度和角度在 ROS 边界使用米、弧度，原厂接口按各自单位转换。
"""Compatibility import: mesh kinematics now uses the original Gento SDK."""
from pathlib import Path
from .gento_kinematics import GentoMeshKinematics


class MeshKinematics(GentoMeshKinematics):
    def __init__(self,sdk_root,model_root):
        root=Path(sdk_root)
        if not (root/'PYTHON_SDK/GentoRobot.py').exists():root=root.parent/'tianji-robot-SDK'
        super().__init__(root,model_root)
