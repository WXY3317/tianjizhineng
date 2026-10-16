"""Compatibility import: mesh kinematics now uses the original Gento SDK."""
from pathlib import Path
from .gento_kinematics import GentoMeshKinematics


class MeshKinematics(GentoMeshKinematics):
    def __init__(self,sdk_root,model_root):
        root=Path(sdk_root)
        if not (root/'PYTHON_SDK/GentoRobot.py').exists():root=root.parent/'tianji-robot-SDK'
        super().__init__(root,model_root)
