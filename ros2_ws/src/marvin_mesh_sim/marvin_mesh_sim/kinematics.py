from pathlib import Path
import yaml

from marvin_sim.kinematics import Kinematics


class MeshKinematics(Kinematics):
    """Actual vendor FK/IK calibrated to the mesh/MJCF arm dimensions."""
    def __init__(self, sdk_root, model_root):
        model_root = Path(model_root)
        super().__init__(sdk_root, model_root, model_root/'config/mesh_arms.MvKDCfg')
        geometry = yaml.safe_load((model_root/'config/sdk_geometry.yaml').read_text())
        for arm in self.arms:
            if not arm.set_tool_kine(geometry['flange_tool_rotation']):
                raise RuntimeError('SDK flange-frame alignment failed')
