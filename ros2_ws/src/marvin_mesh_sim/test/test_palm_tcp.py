"""Palm TCP is attached to the fixed palm geometry, not moving fingers."""
import xml.etree.ElementTree as ET
import numpy as np
import yaml

from test_model_sdk import ROOT,PACKAGE,urdf_fk
from marvin_mesh_sim.frames import J4Frames,J4_FRAME,TCP_FRAMES


def test_palm_tcp_is_fixed_mesh_center_and_matches_flange_tool():
    root=ET.parse(PACKAGE/'urdf/marvin_mesh_hands.urdf').getroot()
    frames=J4Frames(PACKAGE)
    config=yaml.safe_load((PACKAGE/'config/palm_tcp.yaml').read_text())
    dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
    for arm,side in enumerate('LR'):
        path=ROOT/'mjcf'/config[side]['source_mesh']
        vertices=np.fromfile(path,dtype=dtype,offset=84)['vertices'].reshape(-1,3)
        expected=(vertices.min(axis=0).astype(float)+vertices.max(axis=0).astype(float))/2
        local=urdf_fk(root,f'{side}_hand_base',TCP_FRAMES[arm],{})
        assert np.allclose(local[:3,3],expected,atol=1e-12)
        assert np.allclose(local[:3,:3],np.eye(3))
        tool=urdf_fk(root,f'flange_{side}',TCP_FRAMES[arm],{})
        assert np.allclose(frames.tools[arm],tool,atol=1e-12)
        assert np.linalg.norm(tool[:3,3])>.05
        fingers={f'{side}_Joint{i}':.2 for i in range(1,12)}
        assert np.allclose(urdf_fk(root,J4_FRAME,TCP_FRAMES[arm],{}),
                           urdf_fk(root,J4_FRAME,TCP_FRAMES[arm],fingers))
