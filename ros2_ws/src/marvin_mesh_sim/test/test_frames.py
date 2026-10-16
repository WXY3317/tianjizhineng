"""J4 FK/IK compared with independently accumulated full URDF transforms."""
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from marvin_mesh_sim.frames import J4Frames,J4_FRAME
from marvin_mesh_sim.gento_kinematics import GentoMeshKinematics
from marvin_sim.kinematics import INITIAL_DEG
from test_model_sdk import ROOT,PACKAGE,urdf_fk


def test_j4_fk_ik_both_arms_matches_urdf_after_torso_motion():
    frames=J4Frames(PACKAGE);kine=GentoMeshKinematics(ROOT/'tianji-robot-SDK',PACKAGE)
    root=ET.parse(PACKAGE/'urdf/marvin_mesh_hands.urdf').getroot()
    for body in (np.zeros(8),np.array([.1,.1,-.05,.1,.12,-.1,.05,0.])):
        for arm,side in enumerate('LR'):
            q=np.radians(INITIAL_DEG[arm])
            values=dict(zip(frames.body_names,body))
            values.update(dict(zip([f'Joint{i}_{side}' for i in range(1,8)],q)))
            expected=urdf_fk(root,J4_FRAME,f'flange_{side}',values)
            actual=frames.fk(arm,kine.fk(arm,q),body)
            assert np.allclose(actual,expected,atol=1e-9)
            local=frames.ik_target(arm,actual,body)
            solution=kine.ik(arm,local,q)
            assert np.allclose(frames.fk(arm,kine.fk(arm,solution),body),expected,atol=1e-7)
    zero=np.zeros(8);downstream=zero.copy();downstream[4]=.1
    upstream=zero.copy();upstream[:4]=[.1,.1,.1,.1]
    assert np.allclose(frames.shoulder(0,zero),frames.shoulder(0,upstream))
    assert not np.allclose(frames.shoulder(0,zero),frames.shoulder(0,downstream))
    for invalid in ([0.]*7,[float('nan')]*8):
        with pytest.raises(ValueError):frames.shoulder(0,invalid)


def test_j4_ik_waits_for_downstream_torso_settle():
    kine=GentoMeshKinematics(ROOT/'tianji-robot-SDK',PACKAGE)
    rt=kine.module.ROBOT_RT()
    rt.m_BODY.m_BODY_IN.m_BODY_CMD_Joint_Pos[4]=5.
    with pytest.raises(ValueError,match='J5/J6'):
        J4Frames.body_from_rt(rt,require_stationary=True)
    rt.m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_Pos[4]=5.
    assert abs(J4Frames.body_from_rt(rt,require_stationary=True)[4]-np.radians(5.))<1e-10
    rt.m_BODY.m_BODY_OUT.m_BODY_FBK_Joint_Vel[5]=1.
    with pytest.raises(ValueError,match='J5/J6'):
        J4Frames.body_from_rt(rt,require_stationary=True)
