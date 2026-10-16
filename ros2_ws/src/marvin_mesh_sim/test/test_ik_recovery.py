"""Recorded right-arm J4 target rejected by a single reference-dependent IK."""
from pathlib import Path
import numpy as np
import pytest

from marvin_mesh_sim.frames import J4Frames
from marvin_mesh_sim.gento_kinematics import GentoMeshKinematics
from marvin_sim.panel import rpy_matrix
from marvin_sim.kinematics import pose_error

ROOT=Path(__file__).resolve().parents[4]
MODEL=ROOT/'ros2_ws/src/marvin_mesh_sim'
REFERENCE_DEG=[47.3818579,-14.2803343,164.183647,-129.266066,-106.884656,31.7198193,30.3822366]


def screenshot_target():
    target=np.eye(4)
    target[:3,3]=np.array([481.349787,136.538551,347.419851])/1000.
    target[:3,:3]=rpy_matrix([166.600134,-7.594840,-90.867799])
    return target


def test_recorded_right_j4_target_recovers_from_reference_joint_limit():
    kine=GentoMeshKinematics(ROOT/'tianji-robot-SDK',MODEL)
    frames=J4Frames(MODEL);body=np.zeros(8);target=screenshot_target()
    result=kine.ik(1,frames.ik_target(1,target,body),np.radians(REFERENCE_DEG))
    actual=frames.fk(1,kine.fk(1,result),body)
    dp,dr=pose_error(actual,target)
    assert dp<1e-5 and dr<1e-4
    kine.validate_joints(1,result)


def test_unreachable_target_is_still_rejected():
    kine=GentoMeshKinematics(ROOT/'tianji-robot-SDK',MODEL)
    target=np.eye(4);target[0,3]=5.
    with pytest.raises(ValueError):kine.ik(1,target,np.radians(REFERENCE_DEG))
