# 检查 RPY 目标话题的单位、输入校验与位姿转换。
import math

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from std_msgs.msg import Float64MultiArray

from marvin_sim.kinematics import quaternion_matrix
from marvin_sim.node import make_rpy_pose


def test_rpy_target_conversion():
    values=[.1,.2,.3,.4,-.5,.6]
    pose=make_rpy_pose(Float64MultiArray(data=values),'J4',Time())
    assert pose.header.frame_id=='J4'
    assert [pose.pose.position.x,pose.pose.position.y,pose.pose.position.z]==values[:3]
    q=pose.pose.orientation
    actual=quaternion_matrix([q.x,q.y,q.z,q.w])
    roll,pitch,yaw=values[3:]
    cr,sr=math.cos(roll),math.sin(roll);cp,sp=math.cos(pitch),math.sin(pitch)
    cy,sy=math.cos(yaw),math.sin(yaw)
    expected=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                       [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
                       [-sp,cp*sr,cp*cr]])
    assert np.allclose(actual,expected)


@pytest.mark.parametrize('values',([0.]*5,[0.]*7,[0.,0.,0.,float('nan'),0.,0.]))
def test_invalid_rpy_target(values):
        with pytest.raises(ValueError):make_rpy_pose(Float64MultiArray(data=values),'J4',Time())
