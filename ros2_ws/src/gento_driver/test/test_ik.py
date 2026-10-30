from types import SimpleNamespace
import pytest
from gento_driver.engine import Engine
from gento_driver import sdk_ops
from test_bridge import FX_InvKineSolverParams


POSE=[[1,0,0,1],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
REFERENCE=[1.,0.,0.,0.,0.,0.,0.]


def hardware(solver):
    robot=SimpleNamespace(inverse_kinematics=solver,
        forward_kinematics=lambda arm,q:[[1,0,0,q[0]],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    engine=Engine(robot,sdk_ops,FX_InvKineSolverParams,SimpleNamespace(sdk_length_to_m=.001))
    engine.limits=[[[-180.,180.]]*7 for _ in range(2)]
    return engine


def exceeded(params):
    params.m_Output_RetJoint[1]=181.
    params.m_Output_JntExdTags[1]=1
    params.m_Output_IsJntExd=1
    params.m_Output_RunLmtN[1]=-180.
    params.m_Output_RunLmtP[1]=180.
    return -1011


def success(params):
    for i,value in enumerate(params.m_Input_IK_RefJoint):params.m_Output_RetJoint[i]=value
    params.m_Output_RetJoint[0]=1.
    return params


def test_primary_ik_keeps_previous_reference_without_extra_search():
    calls=[]
    def solver(arm,p):calls.append(list(p.m_Input_IK_RefJoint));return success(p)
    result=hardware(solver).solve([POSE,POSE],[REFERENCE,[0.]*7],moving=[0])
    assert calls==[REFERENCE]
    assert result==[REFERENCE,[0.]*7]


def test_local_reference_search_recovers_continuous_legal_solution():
    calls=[]
    def solver(arm,p):
        calls.append(list(p.m_Input_IK_RefJoint))
        return success(p) if p.m_Input_IK_RefJoint[1]>0 else exceeded(p)
    result=hardware(solver).solve([POSE,POSE],[REFERENCE,[0.]*7],moving=[0])[0]
    assert calls[0]==REFERENCE
    assert 0<result[1]<=.1
    assert max(abs(a-b) for a,b in zip(result,REFERENCE))<=1.


def test_sdk_candidate_is_resolved_and_validated_before_use():
    calls=[]
    def solver(arm,p):
        calls.append(list(p.m_Input_IK_RefJoint))
        if abs(p.m_Input_IK_RefJoint[1]-.01)<1e-10:return success(p)
        p.m_OutPut_Result_Num=1
        p.m_OutPut_AllJoint[0]=1.;p.m_OutPut_AllJoint[1]=.01
        return exceeded(p)
    result=hardware(solver).solve([POSE,POSE],[REFERENCE,[0.]*7],moving=[0])[0]
    assert any(q[1]==.01 for q in calls) and result[1]==.01


@pytest.mark.parametrize('damage',['residual','jump','singular'])
def test_fallback_never_relaxes_pose_continuity_or_singularity_checks(damage):
    def solver(arm,p):
        if p.m_Input_IK_RefJoint[1]<=0:return exceeded(p)
        success(p)
        if damage=='residual':p.m_Output_RetJoint[0]=2.
        elif damage=='jump':p.m_Output_RetJoint[2]=5.
        else:p.m_Output_IsDeg[2]=1
        return p
    with pytest.raises(ValueError,match='no continuous legal IK'):
        hardware(solver).solve([POSE,POSE],[REFERENCE,[0.]*7],moving=[0])


def test_real_limit_failure_has_joint_detail_and_bounded_search():
    calls=[]
    def solver(arm,p):calls.append(arm);return exceeded(p)
    with pytest.raises(ValueError,match=r'joint2=181.0000.*limits=\[-180.0000,180.0000\]'):
        hardware(solver).solve([POSE,POSE],[REFERENCE,[0.]*7],moving=[0])
    assert 1<len(calls)<=65


def test_stop_interrupts_fallback_before_more_ik_calls():
    calls=[]
    def solver(arm,p):
        calls.append(arm)
        if len(calls)==2:engine.cancel_requested.set()
        return exceeded(p)
    engine=hardware(solver)
    with pytest.raises(ValueError,match='cancelled'):
        engine.solve([POSE,POSE],[REFERENCE,[0.]*7],moving=[0])
    assert len(calls)==2
