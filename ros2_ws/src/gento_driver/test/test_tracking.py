from types import SimpleNamespace
from gento_driver.sdk_ops import tracking_sample


def test_tracking_uses_controller_feedback_command_and_preserves_actual():
    robot=SimpleNamespace(forward_kinematics=lambda arm,q:[[1,0,0,q[0]],[0,1,0,q[1]],[0,0,1,0],[0,0,0,1]])
    actual=[2.]*7
    result=tracking_sample(robot,0,{'cmd_pos':[1.]*7,'fb_vel':[.5]*7},actual,True)
    assert result['available'] and result['following_error_deg']==[1.]*7
    assert result['command_matrix'][0][3]==1.
    assert actual==[2.]*7


def test_unavailable_command_telemetry_is_reported_without_using_input_mailbox():
    robot=SimpleNamespace(forward_kinematics=lambda *args:None)
    for feedback in ({},{'cmd_pos':[float('nan')]*7},{'cmd_pos':[1.]*6}):
        assert not tracking_sample(robot,0,feedback,[0.]*7)['available']
    result=tracking_sample(robot,0,{'cmd_pos':[0.]*7},[0.]*7)
    assert result['available'] and result['command_matrix'] is None
