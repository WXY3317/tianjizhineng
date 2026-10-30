from types import SimpleNamespace

import pytest

from test_support.legacy_diagnostics import joint_errors


def robot():
    rt = {'arms': [{'state': {'err': 102}}, {'state': {'err': 0}}],
          'body': {'state': {'err': 0}}, 'head': {'state': {'err': 0}}}
    def servo(obj):
        count = {0: 7, 1: 7, 2: 3, 3: 6}[obj]
        return 0, '\n'.join(f'J{i}: 0x{(0x1234 if obj == 0 and i == 2 else 0):04X} - Test fault' for i in range(1, count + 1))
    return SimpleNamespace(check_link_state=lambda: 1, get_rt_dict=lambda: rt,
                           current_state=lambda obj: 100 if obj == 0 else 1,
                           get_ctrl_obj_dof=lambda obj: 2, get_servo_error_codes=servo)


def test_object_and_joint_errors_with_actual_head_dof():
    result = joint_errors(robot(), {102: 'Servo error'}, {100: 'Error', 1: 'Position'})
    assert result['available']
    left, right, body, head = result['objects']
    assert left['error_code'] == 102 and left['error_type'] == 'Servo error'
    assert left['state_name'] == 'Error'
    assert left['joints'][1]['servo_error_code'] == 0x1234
    assert left['joints'][1]['name'] == 'left_joint_2'
    assert len(head['joints']) == 2 and len(body['joints']) == 6
    assert right['error_type'] == 'No error'


@pytest.mark.parametrize('response', [(-1, ''), (0, 'J1: 0x0000 - No error')])
def test_failed_or_incomplete_servo_data_is_not_healthy(response):
    r = robot()
    r.get_servo_error_codes = lambda obj: response
    result = joint_errors(r, {}, {})
    assert all(not group['servo_available'] and not group['joints'] for group in result['objects'])
    assert result['objects'][0]['error_type'] == 'Unknown error'


def test_servo_requests_deferred_during_motion():
    r = robot()
    r.get_servo_error_codes = lambda obj: pytest.fail('No blocking servo query during motion')
    result = joint_errors(r, {102: 'Servo error'}, {}, motion_active=True)
    assert result['objects'][0]['error_code'] == 102
    assert all(not group['servo_available'] for group in result['objects'])


def test_disconnected_diagnostics_unavailable():
    r = robot()
    r.check_link_state = lambda: 0
    with pytest.raises(RuntimeError): joint_errors(r, {}, {})
