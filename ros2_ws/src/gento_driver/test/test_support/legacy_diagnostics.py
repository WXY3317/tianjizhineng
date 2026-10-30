"""Read object errors and per-axis servo diagnostics without issuing control commands."""
import re


def joint_errors(robot, error_names, state_names, motion_active=False):
    if robot.check_link_state() != 1:
        raise RuntimeError('Controller disconnected; error feedback unavailable')
    rt = robot.get_rt_dict()
    groups = []
    for obj, name, dof, data in (
        (0, 'left', 7, rt['arms'][0]),
        (1, 'right', 7, rt['arms'][1]),
        (3, 'body', 6, rt['body']),
        (2, 'head', robot.get_ctrl_obj_dof(2), rt['head']),
    ):
        if name == 'head' and not 0 <= dof <= 3:
            raise ValueError(f'Invalid head DOF: {dof}')
        if dof == 0:
            continue
        state = robot.current_state(obj)  # SDK normalized state, not raw RT state.
        code = int(data['state']['err'])
        group = {'group': name, 'object_id': obj, 'state': state,
                 'state_name': state_names.get(state, 'Unknown state'),
                 'error_code': code,
                 'error_type': 'No error' if code == 0 else error_names.get(code, 'Unknown error'),
                 'servo_available': False, 'joints': []}
        try:
            if motion_active:
                raise RuntimeError('Servo query deferred during motion')
            ret, text = robot.get_servo_error_codes(obj)
            group['servo_query_return'] = ret
            if ret != 0:
                raise RuntimeError(f'Servo query failed: {ret}')
            rows = {}
            for line in text.splitlines():
                match = re.fullmatch(r'J(\d+):\s*(0x[0-9a-fA-F]+)\s*-\s*(.*)', line.strip())
                if match:
                    axis = int(match[1])
                    if axis in rows:
                        raise ValueError('Duplicate servo diagnostic axis')
                    rows[axis] = (int(match[2], 16), match[3])
            if any(i not in rows for i in range(1, dof + 1)):
                raise ValueError('Incomplete servo diagnostic response')
            group['joints'] = [
                {'name': f'{name}_joint_{i}', 'servo_error_code': rows[i][0],
                 'servo_error_hex': f'0x{rows[i][0]:04X}', 'error_type': rows[i][1]}
                for i in range(1, dof + 1)]
            group['servo_available'] = True
        except Exception as exc:
            group['servo_reason'] = str(exc)
        groups.append(group)
    return {'available': True, 'objects': groups}
