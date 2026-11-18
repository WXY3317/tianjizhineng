"""Parse body_base object poses and derive flange targets."""
import json
import math
import re


_BARE_KEYS = re.compile(r'([,{]\s*)(label|position1|position2|orientation)(\s*:)')
VISION_LABELS = ('无遮挡正面', '无遮挡背面', '无遮挡侧面',
                 '有遮挡正面', '有遮挡背面', '有遮挡侧面')


def _vector(value, names, label):
    if not isinstance(value, dict) or set(value) != set(names):
        raise ValueError(f'{label} must contain exactly {", ".join(names)}')
    result = tuple(float(value[name]) for name in names)
    if not all(math.isfinite(v) for v in result):
        raise ValueError(f'{label} must contain finite numbers')
    return result


def _unit_quaternion(value, label):
    q = _vector(value, 'xyzw', label)
    norm = math.sqrt(sum(v*v for v in q))
    if norm < 1e-8 or abs(norm-1.) > .01:
        raise ValueError(f'{label} must be a unit quaternion')
    return tuple(v/norm for v in q)


def _parse_payload(payload):
    if isinstance(payload, bytes):
        payload = payload.decode('utf-8')
    if not isinstance(payload, str) or len(payload) > 4096:
        raise ValueError('Vision payload must be UTF-8 text under 4096 characters')
    payload = _BARE_KEYS.sub(r'\1"\2"\3', payload)
    value = json.loads(payload)
    if not isinstance(value, dict) or not {'position1', 'orientation'} <= set(value):
        raise ValueError('Vision payload needs position1 and orientation')
    if set(value) - {'label', 'position1', 'position2', 'orientation'}:
        raise ValueError('Unexpected vision payload field')
    return value


def parse_targets(payload):
    """Return positions and orientation in body_base; retain an all-zero position2."""
    value = _parse_payload(payload)
    q = _unit_quaternion(value['orientation'], 'orientation')
    return [(_vector(value[key], 'xyz', key), q)
            for key in ('position1', 'position2') if key in value]


def parse_vision_message(payload):
    """Return (label, targets) for the labeled vision protocol."""
    value = _parse_payload(payload)
    label = value.get('label')
    if not isinstance(label, str) or label not in VISION_LABELS:
        raise ValueError('Vision label must be one of: ' + ', '.join(VISION_LABELS))
    return label, parse_targets(payload)


def route_targets(targets, *, label=None):
    """Choose arm by body_base Y sign; omit an all-zero second placeholder."""
    routed = []
    if label not in (None, '无遮挡正面', '无遮挡背面'):
        raise ValueError(f'Unsupported grasp label: {label}')
    for index, target in enumerate(targets):
        position = target[0]
        if index == 1 and position == (0., 0., 0.):
            continue
        if label in ('无遮挡正面', '无遮挡背面') and index == 0 and abs(position[1]) < .05:
            raise ValueError(f'{label} position1 requires Y >= 0.05 m or <= -0.05 m')
        if position[1] == 0.:
            raise ValueError(f'position{index+1} needs nonzero Y to choose an arm')
        routed.append(('left' if position[1] > 0. else 'right', target))
    return routed


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def undo_tool_z_rotation(orientation, angle_deg):
    """Undo a local-tool Z turn, leaving the tool's Z direction unchanged."""
    q = _unit_quaternion(dict(zip('xyzw', orientation)), 'Current orientation')
    if not math.isfinite(angle_deg):
        raise ValueError('Tool Z angle must be finite')
    half = -math.radians(angle_deg) / 2.
    return quaternion_multiply(q, (0., 0., math.sin(half), math.cos(half)))


def tool_z_rotation_matrix(angle_deg):
    """3x3 active rotation about the tool's local Z axis."""
    if not math.isfinite(angle_deg):
        raise ValueError('Tool Z angle must be finite')
    angle = math.radians(angle_deg)
    c, s = math.cos(angle), math.sin(angle)
    return ((c, -s, 0.), (s, c, 0.), (0., 0., 1.))


def undo_tool_z_matrix(orientation, matrix):
    """Apply the inverse of a saved local-Z delta matrix to a current orientation."""
    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError('Tool Z rotation matrix must be 3x3')
    matrix = tuple(tuple(float(value) for value in row) for row in matrix)
    if not all(math.isfinite(value) for row in matrix for value in row):
        raise ValueError('Tool Z rotation matrix must be finite')
    angle = math.degrees(math.atan2(matrix[1][0], matrix[0][0]))
    expected = tool_z_rotation_matrix(angle)
    if any(abs(matrix[i][j] - expected[i][j]) > 1e-5
           for i in range(3) for j in range(3)):
        raise ValueError('Saved matrix is not a local Z rotation')
    return undo_tool_z_rotation(orientation, angle)


def joint5_restore_waypoints(current_joint5_rad, angle_deg, max_step_deg=90.):
    """Joint-5 positions to undo +local-Z angle when wrist joints 6 and 7 are zero."""
    if not all(math.isfinite(v) for v in (current_joint5_rad, angle_deg, max_step_deg)):
        raise ValueError('Joint-5 restore values must be finite')
    if not 0. < max_step_deg <= 90.:
        raise ValueError('Joint-5 restore step must be at most 90 degrees')
    target = current_joint5_rad - math.radians(angle_deg)
    if not -2.9671 <= target <= 2.9671:
        raise ValueError('Joint-5 restore target exceeds model joint limits')
    count = max(1, math.ceil(abs(angle_deg) / max_step_deg))
    return [current_joint5_rad + (target - current_joint5_rad) * i / count
            for i in range(1, count + 1)]


def tool_z_rotations(orientation, step_deg=15):
    """Yield distinct tool orientations with the same local +Z direction.

    Try the requested orientation first, then the smallest positive and negative
    local-Z changes before increasing the absolute angle.
    """
    if not 0 < step_deg <= 90 or 360 % step_deg:
        raise ValueError('tool Z rotation step must divide 360 degrees')
    q = _unit_quaternion(dict(zip('xyzw', orientation)), 'Tool orientation')
    angles = [0.]
    for index in range(1, int(180 / step_deg)):
        angles.extend((index * step_deg, -index * step_deg))
    angles.append(180.)
    for angle in angles:
        half = math.radians(angle) / 2
        candidate = quaternion_multiply(q, (0., 0., math.sin(half), math.cos(half)))
        yield angle, candidate


def rotate_vector(q, vector):
    """Rotate a three-vector by an xyzw unit quaternion."""
    vx, vy, vz = vector
    qx, qy, qz, qw = q
    tx = 2*(qy*vz - qz*vy)
    ty = 2*(qz*vx - qx*vz)
    tz = 2*(qx*vy - qy*vx)
    return (vx + qw*tx + qy*tz - qz*ty,
            vy + qw*ty + qz*tx - qx*tz,
            vz + qw*tz + qx*ty - qy*tx)


def rotation_matrix(q):
    """Return the 3x3 body-frame rotation matrix for an xyzw quaternion."""
    q = _unit_quaternion(dict(zip('xyzw', q)), 'Orientation')
    return tuple(tuple(rotate_vector(q, axis)[row] for axis in
                       ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)))
                 for row in range(3))


def flange_target_from_tip(tip_position, tip_orientation, tool_offset_z_m):
    """Convert a tip target to a flange target for parallel tool/flange axes."""
    if not math.isfinite(tool_offset_z_m):
        raise ValueError('Tool tip offset must be finite')
    q = _unit_quaternion(dict(zip('xyzw', tip_orientation)), 'Tool tip orientation')
    offset = rotate_vector(q, (0., 0., tool_offset_z_m))
    return tuple(value - delta for value, delta in zip(tip_position, offset)), q


def workpiece_pose_from_flange(flange_position, flange_orientation, tool_offset_z_m,
                               tool_z_rotation_deg):
    """Estimate the rigidly attached workpiece pose in body_base from flange FK."""
    position = _vector(dict(zip('xyz', flange_position)), 'xyz', 'Flange position')
    q = _unit_quaternion(dict(zip('xyzw', flange_orientation)), 'Flange orientation')
    if not all(math.isfinite(v) for v in (tool_offset_z_m, tool_z_rotation_deg)):
        raise ValueError('Tool offset and rotation must be finite')
    offset = rotate_vector(q, (0., 0., tool_offset_z_m))
    half = -math.radians(tool_z_rotation_deg) / 2.
    original_orientation = quaternion_multiply(q, (0., 0., math.sin(half), math.cos(half)))
    return tuple(a + b for a, b in zip(position, offset)), original_orientation
