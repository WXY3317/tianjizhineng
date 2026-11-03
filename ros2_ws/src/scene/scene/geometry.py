"""Parse vision targets and transform poses from BD_J6 into body_base."""
import json
import math
import re


_BARE_KEYS = re.compile(r'([,{]\s*)(position1|position2|orientation)(\s*:)')


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


def parse_targets(payload):
    """Return [(position, orientation), ...] in BD_J6, metres and xyzw.

    Accept JSON and the pictured form with bare top-level field names.
    A zero position is retained: its meaning belongs to the scene policy.
    """
    if isinstance(payload, bytes):
        payload = payload.decode('utf-8')
    if not isinstance(payload, str) or len(payload) > 4096:
        raise ValueError('Vision payload must be UTF-8 text under 4096 characters')
    payload = _BARE_KEYS.sub(r'\1"\2"\3', payload)
    value = json.loads(payload)
    if not isinstance(value, dict) or not {'position1', 'orientation'} <= set(value):
        raise ValueError('Vision payload needs position1 and orientation')
    if set(value) - {'position1', 'position2', 'orientation'}:
        raise ValueError('Unexpected vision payload field')
    q = _unit_quaternion(value['orientation'], 'orientation')
    return [(_vector(value[key], 'xyz', key), q)
            for key in ('position1', 'position2') if key in value]


def route_targets(targets):
    """Choose arm by BD_J6 Y sign; omit an all-zero second placeholder."""
    routed = []
    for index, target in enumerate(targets):
        position = target[0]
        if index == 1 and position == (0., 0., 0.):
            continue
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


def transform_target(body_position, body_orientation, target):
    """Compose T_body_base_BD_J6 with T_BD_J6_target."""
    p = tuple(float(v) for v in body_position)
    q = _unit_quaternion(dict(zip('xyzw', body_orientation)), 'BD_J6 orientation')
    if len(p) != 3 or not all(math.isfinite(v) for v in p):
        raise ValueError('BD_J6 position must contain three finite numbers')
    local, local_q = target
    vx, vy, vz = local
    qx, qy, qz, qw = q
    # Rotate the displacement by the measured BD_J6 orientation.
    tx = 2*(qy*vz - qz*vy)
    ty = 2*(qz*vx - qx*vz)
    tz = 2*(qx*vy - qy*vx)
    rotated = (vx + qw*tx + qy*tz - qz*ty,
               vy + qw*ty + qz*tx - qx*tz,
               vz + qw*tz + qx*ty - qy*tx)
    return tuple(a+b for a, b in zip(p, rotated)), quaternion_multiply(q, local_q)
