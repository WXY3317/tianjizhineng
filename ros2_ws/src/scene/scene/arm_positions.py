"""Load named two-arm joint presets from a JSON configuration file."""
import json
import math
from pathlib import Path


def load_arm_positions(path):
    data = json.loads(Path(path).expanduser().read_text(encoding='utf-8'))
    if not isinstance(data, dict) or not data:
        raise ValueError('arm positions must be a nonempty object')
    names, positions = set(), {}
    for key, item in data.items():
        try:
            number = int(key)
        except (TypeError, ValueError) as exc:
            raise ValueError(f'Invalid arm position ID: {key!r}') from exc
        if str(number) != key or not -128 <= number <= 127:
            raise ValueError(f'Arm position ID must fit std_msgs/Int8: {key!r}')
        if not isinstance(item, dict) or not isinstance(item.get('name'), str) or not item['name']:
            raise ValueError(f'Arm position {key} needs a name')
        name = item['name']
        if name in names:
            raise ValueError(f'Duplicate arm position name: {name}')
        names.add(name)
        if any(kind in item for kind in ('body_pose', 'body_pose_ptp', 'body_joints')):
            if any(side in item for side in ('left', 'right')):
                raise ValueError(f'Arm position {key} cannot mix arms and body targets')
            body_kinds = [kind for kind in ('body_pose', 'body_pose_ptp', 'body_joints') if kind in item]
            if len(body_kinds) != 1:
                raise ValueError(f'Arm position {key} must contain one body target kind')
            if 'body_joints' in item:
                values = item['body_joints']
                if (not isinstance(values, list) or len(values) != 6 or
                        any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                            not math.isfinite(v) for v in values)):
                    raise ValueError(f'Arm position {key} body_joints needs six finite radians')
                positions[name, 'body_joints'] = tuple(float(v) for v in values)
                positions[number] = name
                continue
            pose_kind = body_kinds[0]
            pose = item[pose_kind]
            if not isinstance(pose, dict):
                raise ValueError(f'Arm position {key} body_pose must be an object')
            for field, count in (('position', 3), ('orientation', 4)):
                values = pose.get(field)
                if (not isinstance(values, list) or len(values) != count or
                        any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                            not math.isfinite(v) for v in values)):
                    raise ValueError(f'Arm position {key} body_pose {field} needs {count} finite values')
            q = pose['orientation']
            if abs(sum(v*v for v in q) - 1.0) > 1e-3:
                raise ValueError(f'Arm position {key} orientation must be a unit quaternion')
            positions[name, pose_kind] = (tuple(pose['position']), tuple(q))
        else:
            for side in ('left', 'right'):
                values = item.get(side)
                if (not isinstance(values, list) or len(values) != 7 or
                        any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                            not math.isfinite(v) for v in values)):
                    raise ValueError(f'Arm position {key} {side} needs seven finite radians')
                positions[name, side] = tuple(float(v) for v in values)
        positions[number] = name
    if (1 not in positions or positions[1] != 'pre' or
            ('pre', 'left') not in positions or ('pre', 'right') not in positions):
        raise ValueError('Arm position 1 must be named pre for the grasp sequence')
    return positions
