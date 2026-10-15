"""ROS SI units to native SDK units, with strict input checks."""
import math

JOINT_NAMES = [f'{side}_joint_{i}' for side in ('left', 'right') for i in range(1, 8)]


def finite(values):
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite numeric input')


def joint_target(names, positions):
    if len(names) != 14 or len(set(names)) != 14 or set(names) != set(JOINT_NAMES) or len(positions) != 14:
        raise ValueError('Expected exactly the 14 named left_joint_1..7/right_joint_1..7 joints')
    finite(positions)
    values = dict(zip(names, positions))
    return [[math.degrees(values[n]) for n in JOINT_NAMES[a*7:(a+1)*7]] for a in (0, 1)]


def pose_matrix(pose, scale):
    p, q = pose.position, pose.orientation
    finite([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if abs(norm - 1) > 1e-3:
        raise ValueError('Target quaternion must have unit norm')
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), p.x/scale],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), p.y/scale],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), p.z/scale],
            [0, 0, 0, 1]]


def quaternion(m):
    # Branch on the largest component, stable also for rotations near 180 degrees.
    candidates = [1+m[0][0]+m[1][1]+m[2][2], 1+m[0][0]-m[1][1]-m[2][2],
                  1-m[0][0]+m[1][1]-m[2][2], 1-m[0][0]-m[1][1]+m[2][2]]
    k = max(range(4), key=candidates.__getitem__)
    s = 2 * math.sqrt(max(candidates[k], 0))
    if s < 1e-12:
        raise ValueError('Invalid FK rotation matrix')
    if k == 0: q = [(m[2][1]-m[1][2])/s, (m[0][2]-m[2][0])/s, (m[1][0]-m[0][1])/s, s/4]
    elif k == 1: q = [s/4, (m[0][1]+m[1][0])/s, (m[0][2]+m[2][0])/s, (m[2][1]-m[1][2])/s]
    elif k == 2: q = [(m[0][1]+m[1][0])/s, s/4, (m[1][2]+m[2][1])/s, (m[0][2]-m[2][0])/s]
    else: q = [(m[0][2]+m[2][0])/s, (m[1][2]+m[2][1])/s, s/4, (m[1][0]-m[0][1])/s]
    norm = math.sqrt(sum(v*v for v in q))
    return [v/norm for v in q]
