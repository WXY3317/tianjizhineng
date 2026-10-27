"""Offline Cartesian interpolation and sequential IK, sampled at 50 Hz."""
import ctypes
import math
from types import SimpleNamespace

from .conversions import pose_matrix, quaternion

FREQUENCY = 50


def interpolate(start, goal, fraction):
    """Linear position and shortest-arc quaternion SLERP, in SDK units."""
    a, b = quaternion(start), quaternion(goal)
    dot = sum(x*y for x, y in zip(a, b))
    if dot < 0:
        b, dot = [-x for x in b], -dot
    angle = math.acos(min(1., max(-1., dot)))
    if angle < 1e-6:
        q = [(1-fraction)*x + fraction*y for x, y in zip(a, b)]
    else:
        q = [(math.sin((1-fraction)*angle)*x + math.sin(fraction*angle)*y) / math.sin(angle)
             for x, y in zip(a, b)]
    norm = math.sqrt(sum(x*x for x in q))
    position = [(1-fraction)*start[i][3] + fraction*goal[i][3] for i in range(3)]
    pose = SimpleNamespace(position=SimpleNamespace(**dict(zip('xyz', position))),
                           orientation=SimpleNamespace(**dict(zip('xyzw', [x/norm for x in q]))))
    return pose_matrix(pose, 1.)


class RateExceeded(ValueError):
    def __init__(self, arm, point, kind, values, limit, phase='path'):
        limits = list(limit) if isinstance(limit, (tuple, list)) else [limit]*len(values)
        joint = max(range(len(values)), key=lambda i: abs(values[i])/limits[i])
        limit = limits[joint]
        unit = 'deg/s' if kind == 'joint_speed' else 'deg/s^2'
        self.details = {
            'arm': 'left' if arm == 0 else 'right', 'arm_id': arm,
            'joint': joint + 1, 'point_index': point, 'constraint': kind,
            'value': values[joint], 'magnitude': abs(values[joint]),
            'limit': limit, 'unit': unit, 'phase': phase,
        }
        super().__init__(f'arm{arm} joint{joint+1} Cartesian {kind} exceeded at point {point} '
                         f'({phase}): {values[joint]:.6g} {unit}, limit={limit:.6g} {unit}')


def prepare_cartesian(engine, starts, poses, goals):
    c = engine.c
    duration = 1.
    moving = []
    for arm, (start, goal) in enumerate(zip(poses, goals)):
        distance = math.sqrt(sum((start[i][3]-goal[i][3])**2 for i in range(3))) * c.sdk_length_to_m
        dot = abs(sum(x*y for x, y in zip(quaternion(start), quaternion(goal))))
        angle = 2*math.acos(min(1., dot))
        # Hold unchanged arms at the measured joints, without invoking redundant IK.
        if distance <= 1e-5 and angle <= 1e-4:
            continue
        moving.append(arm)
        duration = max(duration, 1.875*distance/c.cartesian_speed,
                       1.875*angle/c.cartesian_angular_speed)
    if not moving:
        return [list(q) for q in starts], {}, {'planned_duration_s': 0., 'retime_count': 0, 'target_hz': FREQUENCY}
    controller_limits = engine.read_cartesian_rate_limits(moving)
    retime_events = []
    for attempt in range(6):
        engine.check_cancel()
        steps = math.ceil(duration*FREQUENCY)
        if steps + 1 > engine.robot.max_point or (steps+1)/FREQUENCY + 1. >= c.timeout:
            raise ValueError('Cartesian trajectory exceeds SDK capacity or motion timeout')
        try:
            targets, plans, peaks = _sample(engine, starts, poses, goals, moving, steps)
            return targets, plans, {'planned_duration_s': steps/FREQUENCY,
                                    'retime_count': attempt, 'target_hz': FREQUENCY,
                                    'retime_events': retime_events, 'peaks': peaks,
                                    'controller_limits': controller_limits}
        except RateExceeded as exc:
            event = {**exc.details, 'attempt': attempt + 1,
                     'duration_s': steps/FREQUENCY, 'will_retry': attempt < 5}
            if attempt < 5:
                event['next_duration_s'] = math.ceil(steps*exc.time_scale)/FREQUENCY
            retime_events.append(event)
            if engine.on_retime is not None:
                engine.on_retime(event)
            if attempt == 5:
                raise
            duration = event['next_duration_s']


def _sample(engine, starts, poses, goals, moving, steps):
    c = engine.c
    peak_rates = {}

    def record(arm, point, kind, values, limit, phase='path'):
        candidate = RateExceeded(arm, point, kind, values, limit, phase)
        ratio = candidate.details['magnitude']/candidate.details['limit']
        if (kind not in peak_rates or
                ratio > peak_rates[kind].details['magnitude']/peak_rates[kind].details['limit']):
            peak_rates[kind] = candidate

    paths = [[list(q)] for q in starts]
    previous = [list(q) for q in starts]
    velocities = [[0.]*7 for _ in starts]
    for index in range(1, steps+1):
        engine.check_cancel()
        t = index/steps
        s = t*t*t*(10 + t*(-15 + 6*t))
        matrices = [interpolate(a, b, s) for a, b in zip(poses, goals)]
        solved = engine.solve(matrices, previous, moving=moving)
        for arm in moving:
            q = solved[arm]
            engine.h.validate_target(starts[arm], q, engine.limits[arm], c.max_delta)
            delta = [x-y for x, y in zip(q, previous[arm])]
            if max(map(abs, delta)) > 1.:
                raise ValueError(f'arm{arm} Cartesian IK jump at point {index}')
            velocity = [x*FREQUENCY for x in delta]
            acceleration = [(x-y)*FREQUENCY for x, y in zip(velocity, velocities[arm])]
            record(arm, index, 'joint_speed', velocity, engine.cartesian_rate_limits[arm]['speed'])
            record(arm, index, 'joint_acceleration', acceleration, engine.cartesian_rate_limits[arm]['acceleration'])
            paths[arm].append(q)
            velocities[arm] = velocity
        previous = solved
    for arm in moving:
        deceleration = [-v*FREQUENCY for v in velocities[arm]]
        record(arm, steps+1, 'joint_acceleration', deceleration,
               engine.cartesian_rate_limits[arm]['acceleration'], phase='terminal_stop')
    peaks = {kind: peak.details.copy() for kind, peak in peak_rates.items()}
    scales = {kind: peak.details['magnitude']/peak.details['limit']
              for kind, peak in peak_rates.items()}
    scales['joint_acceleration'] = math.sqrt(scales['joint_acceleration'])
    limiting = max(scales, key=scales.get)
    if scales[limiting] > 1.:
        failure = peak_rates[limiting]
        # Uniform time scaling reduces speed by k and acceleration by k^2.
        # Re-sample and verify because sequential IK may choose slightly different solutions.
        failure.time_scale = scales[limiting] * 1.02
        failure.details = {**failure.details, 'time_scale': failure.time_scale, 'peaks': peaks}
        raise failure
    plans = {}
    for arm in moving:
        flat = [x for q in paths[arm] for x in q]
        plans[arm] = ((ctypes.c_double * len(flat))(*flat), len(paths[arm]))
    return previous, plans, peaks
