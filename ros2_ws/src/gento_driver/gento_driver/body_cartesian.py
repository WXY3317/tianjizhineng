"""Offline 50 Hz Cartesian torso path; every sample is verified before motion."""
import math
from types import SimpleNamespace

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation, Slerp

from .body_kinematics import LunaBodyKinematics, pose_error

FREQUENCY = 50
ANCHORS = 80


def interpolate(start, goal, fraction):
    rotations = Rotation.from_matrix([[row[:3] for row in start[:3]],
                                      [row[:3] for row in goal[:3]]])
    rotation = Slerp([0., 1.], rotations)([fraction]).as_matrix()[0]
    return [[*rotation[i], (1-fraction)*start[i][3]+fraction*goal[i][3]]
            for i in range(3)]+[[0.,0.,0.,1.]]


def blend(t):
    return t*t*t*(10+t*(-15+6*t))


def plan_body_line(snapshot, goal, rates, checkpoint):
    c = SimpleNamespace(**snapshot['config'])
    start, origin = snapshot['body_matrix'], list(snapshot['body'])
    limits = snapshot['body_limits']
    model = LunaBodyKinematics(limits)
    z_min, z_max = c.body_z_min_m*1000, c.body_z_max_m*1000
    if not z_min <= goal[2][3] <= z_max:
        raise ValueError(f'Body IK target z must be within [{c.body_z_min_m}, {c.body_z_max_m}] m in {c.body_frame}')
    distance_mm, angle_deg = pose_error(start, goal)
    if distance_mm <= .01 and angle_deg <= .001:
        return {}, {'planned_duration_s': 0., 'body_start_matrix': start,
                    'body_target_matrix': goal}
    checkpoint()
    solved_goal = model.inverse(goal, origin, max_joint_delta_deg=c.max_delta,
                                checkpoint=checkpoint)
    if not solved_goal.success:
        raise ValueError(f'Body Cartesian goal IK failed: {solved_goal.message}')
    # Start at the nonsingular goal branch and trace it back to the measured
    # start. This avoids arbitrary elbow flips near the all-zero singularity.
    times, joints = [1.], [solved_goal.joints_deg]
    previous = solved_goal.joints_deg
    for index in range(ANCHORS-1,0,-1):
        checkpoint()
        t = index/ANCHORS
        desired = interpolate(start, goal, blend(t))
        hint = [a+blend(t)*(b-a) for a,b in zip(origin,solved_goal.joints_deg)]
        solved = model.inverse(desired, previous, max_joint_delta_deg=min(c.max_delta,5.),
                               checkpoint=checkpoint, local_only=True, max_nfev=80,
                               seed_joints_deg=hint, position_tolerance_mm=.05,
                               orientation_tolerance_deg=.005)
        if not solved.success:
            raise ValueError(f'Body Cartesian IK failed at anchor {index}/{ANCHORS}')
        times.append(t)
        joints.append(solved.joints_deg)
        previous=solved.joints_deg
    times=np.array([0.]+list(reversed(times)))
    joints=np.array([origin]+list(reversed(joints)))
    spline=CubicSpline(times,joints,axis=0,
                       bc_type=((1,np.zeros(6)),(1,np.zeros(6))))
    joint_distance=np.abs(joints[-1]-joints[0])
    duration=max(1.,1.875*distance_mm/1000/c.cartesian_speed,
                 1.875*math.radians(angle_deg)/c.cartesian_angular_speed,
                 *(1.875*d/v for d,v in zip(joint_distance,rates['speed'])),
                 *(math.sqrt(5.774*d/a) for d,a in zip(joint_distance,rates['acceleration'])))
    path_min,path_max=min(z_min,start[2][3]),max(z_max,start[2][3])
    for attempt in range(8):
        checkpoint()
        steps=math.ceil(duration*FREQUENCY)
        if steps+1>5000 or (steps+1)/FREQUENCY+1>=c.timeout:
            raise ValueError(f'Body Cartesian trajectory exceeds capacity or timeout: duration={duration:.2f}s')
        fractions=np.linspace(0.,1.,steps+1)
        samples=spline(fractions)
        samples[0]=origin
        samples[-1]=joints[-1]
        velocities=np.diff(samples,axis=0)*FREQUENCY
        accelerations=np.diff(np.vstack((np.zeros((1,6)),velocities,np.zeros((1,6)))),axis=0)*FREQUENCY
        speed_ratio=float(np.max(np.abs(velocities)/np.asarray(rates['speed'])))
        acceleration_ratio=float(np.max(np.abs(accelerations)/np.asarray(rates['acceleration'])))
        if speed_ratio>1. or acceleration_ratio>1.:
            duration=math.ceil(steps*max(speed_ratio,math.sqrt(acceleration_ratio))*1.02)/FREQUENCY
            continue
        path=samples.tolist()
        for index,(t,q) in enumerate(zip(fractions,path)):
            checkpoint()
            for joint,(value,initial,(lo,hi)) in enumerate(zip(q,origin,limits)):
                if not lo-1e-10<=value<=hi+1e-10 or abs(value-initial)>c.max_delta+1e-10:
                    raise ValueError(f'Body Cartesian joint{joint+1} exceeds limits/max_delta at point {index}')
            actual=model.forward(q)
            if not path_min-.1<=actual[2][3]<=path_max+.1:
                raise ValueError(f'Body Cartesian height envelope exceeded at point {index}')
            desired=interpolate(start,goal,blend(t))
            position_mm,orientation_deg=pose_error(actual,desired)
            if position_mm>.1 or orientation_deg>.01:
                raise ValueError(f'Body Cartesian FK departs interpolated pose at point {index}: '
                                 f'{position_mm:.4f} mm / {orientation_deg:.4f} deg')
        return {3:path},{'planned_duration_s':steps/FREQUENCY,
                         'retime_count':attempt,'target_hz':FREQUENCY,
                         'body_start_matrix':start,'body_target_matrix':goal,
                         'peak_speed_ratio':speed_ratio,
                         'peak_acceleration_ratio':acceleration_ratio}
    raise ValueError('Body Cartesian speed/acceleration limits could not be met')
