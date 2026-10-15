"""Real vendor FK/IK with SI boundary units and model limit checks.

The vendor library has process-global arm slots 0 and 1. Use one instance per
process and serialize access (the ROS node uses a single-threaded executor).
"""
import importlib
import math
from pathlib import Path
import sys

import numpy as np
import yaml


JOINT_NAMES = [f'Joint{i}_{side}' for side in ('L', 'R') for i in range(1, 8)]
INITIAL_DEG = [[10., -20., 30., -40., 20., 15., -10.],
               [-10., -20., -30., -40., -20., 15., 10.]]


def vendor_modules(sdk_root):
    root = Path(sdk_root).expanduser().resolve()
    if not (root / 'SDK_PYTHON/fx_kine.py').is_file():
        raise ValueError(f'Invalid tianji_armSDK directory: {root}')
    # A single SDK root per process avoids loading inconsistent native globals.
    sys.path.insert(0, str(root))
    kine = importlib.import_module('SDK_PYTHON.fx_kine')
    robot = importlib.import_module('SDK_PYTHON.fx_robot')
    if Path(kine.__file__).resolve().parents[1] != root:
        raise ValueError('A different SDK root is already imported in this process')
    return kine, robot


def validate_transform(matrix):
    value = np.asarray(matrix, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError('Pose must be a finite 4x4 matrix')
    r = value[:3, :3]
    if (not np.allclose(value[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-6)
            or not math.isclose(np.linalg.det(r), 1., abs_tol=1e-6)):
        raise ValueError('Pose rotation must be a proper orthonormal matrix')
    return value


def pose_error(actual, target):
    position = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
    cosine = (np.trace(actual[:3, :3].T @ target[:3, :3]) - 1.) / 2.
    return position, math.acos(float(np.clip(cosine, -1., 1.)))


def quaternion_matrix(values):
    q = np.asarray(values, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError('Quaternion must be finite and nonzero (x y z w)')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def matrix_quaternion(rotation):
    # Branch on the largest component; stable at 180-degree rotations.
    r = np.asarray(rotation)
    candidates = np.array([1+r[0,0]-r[1,1]-r[2,2],
                           1-r[0,0]+r[1,1]-r[2,2],
                           1-r[0,0]-r[1,1]+r[2,2], 1+np.trace(r)])
    i = int(np.argmax(candidates))
    v = math.sqrt(max(0., candidates[i])) / 2
    d = 4*v
    if i == 0:
        q = [v, (r[0,1]+r[1,0])/d, (r[0,2]+r[2,0])/d, (r[2,1]-r[1,2])/d]
    elif i == 1:
        q = [(r[0,1]+r[1,0])/d, v, (r[1,2]+r[2,1])/d, (r[0,2]-r[2,0])/d]
    elif i == 2:
        q = [(r[0,2]+r[2,0])/d, (r[1,2]+r[2,1])/d, v, (r[1,0]-r[0,1])/d]
    else:
        q = [(r[2,1]-r[1,2])/d, (r[0,2]-r[2,0])/d, (r[1,0]-r[0,1])/d, v]
    return (np.array(q) / np.linalg.norm(q)).tolist()


class Kinematics:
    def __init__(self, sdk_root, description_root, config_path=None):
        self.module, self.robot_module = vendor_modules(sdk_root)
        self.arms = []
        self.lower, self.upper, self.velocity, self.acceleration = [], [], [], []
        limits = yaml.safe_load((Path(description_root) / 'config/joint_limits.yaml').read_text())
        config = Path(config_path) if config_path else Path(sdk_root) / 'CommonConfig/ccs_m6_40.MvKDCfg'
        for index in range(2):
            arm = self.module.Marvin_Kine()
            arm.log_switch(0)
            data = arm.load_config(index, str(config))
            if not data or data['TYPE'][index] != 1017:
                raise RuntimeError('Expected CCS M6 type 1017 configuration')
            if not arm.initial_kine(data['TYPE'][index], data['DH'][index],
                                    data['PNVA'][index], data['BD'][index]):
                raise RuntimeError(f'SDK initialization failed for arm {index}')
            arm.remove_tool_kine()
            self.arms.append(arm)
            for j, sdk_limit in enumerate(data['PNVA'][index]):
                model = limits[f'joint{j+1}']['limit']
                # LOADMvCfg returns positive bound, negative bound (PNVA).
                self.lower.append(max(model['lower'], math.radians(sdk_limit[1])))
                self.upper.append(min(model['upper'], math.radians(sdk_limit[0])))
                self.velocity.append(min(model['velocity'], math.radians(sdk_limit[2])))
                self.acceleration.append(math.radians(sdk_limit[3]))
        self.lower, self.upper = np.array(self.lower), np.array(self.upper)
        self.velocity, self.acceleration = np.array(self.velocity), np.array(self.acceleration)

    def validate_joints(self, arm, joints):
        if arm not in (0, 1):
            raise ValueError('Arm must be 0 (left/A) or 1 (right/B)')
        q = np.asarray(joints, dtype=float)
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError('Expected seven finite joint angles in radians')
        s = slice(7*arm, 7*arm+7)
        if np.any(q < self.lower[s]-1e-8) or np.any(q > self.upper[s]+1e-8):
            raise ValueError('Joint target exceeds SDK/model limits')
        return q

    def fk(self, arm, joints):
        q = self.validate_joints(arm, joints)
        result = self.arms[arm].fk(np.degrees(q).tolist())
        if result is False:
            raise ValueError('SDK FK failed')
        result = validate_transform(result).copy()
        result[:3, 3] *= .001
        return result

    def ik(self, arm, target, reference):
        target = validate_transform(target)
        reference = self.validate_joints(arm, reference)
        request = self.module.FX_InvKineSolvePara()
        native = target.copy()
        native[:3, 3] *= 1000.
        request.set_input_ik_target_tcp(native.ravel().tolist())
        request.set_input_ik_ref_joint(np.degrees(reference).tolist())
        request.set_input_ik_zsp_type(0)
        result = self.arms[arm].ik(request)
        if result is False or result.m_Output_IsOutRange or result.m_Output_IsJntExd:
            raise ValueError('SDK IK failed: unreachable, singular reference or joint limits')
        if any(result.m_Output_IsDeg):
            raise ValueError('SDK IK reports a singular configuration')
        q = self.validate_joints(arm, np.radians(result.m_Output_RetJoint.to_list()))
        dp, dr = pose_error(self.fk(arm, q), target)
        if dp > 1e-5 or dr > 1e-4:
            raise ValueError(f'SDK IK FK-check failed: {dp:g} m, {dr:g} rad')
        return q
