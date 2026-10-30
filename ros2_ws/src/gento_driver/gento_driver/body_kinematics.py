"""Offline six-axis Luna torso FK/IK. Lengths: mm; public joint angles: degrees.

B is fixed at the BD_J1 pivot, with axes parallel to the parent AGV frame.
E is BD_FL, including its fixed mounting transform after BD_J6. Geometry and
positive joint directions follow CordDef.Cord and ICBase.cpp, not SkyeBody.
This module computes joint targets only; it never sends motion commands.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional
import re

import numpy as np
from scipy.optimize import least_squares, minimize
from scipy.spatial.transform import Rotation


# J3/J4 signed bounds must come from the controller (the drawing is unsigned).
_CAPS_DEG = (11.0, 50.0, None, None, 30.0, 170.0)
_DEFAULT_CORD = Path(__file__).resolve().parent / "data" / "luna_body.Cord"


def _array(value, shape, name):
    value = np.asarray(value, dtype=float)
    if value.shape != shape or not np.all(np.isfinite(value)):
        raise ValueError(f"{name} must be finite and have shape {shape}")
    return value.copy()


def _pose(value):
    value = _array(value, (4, 4), "target_pose")
    r = value[:3, :3]
    if (not np.allclose(value[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-7, rtol=0)
            or not np.isclose(np.linalg.det(r), 1, atol=1e-7, rtol=0)):
        raise ValueError("target_pose must be a rigid 4x4 transform with a proper rotation")
    return value


def pose_error(actual, target):
    """Return (position error in mm, orientation error in degrees)."""
    actual, target = _pose(actual), _pose(target)
    position = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
    rotation = Rotation.from_matrix(target[:3, :3].T @ actual[:3, :3])
    return position, float(np.rad2deg(np.linalg.norm(rotation.as_rotvec())))


def _load_chain(path):
    """Read x-axis, z-axis, translation triples exactly as ICBase::FX_OnLoadCord."""
    pattern = re.compile(
        r"<S,([^>]+)>:father is\s*<S,([^>]+)>\s*Set as\s*"
        r"<m3,([^>]+)>\s*<I,([^>]+)>")
    nodes = {}
    for name, parent, data, kind in pattern.findall(Path(path).read_text()):
        if name in nodes:
            raise ValueError(f"Duplicate coordinate: {name}")
        data = _array([float(x.strip()) for x in data.split(",")], (9,), name)
        x, z, p = data.reshape(3, 3)
        if np.linalg.norm(z) < 1e-10 or np.linalg.norm(x) < 1e-10:
            raise ValueError(f"Invalid coordinate axes: {name}")
        x, z = x / np.linalg.norm(x), z / np.linalg.norm(z)
        y = np.cross(z, x)
        if np.linalg.norm(y) < 1e-10:
            raise ValueError(f"Parallel coordinate axes: {name}")
        y /= np.linalg.norm(y)
        x = np.cross(y, z)
        transform = np.eye(4)
        transform[:3, :3] = np.column_stack((x, y, z))
        transform[:3, 3] = p
        nodes[name] = (parent, transform, int(kind))

    names = [f"BD_J{i}" for i in range(1, 7)] + ["BD_FL"]
    parent = "AGV"
    chain = []
    for name in names:
        if name not in nodes:
            raise ValueError(f"Missing coordinate: {name}")
        actual_parent, transform, kind = nodes[name]
        if actual_parent != parent or kind != 0:
            raise ValueError(f"Expected revolute {name} under {parent}")
        chain.append(transform.copy())
        parent = name

    # AGV -> B is translation only: B axes stay parallel to AGV. Remove the
    # fixed AGV -> J1 translation but retain J1's installed rotation axes.
    agv_to_base = np.eye(4)
    agv_to_base[:3, 3] = chain[0][:3, 3]
    chain[0][:3, 3] = 0
    return tuple(chain), agv_to_base


@dataclass(frozen=True)
class BodyIKResult:
    success: bool
    joints_deg: Optional[list]
    position_error_mm: float
    orientation_error_deg: Optional[float]
    reference_distance_deg: Optional[float]
    attempts: int
    message: str


class LunaBodyKinematics:
    """Six-axis torso model with explicit signed controller limits.

    joint_limits_deg: six [LimitNeg, LimitPos] pairs in SDK joint order, or None
    for feedback-only FK (inverse is then disabled). Supplied limit pairs
    are intersected with J1 +/-11, J2 +/-50, J5 +/-30, J6 +/-170 degrees.
    J3/J4 keep the controller's signed limits; no sign is guessed from a drawing.
    Current angles may lie outside the stricter user bounds; solved angles never
    do. Such a reference is only clipped to obtain a legal numerical seed.
    """

    def __init__(self, joint_limits_deg=None, cord_path=None):
        # Geometry-only construction supports read-only FK without inventing
        # hardware limits. IK always requires explicit signed controller limits.
        self._bounds_deg = None
        self._chain, self._agv_to_base = _load_chain(
            _DEFAULT_CORD if cord_path is None else cord_path)
        if joint_limits_deg is None:
            return
        bounds = _array(joint_limits_deg, (6, 2), "joint_limits_deg")
        if np.any(bounds[:, 0] >= bounds[:, 1]):
            raise ValueError("Every joint lower limit must be less than its upper limit")
        for i, cap in enumerate(_CAPS_DEG):
            if cap is not None:
                bounds[i, 0] = max(bounds[i, 0], -cap)
                bounds[i, 1] = min(bounds[i, 1], cap)
        if np.any(bounds[:, 0] >= bounds[:, 1]):
            raise ValueError("Controller limits and user caps have no usable intersection")
        self._bounds_deg = bounds

    @property
    def joint_limits_deg(self):
        return None if self._bounds_deg is None else self._bounds_deg.tolist()

    @property
    def agv_to_base(self):
        """Fixed T_AGV_B (mm). T_B_E = inverse(T_AGV_B) @ T_AGV_E."""
        return self._agv_to_base.tolist()

    @classmethod
    def from_robot(cls, robot, cord_path=None):
        """Read limits only. Does not change controller limits, state or position."""
        if not robot._connected:
            raise RuntimeError("Connect the robot before reading torso limits")
        if robot.get_ctrl_obj_dof(3) != 6:
            raise ValueError("Luna torso kinematics requires exactly six body joints")
        limits = []
        for i in range(6):
            pair = []
            for suffix in ("LimitNeg", "LimitPos"):
                name = f"R.B.L{i}.BASIC.{suffix}"
                ret, value = robot.param_get_float(name)
                if ret != 0:
                    raise RuntimeError(f"Cannot read {name}: SDK return code {ret}")
                pair.append(value)
            limits.append(pair)
        return cls(limits, cord_path)

    def _fk(self, radians):
        transform = np.eye(4)
        for fixed, angle in zip(self._chain[:6], radians):
            transform = transform @ fixed
            c, s = np.cos(angle), np.sin(angle)
            rotation = np.array([[c, -s, 0, 0], [s, c, 0, 0],
                                 [0, 0, 1, 0], [0, 0, 0, 1]])
            transform = transform @ rotation
        return transform @ self._chain[6]

    def forward(self, joints_deg):
        """Return T_B_BD_FL, a nested-list 4x4 matrix, in mm.

        FK accepts measured angles outside the user caps as well, so the actual
        pose can still be inspected before solving for a target inside the caps.
        """
        q = _array(joints_deg, (6,), "joints_deg")
        return self._fk(np.deg2rad(q)).tolist()

    def inverse(self, target_pose, reference_joints_deg, *, position_only=False,
                position_tolerance_mm=0.1, orientation_tolerance_deg=0.01,
                max_joint_delta_deg=None, max_nfev=300, checkpoint=None):
        """Solve in B, selecting the nearest feasible candidate to the reference.

        Full-pose mode accepts a rigid 4x4 T_B_BD_FL. Position-only mode accepts
        XYZ (mm) or a 4x4 matrix whose orientation is ignored. In position-only
        mode orientation is free, allowing smaller joint changes.

        A bounded multi-start search finds feasible solutions, then constrained
        local minimization reduces sum((q - reference)**2). This is a local
        numerical method, not a proof of globally nearest solution or reachability.
        max_joint_delta_deg optionally bounds each joint change (scalar or six
        values). On failure joints_deg is None; residuals report the best attempt.
        checkpoint optionally checks cancellation during residual evaluations.
        """
        if self._bounds_deg is None:
            raise ValueError("Controller joint limits are required for torso IK")
        if checkpoint:
            checkpoint()
        ref_deg = _array(reference_joints_deg, (6,), "reference_joints_deg")
        if (not np.isfinite(position_tolerance_mm) or position_tolerance_mm <= 0
                or not np.isfinite(orientation_tolerance_deg) or orientation_tolerance_deg <= 0):
            raise ValueError("Pose tolerances must be positive and finite")
        if isinstance(max_nfev, bool) or not isinstance(max_nfev, int) or max_nfev < 1:
            raise ValueError("max_nfev must be a positive integer")
        target = np.asarray(target_pose, dtype=float)
        if position_only and target.shape == (3,):
            position = _array(target, (3,), "target_position")
            target_rotation = None
        else:
            target = _pose(target)
            position = target[:3, 3]
            target_rotation = None if position_only else target[:3, :3]

        ref = np.deg2rad(ref_deg)
        # Intersect in public degree units before conversion. Subtracting two
        # separately converted radians can turn a single legal angle into a
        # negative or near-zero interval through floating-point cancellation.
        bounds_deg = self._bounds_deg.copy()
        if max_joint_delta_deg is not None:
            delta = np.asarray(max_joint_delta_deg, dtype=float)
            if delta.shape == ():
                delta = np.full(6, float(delta))
            delta = _array(delta, (6,), "max_joint_delta_deg")
            if np.any(delta <= 0):
                raise ValueError("max_joint_delta_deg must be positive")
            bounds_deg[:, 0] = np.maximum(bounds_deg[:, 0], ref_deg - delta)
            bounds_deg[:, 1] = np.minimum(bounds_deg[:, 1], ref_deg + delta)
            if np.any(bounds_deg[:, 0] > bounds_deg[:, 1]):
                raise ValueError("Joint change bounds and joint limits have no intersection")
        bounds = np.deg2rad(bounds_deg)
        lo, hi = bounds.T

        def residual(q):
            if checkpoint:
                checkpoint()
            actual = self._fk(q)
            error = (actual[:3, 3] - position) / 1000.0
            if target_rotation is not None:
                rotation = Rotation.from_matrix(target_rotation.T @ actual[:3, :3]).as_rotvec()
                error = np.concatenate((error, rotation))
            return error

        def errors(q):
            r = residual(q)
            return (float(np.linalg.norm(r[:3]) * 1000),
                    None if target_rotation is None else float(np.rad2deg(np.linalg.norm(r[3:]))))

        def feasible(q):
            if not np.all(np.isfinite(q)) or np.any(q < lo) or np.any(q > hi):
                return False
            p, r = errors(q)
            return p <= position_tolerance_mm and (r is None or r <= orientation_tolerance_deg)

        def distance(q):
            return float(np.sum((q - ref) ** 2))

        def result(q, attempts, success):
            # Conversion back to degrees can overshoot an exact controller
            # bound by one ULP. Return canonical degree values and verify the
            # actual returned vector, not just the optimizer's radian vector.
            degrees = np.clip(np.rad2deg(q), bounds_deg[:, 0], bounds_deg[:, 1])
            if success:
                q = np.deg2rad(degrees)
                success = feasible(q)
            p, r = errors(q)
            return BodyIKResult(
                success, degrees.tolist() if success else None, p, r,
                float(np.rad2deg(np.sqrt(distance(q)))) if success else None, attempts,
                "Converged within pose tolerances and joint limits" if success else
                "No feasible solution found within the bounded search; check target, limits and geometry")

        if feasible(ref):
            return result(ref, 0, True)
        seed = np.clip(ref, lo, hi)
        if feasible(seed):
            # The box projection is already nearest to ref over all legal joints.
            return result(seed, 0, True)
        free = np.flatnonzero(hi > lo)
        if not len(free):
            return result(seed, 1, False)

        def expand(active):
            q = lo.copy()
            q[free] = active
            return q

        def active_residual(active):
            return residual(expand(active))

        seeds = [seed]
        # Opposite bend seeds help leave the zero-pose singularity and find
        # both bend branches. All seeds and optimizer iterates obey the bounds.
        for sign in (-1, 1):
            seeds.append(np.clip(seed + np.deg2rad([0, 20*sign, -40*sign, 20*sign, 0, 0]), lo, hi))
        for joint in (1, 2, 3):
            for sign in (-1, 1):
                offset = np.zeros(6)
                offset[joint] = np.deg2rad(30 * sign)
                seeds.append(np.clip(seed + offset, lo, hi))
        seeds.append((lo + hi) / 2)
        candidates = []
        best, best_error = seed, float(np.linalg.norm(residual(seed)))
        tried = set()
        for seed in seeds:
            key = tuple(np.round(seed, 12))
            if key in tried:
                continue
            tried.add(key)
            # Equal lower/upper bounds are fixed joints, not optimization
            # variables: least_squares requires strictly unequal bounds.
            solved = least_squares(active_residual, seed[free], bounds=(lo[free], hi[free]),
                                   max_nfev=max_nfev, ftol=1e-12, xtol=1e-12, gtol=1e-12)
            q = expand(solved.x)
            error = float(np.linalg.norm(residual(q)))
            if error < best_error:
                best, best_error = q, error
            if not feasible(q):
                continue
            candidates.append(q.copy())
            if len(residual(q)) > len(free):
                # SLSQP cannot handle more equality constraints than variables.
                # Retain the bounded feasible solution found by least squares.
                continue
            # Pose is a constraint, never traded for proximity via a weighted
            # sum. Keep the feasible seed if rank deficiency defeats refinement.
            refined = minimize(lambda v: distance(expand(v)), q[free],
                               jac=lambda v: 2*(v-ref[free]), method="SLSQP",
                               bounds=list(zip(lo[free], hi[free])),
                               constraints={"type": "eq", "fun": active_residual},
                               options={"maxiter": max_nfev, "ftol": 1e-12})
            q_refined = expand(np.clip(refined.x, lo[free], hi[free]))
            if feasible(q_refined):
                candidates.append(q_refined.copy())
        if candidates:
            return result(min(candidates, key=distance), len(tried), True)
        return result(best, len(tried), False)
