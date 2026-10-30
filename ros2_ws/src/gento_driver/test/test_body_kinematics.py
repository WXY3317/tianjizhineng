"""Offline torso tests; native vendor comparison is optional outside the repository."""
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace

import numpy as np
import pytest

SDK = Path(__file__).resolve().parents[4] / 'tianji-robot-SDK'
from gento_driver.body_kinematics import LunaBodyKinematics, pose_error


# Test data only: J3/J4 signed intervals are NOT defaults for real hardware.
LIMITS = [[-11, 11], [-90, 90], [-60, 140], [-55, 110], [-30, 30], [-170, 170]]


@pytest.fixture
def model():
    return LunaBodyKinematics(LIMITS)


def test_zero_pose_base_and_end_mount(model):
    assert np.array(model.forward([0]*6)) == pytest.approx(
        np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 165+300+300+391.5], [0, 0, 0, 1]]))
    base = np.eye(4)
    base[2, 3] = 100
    assert np.array(model.agv_to_base) == pytest.approx(base)


@pytest.fixture(scope="module")
def native_fk(tmp_path_factory):
    compiler = shutil.which("g++")
    if compiler is None:
        pytest.skip("g++ needed for independent vendor FK comparison")
    if not (SDK / 'C_SDK/Interf/ICBase.cpp').is_file():
        pytest.skip("Vendor sources needed only for independent FK comparison")
    binary = tmp_path_factory.mktemp("luna_native") / "fk_reference"
    source = SDK / "C_SDK"
    includes = [source / "Common", source / "Interf", source / "FXUtility/FXCfg",
                source / "FXUtility/FXMath"]
    subprocess.run([compiler, "-std=c++11", "-DCMPL_LIN", "-w",
                    str(Path(__file__).with_name("luna_body_fk_reference.cpp")),
                    str(source / "Interf/ICBase.cpp"),
                    str(source / "FXUtility/FXCfg/CfgBase.cpp"),
                    str(source / "FXUtility/FXMath/FXMath.cpp"),
                    str(source / "FXUtility/FXMath/FXMatrix.cpp"),
                    *[f"-I{p}" for p in includes], "-o", str(binary)],
                   check=True, capture_output=True, text=True)

    def forward(joints):
        cfg = source / "FXConfig/InterferenceCfg"
        output = subprocess.run([str(binary), str(cfg / "CordDef.Cord"),
                                 str(cfg / "CalLinkDef.Links"), str(cfg / "CalInputDef.Joints"),
                                 *[str(q) for q in joints]],
                                check=True, capture_output=True, text=True)
        return np.fromstring(output.stdout, sep=" ").reshape(4, 4)
    return forward


def test_fk_matches_vendor_coordinate_engine(model, native_fk):
    rng = np.random.default_rng(8921)
    bounds = np.array(model.joint_limits_deg)
    # Single-axis probes catch joint direction/order mistakes; mixed poses
    # catch fixed-transform ordering errors. Native FK uses WORLD=AGV here.
    probes = [np.zeros(6), *list(np.eye(6)*5)]
    probes += list(rng.uniform(bounds[:, 0], bounds[:, 1], size=(16, 6)))
    for q in probes:
        expected = native_fk(q)
        expected[2, 3] -= 100  # WORLD/AGV -> fixed torso base at J1.
        assert np.array(model.forward(q)) == pytest.approx(expected, abs=2e-5)


def test_limits_intersect_controller_and_user_caps():
    limits = np.array(LIMITS, dtype=float)
    limits[0] = [-8, 9]
    limits[1] = [-40, 90]
    limits[2] = [-140, 60]  # Preserve the controller's sign choice.
    limits[3] = [-110, 55]
    limits[4] = [-40, 40]
    limits[5] = [-180, 180]
    model = LunaBodyKinematics(limits)
    assert np.array(model.joint_limits_deg) == pytest.approx(
        np.array([[-8, 9], [-40, 50], [-140, 60], [-110, 55], [-30, 30], [-170, 170]]))


def test_controller_limits_are_read_without_commands():
    calls = []
    def read(name):
        calls.append(name)
        joint = int(name.split(".")[2][1:])
        return 0, LIMITS[joint][name.endswith("LimitPos")]
    robot = SimpleNamespace(_connected=True, get_ctrl_obj_dof=lambda obj: 6, param_get_float=read)
    model = LunaBodyKinematics.from_robot(robot)
    assert model.joint_limits_deg[1] == [-50, 50]
    assert len(calls) == 12 and calls[0] == "R.B.L0.BASIC.LimitNeg"
    robot.param_get_float = lambda name: (-1, 0)
    with pytest.raises(RuntimeError, match="Cannot read"):
        LunaBodyKinematics.from_robot(robot)
    robot._connected = False
    with pytest.raises(RuntimeError, match="Connect"):
        LunaBodyKinematics.from_robot(robot)
    robot._connected = True
    robot.get_ctrl_obj_dof = lambda obj: 3
    with pytest.raises(ValueError, match="six"):
        LunaBodyKinematics.from_robot(robot)


def test_reference_pose_returns_identical_joints(model):
    q = [2, 15, -25, 10, 3, 20]
    result = model.inverse(model.forward(q), q)
    assert result.success and result.attempts == 0
    assert result.joints_deg == pytest.approx(q)


@pytest.mark.parametrize("target_q, reference", [
    ([2, 15, -25, 10, 3, 20], [0, 10, -20, 10, 0, 0]),
    ([0, 20, -40, 20, 0, 0], [0]*6),  # leave straight-pose singularity
    ([11, 50, -60, 10, 30, 170], [10, 45, -55, 5, 25, 165]),
    ([0, 10, -20, 10, 0, 165], [0, 10, -20, 10, 0, -165]),
])
def test_full_pose_roundtrip_and_joint_bounds(model, target_q, reference):
    target = model.forward(target_q)
    result = model.inverse(target, reference)
    assert result.success, result
    assert result.position_error_mm <= 0.1
    assert result.orientation_error_deg <= 0.01
    bounds = np.array(model.joint_limits_deg)
    assert np.all(np.array(result.joints_deg) >= bounds[:, 0])
    assert np.all(np.array(result.joints_deg) <= bounds[:, 1])
    assert np.array(model.forward(result.joints_deg)) == pytest.approx(np.array(target), abs=0.1)


def test_two_bend_branches_select_near_reference(model):
    target = model.forward([0, 20, -40, 20, 0, 0])
    # Both [+20,-40,+20] and [-20,+40,-20] have the same end pose.
    ref = [0, -19, 38, -19, 0, 0]
    result = model.inverse(target, ref)
    assert result.success
    assert result.joints_deg == pytest.approx([0, -20, 40, -20, 0, 0], abs=1e-3)


def test_position_only_keeps_free_yaw_and_reduces_joint_change(model):
    reference = [1, 10, -20, 10, 2, 15]
    known = [3, 12, -18, 15, 4, 120]
    target = np.array(model.forward(known))[:3, 3]
    result = model.inverse(target, reference, position_only=True)
    assert result.success and result.orientation_error_deg is None
    assert result.joints_deg[5] == pytest.approx(reference[5], abs=1e-5)
    assert result.reference_distance_deg < np.linalg.norm(np.array(known)-reference)
    assert np.array(model.forward(result.joints_deg))[:3, 3] == pytest.approx(target, abs=0.1)


def test_unreachable_target_never_returns_joint_command(model):
    result = model.inverse([5000, 0, 5000], [0]*6, position_only=True, max_nfev=60)
    assert not result.success and result.joints_deg is None
    assert result.position_error_mm > 1000
    assert result.attempts <= 10


def test_joint_two_cap_and_optional_change_bound_are_hard(model):
    target_q = [0, 60, -100, 40, 0, 0]
    target = model.forward(target_q)
    result = model.inverse(target, target_q, max_nfev=100)
    assert not result.success and result.joints_deg is None
    near = [0, 15, -25, 10, 0, 0]
    result = model.inverse(model.forward(near), [0, 10, -20, 10, 0, 0], max_joint_delta_deg=1)
    assert not result.success and result.joints_deg is None


@pytest.mark.parametrize('delta', [1, 10, 60])
def test_change_bound_touching_limit_is_a_legal_fixed_angle(model, delta):
    reference = [0, 50 + delta, -40, 0, 0, 0]
    target_q = [0, 50, -40, 0, 0, 0]
    result = model.inverse(model.forward(target_q), reference, max_joint_delta_deg=delta)
    assert result.success
    assert result.joints_deg == pytest.approx(target_q, abs=1e-8)


def test_fixed_joint_is_removed_from_optimization(model):
    reference = [0, 51, -40, 0, 0, 0]
    target_q = [0.2, 50, -39.5, -0.5, 0.2, 0.5]
    result = model.inverse(model.forward(target_q), reference, max_joint_delta_deg=1)
    assert result.success, result
    assert result.joints_deg[1] == pytest.approx(50, abs=1e-10)
    assert np.max(np.abs(np.array(result.joints_deg) - reference)) <= 1


def test_disjoint_change_bound_still_rejected(model):
    with pytest.raises(ValueError, match='no intersection'):
        model.inverse(model.forward([0]*6), [0, 60, 0, 0, 0, 0], max_joint_delta_deg=1)


def test_all_joints_fixed_reports_unreachable_pose_without_optimizer(model):
    upper = np.array(model.joint_limits_deg)[:, 1]
    result = model.inverse(model.forward([0]*6), upper+1, max_joint_delta_deg=1)
    assert not result.success and result.joints_deg is None
    assert result.attempts == 1


@pytest.mark.parametrize("bad", [np.zeros((5, 2)), [[0, 0]]*6,
                                   [[float("nan"), 10]]*6, [[60, 70]]*6])
def test_invalid_limits_rejected(bad):
    with pytest.raises(ValueError):
        LunaBodyKinematics(bad)


def test_malformed_pose_and_reference_rejected(model):
    for bad in (np.zeros((4, 4)), np.diag([-1, 1, 1, 1]), np.eye(3), [float("nan")]*3):
        with pytest.raises(ValueError):
            model.inverse(bad, [0]*6)
    with pytest.raises(ValueError):
        model.inverse(np.eye(4), [float("inf")]*6)


def test_model_uses_packaged_geometry_without_sdk_import():
    from gento_driver.body_kinematics import _DEFAULT_CORD
    assert _DEFAULT_CORD.parent.name == 'data'
    assert _DEFAULT_CORD.is_file()
    model = LunaBodyKinematics(LIMITS)
    result = model.inverse(model.forward([0]*6), [0]*6)
    assert result.success and result.joints_deg == pytest.approx([0]*6)


def test_geometry_only_model_supports_feedback_but_rejects_ik():
    model = LunaBodyKinematics()
    assert model.joint_limits_deg is None
    assert model.forward([0]*6)[2][3] == 1156.5
    with pytest.raises(ValueError, match='limits are required'):
        model.inverse(model.forward([0]*6), [0]*6)


def test_checkpoint_interrupts_numerical_search(model):
    calls=[]
    def checkpoint():
        calls.append(True)
        if len(calls)>=4:
            raise ValueError('Planning cancelled')
    with pytest.raises(ValueError, match='cancelled'):
        model.inverse(model.forward([0,20,-40,20,0,0]), [0]*6, checkpoint=checkpoint)
    assert len(calls)==4


def test_joint_tolerance_does_not_imply_tip_position_tolerance(model):
    goal=[0.,20.,-40.,20.,0.,0.]
    measured=list(goal);measured[1]+=.19
    position,orientation=pose_error(model.forward(measured),model.forward(goal))
    assert position>3.0
    assert orientation==pytest.approx(.19)


def test_returned_degree_vector_stays_inside_noninteger_controller_bound():
    upper=50.222154220533334
    limits=np.array(LIMITS,dtype=float);limits[2,1]=upper
    model=LunaBodyKinematics(limits)
    q=[0.,0.,upper,0.,0.,0.]
    result=model.inverse(model.forward(q),q)
    assert result.success
    assert result.joints_deg[2]<=upper
    assert result.joints_deg[2]==upper
