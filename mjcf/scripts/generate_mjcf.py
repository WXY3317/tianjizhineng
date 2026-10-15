"""Generate a self-contained MJCF package from the supplied Luna URDF.

This script is intentionally kept in the output package so the conversion is
repeatable without changing the source URDF or source mesh files.
"""

from __future__ import annotations

import json
import math
import re
import shutil
import struct
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SOURCE_ROOT = PROJECT_ROOT / "Gento Luna_URDF"
SOURCE_URDF = SOURCE_ROOT / "urdf" / "Gento Luna_URDF.urdf"
OUTPUT_ROOT = PROJECT_ROOT / "mjcf"
MESH_ROOT = OUTPUT_ROOT / "assets" / "meshes"
MESH_BOUNDS: dict[str, tuple[list[float], list[float]]] = {}

# Per-joint gains. Two effects set them:
#   * the arm/wrist gains are scaled to the compiled zero-pose effective
#     inertia -- one global kp/kv pair caused either slow motion or overshoot;
#   * the leg chain carries the whole robot under real gravity (gravcomp=0), so
#     its gains also have to hold a static load. A P-only servo has a
#     steady-state error of load_torque / kp, and the hip joints see the largest
#     torque through the longest lever: measured hip droop with target 0.5 rad
#     was 0.022 rad at kp=8000, i.e. ~176 N*m of gravity torque. (When the model
#     was weightless there was no load and the old, lower leg gains looked fine.)
# kv is scaled with sqrt(kp) so the damping ratio of each joint stays put.
CONTROL_PARAMS: dict[str, tuple[float, float]] = {
    "Gento Luna_Leg_J1_Joint": (16000.0, 1500.0),
    "Gento Luna_Leg_J2_Joint": (30000.0, 1750.0),
    "Gento Luna_Leg_J3_Joint": (16000.0, 1000.0),
    "Gento Luna_Leg_J4_Joint": (12000.0, 600.0),
    "Gento Luna_Leg_J5_Joint": (12000.0, 775.0),
    "Gento Luna_Leg_J6_Joint": (8000.0, 360.0),
    "Arm_R1_Joint": (1000.0, 60.0),
    "Arm_R2_Joint": (1500.0, 100.0),
    "Arm_R3_Joint": (1000.0, 60.0),
    "Arm_R4_Joint": (1200.0, 80.0),
    "Arm_R5_Joint": (800.0, 50.0),
    "Arm_R6_Joint": (800.0, 50.0),
    "Arm_R7_Joint": (800.0, 50.0),
    "Arm_L1_Joint": (1000.0, 60.0),
    "Arm_L2_Joint": (1500.0, 100.0),
    "Arm_L3_Joint": (1000.0, 60.0),
    "Arm_L4_Joint": (1200.0, 80.0),
    "Arm_L5_Joint": (800.0, 50.0),
    "Arm_L6_Joint": (800.0, 50.0),
    "Arm_L7_Joint": (800.0, 50.0),
    "Head_J1_joint": (1000.0, 60.0),
    "Head_J2_Joint": (800.0, 50.0),
}


# ---------------------------------------------------------------------------
# Virtual mobile base (2026-09-21)
#
# The source URDF has a fixed root and no wheel hinge, so the MJCF used to weld
# Underpan_Base_link to the world. The base is now a free-floating body under
# real gravity driven by two velocity actuators along the world X / Y axes, which
# is how the virtual driving motion is imitated in simulation:
#
#   * the ground plane sits at z = 0, so the world origin is on the floor;
#   * the base frame starts at wheel-contact height above the floor: the wheels
#     touch the ground, the coordinate frame does not;
#   * the base meets the ground frictionlessly (condim=1, forced from the base
#     side with geom priority so the rest of the scene keeps mu = 0.9). That is
#     the honest idealisation of a swerve / omni-wheel drive: the wheels can
#     always deliver whatever ground-plane force is commanded, so the only thing
#     resisting a slide is the servo. It also means the robot *rests* on the
#     floor rather than hovering, and drops back down after being lifted.
#   * act_base_vx / act_base_vy slide the robot (m/s, world axes). The free
#     joint's translation DOFs are world aligned, so the commands do not rotate
#     with the chassis.
#
# Why the three rotation brakes exist: the sliding command is a generalized
# force on the free joint's translation DOFs, i.e. it pushes the base frame --
# which sits ~0.06 m above the floor while the robot's centre of mass is ~0.4 m
# above it. The mass matrix therefore couples translation and pitch, and the
# servo kicks send the base pitching at tens of rad/s. Measured with the brakes
# removed: 4.3 deg tilt and a 53 mm climb during a 1.44 m slide, plus 208 mm of
# vertical drift when both arms are slammed to 60 % of their range. With a
# velocity brake (ctrl pinned to 0) on roll/pitch/yaw the same runs give
# 0.11 deg tilt, no vertical drift, and unchanged X/Y tracking. A brake on Z is
# not needed: gravity plus the ground contact hold the height.
#
# How the gains were chosen -- a probe script rebuilt the viewer's mouse
# spring-damper (mjv_select -> mjv_initPerturb -> mjv_movePerturb ->
# mjv_applyPerturbForce), so these are measurements, not
# estimates. A viewer drag is a spring of `vis.map.stiffness` = 100 N/m per unit
# localmass; clicking the middle of the base box gives localmass = 78 kg, i.e.
# 7.8 kN/m, and the perturbation's own damper adds another 780 N per m/s. Every
# other resistance therefore has to stay small, because it fights the mouse and
# the velocity command at the same time:
#
#   * BASE_VELOCITY_KV 20000 -> 4000. At 20000 the servo alone was 20 kN per
#     m/s, so a 3 s horizontal drag moved the base 0.54 m; at 4000 the same drag
#     reaches 1.47 m. The velocity-tracking error is unchanged at 2.3 %.
#   * BASE_DAMPING 400 -> 20. Passive damping, so it costs travel and tracking
#     accuracy both; 400 N/(m/s) cost roughly 100 mm of drag travel. A small
#     residual is kept so that no free-joint DOF is left purely undamped.
#   * Real gravity: every link used to carry gravcomp="1", i.e. the whole robot
#     was weightless. That also meant zero ground contact, so the viewer's
#     Ctrl+right-drag -- documented as "translate object in the VERTICAL plane"
#     -- pushed the base into the air and it never came back down. Measured drop
#     from 0.30 m: +300 mm and stuck before, 0.0 mm after.
#
# No wheel joint is added: the contact friction of real omni wheels (3-DOF
# swerve drive) is not modelled, the base simply slides.
# ---------------------------------------------------------------------------
GROUND_Z = 0.0                 # ground plane height (world origin = floor)
BASE_CLEARANCE = 0.001         # gap between the wheel contact plane and the floor
BASE_GRAVCOMP = 0              # 0 = real gravity; 1 makes the whole robot weightless
BASE_DAMPING = 20.0            # uniform damping on all six free-joint DOFs
BASE_ARMATURE = 1.0            # extra inertia, improves the DOF conditioning
BASE_VELOCITY_LIMIT = 2.0      # |v| limit of the sliding command (m/s)
BASE_VELOCITY_KV = 4000.0      # velocity servo gain of the sliding actuators
# Attitude brakes: velocity actuators with ctrl pinned to 0.  Two different
# jobs, so two different gains (probe: user-level skill
# mujoco-viewer-drag-tuning, ROTATE section, configs R_*).
#   roll/pitch: the ONLY thing damping slide-induced tilt (without them a
#     1.44 m slide tilts the chassis 4.3 deg) and the mouse can barely fight
#     them -- which is fine, a robot should not be easy to tip over.
#   yaw: pure rate damper with no other job.  The mouse rotation spring tops
#     out at ~1.1 kN*m, so kv=20000 pinned Ctrl+left-drag at 3 deg/s (looked
#     like "cannot rotate"); kv=2000 turns the same gesture into ~27 deg/s
#     while every stability metric (idle, drop, arm swing, tracking) is
#     unchanged.
BASE_TILT_BRAKE_KV = 20000.0   # roll/pitch brake gain (hold slide-induced tilt)
BASE_YAW_BRAKE_KV = 2000.0     # yaw brake gain (soft: lets the mouse spin the base)
BASE_FREE_JOINT = "base_free_joint"
# Contact of the base with the floor. condim=1 keeps the normal support and drops
# all tangential resistance; priority=1 makes these values win over the ground's
# (MuJoCo otherwise takes condim as the max of the two geoms and averages the
# friction), so the props keep the realistic mu = 0.9 of the ground geom.
BASE_CONTACT = {
    "condim": "1",
    "priority": "1",
    "friction": "0.05 0.02 0.01",
}
BASE_ACTUATORS = (
    # (name suffix, gear axis index, gain, control range, unit, role)
    ("vx", 0, BASE_VELOCITY_KV, BASE_VELOCITY_LIMIT, "m/s", "slide along world +X"),
    ("vy", 1, BASE_VELOCITY_KV, BASE_VELOCITY_LIMIT, "m/s", "slide along world +Y"),
    ("wx", 3, BASE_TILT_BRAKE_KV, 0.0, "rad/s", "hold roll (brake, control pinned to 0)"),
    ("wy", 4, BASE_TILT_BRAKE_KV, 0.0, "rad/s", "hold pitch (brake, control pinned to 0)"),
    ("wz", 5, BASE_YAW_BRAKE_KV, 0.0, "rad/s", "hold yaw (soft brake, lets the mouse spin the base)"),
)

# End-effector frames (2026-09-29). The user asked for a *virtual joint*
# l_ee / r_ee at each wrist that is only a coordinate frame, sitting on the
# circular end face of the end-effector mesh, at its centre.
#
#   * Geometry (verified with trimesh on Arm_X7_Link.STL): the end face is the
#     flange at the extreme y = -0.087 plane of the wrist mesh -- circular,
#     radius ~31.5 mm, centred exactly on the link's y axis (the face carries
#     a bolt pattern of 5 holes on a 25 mm bolt circle plus a 15.8 mm centre
#     bore). The discs at y = -0.058 are the internal flange/housing
#     interface, not the end face.
#   * MJCF has no "fixed" joint type. The MJCF equivalent of a URDF fixed
#     joint is a child body WITHOUT a <joint> element: it adds no DOF, no
#     mass and no contact, i.e. exactly a named coordinate frame welded to
#     the parent. That is how l_ee / r_ee are implemented.
#   * The frame axes stay parallel to the wrist link frame (URDF zero rpy);
#     only the origin is offset to the flange-face centre (0, -0.087, 0).
#   * The <site> of the same name inside each frame is a small red marker so
#     the otherwise invisible frame can be seen and selected in the viewer;
#     sites have no physics.
EE_FRAMES = (
    # (frame name, parent link, position in the parent link frame)
    ("l_ee", "Arm_L7_Link", "0 -0.087 0"),
    ("r_ee", "Arm_R7_Link", "0 -0.087 0"),
)


# ---------------------------------------------------------------------------
# SG100 gripper hands (2026-09-29)
#
# The two SG100 URDF hands (SG100_URDF_260811_align_real/) are mounted on the
# end-effector frames: each hand's base_link is WELDED to l_ee / r_ee via a
# jointless body -- the MJCF equivalent of a URDF fixed joint, exactly like
# the EE frames themselves.
#
#   * Orientation: the hand extends along its own +Z (mount face at z = 0,
#     fingertip joints around z = 0.08..0.11). The flange's outward normal
#     is the wrist frame's -Y (the face sits at y = -0.087). A +90 deg
#     rotation about X maps the hand +Z onto that normal:
#     R_x(90 deg) * (0,0,1) = (0,-1,0).
#   * Palm-down roll (user requirement, 2026-09-29): the SG100 fingers root
#     on the -Y side of base_link and curl toward -Y, i.e. the palm normal is
#     the hand frame's -Y. A residual roll about the flange axis (the hand
#     +Z) turns the palm to point DOWN in the world at the arm zero pose,
#     matching the arm joint limits for top-down grasping. The two hands are
#     mirror pairs, so the roll signs are opposite: L rolls -90 deg about the
#     hand +Z, R rolls +90 deg. Combined mount quats (wxyz), each the product
#     R_x(+90 deg) * R_z(theta):
#       L: 0.5 0.5 0.5 -0.5   (theta = -90 deg)
#       R: 0.5 0.5 -0.5 0.5   (theta = +90 deg)
#     Both still map the hand +Z onto the flange outward normal (the roll is
#     about that very axis), so the mount face stays flush on the flange.
#   * Naming: every hand object is prefixed L_ / R_ (bodies, joints, geoms,
#     mesh assets) because both URDFs reuse the same link and mesh names.
#     Actuators follow the arm convention: act_<l/r>_<joint>.
#   * Drive (user requirement): every finger joint is a position actuator,
#     the same actuator type as the arm joints -- but with a dedicated PD
#     table. The fingers are ~200x lighter than the arm links and their
#     inertias are ~1e-5 kg*m^2, so the arm gains (kp 800..30000, and the
#     generator's 2500/150 fallback) as well as the arm joint armature=0.05
#     / damping=2.5 would swamp or blow up a 20 g finger link. Gains are
#     ordered by the load each joint carries (proximal > middle > distal);
#     kv gives roughly critical-to-slightly-overdamped behaviour with the
#     armature below. Raise kp for a stronger grip; the values here are
#     stable at dt = 0.001 (implicitfast).
# ---------------------------------------------------------------------------
SG100_ROOT = PROJECT_ROOT / "SG100_URDF_260811_align_real"
HANDS = (
    # (side prefix, ee frame, source package dir, mesh subdir)
    ("L", "l_ee", "SG100V3.0_L_URDF_3D", "left_hand"),
    ("R", "r_ee", "SG100V3.0_R_URDF_3D", "right_hand"),
)
# Maps the hand +Z onto the flange outward normal (wrist -Y) AND rolls the
# palm (hand -Y) to face down in the world at the arm zero pose; see the
# comment block above for the derivation. wxyz quaternions, per side.
HAND_MOUNT_QUATS = {
    "L": "0.5 0.5 0.5 -0.5",
    "R": "0.5 0.5 -0.5 0.5",
}
# Per-joint finger PD gains (position mode). All joints hold the URDF zero
# pose against gravity; ranges come from the URDF limits.
HAND_CONTROL_PARAMS: dict[str, tuple[float, float]] = {
    "thumb_j1": (300.0, 2.5),
    "thumb_j2": (250.0, 2.0),
    "thumb_j3": (150.0, 1.2),
    "index_j1": (300.0, 2.5),
    "index_j2": (250.0, 2.0),
    "index_j3": (150.0, 1.2),
    "middle_j1": (300.0, 2.5),
    "middle_j2": (150.0, 1.2),
    "little_j1": (250.0, 2.0),
    "little_j2": (200.0, 1.6),
    "little_j3": (120.0, 1.0),
}
HAND_JOINT_ARMATURE = 0.002   # ~200x the finger link inertia: DOF conditioning
HAND_JOINT_DAMPING = 0.05     # near zero: the servo kv does the damping


def control_params(joint_name: str) -> tuple[float, float]:
    return CONTROL_PARAMS.get(joint_name, (2500.0, 150.0))


def float_text(value: str, default: float = 0.0) -> str:
    try:
        numeric = float(value)
        # The supplied URDF contains one malformed decimal: 0301019860.
        # Treat a long digit string with a leading zero as a missing decimal
        # point, instead of allowing an obviously impossible link offset.
        if re.fullmatch(r"-?0\d{6,}", str(value).strip()):
            sign = "-" if str(value).strip().startswith("-") else ""
            digits = str(value).strip().lstrip("-")[1:]
            numeric = float(f"{sign}0.{digits}")
        return f"{numeric:.9g}"
    except (TypeError, ValueError):
        return f"{default:.9g}"


def vector_text(element: ET.Element | None, attr: str, default: str = "0 0 0") -> str:
    if element is None:
        return default
    return " ".join(float_text(item) for item in element.get(attr, default).split())


def quat_text(element: ET.Element | None) -> str:
    """Convert URDF fixed-axis roll-pitch-yaw to MuJoCo wxyz quaternion."""
    if element is None:
        return "1 0 0 0"
    values = [float(item) for item in element.get("rpy", "0 0 0").split()]
    roll, pitch, yaw = values
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return " ".join(
        f"{value:.9g}"
        for value in (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
    )


def asset_group(filename: str) -> str:
    name = filename.lower()
    if "underpan" in name:
        return "base"
    if name.startswith("arm_l"):
        return "left_arm"
    if name.startswith("arm_r"):
        return "right_arm"
    if "leg" in name:
        return "other"
    if "head" in name:
        return "other"
    if "wheel" in name:
        return "wheels"
    return "other"


def safe_name(name: str) -> str:
    return name.replace(" ", "_").replace("/", "_").replace("\\", "_")


def stl_face_count(path: Path) -> int:
    data = path.read_bytes()
    if len(data) < 84:
        return 0
    return struct.unpack("<I", data[80:84])[0]


def copy_mesh(source: Path, target: Path) -> Path:
    """Copy the source geometry without changing its triangles.

    MuJoCo's binary STL reader has a 200k-face limit. For larger user meshes,
    export the same triangles as OBJ instead of decimating the user's model.
    """
    if source.suffix.lower() != ".stl" or stl_face_count(source) <= 200_000:
        shutil.copy2(source, target)
        return target
    try:
        import trimesh
    except ImportError as exc:
        raise RuntimeError(
            f"{source.name} has {stl_face_count(source)} faces. "
            "Install trimesh and fast-simplification before regenerating."
        ) from exc
    mesh = trimesh.load_mesh(source, process=False)
    obj_target = target.with_suffix(".obj")
    mesh.export(obj_target, file_type="obj")
    print(f"Converted {source.name} to exact OBJ geometry: {len(mesh.faces)} faces.")
    return obj_target


def parse_source() -> tuple[ET.Element, dict[str, ET.Element], list[ET.Element]]:
    root = ET.parse(SOURCE_URDF).getroot()
    links = {link.get("name", ""): link for link in root.findall("link")}
    joints = list(root.findall("joint"))
    return root, links, joints


def copy_assets(root: ET.Element) -> dict[str, str]:
    for directory in ("base", "left_arm", "right_arm", "gripper", "wheels", "other"):
        (MESH_ROOT / directory).mkdir(parents=True, exist_ok=True)
    (OUTPUT_ROOT / "assets" / "textures").mkdir(parents=True, exist_ok=True)
    (OUTPUT_ROOT / "assets" / "materials").mkdir(parents=True, exist_ok=True)

    mapping: dict[str, str] = {}
    for mesh in root.findall(".//mesh"):
        raw = mesh.get("filename", "")
        source_name = Path(raw.replace("package://", "").replace("\\", "/")).name
        if source_name in mapping:
            mapping[raw] = mapping[source_name]
            continue
        source = SOURCE_ROOT / "meshes" / source_name
        if not source.exists():
            raise FileNotFoundError(f"URDF mesh not found: {source}")
        target_name = safe_name(source.name)
        if source.suffix.lower() == ".stl" and stl_face_count(source) > 200_000:
            target_name = Path(target_name).with_suffix(".obj").name
        relative = Path("assets") / "meshes" / asset_group(target_name) / target_name
        target = OUTPUT_ROOT / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        runtime_target = copy_mesh(source, target)
        relative = runtime_target.relative_to(OUTPUT_ROOT)
        try:
            import trimesh
        except ImportError as exc:
            raise RuntimeError("Install trimesh before generating the MJCF package.") from exc
        bounds = trimesh.load_mesh(source, process=False).bounds
        MESH_BOUNDS[relative.as_posix()] = (
            ((bounds[0] + bounds[1]) / 2.0).tolist(),
            ((bounds[1] - bounds[0]) / 2.0).tolist(),
        )
        mapping[raw] = relative.as_posix()
        mapping[source_name] = relative.as_posix()

    # Copy any non-mesh resources that may be present in the source tree.
    known_meshes = {p.resolve() for p in (SOURCE_ROOT / "meshes").glob("*")}
    for source in SOURCE_ROOT.rglob("*"):
        if not source.is_file() or source.resolve() in known_meshes or source.suffix.lower() == ".urdf":
            continue
        relative = Path("assets") / ("textures" if source.suffix.lower() in {".png", ".jpg", ".jpeg", ".bmp", ".tga"} else "materials") / safe_name(source.name)
        target = OUTPUT_ROOT / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists():
            shutil.copy2(source, target)
    return mapping


def _inertia_rpy(origin: ET.Element | None) -> tuple[float, float, float]:
    if origin is None:
        return (0.0, 0.0, 0.0)
    try:
        values = [float(item) for item in origin.get("rpy", "0 0 0").split()]
    except ValueError:
        return (0.0, 0.0, 0.0)
    return (values[0], values[1], values[2])


def _rotate_fullinertia(
    values: tuple[float, float, float, float, float, float],
    roll: float, pitch: float, yaw: float,
) -> tuple[float, float, float, float, float, float]:
    """Rotate an (ixx, iyy, izz, ixy, ixz, iyz) tensor by fixed-axis rpy.

    URDF defines the inertia tensor in the frame given by the inertial
    origin's rpy (e.g. the left SG100 thumb_link1 carries rpy = 0 0 3.14),
    so the tensor must be rotated into the link frame: I' = R I R^T with
    R = Rz(yaw) Ry(pitch) Rx(roll).
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp,     cp * sr,               cp * cr),
    )
    ixx, iyy, izz, ixy, ixz, iyz = values
    I = ((ixx, ixy, ixz), (ixy, iyy, iyz), (ixz, iyz, izz))
    RI = [[sum(R[i][k] * I[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    Ip = [[sum(RI[i][k] * R[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    return (Ip[0][0], Ip[1][1], Ip[2][2], Ip[0][1], Ip[0][2], Ip[1][2])


def add_inertial(body: ET.Element, link: ET.Element) -> None:
    inertial = link.find("inertial")
    if inertial is None:
        return
    origin = inertial.find("origin")
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    if mass is None or inertia is None:
        return
    values = tuple(
        float(float_text(inertia.get(key, "0")))
        for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
    )
    roll, pitch, yaw = _inertia_rpy(origin)
    if roll or pitch or yaw:
        values = _rotate_fullinertia(values, roll, pitch, yaw)
    attrs = {
        "pos": vector_text(origin, "xyz"),
        "mass": float_text(mass.get("value")),
        # Preserve the complete URDF inertia tensor (rotated into the link
        # frame when the inertial origin carries an rpy). MuJoCo's
        # fullinertia order is ixx, iyy, izz, ixy, ixz, iyz, link-local.
        "fullinertia": " ".join(f"{value:.9g}" for value in values),
    }
    ET.SubElement(body, "inertial", attrs)


def add_geoms(body: ET.Element, link: ET.Element, mapping: dict[str, str],
              contact: dict[str, str] | None = None, name_prefix: str = "") -> None:
    """Add the visual and collision geoms of one link.

    ``contact`` overrides attributes of the collision box (used for the base,
    whose floor contact is frictionless); every other link keeps the default
    realistic friction. ``name_prefix`` namespaces the geom names (the SG100
    hands reuse link names between the left and right URDF).
    """
    color = "0.7 0.7 0.7 1"
    visual = link.find("visual")
    if visual is not None:
        material = visual.find("material/color")
        if material is not None:
            color = material.get("rgba", color).replace(",", " ")
        geometry = visual.find("geometry/mesh")
        if geometry is not None:
            raw = geometry.get("filename", "")
            mesh_path = mapping[raw]
            ET.SubElement(
                body,
                "geom",
                {
                    "name": f"visual_{name_prefix}{safe_name(link.get('name', 'link'))}",
                    "type": "mesh",
                    "mesh": Path(mesh_path).stem,
                    "pos": vector_text(visual.find("origin"), "xyz"),
                    "quat": quat_text(visual.find("origin")),
                    "rgba": color,
                    "contype": "0",
                    "conaffinity": "0",
                    "group": "1",
                },
            )

    collision = link.find("collision")
    if collision is not None:
        geometry = collision.find("geometry/mesh")
        if geometry is not None:
            raw = geometry.get("filename", "")
            mesh_path = mapping[raw]
            center, half_size = MESH_BOUNDS[mesh_path]
            attributes = {
                "name": f"collision_{name_prefix}{safe_name(link.get('name', 'link'))}",
                "type": "box",
                "pos": " ".join(f"{value:.9g}" for value in center),
                "size": " ".join(f"{value:.9g}" for value in half_size),
                "contype": "1",
                "conaffinity": "0",
                "group": "3",
                "friction": "0.8 0.02 0.01",
            }
            if contact:
                attributes.update(contact)
            ET.SubElement(body, "geom", attributes)


def add_body(
    parent: ET.Element,
    link_name: str,
    links: dict[str, ET.Element],
    children: dict[str, list[ET.Element]],
    mapping: dict[str, str],
    actuator_rows: list[dict[str, object]],
) -> None:
    link = links[link_name]
    body = ET.SubElement(parent, "body", {"name": link_name, "pos": "0 0 0"})
    add_inertial(body, link)
    add_geoms(body, link, mapping)

    for joint in children.get(link_name, []):
        child_name = joint.find("child").get("link", "")
        origin = joint.find("origin")
        child_body = ET.SubElement(
            body,
            "body",
            {
                "name": child_name,
                "pos": vector_text(origin, "xyz"),
                "quat": quat_text(origin),
                "gravcomp": f"{BASE_GRAVCOMP}",
            },
        )
        joint_type = joint.get("type", "fixed")
        if joint_type in {"revolute", "continuous", "prismatic"}:
            kp, kv = control_params(joint.get("name", ""))
            axis = vector_text(joint.find("axis"), "xyz", "0 0 1")
            limit = joint.find("limit")
            attrs = {
                "name": joint.get("name", ""),
                "type": "hinge" if joint_type != "prismatic" else "slide",
                "axis": axis,
                "armature": "0.05",
                "damping": "2.5",
            }
            if limit is not None and joint_type != "continuous":
                attrs["range"] = f"{float_text(limit.get('lower'))} {float_text(limit.get('upper'))}"
                attrs["limited"] = "true"
            else:
                attrs["limited"] = "false"
            ET.SubElement(child_body, "joint", attrs)
            if limit is not None:
                lower = float(limit.get("lower", "-3.14159"))
                upper = float(limit.get("upper", "3.14159"))
                effort = float(limit.get("effort", "20"))
                initial = max(lower, min(0.0, upper))
            else:
                lower, upper, effort, initial = -math.pi, math.pi, 20.0, 0.0
            actuator_name = f"act_{safe_name(joint.get('name', 'joint')).lower()}"
            ET.SubElement(
                actuator_root,
                "position",
                {
                    "name": actuator_name,
                    "joint": joint.get("name", ""),
                    "kp": f"{kp:.9g}",
                    "kv": f"{kv:.9g}",
                    "ctrlrange": f"{lower:.9g} {upper:.9g}",
                },
            )
            actuator_rows.append(
                {
                    "joint_name": joint.get("name", ""),
                    "joint_type": joint_type,
                    "actuator_name": actuator_name,
                    "joint_range": [lower, upper],
                    "ctrlrange": [lower, upper],
                    "initial_position": initial,
                    "kp": kp,
                    "kv": kv,
                    "position_control": True,
                }
            )
        add_inertial(child_body, links[child_name])
        add_geoms(child_body, links[child_name], mapping)
        add_body_contents(child_body, child_name, links, children, mapping, actuator_rows)


def add_body_contents(
    body: ET.Element,
    link_name: str,
    links: dict[str, ET.Element],
    children: dict[str, list[ET.Element]],
    mapping: dict[str, str],
    actuator_rows: list[dict[str, object]],
) -> None:
    for joint in children.get(link_name, []):
        child_name = joint.find("child").get("link", "")
        origin = joint.find("origin")
        child_body = ET.SubElement(
            body,
            "body",
            {
                "name": child_name,
                "pos": vector_text(origin, "xyz"),
                "quat": quat_text(origin),
                "gravcomp": f"{BASE_GRAVCOMP}",
            },
        )
        joint_type = joint.get("type", "fixed")
        if joint_type in {"revolute", "continuous", "prismatic"}:
            kp, kv = control_params(joint.get("name", ""))
            limit = joint.find("limit")
            attrs = {
                "name": joint.get("name", ""),
                "type": "hinge" if joint_type != "prismatic" else "slide",
                "axis": vector_text(joint.find("axis"), "xyz", "0 0 1"),
                "limited": "false" if joint_type == "continuous" else "true",
                "armature": "0.05",
                "damping": "2.5",
            }
            if limit is not None and joint_type != "continuous":
                lower = float(limit.get("lower", "-3.14159"))
                upper = float(limit.get("upper", "3.14159"))
                attrs["range"] = f"{lower:.9g} {upper:.9g}"
                effort = float(limit.get("effort", "20"))
            else:
                lower, upper, effort = -math.pi, math.pi, 20.0
            ET.SubElement(child_body, "joint", attrs)
            actuator_name = f"act_{safe_name(joint.get('name', 'joint')).lower()}"
            ET.SubElement(
                actuator_root,
                "position",
                {
                    "name": actuator_name,
                    "joint": joint.get("name", ""),
                    "kp": f"{kp:.9g}",
                    "kv": f"{kv:.9g}",
                    "ctrlrange": f"{lower:.9g} {upper:.9g}",
                },
            )
            actuator_rows.append(
                {
                    "joint_name": joint.get("name", ""),
                    "joint_type": joint_type,
                    "actuator_name": actuator_name,
                    "joint_range": [lower, upper],
                    "ctrlrange": [lower, upper],
                    "initial_position": 0.0,
                    "kp": kp,
                    "kv": kv,
                    "position_control": True,
                }
            )
        add_inertial(child_body, links[child_name])
        add_geoms(child_body, links[child_name], mapping)
        add_body_contents(child_body, child_name, links, children, mapping, actuator_rows)


def find_body(element: ET.Element, name: str) -> ET.Element | None:
    for body in element.iter("body"):
        if body.get("name") == name:
            return body
    return None


def add_ee_frames(parent_element: ET.Element) -> list[dict[str, object]]:
    """Attach the URDF-style fixed end-effector frames l_ee / r_ee.

    See the EE_FRAMES comment block for the geometry derivation and why the
    frames are jointless bodies (MJCF fixed-joint equivalent).
    """
    rows: list[dict[str, object]] = []
    for name, parent_link, pos in EE_FRAMES:
        parent_body = find_body(parent_element, parent_link)
        if parent_body is None:
            raise RuntimeError(f"EE-frame parent body not found in the tree: {parent_link}")
        body = ET.SubElement(parent_body, "body", {"name": name, "pos": pos})
        ET.SubElement(
            body,
            "site",
            {"name": name, "size": "0.003", "rgba": "1 0 0 1"},
        )
        rows.append(
            {
                "frame": name,
                "parent_link": parent_link,
                "pos_in_parent": pos,
                "type": "fixed_frame_body",
                "note": (
                    "URDF fixed-joint equivalent: welded body without a joint, "
                    "no DOF/mass; axes parallel to the parent link frame; origin "
                    "at the centre of the circular flange face (y = -0.087)"
                ),
            }
        )
    return rows


def _hand_control(joint_name: str) -> tuple[float, float]:
    """PD gains of one SG100 finger joint (no arm fallback, by design)."""
    try:
        return HAND_CONTROL_PARAMS[joint_name]
    except KeyError as exc:
        raise RuntimeError(
            f"SG100 joint {joint_name} has no entry in HAND_CONTROL_PARAMS; "
            "finger gains must never fall back to the arm table"
        ) from exc


def copy_hand_assets(
    side: str, package: str, hand_root: ET.Element, subdir: str
) -> dict[str, str]:
    """Copy one SG100 hand's meshes into the MJCF asset tree.

    Both hand URDFs reuse the same mesh file names, so the copied files -- and
    therefore the mesh asset names -- are prefixed with the side. The
    <asset><mesh> entries are registered here because the Luna registration
    loop has already run by the time the hands are parsed.
    """
    import trimesh

    hand_dir = MESH_ROOT / "hands" / subdir
    hand_dir.mkdir(parents=True, exist_ok=True)
    mapping: dict[str, str] = {}
    for mesh in hand_root.findall(".//mesh"):
        raw = mesh.get("filename", "")
        if raw in mapping:
            continue
        source_name = Path(raw.replace("package://", "").replace("\\", "/")).name
        source = SG100_ROOT / package / "meshes" / source_name
        if not source.exists():
            raise FileNotFoundError(f"SG100 mesh not found: {source}")
        target = hand_dir / f"{side}_{source_name}"
        copy_mesh(source, target)
        relative = (Path("assets") / "meshes" / "hands" / subdir / target.name).as_posix()
        bounds = trimesh.load_mesh(source, process=False).bounds
        MESH_BOUNDS[relative] = (
            ((bounds[0] + bounds[1]) / 2.0).tolist(),
            ((bounds[1] - bounds[0]) / 2.0).tolist(),
        )
        mapping[raw] = relative
        mesh_name = Path(relative).stem
        if asset_root.find(f"mesh[@name='{mesh_name}']") is None:
            ET.SubElement(asset_root, "mesh", {"name": mesh_name, "file": relative})
    return mapping


def add_hand_contents(
    body: ET.Element,
    link_name: str,
    links: dict[str, ET.Element],
    children: dict[str, list[ET.Element]],
    mapping: dict[str, str],
    side: str,
    actuator_rows: list[dict[str, object]],
) -> None:
    """Recursively convert one SG100 hand below its mounted base body.

    Every finger joint becomes a hinge plus a position actuator (the same
    drive as the arm joints, per the user requirement) with the dedicated
    finger PD table and much smaller armature/damping than the arm joints.
    """
    for joint in children.get(link_name, []):
        joint_name = joint.get("name", "")
        if joint.get("type") != "revolute":
            raise RuntimeError(
                f"unexpected SG100 joint type for {side}/{joint_name}: "
                f"{joint.get('type')}"
            )
        child_name = joint.find("child").get("link", "")
        origin = joint.find("origin")
        child_body = ET.SubElement(
            body,
            "body",
            {
                "name": f"{side}_{child_name}",
                "pos": vector_text(origin, "xyz"),
                "quat": quat_text(origin),
                "gravcomp": f"{BASE_GRAVCOMP}",
            },
        )
        limit = joint.find("limit")
        if limit is None:
            raise RuntimeError(f"SG100 joint {side}/{joint_name} has no limit")
        lower = float(limit.get("lower", "-3.14159"))
        upper = float(limit.get("upper", "3.14159"))
        kp, kv = _hand_control(joint_name)
        ET.SubElement(
            child_body,
            "joint",
            {
                "name": f"{side}_{joint_name}",
                "type": "hinge",
                "axis": vector_text(joint.find("axis"), "xyz", "0 0 1"),
                "range": f"{lower:.9g} {upper:.9g}",
                "limited": "true",
                "armature": f"{HAND_JOINT_ARMATURE:.9g}",
                "damping": f"{HAND_JOINT_DAMPING:.9g}",
            },
        )
        add_inertial(child_body, links[child_name])
        add_geoms(child_body, links[child_name], mapping, name_prefix=f"{side}_")
        actuator_name = f"act_{side.lower()}_{joint_name}".lower()
        ET.SubElement(
            actuator_root,
            "position",
            {
                "name": actuator_name,
                "joint": f"{side}_{joint_name}",
                "kp": f"{kp:.9g}",
                "kv": f"{kv:.9g}",
                "ctrlrange": f"{lower:.9g} {upper:.9g}",
            },
        )
        actuator_rows.append(
            {
                "joint_name": f"{side}_{joint_name}",
                "joint_type": "revolute",
                "actuator_name": actuator_name,
                "joint_range": [lower, upper],
                "ctrlrange": [lower, upper],
                "initial_position": 0.0,
                "kp": kp,
                "kv": kv,
                "position_control": True,
                "hand": side,
            }
        )
        add_hand_contents(
            child_body, child_name, links, children, mapping, side, actuator_rows
        )


def add_hands(parent_element: ET.Element) -> list[dict[str, object]]:
    """Weld both SG100 hands onto the l_ee / r_ee end-effector frames.

    Returns one mount description row per hand. The finger actuators are
    collected in the module-level ``hand_actuator_rows`` list, kept separate
    from the arm rows so the arm contract in the config stays at 22 joints.
    """
    mounts: list[dict[str, object]] = []
    for side, ee_name, package, subdir in HANDS:
        urdf_path = SG100_ROOT / package / "urdf" / f"{package}.urdf"
        hand_root = ET.parse(urdf_path).getroot()
        hand_links = {node.get("name", ""): node for node in hand_root.findall("link")}
        hand_joints = list(hand_root.findall("joint"))
        parented = {node.find("child").get("link", "") for node in hand_joints}
        roots = sorted(name for name in hand_links if name not in parented)
        if roots != ["base_link"]:
            raise RuntimeError(f"unexpected SG100 root links for {side}: {roots}")
        hand_children: dict[str, list[ET.Element]] = {}
        for node in hand_joints:
            hand_children.setdefault(
                node.find("parent").get("link", ""), []
            ).append(node)
        mapping = copy_hand_assets(side, package, hand_root, subdir)
        ee_body = find_body(parent_element, ee_name)
        if ee_body is None:
            raise RuntimeError(
                f"EE frame body not found for the {side} hand: {ee_name}"
            )
        # The mount body itself is jointless (welded to the EE frame) and
        # carries the hand base_link's inertial/geoms in its own frame.
        base_body = ET.SubElement(
            ee_body,
            "body",
            {
                "name": f"{side}_hand_base",
                "pos": "0 0 0",
                "quat": HAND_MOUNT_QUATS[side],
                "gravcomp": f"{BASE_GRAVCOMP}",
            },
        )
        add_inertial(base_body, hand_links["base_link"])
        add_geoms(base_body, hand_links["base_link"], mapping, name_prefix=f"{side}_")
        add_hand_contents(
            base_body, "base_link", hand_links, hand_children, mapping, side,
            hand_actuator_rows,
        )
        mounts.append(
            {
                "side": side,
                "ee_frame": ee_name,
                "mount_body": f"{side}_hand_base",
                "pos_in_ee": "0 0 0",
                "quat_in_ee": HAND_MOUNT_QUATS[side],
                "urdf": f"SG100_URDF_260811_align_real/{package}",
                "note": (
                    "base_link welded to the EE frame (jointless body); hand "
                    "+Z mapped onto the flange outward normal (wrist -Y) by a "
                    "+90 deg roll about X, then rolled about that normal so "
                    "the palm (hand -Y) faces DOWN in the world at the arm "
                    "zero pose (L: -90 deg, R: +90 deg, mirror hands); every "
                    "finger joint is a position actuator with the dedicated "
                    "HAND_CONTROL_PARAMS PD table"
                ),
            }
        )
    return mounts


def write_xml(root: ET.Element, output: Path) -> None:
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)


root, links, joints = parse_source()
mapping = copy_assets(root)
parented = {joint.find("child").get("link", "") for joint in joints}
root_link = "Underpan_Base_link"
if root_link not in links:
    raise RuntimeError("Underpan_Base_link was not found in the URDF")
if root_link in parented:
    raise RuntimeError("Underpan_Base_link is not a root link in the source URDF")

children: dict[str, list[ET.Element]] = {}
for joint in joints:
    children.setdefault(joint.find("parent").get("link", ""), []).append(joint)

robot = ET.Element(
    "mujoco",
    {"model": "luna_m6_robot"},
)
ET.SubElement(robot, "compiler", {"angle": "radian", "coordinate": "local"})
asset_root = ET.SubElement(robot, "asset")
for raw, relative in mapping.items():
    if "/" not in relative:
        continue
    mesh_name = Path(relative).stem
    if asset_root.find(f"mesh[@name='{mesh_name}']") is None:
        ET.SubElement(asset_root, "mesh", {"name": mesh_name, "file": relative})
worldbody = ET.SubElement(robot, "worldbody")
# The base frame is lifted so the wheels (the lowest point of the base mesh)
# touch the ground; see the BASE_* constants above.
base_collision_mesh = links[root_link].find("collision/geometry/mesh")
if base_collision_mesh is None:
    raise RuntimeError("Underpan_Base_link has no collision mesh in the source URDF")
base_center, base_half = MESH_BOUNDS[mapping[base_collision_mesh.get("filename", "")]]
wheel_bottom = base_center[2] - base_half[2]          # base-local wheel contact plane
base_lift = GROUND_Z + BASE_CLEARANCE - wheel_bottom
base = ET.SubElement(
    worldbody,
    "body",
    {"name": root_link, "pos": f"0 0 {base_lift:.9g}", "gravcomp": f"{BASE_GRAVCOMP}"},
)
# <joint type="free"> is the general form of <freejoint>; the dedicated
# <freejoint> element has no damping attribute in MuJoCo 3.x, and the uniform
# damping is what keeps the floating base from drifting while the arms move.
ET.SubElement(
    base,
    "joint",
    {
        "name": BASE_FREE_JOINT,
        "type": "free",
        "damping": f"{BASE_DAMPING:.9g}",
        "armature": f"{BASE_ARMATURE:.9g}",
    },
)
add_inertial(base, links[root_link])
add_geoms(base, links[root_link], mapping, contact=BASE_CONTACT)
actuator_rows: list[dict[str, object]] = []
actuator_root = ET.SubElement(robot, "actuator")
# Virtual mobile base first, so its sliders sit at the top of the viewer's
# Control panel. This shifts every position-actuator index by +5;
# luna_m6_scene.xml keeps the test indices in <custom><numeric>.
base_actuator_rows: list[dict[str, object]] = []
for suffix, axis, gain, limit, unit, role in BASE_ACTUATORS:
    gear = ["0"] * 6
    gear[axis] = "1"
    actuator_name = f"act_base_{suffix}"
    lower = -limit if limit else 0.0            # avoid the "-0" spelling
    ET.SubElement(
        actuator_root,
        "velocity",
        {
            "name": actuator_name,
            "joint": BASE_FREE_JOINT,
            "kv": f"{gain:.9g}",
            "gear": " ".join(gear),
            "ctrlrange": f"{lower:.9g} {limit:.9g}",
        },
    )
    base_actuator_rows.append(
        {
            "actuator_name": actuator_name,
            "joint_name": BASE_FREE_JOINT,
            "joint_type": "free",
            "dof": "xyz"[axis] if axis < 3 else f"rot{'xyz'[axis - 3]}",
            "unit": unit,
            "role": role,
            "ctrlrange": [-limit, limit],
            "actuator_type": "velocity",
            "kv": gain,
            "gear": gear,
        }
    )
for joint in children[root_link]:
    child_name = joint.find("child").get("link", "")
    origin = joint.find("origin")
    child_body = ET.SubElement(
        base,
        "body",
        {
            "name": child_name,
            "pos": vector_text(origin, "xyz"),
            "quat": quat_text(origin),
            "gravcomp": f"{BASE_GRAVCOMP}",
        },
    )
    joint_type = joint.get("type", "fixed")
    if joint_type in {"revolute", "continuous", "prismatic"}:
        kp, kv = control_params(joint.get("name", ""))
        limit = joint.find("limit")
        lower = float(limit.get("lower", "-3.14159")) if limit is not None else -math.pi
        upper = float(limit.get("upper", "3.14159")) if limit is not None else math.pi
        effort = float(limit.get("effort", "20")) if limit is not None else 20.0
        ET.SubElement(
            child_body,
            "joint",
            {
                "name": joint.get("name", ""),
                "type": "hinge" if joint_type != "prismatic" else "slide",
                "axis": vector_text(joint.find("axis"), "xyz", "0 0 1"),
                "range": f"{lower:.9g} {upper:.9g}",
                "limited": "true",
                "armature": "0.05",
                "damping": "2.5",
            },
        )
        actuator_name = f"act_{safe_name(joint.get('name', 'joint')).lower()}"
        ET.SubElement(
            actuator_root,
            "position",
            {
                "name": actuator_name,
                "joint": joint.get("name", ""),
                # Hold the URDF zero pose against gravity. Do not use the
                # URDF effort value as an artificial force clamp here.
                "kp": f"{kp:.9g}",
                "kv": f"{kv:.9g}",
                "ctrlrange": f"{lower:.9g} {upper:.9g}",
            },
        )
        actuator_rows.append(
            {
                "joint_name": joint.get("name", ""),
                "joint_type": joint_type,
                "actuator_name": actuator_name,
                "joint_range": [lower, upper],
                "ctrlrange": [lower, upper],
                "initial_position": 0.0,
                "kp": kp,
                "kv": kv,
                "position_control": True,
            }
        )
    add_inertial(child_body, links[child_name])
    add_geoms(child_body, links[child_name], mapping)
    add_body_contents(child_body, child_name, links, children, mapping, actuator_rows)

ee_frame_rows = add_ee_frames(worldbody)

# SG100 hands: welded to the EE frames, appended AFTER the arm tree walk so
# the arm actuator indices (and the scene's test_position_actuators numerics)
# keep their meaning. Hand actuators go into their own config list.
hand_actuator_rows: list[dict[str, object]] = []
hand_mount_rows = add_hands(worldbody)

write_xml(robot, OUTPUT_ROOT / "luna_m6_robot.xml")

config = {
    "source_urdf": "not referenced at runtime",
    "root_link": "Underpan_Base_link",
    "root_pose": {
        "position": [0.0, 0.0, round(base_lift, 9)],
        "quaternion": [1.0, 0.0, 0.0, 0.0],
    },
    "base_mode": "virtual_mobile_base_sliding_xy",
    "ground_z": GROUND_Z,
    "wheel_contact_plane_z": round(wheel_bottom + base_lift, 9),
    "base_gravity_compensation": BASE_GRAVCOMP,
    "base_floor_contact": dict(BASE_CONTACT),
    "base_free_joint": {
        "name": BASE_FREE_JOINT,
        "type": "free",
        "damping": BASE_DAMPING,
        "armature": BASE_ARMATURE,
        "note": (
            "real gravity (gravcomp=0); the base rests on a frictionless floor "
            "contact (condim=1 + geom priority, so the props keep mu=0.9). X/Y "
            "are driven by act_base_vx / act_base_vy, roll/pitch/yaw are held by "
            "the act_base_w* velocity brakes and Z by gravity plus that contact"
        ),
    },
    "base_actuators": base_actuator_rows,
    "wheel_joints_added": [],
    "ee_frames": ee_frame_rows,
    "hand_mounts": hand_mount_rows,
    "hand_actuators": hand_actuator_rows,
    "actuated_joints": actuator_rows,
}
(OUTPUT_ROOT / "config").mkdir(exist_ok=True)
(OUTPUT_ROOT / "config" / "joint_actuator_config.json").write_text(
    json.dumps(config, indent=2, ensure_ascii=False) + "\n",
    encoding="utf-8",
)

scene = f"""<?xml version="1.0" encoding="utf-8"?>
<mujoco model="luna_m6_scene">
  <compiler angle="radian" coordinate="local" autolimits="true"/>
  <option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81" iterations="100"/>
  <!-- Test and viewer settings are data, so scripts do not duplicate them. -->
  <custom>
    <numeric name="test_position_actuators" data="5 6 11 20"/>
    <numeric name="test_position_targets" data="0.1 0.5 0.5 -0.5"/>
    <numeric name="test_position_steps" data="3000"/>
    <numeric name="test_position_error_tolerance" data="0.01"/>
    <numeric name="test_position_velocity_tolerance" data="0.05"/>
    <numeric name="test_environment_steps" data="5000"/>
    <!-- Base sliding test: vx vy are world-axis speeds in m/s. The default
         direction is -x/-y because the roller line stands at x = 0.37..1.65:
         driving +x would run the robot into it (which is correct physics, but
         not what a tracking check should measure). -->
    <numeric name="test_base_velocity" data="-0.6 -0.4"/>
    <numeric name="test_base_duration" data="2.0"/>
    <numeric name="test_base_position_tolerance" data="0.05"/>
    <numeric name="test_base_height_tolerance" data="0.002"/>
    <numeric name="viewer_free_camera" data="0 0 1.05 3.6 135 -45"/>
    <numeric name="viewer_fps" data="60"/>
  </custom>
  <statistic center="0 0 1.05" extent="1.8"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-130" elevation="-20"/>
    <map znear="0.01" zfar="50"/>
  </visual>
  <asset>
    <texture name="skybox" type="skybox" builtin="gradient"
             rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture name="groundplane" type="2d" builtin="checker" mark="edge"
             rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
             markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane"
              texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>
  <worldbody>
    <!-- The world origin is on the floor. The robot base is lifted instead, to
         the wheel contact plane: base-local z={wheel_bottom:.6f} (lowest point of the
         source base mesh) plus {BASE_CLEARANCE * 1000:.0f} mm clearance, i.e. the base frame starts
         {base_lift:.6f} m above the ground and the wheels rest on it. -->
    <!-- 400 m^2 square arena: 20 m x 20 m; MuJoCo size is half-extent.
         mu = 0.9 here applies to the props. The robot base deliberately does
         NOT use it: its collision geom carries priority="1", so the base slides
         on the floor without tangential resistance (see luna_m6_robot.xml). -->
    <geom name="ground" type="plane" pos="0 0 {GROUND_Z:.9g}" size="10 10 0.1"
          material="groundplane" friction="0.9 0.03 0.01"
          contype="1" conaffinity="1"/>
    <!-- About 45 degrees above horizontal, aimed at the chest. -->
    <camera name="camera_main" pos="1.6 -1.6 3.25"
            xyaxes="0.7071068 0.7071068 0 -0.497468 0.497468 0.718421"/>
    <camera name="camera_debug" pos="2.4 -2.4 2.8"
            xyaxes="0.7071068 0.7071068 0 -0.351123 0.351123 0.866025"/>
    <light name="light_overhead" pos="0 0 1.5" dir="0 0 -1" directional="true"/>
  </worldbody>
  <include file="luna_m6_robot.xml"/>
  <!-- 场景道具（输送线台子等）。单独成文件，重新运行 generate_mjcf.py 不会被覆盖。 -->
  <include file="luna_m6_props.xml"/>
</mujoco>
"""
(OUTPUT_ROOT / "luna_m6_scene.xml").write_text(scene, encoding="utf-8")
print(
    f"Generated {len(mapping)} mesh references, {len(base_actuator_rows)} base "
    f"actuators, {len(actuator_rows)} position actuators and "
    f"{len(hand_actuator_rows)} SG100 finger actuators "
    f"({len(hand_mount_rows)} hands mounted)."
)
print(f"Base frame lifted to z = {base_lift:.6f} (wheel contact plane z = {wheel_bottom:.6f}).")
