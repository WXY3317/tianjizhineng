# Luna M6 MuJoCo Package

This directory is a self-contained MuJoCo model package generated from the
source URDF in `..\Gento Luna_URDF`. The source URDF and its resources are not
modified and are not required at runtime.

## Directory layout

```text
mjcf/
  luna_m6_scene.xml
  luna_m6_robot.xml
  assets/
    meshes/
      base/
      left_arm/
      right_arm/
      gripper/
      wheels/
      other/
    textures/
    materials/
  config/joint_actuator_config.json
  scripts/
    generate_mjcf.py
    generate_workpieces.py
```

All test scripts live in the project-level `tests/` directory (see below); the
`mjcf/scripts/` folder only holds the two generators.

The supplied URDF contains 23 STL meshes, no texture files, no material files,
and 22 revolute joints. The package therefore contains 23 runtime mesh assets,
one generated checker texture, and 22 position actuators. No gripper or wheel
joint exists in the source model.

The root link is not welded to the world: `base_free_joint` makes it a virtual
mobile base driven by two sliding actuators (`act_base_vx` / `act_base_vy`,
world X/Y). See "Root, base constraint and driving" below.

Two source STL files exceed MuJoCo's 200,000-triangle STL decoder limit. The
generator reads the current source STL files on every regeneration, leaves the
originals untouched, and writes exact, lossless OBJ runtime copies for those
two meshes. The OBJ files preserve the complete source triangle counts. Visual meshes remain available in the model;
collision uses stable per-link box approximations with robot self-collision
disabled and ground collision enabled.

## Install dependencies on Windows

From the project root:

```powershell
uv run --with mujoco --with numpy python -c "import mujoco; print(mujoco.__version__)"
```

The generators are under `mjcf/scripts`; every test script is under the
project-level `tests/` (test scripts never go into `scripts/`, which is
reserved for simulation-control utilities such as `view_environment.py`,
`render_probe.py` and `render_check_hands.py`):

```powershell
uv run --with mujoco --with numpy python .\tests\check_resource_paths.py
uv run --with mujoco --with numpy python .\tests\test_luna_m6.py
.\.venv\Scripts\python.exe .\tests\test_position_control.py
.\.venv\Scripts\python.exe .\tests\test_environment.py
```

To regenerate the package after replacing the source URDF or its resources:

```powershell
uv run --with trimesh python .\mjcf\scripts\generate_mjcf.py
uv run python .\mjcf\scripts\generate_workpieces.py
```

`trimesh` is required by the generator only (it reads the mesh bounding boxes
that become the collision boxes), so it is passed with `--with` instead of being
added to the runtime dependencies. Regenerating rewrites `luna_m6_robot.xml` and
`luna_m6_scene.xml`; `luna_m6_props.xml` is hand-maintained and only its marked
`WORKPIECES` block is rewritten by `generate_workpieces.py`.

If the global uv cache is not writable, set a project-local cache first:

```powershell
$env:UV_CACHE_DIR = ".uv-cache"
```

## Loading the scene

The main scene is `luna_m6_scene.xml`. In Python:

```python
import mujoco

model = mujoco.MjModel.from_xml_path(r"mjcf\luna_m6_scene.xml")
data = mujoco.MjData(model)
```

The model can also be opened with the MuJoCo `simulate` executable by passing
the absolute path to `luna_m6_scene.xml`.

The generated scene XML is the runtime parameter source. It contains the
simulation options, lighting, floor, cameras, actuator gains, joint damping,
and test/viewer values under `<custom><numeric>`. The scripts only load these
values and perform checks; they do not maintain a second set of physics or
viewer parameters. Run `generate_mjcf.py` only when regenerating the XML from
the URDF and mesh assets.

## Root, base constraint and driving

`Underpan_Base_link` is the first body in the world model. The source URDF has
no floating-base joint, so the generated model *adds* one, deliberately:

```xml
<body name="Underpan_Base_link" pos="0 0 0.061002" gravcomp="0">
  <joint name="base_free_joint" type="free" damping="20" armature="1"/>
  <geom name="collision_Underpan_Base_link" type="box" ...
        friction="0.05 0.02 0.01" condim="1" priority="1"/>
```

| Item | Value | Why |
| --- | --- | --- |
| ground plane | `z = 0` | The world origin is on the floor; a mobile base drives around it. |
| base frame height | `z = 0.061002` | The lowest point of the base mesh (the wheel contact plane) is at local `z = -0.060002`; the extra 1 mm is clearance. So **the wheels touch the ground and the frame does not**. |
| gravity | `gravcomp="0"` | Real gravity. The robot has weight, rests on the floor, and falls back down when it is dropped. (An earlier revision put `gravcomp="1"` on every link, which made the whole robot weightless: it hovered at its placement height, never touched the floor, and after a drag into the air never came back down.) |
| floor contact | `condim="1"` + `priority="1"` | The base meets the floor with a normal force but **no tangential resistance** — the idealisation of a swerve / omni-wheel drive, which can always deliver the commanded ground-plane force. `priority` keeps this local to the base, so the props still use the ground's `mu = 0.9`. |
| sliding | `act_base_vx`, `act_base_vy` | Velocity actuators on the free joint's world-aligned translation DOFs, `kv = 4000`, range ±2 m/s. |
| attitude | `act_base_wx/wy/wz` | Velocity actuators with the control pinned to `0` — pure dampers. Two different gains: roll/pitch `kv = 20000` (the only thing damping slide-induced tilt), yaw `kv = 2000` (soft, so `Ctrl + left drag` can spin the base — see below). |
| damping | `damping="20"` | Light residual damping on the six free-joint DOFs. It is *passive*, so it fights the mouse and the velocity command alike; it was cut from 400, which on its own cost roughly 100 mm of drag travel and 2 % of tracking accuracy. |

The attitude brakes are not optional. The sliding command is a generalized force
on the base frame, which sits 0.061 m above the floor while the robot's centre of
mass is ~0.4 m above it, so the mass matrix couples translation and pitch.
Measured without them: a 1.44 m slide tilts the chassis 4.3° and lifts it 53 mm,
and slamming both arms to 60 % of their range drifts it 208 mm vertically. With
them: 0.11° tilt, zero vertical drift, identical X/Y tracking. A brake on Z is
unnecessary: gravity plus the floor contact hold the height. The yaw brake was
later softened from 20000 to 2000 (it has no tilt duty — only the roll/pitch
brakes do), which is what makes `Ctrl + left drag` able to spin the robot.

### Dragging the base in the viewer

The viewer never moves a body directly while the simulation runs. `simulate`
attaches a spring-damper to the selected body and lets the mouse move the other
end of it: `m.vis.map.stiffness = 100` N/m per unit `localmass`, and clicking the
middle of the base box gives `localmass = 78 kg`, i.e. a 7.8 kN/m spring. The
mouse map matters as much as the gains:

| Gesture | Effect |
| --- | --- |
| `Ctrl` + right drag | translate the object in the **vertical** plane (includes Z) — drag *up* to lift it |
| `Ctrl` + `Shift` + right drag | translate the object in the **horizontal** plane |
| `Ctrl` + left drag, **horizontal** move | rotate the object **around the vertical axis** (yaw) |
| `Ctrl` + left drag, **vertical** move | rotate the object in the screen plane (pitch) |
| `Ctrl` + `Shift` + left drag | roll the object around the view axis (steering wheel) |
| while paused | the perturbation is applied directly, no spring, no force limit |

So `Ctrl` + right drag pushes the base straight into the floor (measured: 20 kN
along `(0, 0, -1)`) or lifts it up — it cannot slide the robot. Use
`Ctrl + Shift + right drag` to drive it across the floor, and `Ctrl + left
drag` with a horizontal mouse move to spin it in place (yaw).

Whether the mouse can actually *rotate* the robot is decided by the yaw brake:
the rotation spring tops out at ~1.1 kN*m of torque (stiffnessrot 500 x
localmass 78), and the brake is a rate damper, so the steady spin rate is
1101/kv. With the original kv=20000 that was 0.055 rad/s (~3 deg/s — looked
like "cannot rotate"); with kv=2000 the same gesture spins the base ~27 deg/s
and every stability metric (idle, drop, arm swing, tracking) is unchanged.
The roll/pitch brakes stay at kv=20000 on purpose: they are the only thing
damping slide-induced tilt, and they keep a mouse pitch-drag from flipping the
robot (a pitched drag then settles back to under 1 deg instead of staying 11
deg over).

The gains above were chosen by a drag probe script that reproduced that
pipeline (`mjv_select` → `mjv_initPerturb` → `mjv_movePerturb` →
`mjv_applyPerturbForce`); the probe now lives in the user-level skill
`mujoco-viewer-drag-tuning` (a project copy is archived under
`.workbuddy/artifacts/removed_scripts/`). Measured on the empty arena for a 3 s
horizontal drag: 0.54 m of travel with the original `kv = 20000` against 1.25 m
now, with unchanged velocity tracking (2.3 % position error over the 2 s move).

Those numbers are tied to this exact model revision: any change to the model
(even an actuator gain) shifts the absolute values (the same probe measured
1.47 m before the leg-gain retune and 1.25 m after it, same conclusions).
Re-run the probe before quoting numbers after any model change. The probe
writes its variants to a throw-away temp directory and never touches the
repository; its patch step is idempotent and asserts that every substitution
actually matched (a silently failed patch once produced a bogus "control"
group -- see `MUJOCO_CONVERSION_NOTES.md` section 4.10 for the full issue log).

No wheel joint is added: real omni wheels have 3-DOF contact (drive + lateral
slide + swivel) which a hinge cannot express, so the base slides instead — the
virtual driving motion. Use `tests/test_base_mobility.py` to check the height,
the real weight, the landing after a drop, the idle stability and the sliding
response.

## End-effector frames `l_ee` / `r_ee`

Each wrist carries a **virtual end-effector frame**: a pure coordinate frame at
the centre of the circular flange face of the wrist mesh (`Arm_X7_Link.STL`,
measured with trimesh: the flange is the extreme `y = -0.087` plane, radius
~31.5 mm, centred on the link's y axis, with a 5-hole bolt circle and a centre
bore).

MJCF has no "fixed" joint type, so the URDF-style fixed joint is a child body
**without a `<joint>`** — welded, zero DOF, zero mass, no contact. The frames
are defined in `generate_mjcf.py` (`EE_FRAMES`) as

```text
Arm_L7_Link ── <body name="l_ee" pos="0 -0.087 0">   (axes parallel to the wrist)
Arm_R7_Link ── <body name="r_ee" pos="0 -0.087 0">
```

Each frame contains a same-named `<site>` (a 3 mm red marker dot, no physics)
so the otherwise invisible frame can be seen, selected and referenced in the
viewer.

## SG100 hands mounted on `l_ee` / `r_ee`

The two SG100 hands (`SG100_URDF_260811_align_real/`) are welded onto the EE
frames: each `base_link` hangs directly under `l_ee` / `r_ee` as a jointless
body (`L_hand_base` / `R_hand_base`, no DOF). The mount rotation is
**+90 deg about X** — so the hand +Z (its extension axis, mount face at
z = 0) points along the flange outward normal (wrist -Y) — followed by a
**palm-down roll about that same axis**: the SG100 fingers root on the -Y
side of `base_link` and curl toward -Y, i.e. the palm normal is the hand
frame's -Y, and the roll turns that palm to face **down in the world** at
the arm zero pose. The hands are mirror pairs, so the roll signs are
opposite: L rolls **-90 deg**, R rolls **+90 deg** about the hand +Z
(mount quats wxyz: L `0.5 0.5 0.5 -0.5`, R `0.5 0.5 -0.5 0.5`).
All hand objects carry an `L_` / `R_`
prefix because both URDFs reuse the same link and mesh names.

Drive (same actuator type as the arm joints): every one of the 22 finger
joints (11 per hand: thumb 3, index 3, middle 2, little 3) is a `<position>`
actuator `act_<l/r>_<joint>` with a **dedicated finger PD table**
(`HAND_CONTROL_PARAMS` in `generate_mjcf.py`) — proximal (300, 2.5) >
middle (250, 2.0) > distal (150, 1.2), plus finger-specific
`armature = 0.002` / `damping = 0.05` (the arm's 0.05 / 2.5 would swamp a
~20 g finger link). Raise `kp` per joint for a stronger grip; keep
`omega*dt < 0.5` for stability at the 1000 Hz step.

Actuator order: the hands are appended **after** the arm tree, so the arm
actuator indices (and the scene's `test_position_actuators` numerics) are
unchanged. `scripts/render_check_hands.py` renders the mounted hands for a
visual check (mind the 640x480 offscreen framebuffer limit). See
`MUJOCO_CONVERSION_NOTES.md` sections 4.11 / 4.12 for the full derivation.

## Joint targets and actuator gains

The actuator-to-joint table is in
`config/joint_actuator_config.json`. To command a position in Python:

```python
joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Arm_L1_Joint")
actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                 "act_arm_l1_joint")
data.ctrl[actuator_id] = 0.25

# finger example: close the left index proximal joint (act_l_index_j1)
act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_l_index_j1")
data.ctrl[act_id] = 0.5   # rad, clamped to the URDF range -1.57..0
```

Each joint uses a MuJoCo `<position>` actuator. The `kp` and `kv` values are
written into `luna_m6_robot.xml` per joint using the effective zero-pose
articulated inertia. This avoids using one gain pair for both the heavy leg
joints and lightweight wrist joints. The current model uses
`armature="0.05"`, `damping="2.5"`, and a 1000 Hz simulation step (the SG100
finger joints use their own smaller armature/damping and PD gains, see the
previous section).

Use `tests/test_position_control.py` after changing gains. It checks several
representative target steps for convergence and low final velocity.

## Camera, lights, and floor

The viewer starts with a free camera aimed at the chest from approximately
45 degrees above the robot. Mouse orbit, pan, and zoom remain enabled. The
scene uses the Unitree G1 `scene_23dof.xml` skybox, checker floor, headlight,
haze, and overhead directional light parameters.

The checker floor is 20 m by 20 m, covering 400 m². MuJoCo plane sizes are
half-extents, so the XML uses `size="10 10 0.1"`.

## Resource-path validation

Run `check_resource_paths.py` after edits. It scans every XML file and checks
that resources are relative, resolve inside `mjcf`, and do not contain
`package://`, absolute Windows paths, or references to `Gento Luna_URDF`.

## Workpieces in the storage cages

`luna_m6_props.xml` places 10 copies of the cast-part workpiece (s01, decimated
to 12,246 triangles) inside the two storage cages, 5 per cage. The bodies are
`Workpiece_B_1..5` / `Workpiece_I_1..5`, each with a free joint, a normalized
mesh asset (`assets/meshes/Workpiece_S01_162x162x63.STL`, origin at the bottom
center so body z equals the resting height), an equal-size cylinder collision
geom, and a mass of 1.43 kg (aluminum estimate; edit the `<inertial>` line in
`luna_m6_props.xml` for a different material).

The random layout is generated by `mjcf/scripts/generate_workpieces.py`
(`--seed N` for a different arrangement). The script rewrites the marked
`WORKPIECES BEGIN/END` block inside `luna_m6_props.xml` and enforces: parts lie
flat on the cage floor plate, stay clear of the corner posts and walls, and do
not overlap each other (OBB separation check).

### Collision alignment and drag stability (fixed 2026-09-21)

Dragging a workpiece with the viewer mouse used to make it oscillate forever and
clip through the cage. Root causes, both measured on the `StorageCage` mesh:

1. **Collision walls were 20 mm outside the visual mesh.** The cage STL walls
   (solid skirt + wire mesh) sit at |x|,|y| = 0.3455..0.3505 and the corner posts
   span 0.350..0.400. The old collision walls sat at ±0.372 (inner face 0.366),
   so a part pushed against them visually sank ~20 mm into the wire mesh.
   The walls now sit at ±0.371 with half-thickness 0.029: inner face 0.342
   (3.5 mm inside the visual surface), outer face 0.400 flush with the posts.
   The 58 mm effective thickness (was 12 mm) also stops a dragged part from
   tunnelling straight through, which was the main source of the endless
   jitter once a part got stuck half-inside a thin wall.

2. **Torsional friction was silently disabled.** The workpiece geoms used
   `condim=3` (MuJoCo default), which ignores the 2nd/3rd `friction` numbers —
   a kicked disc kept spinning. They now use `condim="6"`
   `friction="0.9 0.02 0.005"`, so torsional + rolling friction act and the
   part comes to rest. `margin="0.0003"` pre-activates contacts slightly to
   avoid contact popping; the side effect is that a part floats `margin` above
   the floor (0.27 mm, invisible).

Verified with the real viewer perturbation model (`kp = 100 * mass`, critically
damped, i.e. k ~ 143 N/m for these parts): pressing a part against a wall now
gives ~0 mm visual penetration (was up to 10 mm, i.e. through the old 12 mm
wall), and the whole scene returns to rest with residual jitter < 0.06 mm/s.
Note that an extreme fling (drag target thrown far outside the cage) can still
launch a part over the 0.45 m wall — that is inherent to the viewer's
perturbation spring, not a collision problem; lower `timestep` in
`luna_m6_scene.xml` if transient overlap during such flings matters.

## Slot pallet next to the storage cages

`luna_m6_props.xml` adds a `Pallet_S02` body (the s02 slot pallet,
decimated to 9,392 triangles, max deviation 0.86 mm). The mesh asset
`assets/meshes/s02_pallet_1000x500x185.STL` is normalized (origin at the
bottom center, 1000 x 500 x 185 mm) and — note — uses an ASCII filename
because MuJoCo's `fopen` on Windows cannot open non-ASCII paths.

Placement: horizontally on the ground (`pos="-3.0 1.2 0"` — the ground plane is
at z = 0), long edge parallel to the cage row, 0.1 m clear of cage B's right
edge, rendered in the same blue (`rgba="0.07 0.16 0.75 1"`) as the storage cages.

Collision follows the measured STL structure: one deck box (z 0.09..0.185,
full 1.0 x 0.5) plus six foot boxes (z 0..0.095 at x = 0 / ±0.4725,
y = ±0.2275, feet bottoms flush with the visual mesh), so the space under the
deck between the feet stays traversable. Mass is an explicit 14.5 kg (envelope
volume 0.0152 m3 x 950 kg/m3, plastic pallet estimate; for a steel pallet set
mass ~110 and scale diaginertia x7.6).

The pallet and all workpieces are **movable**: each has a `<freejoint/>`, so
the viewer mouse can drag, rotate and lift them (`Ctrl + right` drag up lifts
the pallet, release drops it back — measured 2.8 m lift, 0 mm residual after
settling) and the robot can push them around. The **storage cages and the
conveyor line are fixed installations by hard requirement** — they have no
joint, so no perturbation can move them (regression-checked at full mouse
force: 0.000 mm / 0.000 deg). When a body gains its freejoint its collision
floor must already sit at z = 0 — an overhanging collision box makes the body
sink by the overhang (the pallet's feet used to start at z = 0.01, which
would have sunk it 10 mm through its visual mesh), so the pallet's foot boxes
were extended down to the visual mesh bottom with their tops unchanged. The
cages keep their original floating floor (z = 0.035..0.085) precisely because
they stay static. `tests/check_props_movable.py` asserts both directions.

