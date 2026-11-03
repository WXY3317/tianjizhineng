# 加载 Luna MuJoCo 模型并执行模型检查，辅助发现生成和资源配置问题。
from __future__ import annotations

import json
import sys
from pathlib import Path

import mujoco
import numpy as np


MJCF_ROOT = Path(__file__).resolve().parents[1]
ROBOT = MJCF_ROOT / "luna_m6_robot.xml"
SCENE = MJCF_ROOT / "luna_m6_scene.xml"
CONFIG = MJCF_ROOT / "config" / "joint_actuator_config.json"


def main() -> int:
    config = json.loads(CONFIG.read_text(encoding="utf-8"))

    # --- the generated robot asset on its own ---------------------------------
    robot = mujoco.MjModel.from_xml_path(str(ROBOT))
    root_id = mujoco.mj_name2id(robot, mujoco.mjtObj.mjOBJ_BODY, "Underpan_Base_link")
    assert root_id >= 0, "Underpan_Base_link is missing"
    base_height = float(robot.body_pos[root_id][2])
    assert base_height > 0.0, (
        f"the base frame must sit at wheel-contact height, got {robot.body_pos[root_id]}"
    )

    free_id = mujoco.mj_name2id(robot, mujoco.mjtObj.mjOBJ_JOINT, "base_free_joint")
    assert free_id >= 0, "base_free_joint is missing"
    assert robot.jnt_type[free_id] == mujoco.mjtJoint.mjJNT_FREE

    joint_names = {
        mujoco.mj_id2name(robot, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(robot.njnt)
    }
    assert not any("wheel" in (name or "").lower() for name in joint_names)
    assert len(config["actuated_joints"]) == 22
    assert len(config["base_actuators"]) == 5
    assert robot.nu == 22 + 5
    assert robot.nmesh == 23

    actuator_joints = {
        mujoco.mj_id2name(robot, mujoco.mjtObj.mjOBJ_JOINT, int(robot.actuator_trnid[i, 0]))
        for i in range(robot.nu)
    }
    expected_joints = {row["joint_name"] for row in config["actuated_joints"]} | {"base_free_joint"}
    assert actuator_joints == expected_joints

    # --- the full scene: the floating base must hold its pose -----------------
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    start = data.qpos[:3].copy()
    settle_steps = 1000
    for _ in range(settle_steps):
        mujoco.mj_step(model, data)
    # The base starts one clearance above the floor and sits down onto it, so the
    # drift is measured from the seated pose, not from the model default.
    seated = data.qpos[:3].copy()
    for _ in range(2500 - settle_steps):
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    seat = float(start[2] - seated[2])
    assert seat < 3e-3, f"the base sank {seat * 1000:.2f} mm onto the floor"
    drift = float(np.linalg.norm(data.qpos[:3] - seated))
    assert drift < 1e-6, f"the floating base drifted {drift * 1000:.4f} mm while idle"

    print("PASS: MuJoCo loaded the robot and stepped the scene for 5 seconds.")
    print(f"  robot: bodies={robot.nbody} joints={robot.njnt} actuators={robot.nu} meshes={robot.nmesh}")
    print(f"  scene: bodies={model.nbody} joints={model.njnt} actuators={model.nu} textures={model.ntex}")
    print(f"  base frame z={base_height:.6f} m, root_z after 5 s={data.qpos[2]:.6f} m")
    print("  no wheel hinge was added; the base slides through act_base_vx / act_base_vy")
    return 0


if __name__ == "__main__":
    sys.exit(main())
