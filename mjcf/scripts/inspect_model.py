# 加载 MuJoCo 模型并输出模型结构，辅助核对关节、刚体及资源是否成功导入。
from __future__ import annotations

import sys
from pathlib import Path

import mujoco


MJCF_ROOT = Path(__file__).resolve().parents[1]
SCENE = MJCF_ROOT / "luna_m6_scene.xml"


def names(model: mujoco.MjModel, obj_type: mujoco.mjtObj) -> list[str]:
    count = mujoco.mj_name2id(model, obj_type, "")
    del count
    size = {
        mujoco.mjtObj.mjOBJ_BODY: model.nbody,
        mujoco.mjtObj.mjOBJ_JOINT: model.njnt,
        mujoco.mjtObj.mjOBJ_ACTUATOR: model.nu,
        mujoco.mjtObj.mjOBJ_GEOM: model.ngeom,
        mujoco.mjtObj.mjOBJ_MESH: model.nmesh,
        mujoco.mjtObj.mjOBJ_CAMERA: model.ncam,
        mujoco.mjtObj.mjOBJ_LIGHT: model.nlight,
    }[obj_type]
    return [mujoco.mj_id2name(model, obj_type, index) or "<unnamed>" for index in range(size)]


def main() -> int:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    print(f"model: {SCENE}")
    for label, obj_type in (
        ("bodies", mujoco.mjtObj.mjOBJ_BODY),
        ("joints", mujoco.mjtObj.mjOBJ_JOINT),
        ("actuators", mujoco.mjtObj.mjOBJ_ACTUATOR),
        ("geoms", mujoco.mjtObj.mjOBJ_GEOM),
        ("meshes", mujoco.mjtObj.mjOBJ_MESH),
        ("cameras", mujoco.mjtObj.mjOBJ_CAMERA),
        ("lights", mujoco.mjtObj.mjOBJ_LIGHT),
    ):
        print(f"\n{label}:")
        for item in names(model, obj_type):
            print(f"  {item}")

    root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Underpan_Base_link")
    free_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free_joint")
    print("\nUnderpan_Base_link:")
    print(f"  body id: {root_id}")
    print(f"  local pos: {model.body_pos[root_id].tolist()}")
    print(f"  local quat: {model.body_quat[root_id].tolist()}")
    print("\nbase_free_joint:")
    print(f"  joint id: {free_id}")
    print(f"  type: {mujoco.mjtJoint(model.jnt_type[free_id]).name}")
    print(f"  qpos address: {model.jnt_qposadr[free_id]}")
    print(f"  actuator ids: {[i for i in range(model.nu) if model.actuator_trnid[i, 0] == free_id]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
