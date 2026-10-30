"""Offline torso FK/IK: load six signed limit pairs exported from the controller.

ros2 run gento_driver body_ik_example --limits body_limits.json \
    --reference 0 10 -20 10 0 0 --offset 10 0 0 --position-only

This example never loads the native SDK library or connects to a controller.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
from gento_driver.body_kinematics import LunaBodyKinematics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--limits", required=True, type=Path,
                        help="JSON file: six [LimitNeg, LimitPos] pairs in degrees")
    parser.add_argument("--reference", required=True, nargs=6, type=float,
                        help="Current/reference torso joint angles in degrees")
    parser.add_argument("--offset", nargs=3, type=float, default=[10, 0, 0],
                        help="Target XYZ offset in the fixed torso base axes, mm")
    parser.add_argument("--position-only", action="store_true",
                        help="Leave end orientation free to reduce joint movement")
    parser.add_argument("--max-joint-delta", type=float,
                        help="Optional maximum per-joint change in degrees")
    args = parser.parse_args()
    model = LunaBodyKinematics(json.loads(args.limits.read_text()))
    target = model.forward(args.reference)
    for i, offset in enumerate(args.offset):
        target[i][3] += offset
    result = model.inverse(target, args.reference, position_only=args.position_only,
                           max_joint_delta_deg=args.max_joint_delta)
    print(json.dumps({"effective_limits_deg": model.joint_limits_deg,
                      "target_pose_in_body_base_mm": target,
                      "result": asdict(result)}, indent=2, ensure_ascii=False))
    return 0 if result.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
