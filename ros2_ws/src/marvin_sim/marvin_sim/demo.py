# 命令行仿真示例：连接 SDK 后展示状态读取及运动控制流程，参数由命令行提供。
"""Headless end-to-end demo: real SDK FK -> IK -> UDP motion -> feedback."""
import argparse
import json
import time

import numpy as np

from .client import SDKClient
from .kinematics import Kinematics, pose_error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk-root', required=True)
    parser.add_argument('--description-root')
    parser.add_argument('--sim-ip', default='')
    args = parser.parse_args()
    if not args.description_root:
        from ament_index_python.packages import get_package_share_directory
        args.description_root = get_package_share_directory('marvin_description')
    kine = Kinematics(args.sdk_root, args.description_root)
    client = SDKClient(kine, sim_ip=args.sim_ip)
    try:
        current = client.read()
        targets, errors = {}, []
        for arm in range(2):
            reference = current[7*arm:7*arm+7]
            requested = reference.copy()
            requested[0] += np.radians(5 if arm == 0 else -5)
            pose = kine.fk(arm, requested)
            targets[arm] = kine.ik(arm, pose, reference)
            errors.append(pose_error(kine.fk(arm, targets[arm]), pose))
        client.move(targets)
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            feedback = client.read()
            error = max(float(np.max(np.abs(feedback[7*a:7*a+7]-q))) for a, q in targets.items())
            if error < 1e-5:
                print(json.dumps({'result': 'PASS', 'sdk_fk_ik_errors_m_rad': errors,
                                  'sdk_feedback_max_joint_error_rad': error,
                                  'sdk_controller_ip': client.address}, indent=2), flush=True)
                return
            time.sleep(.02)
        raise RuntimeError('SDK feedback did not reach the FK/IK targets within 10 seconds')
    finally:
        client.close()
