# 实体单臂 SDK 控制脚本；默认连接后只读预览，执行运动受命令行选项及目标校验约束。
"""Single-arm position control; default is connected, read-only preview."""
import argparse
import ipaddress
import math
import sys
import time
from pathlib import Path

from .sdk_loader import load_sdk

_sdk = load_sdk()
GentoRobot = _sdk.GentoRobot
FX_InvKineSolverParams = _sdk.FX_InvKineSolverParams
FXStateType = _sdk.FXStateType


def check(ret, operation):
    if ret != 0:
        raise RuntimeError(f'{operation}失败，SDK 返回值：{ret}')


def finite(values):
    if not all(math.isfinite(x) for x in values):
        raise ValueError('参数包含 NaN 或无穷大')


def feedback(robot, arm):
    if robot.check_link_state() != 1:
        raise RuntimeError('控制器连接丢失或反馈中断')
    q = list(robot.get_rt_dict()['arms'][arm]['fb']['fb_pos'])
    if len(q) != 7:
        raise RuntimeError('手臂反馈不是 7 个关节')
    finite(q)
    return q


def fk(robot, arm, q):
    matrix = robot.forward_kinematics(arm, q)
    if not isinstance(matrix, list):
        raise RuntimeError(f'正解失败：{matrix}')
    finite([v for row in matrix for v in row])
    return matrix


def validate_target(current, target, limits, max_delta):
    if len(target) != 7:
        raise ValueError('目标必须包含 7 个关节角')
    finite(target)
    for i, (q, goal, (lo, hi)) in enumerate(zip(current, target, limits)):
        finite([lo, hi])
        if lo >= hi or not lo <= goal <= hi:
            raise ValueError(f'关节 {i + 1} 目标 {goal} 超出有效限位 [{lo}, {hi}]')
        if abs(goal - q) > max_delta:
            raise ValueError(f'关节 {i + 1} 位移超过单次上限 {max_delta} 度')


def run(args, robot):
    active = False
    try:
        ret = robot.link(*map(int, str(ipaddress.IPv4Address(args.ip)).split('.')))
        if ret < 0:
            raise RuntimeError(f'连接失败：{ret}')
        check(robot.init_single_arm_config(args.arm), '初始化运动学')
        current = feedback(robot, args.arm)
        current_pose = fk(robot, args.arm, current)
        print('当前关节角（度）：', current)
        print('当前 TCP XYZABC：', robot.matrix2xyzabc(current_pose))
        if args.joints is not None:
            target = args.joints
        elif args.delta is not None:
            target = [q + d for q, d in zip(current, args.delta)]
        elif args.tcp is not None:
            target_pose = robot.xyzabc2matrix(args.tcp)
            params = FX_InvKineSolverParams()
            for i in range(7):
                params.m_Input_IK_RefJoint[i] = current[i]
            for i in range(16):
                params.m_Input_IK_TargetTCP[i] = target_pose[i // 4][i % 4]
            result = robot.inverse_kinematics(args.arm, params)
            if not isinstance(result, FX_InvKineSolverParams):
                raise RuntimeError(f'逆解失败：{result}')
            if result.m_Output_IsJntExd or any(result.m_Output_JntExdTags):
                raise ValueError('逆解超出关节限位')
            target = list(result.m_Output_RetJoint)
            actual = fk(robot, args.arm, target)
            # Position uses the model length unit; rotation is dimensionless.
            pos_err = math.sqrt(sum((actual[i][3] - target_pose[i][3]) ** 2 for i in range(3)))
            rot_err = max(abs(actual[i][j] - target_pose[i][j]) for i in range(3) for j in range(3))
            if pos_err > 0.1 or rot_err > 1e-3:
                raise ValueError(f'IK 回代误差过大：位置={pos_err}，旋转矩阵={rot_err}')
        else:
            print('未提供目标，仅显示当前位姿。')
            return

        limits = []
        for i in range(7):
            pair = []
            for name in ('LimitNeg', 'LimitPos'):
                ret, value = robot.param_get_float(f'R.A{args.arm}.L{i}.BASIC.{name}')
                check(ret, f'读取关节 {i + 1} {name}')
                pair.append(value)
            limits.append(pair)
        validate_target(current, target, limits, args.max_delta)
        print('目标关节角（度）：', target)
        print('目标 TCP XYZABC：', robot.matrix2xyzabc(fk(robot, args.arm, target)))
        if not args.execute:
            print('预览完成，未切换模式或发送运动指令。添加 --execute 执行。')
            return

        # Refuse to take over an arm already running another mode, or reset errors.
        if robot.current_state(args.arm) != FXStateType.FX_STATE_IDLE:
            raise RuntimeError('执行前手臂必须为 IDLE；请先由操作员处理当前模式或故障')
        latest = feedback(robot, args.arm)
        if max(abs(a - b) for a, b in zip(latest, current)) > 0.5:
            raise RuntimeError('准备期间手臂位置发生变化，请重新预览目标')
        validate_target(latest, target, limits, args.max_delta)
        active = True  # A failed/interruptible transition may still have taken effect.
        check(robot.switch_to_position_mode(args.arm, 3000, args.speed, args.acc), '切换位置模式')
        check(robot.runtime_set_joint_pos_cmd(args.arm, target), '发送目标关节角')
        print('运动中，Ctrl+C 请求软件急停；硬件急停应随时可用。')
        deadline = time.monotonic() + args.timeout
        settled = None
        while time.monotonic() < deadline:
            q = feedback(robot, args.arm)
            if robot.current_state(args.arm) != FXStateType.FX_STATE_POSITION:
                raise RuntimeError('运动期间手臂离开 POSITION 状态')
            if max(abs(a - b) for a, b in zip(q, target)) <= args.tolerance:
                settled = settled if settled is not None else time.monotonic()
                if time.monotonic() - settled >= 0.5:
                    break
            else:
                settled = None
            time.sleep(0.02)
        else:
            raise TimeoutError('等待到位超时')
        check(robot.switch_to_idle(args.arm, 3000), '到位后切换 IDLE')
        active = False
        print('到位完成，手臂已切换 IDLE。最终反馈：', q)
    finally:
        if active:
            try:
                mask = 1 << args.arm
                ret = robot.emergency_stop(mask)
                if ret != mask:
                    print(f'软件急停未确认成功（{ret}），请使用硬件急停！', file=sys.stderr)
            except Exception as exc:
                print(f'软件急停失败：{exc}；请使用硬件急停！', file=sys.stderr)
        try:
            robot.cleanup()  # SDK cleanup only destroys the kinematics handle.
        finally:
            robot.unlink()


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ip', required=True, help='实际控制器 IPv4 地址')
    p.add_argument('--arm', type=int, choices=(0, 1), default=0)
    group = p.add_mutually_exclusive_group()
    group.add_argument('--joints', nargs=7, type=float, help='绝对关节角，度')
    group.add_argument('--delta', nargs=7, type=float, help='相对当前关节角增量，度')
    group.add_argument('--tcp', nargs=6, type=float, help='绝对 TCP XYZABC；XYZ 使用模型长度单位，ABC 为度')
    p.add_argument('--execute', action='store_true', help='明确启用真机运动')
    p.add_argument('--speed', type=float, default=5, help='速度比例百分数，默认 5，上限 10')
    p.add_argument('--acc', type=float, default=5, help='加速度比例百分数，默认 5，上限 10')
    p.add_argument('--max-delta', type=float, default=5, help='单次各关节位移上限（度）')
    p.add_argument('--timeout', type=float, default=30)
    p.add_argument('--tolerance', type=float, default=0.2, help='关节到位容差（度）')
    a = p.parse_args(argv)
    try:
        ipaddress.IPv4Address(a.ip)
        values = [a.speed, a.acc, a.max_delta, a.timeout, a.tolerance]
        finite(values + (a.joints or a.delta or a.tcp or []))
        if any(x <= 0 for x in values) or a.speed > 10 or a.acc > 10:
            raise ValueError('参数必须为正数，speed/acc 不得超过 10')
        if a.execute and all(x is None for x in (a.joints, a.delta, a.tcp)):
            raise ValueError('--execute 必须指定目标')
    except ValueError as exc:
        p.error(str(exc))
    return a


def main():
    args = parse_args()
    try:
        run(args, GentoRobot())
        return 0
    except KeyboardInterrupt:
        print('操作中断。', file=sys.stderr)
        return 130
    except Exception as exc:
        print(f'失败：{exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
