"""Dual-arm joint trajectories using one SDK connection and a combined start mask."""
import argparse
import ipaddress
import sys
import time

from arm_control import GentoRobot, FXStateType, check, feedback, finite, fk, validate_target

BOTH_ARMS = 3


def read_limits(robot, arm):
    limits = []
    for joint in range(7):
        pair = []
        for suffix in ('LimitNeg', 'LimitPos'):
            ret, value = robot.param_get_float(f'R.A{arm}.L{joint}.BASIC.{suffix}')
            check(ret, f'读取 arm{arm} joint{joint + 1} {suffix}')
            pair.append(value)
        limits.append(pair)
    return limits


def prepare_plan(robot, arm, start, target, limits, args):
    validate_target(start, target, limits, args.max_delta)
    # The native planner uses ratios in (0, 1], unlike position-mode percentages.
    plan = robot.plan_joints(arm, start, target, args.plan_speed, args.plan_acc, 50)
    if not isinstance(plan, tuple) or len(plan) != 2:
        raise RuntimeError(f'arm{arm} 轨迹规划失败：{plan}')
    points, count = plan
    if not 1 <= count <= robot.max_point or len(points) < count * 7:
        raise RuntimeError(f'arm{arm} 轨迹点数无效：{count}')
    for i in range(count):
        validate_target(start, list(points[i * 7:(i + 1) * 7]), limits, args.max_delta)
    # Do not execute a trajectory whose endpoint does not match the requested goal.
    end = list(points[(count - 1) * 7:count * 7])
    if max(abs(a - b) for a, b in zip(end, target)) > 0.05:
        raise RuntimeError(f'arm{arm} 轨迹终点与目标不符')
    return points, count


def run(args, robot):
    armed = False
    try:
        ret = robot.link(*map(int, args.ip.split('.')))
        if ret < 0:
            raise RuntimeError(f'连接失败：{ret}')
        starts, targets, plans = [], [], []
        for arm in (0, 1):
            check(robot.init_single_arm_config(arm), f'初始化 arm{arm}')
            starts.append(feedback(robot, arm))
            print(f'arm{arm} 当前关节角（度）：{starts[arm]}')
            print(f'arm{arm} 当前 TCP：{robot.matrix2xyzabc(fk(robot, arm, starts[arm]))}')
        if args.joints is None and args.delta is None:
            return
        for arm in (0, 1):
            values = (args.joints if args.joints is not None else args.delta)[arm * 7:(arm + 1) * 7]
            target = values if args.joints is not None else [q + d for q, d in zip(starts[arm], values)]
            targets.append(target)
            plan = prepare_plan(robot, arm, starts[arm], target, read_limits(robot, arm), args)
            plans.append(plan)
            print(f'arm{arm} 目标（度）：{target}；轨迹点数：{plan[1]}')
        print('两臂分别规划、通过同一个双臂掩码启动；不保证同时到达。未进行碰撞检查。')
        if not args.execute:
            print('仅预览；未切换模式、上传或执行轨迹。')
            return

        for arm in (0, 1):
            if robot.current_state(arm) != FXStateType.FX_STATE_IDLE:
                raise RuntimeError(f'arm{arm} 必须处于 IDLE；不自动复位或接管其他模式')
            if max(abs(a - b) for a, b in zip(feedback(robot, arm), starts[arm])) > 0.2:
                raise RuntimeError(f'arm{arm} 起点变化，请重新规划')
        armed = True
        for arm in (0, 1):
            check(robot.switch_to_position_mode(arm, 3000, args.speed, args.acc), f'arm{arm} 切换 POSITION')
        for arm in (0, 1):
            check(robot.config_set_traj(arm, *plans[arm]), f'arm{arm} 上传轨迹')
        # Uploads can block; recheck both arms before starting either trajectory.
        for arm in (0, 1):
            if robot.current_state(arm) != FXStateType.FX_STATE_POSITION:
                raise RuntimeError(f'arm{arm} 状态变化')
            if max(abs(a - b) for a, b in zip(feedback(robot, arm), starts[arm])) > 0.2:
                raise RuntimeError(f'arm{arm} 上传期间起点变化')
        ret = robot.runtime_run_traj(BOTH_ARMS)
        if ret != BOTH_ARMS:
            raise RuntimeError(f'双臂启动未全部成功：返回掩码 {ret}')
        print('双臂执行中；Ctrl+C 请求停止双臂。')
        deadline = time.monotonic() + args.timeout
        settled = None
        while time.monotonic() < deadline:
            reached = True
            for arm in (0, 1):
                q = feedback(robot, arm)
                if robot.current_state(arm) != FXStateType.FX_STATE_POSITION:
                    raise RuntimeError(f'arm{arm} 运动状态异常')
                reached = reached and max(abs(a - b) for a, b in zip(q, targets[arm])) <= args.tolerance
            sg = robot.get_sg_dict()
            completed = all(sg['arms'][arm]['get']['traj_state'] == 0 for arm in (0, 1))
            if reached and completed:
                settled = time.monotonic() if settled is None else settled
                if time.monotonic() - settled >= 0.5:
                    break
            else:
                settled = None
            time.sleep(0.02)
        else:
            raise TimeoutError('双臂等待到位超时')
        for arm in (0, 1):
            check(robot.switch_to_idle(arm, 3000), f'arm{arm} 返回 IDLE')
        armed = False
        print('双臂到位，均已切换 IDLE。')
    finally:
        if armed:
            # Also covers partial start, upload failure and partial mode transition.
            try:
                ret = robot.emergency_stop(BOTH_ARMS)
                if ret != BOTH_ARMS:
                    print(f'双臂急停未全部确认：{ret}；请使用硬件急停！', file=sys.stderr)
            except Exception as exc:
                print(f'双臂软件急停失败：{exc}；请使用硬件急停！', file=sys.stderr)
        try:
            robot.cleanup()
        finally:
            robot.unlink()


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ip', required=True)
    g = p.add_mutually_exclusive_group()
    g.add_argument('--joints', type=float, nargs=14, help='左臂 7 个角度，随后右臂 7 个角度，单位度')
    g.add_argument('--delta', type=float, nargs=14, help='双臂相对增量，左 7 个随后右 7 个，单位度')
    p.add_argument('--execute', action='store_true')
    p.add_argument('--speed', type=float, default=5, help='控制器速度比例百分数，上限 10')
    p.add_argument('--acc', type=float, default=5, help='控制器加速度比例百分数，上限 10')
    p.add_argument('--plan-speed', type=float, default=0.05, help='规划速度比例，0~0.1')
    p.add_argument('--plan-acc', type=float, default=0.05, help='规划加速度比例，0~0.1')
    p.add_argument('--max-delta', type=float, default=5)
    p.add_argument('--timeout', type=float, default=60)
    p.add_argument('--tolerance', type=float, default=0.2)
    a = p.parse_args(argv)
    try:
        a.ip = str(ipaddress.IPv4Address(a.ip))
        values = [a.speed, a.acc, a.plan_speed, a.plan_acc, a.max_delta, a.timeout, a.tolerance]
        finite(values + (a.joints or a.delta or []))
        if any(x <= 0 for x in values) or max(a.speed, a.acc) > 10 or max(a.plan_speed, a.plan_acc) > 0.1:
            raise ValueError('参数必须为正数；控制比例不超过 10%，规划比例不超过 0.1')
        if a.execute and a.joints is None and a.delta is None:
            raise ValueError('--execute 必须指定双臂目标')
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
