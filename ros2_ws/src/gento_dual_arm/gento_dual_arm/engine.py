# 串行执行 SDK 连接、目标下发及反馈读取，管理双臂控制状态；调用应由同一执行器串行调度。
"""Single-thread SDK state machine. All calls must be serialized by the ROS executor."""
import math
import time


class Engine:
    def __init__(self, robot, helpers, ik_type, config):
        self.robot, self.h, self.ik_type, self.c = robot, helpers, ik_type, config
        self.active = False
        self.faulted = False
        self.targets = None
        self.settled = None
        self.deadline = 0

    def connect(self):
        ret = self.robot.link(*map(int, self.c.ip.split('.')))
        if ret < 0:
            raise RuntimeError(f'Link failed: {ret}')
        for arm in (0, 1):
            self.h.check(self.robot.init_single_arm_config(arm), f'Initialize arm{arm}')
        self.limits = [self.h.read_limits(self.robot, a) for a in (0, 1)]

    def sample(self):
        if self.robot.check_link_state() != 1:
            raise RuntimeError('Controller disconnected or feedback interrupted')
        data = self.robot.get_rt_dict()
        joints = [list(data['arms'][a]['fb']['fb_pos']) for a in (0, 1)]
        for q in joints:
            if len(q) != 7:
                raise ValueError('Invalid feedback length')
            self.h.finite(q)
        matrices = [self.h.fk(self.robot, a, joints[a]) for a in (0, 1)]
        return joints, matrices

    def solve(self, matrices, starts):
        targets = []
        for arm in (0, 1):
            p = self.ik_type()
            for i in range(7): p.m_Input_IK_RefJoint[i] = starts[arm][i]
            for i in range(16): p.m_Input_IK_TargetTCP[i] = matrices[arm][i//4][i%4]
            result = self.robot.inverse_kinematics(arm, p)
            if not isinstance(result, self.ik_type):
                raise ValueError(f'arm{arm} IK failed: {result}')
            if result.m_Output_IsJntExd or any(result.m_Output_JntExdTags):
                raise ValueError(f'arm{arm} IK exceeds joint limits')
            q = list(result.m_Output_RetJoint)
            actual = self.h.fk(self.robot, arm, q)
            error_m = math.sqrt(sum((actual[i][3]-matrices[arm][i][3])**2 for i in range(3))) * self.c.sdk_length_to_m
            rot_error = max(abs(actual[i][j]-matrices[arm][i][j]) for i in range(3) for j in range(3))
            if error_m > 0.0001 or rot_error > 0.001:
                raise ValueError(f'arm{arm} IK residual too large: {error_m} m / {rot_error}')
            targets.append(q)
        return targets

    def start(self, targets=None, matrices=None):
        if self.active or self.faulted:
            raise RuntimeError('Busy or faulted; command rejected')
        starts, _ = self.sample()
        if matrices is not None:
            targets = self.solve(matrices, starts)
        plans = [self.h.prepare_plan(self.robot, a, starts[a], targets[a], self.limits[a], self.c) for a in (0, 1)]
        preview = {'joints_deg': targets, 'points': [p[1] for p in plans]}
        if not self.c.execute:
            return 'preview', preview
        for arm in (0, 1):
            if self.robot.current_state(arm) != 0:
                raise ValueError(f'arm{arm} must be IDLE; no automatic fault reset')
        self.active = True
        try:
            self.check_start(starts, 0)
            for a in (0, 1):
                self.h.check(self.robot.switch_to_position_mode(a, 3000, self.c.speed, self.c.acc), f'arm{a} POSITION')
            for a in (0, 1):
                self.h.check(self.robot.config_set_traj(a, *plans[a]), f'arm{a} upload')
            self.check_start(starts, 1)
            ret = self.robot.runtime_run_traj(3)
            if ret != 3:
                raise RuntimeError(f'Partial start: mask={ret}')
            self.targets = targets
            self.deadline = time.monotonic() + self.c.timeout
            self.settled = None
        except BaseException:
            self.stop()
            raise
        return 'running', preview

    def check_start(self, starts, state):
        latest, _ = self.sample()
        for a in (0, 1):
            if self.robot.current_state(a) != state or max(abs(x-y) for x,y in zip(latest[a], starts[a])) > 0.2:
                raise RuntimeError(f'arm{a} state/start changed during preparation')

    def poll(self, joints):
        if not self.active:
            return False
        if time.monotonic() >= self.deadline:
            raise TimeoutError('Motion timeout')
        if any(self.robot.current_state(a) != 1 for a in (0, 1)):
            raise RuntimeError('Arm left POSITION during execution')
        sg = self.robot.get_sg_dict()
        done = all(sg['arms'][a]['get']['traj_state'] == 0 and
                   max(abs(x-y) for x,y in zip(joints[a], self.targets[a])) <= self.c.tolerance for a in (0, 1))
        if not done:
            self.settled = None
            return False
        now = time.monotonic()
        self.settled = now if self.settled is None else self.settled
        if now-self.settled < 0.5:
            return False
        for a in (0, 1): self.h.check(self.robot.switch_to_idle(a, 3000), f'arm{a} IDLE')
        self.active = False
        return True

    def stop(self, force=False):
        self.faulted = True
        try:
            if self.active or force:
                ret = self.robot.emergency_stop(3)
                if ret != 3:
                    raise RuntimeError(f'Emergency stop unconfirmed: mask={ret}; use hardware stop')
        finally:
            self.active = False

    def close(self):
        try:
            if self.active: self.stop()
        finally:
            try: self.robot.cleanup()
            finally: self.robot.unlink()
