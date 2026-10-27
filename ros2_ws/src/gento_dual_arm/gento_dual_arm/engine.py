# 串行执行 SDK 连接、目标下发及反馈读取，管理双臂控制状态；调用应由同一执行器串行调度。
"""Single-thread SDK state machine. All calls must be serialized by the ROS executor."""
import math
import time

BODY = 3
CONTROLLED_OBJECTS = (0, 1, BODY)
STOP_MASK = (1 << 0) | (1 << 1) | (1 << BODY)


class Engine:
    def __init__(self, robot, helpers, ik_type, config):
        self.robot, self.h, self.ik_type, self.c = robot, helpers, ik_type, config
        self.active = False
        self.faulted = False
        self.targets = None
        self.settled = None
        self.deadline = 0
        self.motion_group = 'arms'
        self.linked = False
        self.closed = False
        self.recovery_stage = None
        self.recovery_stop_requested = False

    def connect(self):
        self.closed = False
        ret = self.robot.link(*map(int, self.c.ip.split('.')))
        if ret < 0:
            raise RuntimeError(f'Link failed: {ret}')
        self.linked = True
        self.reset_errors()

    def reset_object_error(self, obj):
        """Mirror a vendor Reset button: reset one object, without enabling motion."""
        if obj not in CONTROLLED_OBJECTS:
            raise ValueError('Reset object must be left=0, right=1 or body=3')
        if self.active:
            raise ValueError('Motion active; stop before requesting error reset')
        self.faulted = True
        if not self.linked or self.robot.check_link_state() != 1:
            raise RuntimeError('Controller not connected; restart node to reconnect')
        before = self.robot.current_state(obj)
        ret, code = self.robot.reset_error(obj, 1000)
        after = self.robot.current_state(obj)
        if ret != 0 or after != 0:
            raise RuntimeError(f'object{obj} reset failed: ret={ret}, error={code}, '
                               f'state_before={before}, state_after={after}')
        return {'object_id': obj, 'return_code': ret, 'error_code': code,
                'state_before': before, 'state_after': after}

    def reset_errors(self):
        if self.active:
            raise ValueError('Motion active; stop before requesting error reset')
        self.faulted = True
        self.recovery_stop_requested = False
        self.recovery_stage = 'link_check'
        mode_transition_pending = False
        position_attempted = False
        if not self.linked or self.robot.check_link_state() != 1:
            raise RuntimeError('Controller not connected; restart node to reconnect')
        try:
            reset_failures = []
            for arm in CONTROLLED_OBJECTS:
                label = 'body' if arm == BODY else f'arm{arm}'
                before = self.robot.current_state(arm)
                # ResetError waits for IDLE; it does not switch a healthy
                # POSITION/PD/release object back to IDLE on every controller.
                if before not in (0, 100):  # IDLE / ERROR
                    self.recovery_stage = f'{label}:idle'
                    mode_transition_pending = True
                    self.h.check(self.robot.switch_to_idle(arm, 3000), f'{label} IDLE before reset')
                    if self.robot.current_state(arm) != 0:
                        raise RuntimeError(f'{label} did not reach IDLE before reset: state={self.robot.current_state(arm)}')
                mode_transition_pending = False
                if before == 100:  # Match the vendor example: reset only ERROR.
                    self.recovery_stage = f'{label}:reset'
                    try:
                        ret, error_code = self.robot.reset_error(arm, 3000)
                        after = self.robot.current_state(arm)
                        if ret != 0 or after != 0:
                            reset_failures.append(f'{label} reset failed: ret={ret}, error={error_code}, '
                                                  f'state_before={before}, state_after={after}')
                    except Exception as exc:
                        reset_failures.append(f'{label} reset failed: {exc}')
            if reset_failures:
                self.recovery_stage = 'reset_summary'
                raise RuntimeError('; '.join(reset_failures))
            for arm in CONTROLLED_OBJECTS:
                if self.robot.current_state(arm) != 0:
                    mode_transition_pending = True
                    self.h.check(self.robot.switch_to_idle(arm, 3000), f'arm{arm} IDLE')
                if self.robot.current_state(arm) != 0:
                    raise RuntimeError(f'arm{arm} did not reach IDLE after reset')
            mode_transition_pending = False
            self.recovery_stage = 'model_and_limits'
            for arm in (0, 1):
                self.h.check(self.robot.init_single_arm_config(arm), f'Initialize arm{arm}')
            self.limits = [self.h.read_limits(self.robot, a) for a in (0, 1)]
            self.body_limits = []
            for joint in range(6):
                pair = []
                for suffix in ('LimitNeg', 'LimitPos'):
                    ret, value = self.robot.param_get_float(f'R.B.L{joint}.BASIC.{suffix}')
                    self.h.check(ret, f'Body joint{joint + 1} {suffix}')
                    pair.append(value)
                self.h.finite(pair)
                if pair[0] >= pair[1]:
                    raise ValueError(f'Invalid body joint{joint + 1} limits: {pair}')
                self.body_limits.append(pair)
            self.head_dof = self.robot.get_ctrl_obj_dof(2)
            if not 0 <= self.head_dof <= 3:
                raise ValueError(f'Invalid head DOF from controller: {self.head_dof}')
            for arm in CONTROLLED_OBJECTS:
                self.recovery_stage = f'object{arm}:position'
                position_attempted = True
                self.h.check(self.robot.switch_to_position_mode(arm, 3000, self.c.speed, self.c.acc), f'arm{arm} POSITION')
                if self.robot.current_state(arm) != 1:
                    raise RuntimeError(f'arm{arm} did not reach POSITION')
            self.recovery_stage = 'feedback_check'
            data = self.robot.get_rt_dict()
            self.sample(data)
            self.sample_body(data)
            self.sample_head(data)
            self.targets = None
            self.settled = None
            self.deadline = 0
            self.faulted = False
            self.recovery_stage = 'ready'
        except BaseException as exc:
            # An existing reset/parameter failure is not itself a new motion.
            # Stop if mode switching was attempted and its result is uncertain,
            # or POSITION was entered on any controlled object.
            if mode_transition_pending or position_attempted:
                self.recovery_stop_requested = True
                try:
                    self.stop(force=True)
                except Exception as stop_exc:
                    raise RuntimeError(f'{exc}; recovery stop failed: {stop_exc}') from exc
            raise

    def sample(self, data=None):
        if self.robot.check_link_state() != 1:
            raise RuntimeError('Controller disconnected or feedback interrupted')
        data = self.robot.get_rt_dict() if data is None else data
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
            if result.m_Output_IsOutRange:
                raise ValueError(f'arm{arm} IK outside workspace')
            if any(result.m_Output_IsDeg):
                raise ValueError(f'arm{arm} IK is singular')
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

    def sample_body(self, data=None):
        if self.robot.check_link_state() != 1:
            raise RuntimeError('Controller disconnected or feedback interrupted')
        data = self.robot.get_rt_dict() if data is None else data
        joints = list(data['body']['fb_pos'])
        if len(joints) != 6:
            raise ValueError('Expected six body feedback positions')
        self.h.finite(joints)
        return joints

    def sample_head(self, data=None):
        if self.robot.check_link_state() != 1:
            raise RuntimeError('Controller disconnected or feedback interrupted')
        if self.head_dof == 0:
            return []
        data = self.robot.get_rt_dict() if data is None else data
        values = list(data['head']['fb_pos'])
        if len(values) < self.head_dof:
            raise ValueError('Head feedback is shorter than controller DOF')
        joints = values[:self.head_dof]
        self.h.finite(joints)
        return joints

    def start_body(self, target):
        if self.active or self.faulted:
            raise RuntimeError('Busy or faulted; command rejected')
        start = self.sample_body()
        if len(target) != 6:
            raise ValueError('Expected six body joint positions')
        self.h.finite(target)
        for i, (current, goal, (lo, hi)) in enumerate(zip(start, target, self.body_limits)):
            if not lo <= goal <= hi or abs(goal - current) > self.c.max_delta:
                raise ValueError(f'Body joint{i + 1} exceeds limit or max_delta')
        if self.robot.current_state(BODY) not in (0, 1):
            raise ValueError('Body must be IDLE or POSITION')
        self.active = True
        self.motion_group = 'body'
        try:
            if self.robot.current_state(BODY) == 0:
                self.h.check(self.robot.switch_to_position_mode(BODY, 3000, self.c.speed, self.c.acc), 'Body POSITION')
            latest = self.sample_body()
            if self.robot.current_state(BODY) != 1 or max(abs(x-y) for x, y in zip(latest, start)) > 0.2:
                raise RuntimeError('Body state/start changed during preparation')
            self.h.check(self.robot.runtime_set_joint_pos_cmd(BODY, target), 'Body position command')
            self.targets = list(target)
            self.deadline = time.monotonic() + self.c.timeout
            self.settled = None
        except BaseException:
            self.stop()
            raise
        return 'running', {'group': 'body', 'joints_deg': target}

    def start(self, targets=None, matrices=None, arm=None, target=None):
        if self.active or self.faulted:
            raise RuntimeError('Busy or faulted; command rejected')
        starts, _ = self.sample()
        if arm is not None:
            if arm not in (0, 1) or target is None or targets is not None or matrices is not None:
                raise ValueError('A single-arm command requires arm=0/1 and target only')
            targets = [list(q) for q in starts]
            targets[arm] = list(target)
        if matrices is not None:
            targets = self.solve(matrices, starts)
        # Validate both targets even when one arm does not need to move.
        for a in (0, 1):
            self.h.validate_target(starts[a], targets[a], self.limits[a], self.c.max_delta)
        initial_states = [self.robot.current_state(a) for a in (0, 1)]
        if any(state not in (0, 1) for state in initial_states):
            raise ValueError('Both arms must be IDLE or POSITION; restart to reset faults')
        moving = [a for a in (0, 1)
                  if max(abs(x-y) for x, y in zip(starts[a], targets[a])) > self.c.tolerance]
        plans = {a: self.h.prepare_plan(self.robot, a, starts[a], targets[a], self.limits[a], self.c)
                 for a in moving}
        mask = sum(1 << a for a in moving)
        details = {'group': 'arms', 'joints_deg': targets,
                   'points': [plans[a][1] if a in plans else 0 for a in (0, 1)],
                   'moving_arms': moving, 'run_mask': mask}
        self.check_start(starts, initial_states)
        if not moving:
            return 'completed', {**details, 'reason': 'Both arms already within tolerance'}
        self.active = True
        self.motion_group = 'arms'
        try:
            self.check_start(starts, initial_states)
            for a in moving:
                if initial_states[a] == 0:
                    self.h.check(self.robot.switch_to_position_mode(a, 3000, self.c.speed, self.c.acc), f'arm{a} POSITION')
            for a in moving:
                self.h.check(self.robot.config_set_traj(a, *plans[a]), f'arm{a} upload')
            self.arm_states = [1 if a in moving else initial_states[a] for a in (0, 1)]
            self.check_start(starts, self.arm_states)
            ret = self.robot.runtime_run_traj(mask)
            if ret != mask:
                raise RuntimeError(f'Partial start: mask={ret}')
            self.moving_arms = moving
            self.targets = targets
            self.deadline = time.monotonic() + self.c.timeout
            self.settled = None
        except BaseException:
            self.stop()
            raise
        return 'running', details

    def check_start(self, starts, state):
        latest, _ = self.sample()
        for a in (0, 1):
            expected = state[a] if isinstance(state, (list, tuple)) else state
            if self.robot.current_state(a) != expected or max(abs(x-y) for x,y in zip(latest[a], starts[a])) > 0.2:
                raise RuntimeError(f'arm{a} state/start changed during preparation')

    def poll(self, joints, body_joints=None):
        if not self.active:
            return False
        if time.monotonic() >= self.deadline:
            raise TimeoutError('Motion timeout')
        if self.motion_group == 'body':
            if self.robot.current_state(BODY) != 1:
                raise RuntimeError('Body left POSITION during execution')
            current = body_joints if body_joints is not None else self.sample_body()
            done = max(abs(x-y) for x, y in zip(current, self.targets)) <= self.c.tolerance
            if not done:
                self.settled = None
                return False
            now = time.monotonic()
            self.settled = now if self.settled is None else self.settled
            if now - self.settled < 0.5:
                return False
            self.active = False
            return True
        if any(self.robot.current_state(a) != self.arm_states[a] for a in (0, 1)):
            raise RuntimeError('Arm state changed during execution')
        sg = self.robot.get_sg_dict()
        done = (all(sg['arms'][a]['get']['traj_state'] == 0 for a in self.moving_arms) and
                all(max(abs(x-y) for x,y in zip(joints[a], self.targets[a])) <= self.c.tolerance
                    for a in (0, 1)))
        if not done:
            self.settled = None
            return False
        now = time.monotonic()
        self.settled = now if self.settled is None else self.settled
        if now-self.settled < 0.5:
            return False
        self.active = False
        return True

    def stop(self, force=False):
        self.faulted = True
        try:
            if self.active or force:
                ret = self.robot.emergency_stop(STOP_MASK)
                if ret != STOP_MASK:
                    raise RuntimeError(f'Emergency stop unconfirmed: mask={ret}; use hardware stop')
        finally:
            self.active = False

    def close(self):
        if self.closed:
            return
        self.closed = True
        failures = []
        try:
            if self.active:
                # Preserve emergency handling for interrupted motion. Do not
                # automatically reset or leave ERROR after an emergency stop.
                self.stop()
            elif self.linked:
                if self.robot.check_link_state() != 1:
                    failures.append('Controller disconnected; IDLE could not be confirmed')
                else:
                    for obj in CONTROLLED_OBJECTS:
                        label = 'body' if obj == BODY else f'arm{obj}'
                        try:
                            state = self.robot.current_state(obj)
                            if state == 100:  # Preserve ERROR for diagnosis.
                                continue
                            if state != 0:
                                self.h.check(self.robot.switch_to_idle(obj, 3000), f'{label} IDLE on exit')
                            if self.robot.current_state(obj) != 0:
                                raise RuntimeError(f'{label} did not reach IDLE on exit')
                        except Exception as exc:
                            failures.append(str(exc))
                    if failures:
                        try:
                            self.stop(force=True)
                        except Exception as exc:
                            failures.append(f'Exit emergency stop failed: {exc}')
        finally:
            self.linked = False
            try:
                self.robot.cleanup()
            finally:
                self.robot.unlink()
        if failures:
            raise RuntimeError('; '.join(failures))
