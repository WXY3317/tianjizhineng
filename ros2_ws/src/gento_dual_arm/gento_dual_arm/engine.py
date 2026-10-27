# 串行执行 SDK 连接、目标下发及反馈读取，管理双臂控制状态；调用应由同一执行器串行调度。
"""Single-thread SDK state machine. All calls must be serialized by the ROS executor."""
import math
import time
import threading

from .cartesian import FREQUENCY

STREAM_PERIOD = 1. / FREQUENCY

BODY = 3
CONTROLLED_OBJECTS = (0, 1, BODY)
STOP_MASK = (1 << 0) | (1 << 1) | (1 << BODY)
# Keep vendor warnings/errors; the ROS node reports essential lifecycle events.
SDK_LOG_MASK = (1 << 2) | (1 << 3)


class Engine:
    def __init__(self, robot, helpers, ik_type, config, on_retime=None, on_upload=None):
        self.robot, self.h, self.ik_type, self.c = robot, helpers, ik_type, config
        self.on_retime = on_retime
        self.on_upload = on_upload
        self.cancel_requested = threading.Event()
        self.cartesian_stream = None
        self.cartesian_tracking = None
        self.trajectory_diagnostic = None
        self.last_tracking_sample = None
        self.cartesian_rate_limits = None
        self.motion_metrics = {}
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
        ret = self.robot.link(*map(int, self.c.ip.split('.')), SDK_LOG_MASK)
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
        before = self.recovery_call(self.robot.current_state, obj)
        ret, code = self.recovery_call(self.robot.reset_error, obj, 1000)
        after = self.recovery_call(self.robot.current_state, obj)
        if ret != 0 or after != 0:
            raise RuntimeError(f'object{obj} reset failed: ret={ret}, error={code}, '
                               f'state_before={before}, state_after={after}')
        return {'object_id': obj, 'return_code': ret, 'error_code': code,
                'state_before': before, 'state_after': after}

    def recovery_call(self, operation, *args):
        self.check_cancel()
        result = operation(*args)
        self.check_cancel()
        return result

    def reset_errors(self):
        if self.active:
            raise ValueError('Motion active; stop before requesting error reset')
        self.faulted = True
        self.recovery_stop_requested = False
        self.recovery_stage = 'link_check'
        mode_transition_pending = False
        position_attempted = False
        if not self.linked or self.recovery_call(self.robot.check_link_state) != 1:
            raise RuntimeError('Controller not connected; restart node to reconnect')
        try:
            reset_failures = []
            for arm in CONTROLLED_OBJECTS:
                label = 'body' if arm == BODY else f'arm{arm}'
                before = self.recovery_call(self.robot.current_state, arm)
                # ResetError waits for IDLE; it does not switch a healthy
                # POSITION/PD/release object back to IDLE on every controller.
                if before not in (0, 100):  # IDLE / ERROR
                    self.recovery_stage = f'{label}:idle'
                    mode_transition_pending = True
                    self.h.check(self.recovery_call(self.robot.switch_to_idle, arm, 3000), f'{label} IDLE before reset')
                    if self.recovery_call(self.robot.current_state, arm) != 0:
                        raise RuntimeError(f'{label} did not reach IDLE before reset: state={self.recovery_call(self.robot.current_state, arm)}')
                mode_transition_pending = False
                if before == 100:  # Match the vendor example: reset only ERROR.
                    self.recovery_stage = f'{label}:reset'
                    try:
                        ret, error_code = self.recovery_call(self.robot.reset_error, arm, 3000)
                        after = self.recovery_call(self.robot.current_state, arm)
                        if ret != 0 or after != 0:
                            reset_failures.append(f'{label} reset failed: ret={ret}, error={error_code}, '
                                                  f'state_before={before}, state_after={after}')
                    except Exception as exc:
                        reset_failures.append(f'{label} reset failed: {exc}')
            if reset_failures:
                self.recovery_stage = 'reset_summary'
                raise RuntimeError('; '.join(reset_failures))
            for arm in CONTROLLED_OBJECTS:
                if self.recovery_call(self.robot.current_state, arm) != 0:
                    mode_transition_pending = True
                    self.h.check(self.recovery_call(self.robot.switch_to_idle, arm, 3000), f'arm{arm} IDLE')
                if self.recovery_call(self.robot.current_state, arm) != 0:
                    raise RuntimeError(f'arm{arm} did not reach IDLE after reset')
            mode_transition_pending = False
            self.recovery_stage = 'model_and_limits'
            for arm in (0, 1):
                self.h.check(self.recovery_call(self.robot.init_single_arm_config, arm), f'Initialize arm{arm}')
            self.limits = [self.h.read_limits(self.robot, a, self.check_cancel) for a in (0, 1)]
            self.body_limits = []
            for joint in range(6):
                pair = []
                for suffix in ('LimitNeg', 'LimitPos'):
                    ret, value = self.recovery_call(self.robot.param_get_float, f'R.B.L{joint}.BASIC.{suffix}')
                    self.h.check(ret, f'Body joint{joint + 1} {suffix}')
                    pair.append(value)
                self.h.finite(pair)
                if pair[0] >= pair[1]:
                    raise ValueError(f'Invalid body joint{joint + 1} limits: {pair}')
                self.body_limits.append(pair)
            self.head_dof = self.recovery_call(self.robot.get_ctrl_obj_dof, 2)
            if not 0 <= self.head_dof <= 3:
                raise ValueError(f'Invalid head DOF from controller: {self.head_dof}')
            for arm in CONTROLLED_OBJECTS:
                self.recovery_stage = f'object{arm}:position'
                position_attempted = True
                self.h.check(self.recovery_call(self.robot.switch_to_position_mode, arm, 3000, self.c.speed, self.c.acc), f'arm{arm} POSITION')
                if self.recovery_call(self.robot.current_state, arm) != 1:
                    raise RuntimeError(f'arm{arm} did not reach POSITION')
            self.recovery_stage = 'feedback_check'
            data = self.recovery_call(self.robot.get_rt_dict)
            self.sample(data)
            self.sample_body(data)
            self.sample_head(data)
            self.targets = None
            self.settled = None
            self.deadline = 0
            self.check_cancel()
            self.faulted = False
            self.recovery_stage = 'ready'
        except BaseException as exc:
            # An existing reset/parameter failure is not itself a new motion.
            # Stop if mode switching was attempted and its result is uncertain,
            # or POSITION was entered on any controlled object.
            if mode_transition_pending or position_attempted or self.cancel_requested.is_set():
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

    def check_cancel(self):
        if self.cancel_requested.is_set():
            raise ValueError("Motion cancelled by stop request")

    def solve(self, matrices, starts, moving=(0, 1)):
        targets = []
        for arm in (0, 1):
            self.check_cancel()
            if arm not in moving:
                targets.append(list(starts[arm]))
                continue
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
        self.check_cancel()
        if self.active or self.faulted:
            raise RuntimeError('Busy or faulted; command rejected')
        self.motion_metrics = {}
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
            self.check_cancel()
            self.h.check(self.robot.runtime_set_joint_pos_cmd(BODY, target), 'Body position command')
            self.targets = list(target)
            self.deadline = time.monotonic() + self.c.timeout
            self.settled = None
        except BaseException:
            self.stop()
            raise
        return 'running', {'group': 'body', 'joints_deg': target}

    def start(self, targets=None, matrices=None, arm=None, target=None, target_matrix=None):
        self.check_cancel()
        if self.active or self.faulted:
            raise RuntimeError('Busy or faulted; command rejected')
        starts, poses = self.sample()
        if target_matrix is not None:
            if arm not in (0, 1) or target is not None or targets is not None or matrices is not None:
                raise ValueError('Single-arm pose requires arm=0/1 and target_matrix only')
            matrices = [pose for pose in poses]
            matrices[arm] = target_matrix
        elif arm is not None:
            if arm not in (0, 1) or target is None or targets is not None or matrices is not None:
                raise ValueError('A single-arm command requires arm=0/1 and target only')
            targets = [list(q) for q in starts]
            targets[arm] = list(target)
        self.motion_metrics = {}
        self.cartesian_rate_limits = None
        cartesian_plans = None
        plan_info = None
        execution = getattr(self.c, 'cartesian_execution', 'trajectory')
        if execution not in ('trajectory', 'stream'):
            raise ValueError('cartesian_execution must be trajectory or stream')
        if matrices is not None:
            from .cartesian import prepare_cartesian
            targets, cartesian_plans, plan_info = prepare_cartesian(self, starts, poses, matrices)
        # Validate both targets even when one arm does not need to move.
        for a in (0, 1):
            self.h.validate_target(starts[a], targets[a], self.limits[a], self.c.max_delta)
        initial_states = [self.robot.current_state(a) for a in (0, 1)]
        if any(state not in (0, 1) for state in initial_states):
            raise ValueError('Both arms must be IDLE or POSITION; restart to reset faults')
        moving = [a for a in (0, 1)
                  if max(abs(x-y) for x, y in zip(starts[a], targets[a])) > self.c.tolerance]
        plans = {a: self.h.prepare_plan(self.robot, a, starts[a], targets[a], self.limits[a], self.c)
                 for a in moving} if cartesian_plans is None else cartesian_plans
        if cartesian_plans is not None:
            moving = list(plans)
        mask = sum(1 << a for a in moving)
        details = {'group': 'arms', 'joints_deg': targets,
                   'points': [plans[a][1] if a in plans else 0 for a in (0, 1)],
                   'moving_arms': moving, 'run_mask': mask}
        if plan_info is not None:
            details['cartesian_plan'] = plan_info
            details['cartesian_execution'] = execution
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
            if cartesian_plans is not None:
                self.check_cartesian_controller_ratios()
            if cartesian_plans is not None and getattr(self.c, 'trajectory_log_dir', ''):
                from .trajectory_diagnostics import TrajectoryDiagnostic
                self.trajectory_diagnostic = TrajectoryDiagnostic(plans, poses, matrices, execution, self.c)
                self.trajectory_diagnostic.data['controller_limits'] = plan_info['controller_limits']
                details['trajectory_log'] = self.trajectory_diagnostic.path
            for a in moving if cartesian_plans is None or execution == 'trajectory' else ():
                self.check_cancel()
                self.upload_trajectory(a, *plans[a])
            self.arm_states = [1 if a in moving else initial_states[a] for a in (0, 1)]
            self.check_start(starts, self.arm_states)
            self.check_cancel()
            if cartesian_plans is not None:
                self.check_cartesian_controller_ratios()
                self.check_stream_entry(plans)
                self.check_cancel()
                self.last_tracking_sample = None
                self.cartesian_tracking = {'starts': poses, 'goals': matrices,
                                           'max_error_m': {a: 0. for a in moving}, 'samples': 0}
            if cartesian_plans is None or execution == 'trajectory':
                self.check_uploaded_trajectories(moving)
                self.check_cancel()
                if self.trajectory_diagnostic is not None:
                    self.trajectory_diagnostic.started_ns = time.monotonic_ns()
                    self.trajectory_diagnostic.data['execution_start_unix_ns'] = time.time_ns()
                ret = self.robot.runtime_run_traj(mask)
                if ret != mask:
                    raise RuntimeError(f'Partial start: mask={ret}')
            else:
                self.cartesian_stream = {'plans': plans, 'index': 0,
                                         'next': time.monotonic(), 'last': None,
                                         'first': None, 'max_gap': 0.}
            self.moving_arms = moving
            self.targets = targets
            self.deadline = time.monotonic() + self.c.timeout
            self.settled = None
        except BaseException:
            self.stop()
            raise
        return 'running', details

    def upload_trajectory(self, arm, points, count):
        self.check_cancel()
        started = time.monotonic_ns()
        def snapshot():
            try:
                return {'link': self.robot.check_link_state(), 'object_state': self.robot.current_state(arm),
                        'traj_state': self.robot.get_sg_dict()['arms'][arm]['get']['traj_state']}
            except Exception as exc:
                return {'unavailable': str(exc)}
        before = snapshot()
        if self.on_upload is not None:
            self.on_upload({'phase': 'begin', 'arm_id': arm, 'points': count, 'before': before})
        ret = self.robot.config_set_traj(arm, points, count)
        event = {'phase': 'confirmed' if ret == 0 else 'failed', 'arm_id': arm, 'points': count,
                 'sdk_return': ret, 'elapsed_s': (time.monotonic_ns()-started)/1e9,
                 'before': before, 'after': snapshot()}
        if self.trajectory_diagnostic is not None:
            self.trajectory_diagnostic.data.setdefault('uploads', []).append(event)
        if self.on_upload is not None:
            self.on_upload(event)
        self.check_cancel()
        if ret != 0:
            raise RuntimeError(f'arm{arm} upload failed: SDK={ret}, points={count}, '
                               f"elapsed_s={event['elapsed_s']:.3f}, before={before}, after={event['after']}")

    def check_uploaded_trajectories(self, moving):
        # RunTraj only queues a command; it does not validate upload completion itself.
        sg = self.robot.get_sg_dict()
        for arm in moving:
            self.check_cancel()
            state = sg['arms'][arm]['get']['traj_state']
            if state != 2:
                raise RuntimeError(f'arm{arm} upload not confirmed before start: traj_state={state}, expected=2')

    def check_start(self, starts, state):
        latest, _ = self.sample()
        for a in (0, 1):
            expected = state[a] if isinstance(state, (list, tuple)) else state
            if self.robot.current_state(a) != expected or max(abs(x-y) for x,y in zip(latest[a], starts[a])) > 0.2:
                raise RuntimeError(f'arm{a} state/start changed during preparation')

    def read_cartesian_rate_limits(self, moving):
        """Respect per-joint controller ratings and active POSITION percentages."""
        sg = self.robot.get_sg_dict()
        result = {}
        for arm in moving:
            self.check_cancel()
            settings = sg['arms'][arm]['set']
            ratios = [float(settings['vel_ratio']), float(settings['acc_ratio'])]
            self.h.finite(ratios)
            if not all(0 < value <= 100 for value in ratios):
                raise ValueError(f'arm{arm} invalid controller speed/acceleration ratios: {ratios}')
            rated = [[], []]
            for joint in range(7):
                for kind, suffix in enumerate(('VelMax', 'AccMax')):
                    ret, value = self.recovery_call(self.robot.param_get_float,
                                                   f'R.A{arm}.L{joint}.BASIC.{suffix}')
                    self.h.check(ret, f'arm{arm} joint{joint+1} {suffix}')
                    self.h.finite([value])
                    if value <= 0:
                        raise ValueError(f'arm{arm} joint{joint+1} invalid {suffix}: {value}')
                    rated[kind].append(value)
            result[arm] = {
                'vel_ratio': ratios[0], 'acc_ratio': ratios[1], 'headroom': .9,
                'rated_speed_deg_s': rated[0], 'rated_acc_deg_s2': rated[1],
                'speed': [min(self.c.cartesian_joint_speed, v*ratios[0]/100*.9) for v in rated[0]],
                'acceleration': [min(self.c.cartesian_joint_acc, a*ratios[1]/100*.9) for a in rated[1]],
            }
        self.cartesian_rate_limits = result
        return result

    def check_cartesian_controller_ratios(self):
        if not self.cartesian_rate_limits:
            return
        sg = self.robot.get_sg_dict()
        for arm, limits in self.cartesian_rate_limits.items():
            current = sg['arms'][arm]['set']
            for key in ('vel_ratio', 'acc_ratio'):
                value = float(current[key])
                if not math.isfinite(value) or abs(value-limits[key]) > 1e-6:
                    raise RuntimeError(f'arm{arm} controller {key} changed since planning; replan required')

    def check_stream_entry(self, plans):
        """Validate the measured-to-first-point join, including its next segment."""
        latest, _ = self.sample()
        for arm, (points, count) in plans.items():
            first = list(points[:7])
            second = list(points[7:14]) if count > 1 else first
            velocity = [(q-x)*50 for q, x in zip(first, latest[arm])]
            next_velocity = [(q-x)*50 for q, x in zip(second, first)]
            acceleration = [abs(v)*50 for v in velocity]
            acceleration += [abs(v-u)*50 for v, u in zip(next_velocity, velocity)]
            limits = (self.cartesian_rate_limits or {}).get(arm)
            speed_limits = limits['speed'] if limits else [self.c.cartesian_joint_speed]*7
            acc_limits = limits['acceleration'] if limits else [self.c.cartesian_joint_acc]*7
            if (any(abs(v) > cap for v, cap in zip(velocity, speed_limits)) or
                    any(a > cap for a, cap in zip(acceleration, acc_limits*2))):
                raise RuntimeError(f'arm{arm} moved since planning; Cartesian entry exceeds rate limits; replan required')

    def stream_tick(self):
        """Called at 50 Hz by the same callback group as every SDK operation."""
        if self.cancel_requested.is_set():
            self.stop(force=True)
            self.cancel_requested.clear()
            return 'stopped'
        stream = self.cartesian_stream
        if not self.active or stream is None:
            return None
        now = time.monotonic()
        if now >= self.deadline:
            raise TimeoutError('Motion timeout')
        if now < stream['next']:
            return None
        index = stream['index']
        count = next(iter(stream['plans'].values()))[1]
        if index >= count:
            return None
        if stream['last'] is not None and now - stream['last'] > 0.1:
            raise RuntimeError('Cartesian command stream stalled for more than 100 ms')
        if index == 0:
            self.check_stream_entry(stream['plans'])
        for arm, (points, _) in stream['plans'].items():
            self.check_cancel()
            if self.robot.current_state(arm) != 1:
                raise RuntimeError(f'arm{arm} left POSITION during Cartesian execution')
            self.h.check(self.robot.runtime_set_joint_pos_cmd(arm, list(points[index*7:(index+1)*7])),
                         f'arm{arm} Cartesian point {index}')
        stream['index'] += 1
        sent_at = time.monotonic()
        if stream['first'] is None:
            stream['first'] = sent_at
        if stream['last'] is not None:
            stream['max_gap'] = max(stream['max_gap'], sent_at - stream['last'])
        stream['last'] = sent_at
        if stream['index'] == count:
            elapsed = sent_at - stream['first']
            self.motion_metrics['cartesian_stream'] = {
                'sent_points': count, 'send_duration_s': elapsed,
                'actual_hz': (count-1)/elapsed if elapsed > 0 else 0.,
                'max_interval_s': stream['max_gap'], 'target_hz': FREQUENCY}
        # Measure from completion, including the time spent inside SDK calls.
        stream['next'] = sent_at + STREAM_PERIOD
        return None

    def check_cartesian_tracking(self, joints, matrices=None, sg=None):
        track = self.cartesian_tracking
        if track is None:
            return
        limit = getattr(self.c, 'cartesian_path_tolerance', .005)
        snapshot = {'arms': {}}
        failure = None
        for arm in self.moving_arms:
            actual = matrices[arm] if matrices is not None else self.h.fk(self.robot, arm, joints[arm])
            start, goal = track['starts'][arm], track['goals'][arm]
            direction = [goal[i][3]-start[i][3] for i in range(3)]
            offset = [actual[i][3]-start[i][3] for i in range(3)]
            length2 = sum(x*x for x in direction)
            fraction = min(1., max(0., sum(x*y for x, y in zip(offset, direction))/length2)) if length2 > 1e-12 else 0.
            error = math.sqrt(sum((x-fraction*y)**2 for x, y in zip(offset, direction))) * self.c.sdk_length_to_m
            self.h.finite([error])
            track['max_error_m'][arm] = max(track['max_error_m'][arm], error)
            residual = [(x-fraction*y)*self.c.sdk_length_to_m for x, y in zip(offset, direction)]
            state = sg['arms'][arm] if sg is not None else {}
            snapshot['arms'][str(arm)] = {
                'joints_deg': list(joints[arm]),
                'tcp_position_m': [actual[i][3]*self.c.sdk_length_to_m for i in range(3)],
                'tcp_rotation': [row[:3] for row in actual[:3]],
                'line_projection_fraction': fraction, 'line_error_m': error,
                'line_error_vector_m': residual,
                'traj_state': state.get('get', {}).get('traj_state'),
                'controller_set': state.get('set', {}),
            }
            if error > limit:
                failure = f'arm{arm} TCP departed Cartesian line: {error:.6f} m > {limit:.6f} m'
        self.last_tracking_sample = snapshot
        if self.trajectory_diagnostic is not None:
            self.trajectory_diagnostic.append(snapshot)
        if failure is not None:
            raise RuntimeError(failure)
        track['samples'] += 1
        self.motion_metrics['cartesian_tracking'] = {
            'samples': track['samples'], 'max_line_error_m': dict(track['max_error_m']),
            'tolerance_m': limit}

    def poll(self, joints, body_joints=None, matrices=None):
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
        sg = self.robot.get_sg_dict()
        self.check_cartesian_tracking(joints, matrices, sg)
        if self.cartesian_tracking is not None:
            for arm, limits in (self.cartesian_rate_limits or {}).items():
                for key in ('vel_ratio', 'acc_ratio'):
                    value = float(sg['arms'][arm]['set'][key])
                    if not math.isfinite(value) or abs(value-limits[key]) > 1e-6:
                        raise RuntimeError(f'arm{arm} controller {key} changed during Cartesian execution')
        if any(self.robot.current_state(a) != self.arm_states[a] for a in (0, 1)):
            raise RuntimeError('Arm state changed during execution')
        if self.cartesian_stream is not None:
            stream = self.cartesian_stream
            finished = stream['index'] >= next(iter(stream['plans'].values()))[1]
        else:
            finished = all(sg['arms'][a]['get']['traj_state'] == 0 for a in self.moving_arms)
        done = (finished and
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
        self.cartesian_stream = None
        self.cartesian_tracking = None
        return True

    def stop(self, force=False):
        self.cartesian_tracking = None
        self.cartesian_stream = None
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
