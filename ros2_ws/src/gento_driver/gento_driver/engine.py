# 串行执行 SDK 连接、目标下发及反馈读取，管理双臂控制状态；调用应由同一执行器串行调度。
"""Single-thread SDK state machine. All calls must be serialized by the ROS executor."""
import math
import time
import threading
from .body_kinematics import LunaBodyKinematics

FREQUENCY = 50

STREAM_PERIOD = 1. / FREQUENCY

BODY = 3
CONTROLLED_OBJECTS = (0, 1, BODY)
STOP_MASK = (1 << 0) | (1 << 1) | (1 << BODY)
# Keep vendor warnings/errors; the ROS node reports essential lifecycle events.
SDK_LOG_MASK = (1 << 2) | (1 << 3)


class Engine:
    def __init__(self, robot, helpers, ik_type, config, on_upload=None):
        self.robot, self.h, self.ik_type, self.c = robot, helpers, ik_type, config
        self.control_enabled = getattr(config, 'enable_control', True)
        self.on_upload = on_upload
        self.cancel_requested = threading.Event()
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
        if self.control_enabled:
            self.reset_errors()
        else:
            for arm in (0, 1):
                self.h.check(self.robot.init_single_arm_config(arm), f'Initialize arm{arm} feedback model')
            self.head_dof = self.robot.get_ctrl_obj_dof(2)
            if not 0 <= self.head_dof <= 3:
                raise ValueError(f'Invalid head DOF: {self.head_dof}')
            self.sample()
            self.sample_body()
            self.sample_head()

    def require_control(self):
        if not self.control_enabled:
            raise ValueError('Driver is feedback-only; control is disabled')

    def reset_object_error(self, obj):
        """Mirror a vendor Reset button: reset one object, without enabling motion."""
        self.require_control()
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
        self.require_control()
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
            self.body_limits = LunaBodyKinematics(self.body_limits).joint_limits_deg
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
        from .ik import solve_step
        targets=[]
        for arm in (0,1):
            self.check_cancel()
            targets.append(solve_step(self,arm,matrices[arm],starts[arm]) if arm in moving else list(starts[arm]))
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



    def upload_trajectory(self, arm, points, count):
        self.require_control()
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
            planning_speed=min(ratios[0],self.c.speed)
            planning_acc=min(ratios[1],self.c.acc)
            result[arm] = {
                'planning_vel_ratio':planning_speed,'planning_acc_ratio':planning_acc,
                'vel_ratio': ratios[0], 'acc_ratio': ratios[1], 'headroom': .9,
                'rated_speed_deg_s': rated[0], 'rated_acc_deg_s2': rated[1],
                'speed': [min(self.c.cartesian_joint_speed, v*planning_speed/100*.9) for v in rated[0]],
                'acceleration': [min(self.c.cartesian_joint_acc, a*planning_acc/100*.9) for a in rated[1]],
            }
        self.cartesian_rate_limits = result
        return result

    def set_arm_ratios(self,ratios,allow_cancel=True,timeout=.5):
        """Set and confirm global ratios; callers retain independently checked path limits."""
        self.require_control()
        for arm,values in ratios.items():
            if arm not in (0,1) or len(values)!=2:raise ValueError('Invalid arm ratios')
            self.h.finite(values)
            if not all(1<=v<=100 for v in values):raise ValueError('Ratios must be within [1,100]')
        failures=[]
        for arm,values in ratios.items():
            if allow_cancel:self.check_cancel()
            try:self.h.check(self.robot.runtime_set_speed_ratio(arm,*values),f'arm{arm} speed ratios')
            except Exception as exc:
                if allow_cancel:raise
                failures.append(str(exc))
        if failures:raise RuntimeError('; '.join(failures))
        deadline=time.monotonic()+timeout
        while True:
            if allow_cancel:self.check_cancel()
            if self.robot.check_link_state()!=1:raise RuntimeError('Controller disconnected while setting ratios')
            sg=self.robot.get_sg_dict()
            confirmed=True
            for arm,values in ratios.items():
                settings=sg['arms'][arm]['set']
                current=[float(settings[k]) for k in ('vel_ratio','acc_ratio')]
                self.h.finite(current)
                confirmed=confirmed and all(abs(a-b)<=1e-6 for a,b in zip(current,values))
            if confirmed:return
            if time.monotonic()>=deadline:raise TimeoutError('Controller speed ratios were not confirmed')
            time.sleep(.005)

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





    def stop(self, force=False):
        self.faulted = True
        try:
            if self.control_enabled and (self.active or force):
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
            elif self.linked and self.control_enabled:
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
