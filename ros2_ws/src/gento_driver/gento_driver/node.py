"""The sole SDK owner: serialized RPC, validated execution and independent stops."""
import ctypes
import copy
import json
import math
import threading
import time
import uuid
from pathlib import Path
from dataclasses import asdict
from types import SimpleNamespace

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.task import Future
from std_msgs.msg import String, Empty
from rcl_interfaces.msg import ParameterDescriptor
from gento_interfaces.srv import DriverQuery
from gento_interfaces.action import ExecuteTrajectory

from .config import DEFAULTS
from .conversions import finite
from .engine import Engine
from .native_ik import bind_native_ik
from .sdk_ops import tracking_sample
from .safety import validate_goal, line_error
from .body_kinematics import LunaBodyKinematics, pose_error


def wire_json(value):
    # Preserve invalid controller telemetry as null in diagnostic JSON, not NaN tokens.
    def clean(item):
        if isinstance(item,float) and not math.isfinite(item):return None
        if isinstance(item,dict):return {k:clean(v) for k,v in item.items()}
        if isinstance(item,(list,tuple)):return [clean(v) for v in item]
        return item
    return json.dumps(clean(value),allow_nan=False)


class Bridge(Node):
    def __init__(self, hardware=None):
        super().__init__('gento_driver')
        for key,value in DEFAULTS.items(): self.declare_parameter(key,value,ParameterDescriptor(read_only=True))
        self.c=SimpleNamespace(**{key:self.get_parameter(key).value for key in DEFAULTS})
        if self.get_parameter('use_sim_time').value: raise ValueError('Real hardware requires wall clock')
        import ipaddress
        self.c.ip=str(ipaddress.IPv4Address(self.c.ip))
        values=[self.c.speed,self.c.acc,self.c.max_delta,self.c.timeout,self.c.tolerance,
                self.c.publish_rate,self.c.command_max_age,self.c.sdk_length_to_m,self.c.cartesian_path_tolerance,
                self.c.cartesian_speed,self.c.cartesian_angular_speed,self.c.cartesian_joint_speed,self.c.cartesian_joint_acc,
                self.c.body_position_tolerance_mm,self.c.body_orientation_tolerance_deg]
        finite(values)
        if min(values)<=0 or max(self.c.speed,self.c.acc)>10 or self.c.publish_rate>100:
            raise ValueError('Invalid hardware limits/configuration')
        if self.c.cartesian_execution!='trajectory': raise ValueError('Split execution requires controller trajectory playback')
        frames=[self.c.left_frame,self.c.right_frame,self.c.body_frame]
        if any(not isinstance(f,str) or not f.strip() for f in frames) or len(set(frames))!=3:
            raise ValueError('Arm and body frame names must be nonempty and distinct')
        self.body_geometry=LunaBodyKinematics()
        self.sdk_lock=threading.RLock()
        self.lock=threading.Lock()
        self.stop_event=threading.Event()
        self.session=uuid.uuid4().hex
        self.reserved=None
        self.pending=None
        self.active=None
        self.seen=set()
        self.heartbeat=None
        self.sequence=0
        self.last_publish=0.
        self.latest=None
        self.failed=False
        self.stop_confirmed=False
        self.last_fault=None
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.raw_pub=self.create_publisher(String,'/gento_driver/raw_state',1)
        self.status_pub=self.create_publisher(String,'/gento_driver/status',latched)
        if hardware is None:
            from .sdk_loader import load_sdk
            from . import sdk_ops as helpers
            sdk=load_sdk(Path(self.c.sdk_root).expanduser().resolve())
            robot=sdk.GentoRobot()
            hardware=Engine(robot,helpers,bind_native_ik(robot,sdk),self.c)
        self.engine=hardware
        self.engine.c=self.c
        self.engine.control_enabled=self.c.enable_control
        self.engine.cancel_requested=self.stop_event
        try:
            self.engine.connect()
        except Exception:
            self.engine.close()
            raise
        self.create_service(DriverQuery,'/gento_driver/query',self.query)
        self.interrupt_group=ReentrantCallbackGroup()
        self.action=None
        if self.c.enable_control:
            self.create_subscription(Empty,'/gento_driver/stop',lambda _:self.stop_event.set(),1,
                                     callback_group=self.interrupt_group)
            self.create_subscription(String,'/gento_driver/heartbeat',self.receive_heartbeat,1,
                                     callback_group=self.interrupt_group)
            self.action=ActionServer(self,ExecuteTrajectory,'/gento_driver/execute',self.execute,
                                    goal_callback=self.accept_goal,cancel_callback=self.cancel_goal,
                                    callback_group=self.interrupt_group)
        self.timer=self.create_timer(.02,self.tick)
        self.status('ready',control_enabled=self.c.enable_control)

    def status(self,state,**details):
        payload={'state':state,'session_id':self.session,**details}
        if self.context.ok(): self.status_pub.publish(String(data=json.dumps(payload)))
        if state=='fault': self.get_logger().error(str(details.get('reason','fault')))
        elif state in ('ready','running','completed','stopped'):self.get_logger().info(state)
        else:self.get_logger().debug(state)

    def snapshot(self):
        data=self.engine.robot.get_rt_dict()
        joints,matrices=self.engine.sample(data)
        body=self.engine.sample_body(data)
        body_matrix=self.body_geometry.forward(body)
        head=self.engine.sample_head(data)
        lines=self.active.get('lines',{}) if self.active else {}
        tracking=[tracking_sample(self.engine.robot,a,data['arms'][a]['fb'],joints[a],a in lines) for a in (0,1)]
        self.sequence+=1
        stamp=self.get_clock().now().to_msg()
        objects=[]
        for obj,source in ((0,data['arms'][0]),(1,data['arms'][1]),(3,data['body']),(2,data.get('head',{}))):
            objects.append({'object_id':obj,'state':self.engine.robot.current_state(obj),
                            'error_code':source.get('state',{}).get('err',0)})
        return {'session_id':self.session,'sequence':self.sequence,'stamp':{'sec':stamp.sec,'nanosec':stamp.nanosec},
                'joints':joints,'body':body,'body_matrix':body_matrix,'head':head,'matrices':matrices,'objects':objects,'tracking':tracking,
                'controller':self.engine.robot.get_sg_dict(), 'config':vars(self.c),
                'limits':getattr(self.engine,'limits',None),'body_limits':getattr(self.engine,'body_limits',None),
                'active_task':self.active['goal'].task_id if self.active else None,
                'ready':not self.failed and not self.engine.faulted,
                'heartbeat_task_id':self.heartbeat[1] if self.heartbeat and self.heartbeat[0]==self.session and time.monotonic()-self.heartbeat[2]<1. else None}

    def query(self,request,response):
        with self.sdk_lock:
            response.session_id=self.session
            try:
                if request.operation!='snapshot' and request.session_id!=self.session:
                    raise ValueError('Stale session')
                if len(request.payload)>65536: raise ValueError('RPC payload too large')
                args=json.loads(request.payload or '{}')
                if not isinstance(args,dict): raise ValueError('Expected RPC object')
                if request.operation=='snapshot':
                    result=self.snapshot()
                    if (args.get('include_arm_limits',True) and self.c.enable_control
                            and not self.active and not self.failed and not self.engine.faulted):
                        result['controller_limits']=self.engine.read_cartesian_rate_limits([0,1])
                else:
                    self.engine.require_control()
                    if self.active or self.reserved: raise ValueError('Driver busy')
                    self.engine.check_cancel()
                    if request.operation=='body_ik':
                        model=LunaBodyKinematics(self.engine.body_limits)
                        solved=model.inverse(args['matrix'],args['reference'],
                                             max_joint_delta_deg=self.c.max_delta,
                                             checkpoint=self.engine.check_cancel)
                        self.engine.check_cancel()
                        if not solved.success:
                            raise ValueError(f'Body IK failed: {solved.message}; '
                                             f'position_error_mm={solved.position_error_mm:.6f}; '
                                             f'orientation_error_deg={solved.orientation_error_deg}')
                        result=asdict(solved)
                    elif request.operation=='ik':
                        matrices,starts,moving=args['matrices'],args['starts'],args['moving']
                        if len(matrices)!=2 or len(starts)!=2 or not moving or len(set(moving))!=len(moving) or any(a not in (0,1) for a in moving):
                            raise ValueError('Invalid IK inputs')
                        for matrix in matrices:
                            if len(matrix)!=4 or any(len(row)!=4 for row in matrix): raise ValueError('Invalid pose matrix')
                            finite([x for row in matrix for x in row])
                        for q in starts:
                            if len(q)!=7: raise ValueError('Invalid IK reference')
                            finite(q)
                        result=self.engine.solve(matrices,starts,moving)
                    elif request.operation=='fk':
                        joints=args['joints']
                        if len(joints)!=2 or any(len(q)!=7 for q in joints):raise ValueError('Expected two seven-joint vectors')
                        for q in joints:finite(q)
                        result=[self.engine.h.fk(self.engine.robot,a,joints[a]) for a in (0,1)]
                    elif request.operation in ('reset','object_reset'):
                        if self.stop_event.is_set(): raise ValueError('Stop pending')
                        self.stop_confirmed=False
                        result=self.engine.reset_errors() if request.operation=='reset' else self.engine.reset_object_error(args['object'])
                        self.failed=self.engine.faulted
                        self.last_fault=None
                        self.session=uuid.uuid4().hex
                        self.seen.clear()
                        self.timer.reset()
                        response.session_id=self.session
                        self.status('ready' if not self.failed else 'fault',reason='Recovery completed' if not self.failed else 'Object is IDLE; full reset required')
                    else: raise ValueError('Unsupported driver operation')
                response.success=True
                response.result=json.dumps(result)
            except Exception as exc:
                response.error=str(exc)
                if request.operation in ('reset','object_reset') and self.engine.faulted:
                    self.failed=True
                    self.status('fault',reason=str(exc))
            return response

    def receive_heartbeat(self,msg):
        try:
            if len(msg.data)>1024:return
            value=json.loads(msg.data)
            if not isinstance(value,dict) or type(value.get('stamp_ns')) is not int:return
            age=self.get_clock().now().nanoseconds-int(value['stamp_ns'])
            if (value['session_id']==self.session and isinstance(value['task_id'],str) and
                    len(value['task_id'])<=128 and 0<=age<=500000000):
                if self.heartbeat and self.heartbeat[:2]==(value['session_id'],value['task_id']) and int(value['stamp_ns'])<=self.heartbeat[3]:
                    return
                self.heartbeat=(value['session_id'],value['task_id'],time.monotonic(),int(value['stamp_ns']))
        except (ValueError,KeyError,TypeError): pass

    def alive(self,goal):
        value=self.heartbeat
        return value is not None and value[:2]==(goal.session_id,goal.task_id) and time.monotonic()-value[2]<1.

    def accept_goal(self,goal):
        with self.lock:
            if (not self.c.enable_control or self.failed or self.engine.faulted or self.stop_event.is_set()
                    or self.reserved or len(self.seen)>=4096 or goal.session_id!=self.session or goal.task_id in self.seen):
                return GoalResponse.REJECT
            if not goal.task_id or len(goal.task_id)>128: return GoalResponse.REJECT
            self.reserved=goal.task_id
            return GoalResponse.ACCEPT

    def cancel_goal(self,handle):
        self.stop_event.set()
        return CancelResponse.ACCEPT

    async def execute(self,handle):
        future=Future()
        with self.lock: self.pending=(handle,future)
        return await future

    def finish(self,success,reason,details=None):
        active=self.active
        if active is None: return
        if active.get('restore_ratios'):
            try:
                # Restoration must also run when stop/cancel is already latched.
                self.engine.set_arm_ratios(active['restore_ratios'],allow_cancel=False)
            except Exception as exc:
                success=False
                reason+=f'; speed ratio restoration failed: {exc}'
                self.failed=True
                try:self.emergency_stop()
                except Exception as stop_exc:reason+=f'; stop failed: {stop_exc}'
                self.status('fault',task_id=active['goal'].task_id,reason=reason)
        if success:
            self.status('completed',task_id=active['goal'].task_id,max_line_error_m=active['max_error'])
        handle=active['handle']
        result=ExecuteTrajectory.Result(success=success,message=reason,details=json.dumps(details or {}))
        try:
            if handle.is_cancel_requested: handle.canceled()
            elif success: handle.succeed()
            else: handle.abort()
        except Exception: pass
        active['future'].set_result(result)
        with self.lock:
            self.seen.add(active['goal'].task_id)
            self.reserved=None
        self.active=None

    def start_pending(self):
        with self.lock:
            pending=self.pending
            self.pending=None
        if pending is None: return
        handle,future=pending
        goal=handle.request
        self.active={'handle':handle,'future':future,'goal':goal,'started':time.monotonic(),'settled':None}
        try:
            self.engine.require_control()
            if self.failed or self.engine.faulted: raise ValueError('Driver fault is latched')
            self.engine.check_cancel()
            if not self.alive(goal): raise ValueError('Control heartbeat unavailable')
            self.status('validating',task_id=goal.task_id)
            snapshot=self.snapshot()
            if any(o['object_id'] in (0,1,3) and (o['state'] not in (0,1) or o['error_code']) for o in snapshot['objects']):
                raise ValueError('Controlled object has a fault or unsupported mode')
            plans,lines=validate_goal(goal,snapshot,self.engine,self.get_clock().now().nanoseconds)
            self.active.update(plans=plans,lines=lines,max_error={a:0. for a in lines})
            self.engine.active=True
            if 3 in plans:
                self.active['body_goal_matrix']=self.body_geometry.forward(plans[3][0])
                if self.engine.robot.current_state(3)!=1: raise ValueError('Body not in POSITION')
                self.engine.check_cancel()
                self.engine.h.check(self.engine.robot.runtime_set_joint_pos_cmd(3,plans[3][0]),'Body position command')
            else:
                self.status('uploading',task_id=goal.task_id)
                for arm,points in plans.items():
                    if self.engine.robot.current_state(arm)!=1: raise ValueError('Arm not in POSITION')
                    flat=[v for q in points for v in q]
                    self.engine.upload_trajectory(arm,(ctypes.c_double*len(flat))(*flat),len(points))
                self.engine.check_cartesian_controller_ratios()
                self.engine.check_uploaded_trajectories(plans)
                # Upload may block: validate actual entry and heartbeat again before RunTraj.
                fresh=self.snapshot()
                validate_goal(goal,fresh,self.engine,self.get_clock().now().nanoseconds,check_freshness=False)
                # Vendor MoveL example sets global ratios to 100% during playback
                # so firmware does not re-limit an already time-parameterized path.
                # Keep the measured low-speed limits for every trajectory check.
                limits=copy.deepcopy(self.engine.cartesian_rate_limits)
                restore={a:list(goal.controller_ratios[i*2:i*2+2]) for i,a in enumerate(plans)}
                self.active['restore_ratios']=restore
                self.active['playback_ratios']={a:[100.,100.] for a in plans}
                self.engine.set_arm_ratios(self.active['playback_ratios'])
                fresh=self.snapshot()
                validate_goal(goal,fresh,self.engine,self.get_clock().now().nanoseconds,
                              check_freshness=False,planning_limits=limits)
                if any(o['object_id'] in (0,1,3) and (o['state'] not in (0,1) or o['error_code']) for o in fresh['objects']):
                    raise ValueError('Controlled object changed mode/faulted before playback')
                self.engine.check_uploaded_trajectories(plans)
                self.check_playback_ratios({'controller':self.engine.robot.get_sg_dict()})
                self.engine.check_cancel()
                if not self.alive(goal): raise ValueError('Control lost during upload')
                mask=sum(1<<a for a in plans)
                if self.engine.robot.runtime_run_traj(mask)!=mask: raise RuntimeError('Partial trajectory start')
            self.active['started']=time.monotonic()
            self.status('running',task_id=goal.task_id,objects=list(plans))
        except Exception as exc:
            self.emergency_stop()
            self.failed=True
            self.status('fault',task_id=goal.task_id,reason=str(exc))
            self.finish(False,str(exc))

    def check_playback_ratios(self,raw):
        active=self.active
        for index,obj in enumerate(active['plans']):
            settings=raw['controller']['body']['set'] if obj==3 else raw['controller']['arms'][obj]['set']
            expected=active.get('playback_ratios',{}).get(obj,active['goal'].controller_ratios[index*2:index*2+2])
            for cap,key in zip(expected,('vel_ratio','acc_ratio')):
                ratio=float(settings[key])
                if not math.isfinite(ratio) or abs(ratio-cap)>1e-6:
                    raise RuntimeError('Controller ratios changed during execution')

    def emergency_stop(self):
        self.stop_confirmed=False
        self.engine.stop(force=True)
        self.stop_confirmed=True

    def tick(self):
        with self.sdk_lock:
            try:
                if self.stop_event.is_set():
                    self.emergency_stop()
                    self.stop_event.clear()
                    self.failed=True
                    task=self.active['goal'].task_id if self.active else self.reserved
                    self.status('stopped',task_id=task,reason='Stop latched; reset required')
                    self.finish(False,'Stopped')
                self.start_pending()
                raw=self.snapshot()
                self.latest=raw
                if self.active:
                    active=self.active
                    if not self.alive(active['goal']): raise RuntimeError('Control heartbeat timed out')
                    if time.monotonic()-active['started']>self.c.timeout: raise TimeoutError('Execution timeout')
                    if any(o['object_id'] in (0,1,3) and (o['state']==100 or o['error_code']) for o in raw['objects']):
                        raise RuntimeError('Controlled object fault during execution')
                    done=True
                    for obj,points in active['plans'].items():
                        if self.engine.robot.current_state(obj)!=1: raise RuntimeError(f'object{obj} left POSITION')
                        q=raw['body'] if obj==3 else raw['joints'][obj]
                        done=done and max(abs(x-y) for x,y in zip(q,points[-1]))<=self.c.tolerance
                        if obj==3:
                            position_error,orientation_error=pose_error(raw['body_matrix'],active['body_goal_matrix'])
                            active['body_pose_error']={'position_mm':position_error,'orientation_deg':orientation_error}
                            done=done and position_error<=self.c.body_position_tolerance_mm and orientation_error<=self.c.body_orientation_tolerance_deg
                        if obj!=3: done=done and raw['controller']['arms'][obj]['get']['traj_state']==0
                        if obj in active['lines']:
                            error,_=line_error(raw['matrices'][obj],*active['lines'][obj],self.c.sdk_length_to_m)
                            active['max_error'][obj]=max(active['max_error'][obj],error)
                            if error>self.c.cartesian_path_tolerance:
                                reason=f'arm{obj} TCP line error {error:.6f} m'
                                tracking=raw['tracking'][obj]
                                if tracking.get('available') and tracking.get('command_matrix') is not None:
                                    commanded,_=line_error(tracking['command_matrix'],*active['lines'][obj],self.c.sdk_length_to_m)
                                    following=max(abs(v) for v in tracking['following_error_deg'])
                                    reason+=f'; controller_command_line_error={commanded:.6f} m; max_joint_following_error={following:.4f} deg'
                                raise RuntimeError(reason)
                    self.check_playback_ratios(raw)
                    active['settled']=(active['settled'] or time.monotonic()) if done else None
                    if active['settled'] is not None and time.monotonic()-active['settled']>=.5:
                        self.engine.active=False
                        self.finish(True,'Completed',{'max_line_error_m':active['max_error'],
                                                      'body_pose_error':active.get('body_pose_error')})
                    elif self.active:
                        feedback=ExecuteTrajectory.Feedback(state='running',progress=-1.)
                        self.active['handle'].publish_feedback(feedback)
                if time.monotonic()-self.last_publish>=1/self.c.publish_rate:
                    self.last_publish=time.monotonic()
                    if self.context.ok(): self.raw_pub.publish(String(data=wire_json(raw)))
            except Exception as exc:
                try:
                    if not self.stop_confirmed or self.engine.active:self.emergency_stop()
                except Exception as stop_exc: self.get_logger().error(str(stop_exc))
                self.failed=True
                if 'raw' in locals() and self.context.ok():
                    self.raw_pub.publish(String(data=wire_json(raw)))
                task=self.active['goal'].task_id if self.active else None
                if self.last_fault!=str(exc) or self.active:
                    self.status('fault',task_id=task,reason=str(exc))
                self.last_fault=str(exc)
                self.finish(False,str(exc))
                # Keep servicing stop requests and collecting feedback while motion stays latched.

    def close(self):
        self.timer.cancel()
        self.stop_event.set()
        with self.sdk_lock:
            try:
                if self.engine.active:self.emergency_stop()
            finally:
                try:self.finish(False,'Driver shutting down')
                finally:
                    try:self.engine.close()
                    finally:
                        with self.lock:
                            if self.pending:
                                handle,future=self.pending
                                try:handle.abort()
                                except Exception:pass
                                future.set_result(ExecuteTrajectory.Result(success=False,message='Driver shutting down',details='{}'))
                                self.pending=None



def main(args=None):
    rclpy.init(args=args)
    node=None
    executor=MultiThreadedExecutor(num_threads=3)
    try:
        node=Bridge()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        if node:
            try:node.close()
            except Exception as exc:node.get_logger().error(f'Shutdown could not confirm safe state: {exc}')
        executor.shutdown()
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
