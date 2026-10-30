"""Own motion tasks, Cartesian/joint planning, action execution and cancellation."""
import json
import math
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String, Empty
from gento_interfaces.srv import DriverQuery
from gento_interfaces.action import ExecuteTrajectory
from .conversions import JOINT_NAMES,BODY_JOINT_NAMES,joint_target,body_target,pose_matrix,quaternion
from .planning import Planner


RECOVERY_TIMEOUT = 60.

def as_pose(matrix,scale):
    pose=Pose()
    pose.position.x,pose.position.y,pose.position.z=[matrix[i][3]*scale for i in range(3)]
    pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=quaternion(matrix)
    return pose


class Control(Node):
    def __init__(self):
        super().__init__('gento_control')
        self.group=ReentrantCallbackGroup()
        self.rpc_client=self.create_client(DriverQuery,'/gento_driver/query',callback_group=self.group)
        self.action=ActionClient(self,ExecuteTrajectory,'/gento_driver/execute',callback_group=self.group)
        self.heartbeat_pub=self.create_publisher(String,'/gento_driver/heartbeat',1)
        self.stop_pub=self.create_publisher(Empty,'/gento_driver/stop',1)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub=self.create_publisher(String,'/gento_control/status',latched)
        self.plan_pub=self.create_publisher(String,'/gento_control/plan',latched)
        self.lock=threading.Lock()
        self.cancel=threading.Event()
        self.busy=False
        self.recovering=False
        self.closing=threading.Event()
        self.session=''
        self.command_max_age=None
        self.task_id=''
        self.accept_after=self.get_clock().now().nanoseconds
        self.worker=ThreadPoolExecutor(max_workers=1,thread_name_prefix='motion-planner')
        self.subs=[]
        prefix='/gento_control'
        for arm,side in enumerate(('left','right')):
            self.subs.append(self.create_subscription(PoseStamped,f'{prefix}/{side}/target_pose',
                lambda msg,a=arm:self.submit('pose',a,msg),1))
            self.subs.append(self.create_subscription(JointState,f'{prefix}/{side}/target_joints',
                lambda msg,a=arm:self.submit('joints',a,msg),1))
        self.subs.append(self.create_subscription(JointState,f'{prefix}/body/target_joints',lambda m:self.submit('body',3,m),1))
        self.subs.append(self.create_subscription(PoseStamped,f'{prefix}/body/target_pose',lambda m:self.submit('body_pose',3,m),1))
        self.subs.append(self.create_subscription(Empty,f'{prefix}/reset_error',lambda _:self.reset(),1))
        for obj,side in ((0,'left'),(1,'right'),(3,'body')):
            self.subs.append(self.create_subscription(Empty,f'{prefix}/{side}/reset_error',lambda _,o=obj:self.reset(o),1))
        # Observe the single stop topic to cancel planning immediately; only driver calls SDK stop.
        self.subs.append(self.create_subscription(Empty,'/gento_driver/stop',lambda _:self.cancel.set(),1,callback_group=self.group))
        self.create_subscription(String,'/gento_driver/status',self.driver_status,latched,callback_group=self.group)
        self.timer=self.create_timer(.1,self.heartbeat,callback_group=self.group)

    def status(self,state,**details):
        if self.context.ok(): self.status_pub.publish(String(data=json.dumps({'state':state,'session_id':self.session,'task_id':self.task_id,**details})))
        # Driver reports actual execution; keep control topics without duplicate console logs.
        if state=='running' or (state=='completed' and details.get('terminal')):return
        if state in ('fault','rejected'): self.get_logger().warning(state+': '+str(details.get('reason','')))
        else: self.get_logger().info(state)

    def driver_status(self,msg):
        try:
            value=json.loads(msg.data)
            with self.lock:
                if (self.busy and not self.recovering and value['state'] in ('fault','stopped')
                        and value.get('session_id')==self.session
                        and value.get('task_id') in (None,self.task_id)):
                    self.cancel.set()
        except (KeyError,ValueError,TypeError):pass

    def heartbeat(self):
        if self.context.ok() and self.busy and not self.recovering and self.session:
            self.heartbeat_pub.publish(String(data=json.dumps({'session_id':self.session,'task_id':self.task_id,'stamp_ns':self.get_clock().now().nanoseconds})))

    def wait(self,future,timeout=5.,interruptible=True):
        complete=threading.Event()
        future.add_done_callback(lambda _:complete.set())
        deadline=None if timeout is None else time.monotonic()+timeout
        while not complete.wait(.02):
            if not self.context.ok() or self.closing.is_set(): raise RuntimeError('ROS shutting down')
            if deadline is not None and time.monotonic()>deadline: raise TimeoutError('Driver RPC/action timeout')
            if interruptible and self.cancel.is_set(): raise ValueError('Task cancelled')
        return future.result()

    def rpc(self,operation,payload=None):
        if not self.rpc_client.wait_for_service(timeout_sec=2.): raise RuntimeError('Driver unavailable')
        future=self.rpc_client.call_async(DriverQuery.Request(
            session_id=self.session,operation=operation,payload=json.dumps(payload or {})))
        if operation in ('reset','object_reset'):
            try:
                # A full reset can contain several sequential 3-second SDK waits.
                response=self.wait(future,timeout=RECOVERY_TIMEOUT,interruptible=False)
            except TimeoutError:
                self.stop()
                self.status('recovering',reason='Recovery timeout; stop requested; awaiting driver confirmation')
                # RPC timeout does not cancel work in the driver. Keep the task slot
                # reserved until the reply arrives, or until this node shuts down.
                response=self.wait(future,timeout=None,interruptible=False)
        else:response=self.wait(future)
        if not response.success: raise ValueError(response.error)
        if operation in ('snapshot','reset','object_reset'): self.session=response.session_id
        result=json.loads(response.result or 'null')
        if operation=='snapshot':self.command_max_age=result['config']['command_max_age']
        return result

    def check_stamp(self,msg):
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        now=self.get_clock().now().nanoseconds
        if (stamp<=self.accept_after or now<stamp or
                (self.command_max_age is not None and now-stamp>self.command_max_age*10**9)):
            raise ValueError('Stale/future target; send a fresh stamped command after completion')

    def submit(self,kind,arm,msg):
        try:
            self.check_stamp(msg)
            with self.lock:
                if self.busy: raise ValueError('Busy; commands are not queued')
                self.busy=True
                self.cancel.clear()
                self.task_id=uuid.uuid4().hex
            self.worker.submit(self.run_task,kind,arm,msg)
        except Exception as exc: self.status('rejected',reason=str(exc))

    def run_task(self,kind,arm,msg):
        handle=None
        submitted=False
        try:
            body_task=kind in ('body','body_pose')
            snapshot_payload={'include_arm_limits':False} if body_task else None
            snapshot=self.rpc('snapshot',snapshot_payload)
            self.check_stamp(msg)
            if not snapshot['config']['enable_control']: raise ValueError('Driver is feedback-only')
            if not snapshot['ready']: raise ValueError('Driver fault is latched; reset required')
            if snapshot['active_task']:raise ValueError('Driver busy; commands are not queued')
            self.heartbeat()
            planner=Planner(snapshot,self.rpc,self.cancel)
            lines=[]
            goals=snapshot['matrices']
            if kind=='pose':
                expected=snapshot['config']['left_frame' if arm==0 else 'right_frame']
                if msg.header.frame_id!=expected: raise ValueError(f'Expected frame {expected}; no TF conversion')
                plans,goals,info=planner.cartesian(arm,pose_matrix(msg.pose,planner.c.sdk_length_to_m))
                lines=list(plans)
            elif kind=='joints':
                if msg.velocity or msg.effort: raise ValueError('Position targets only')
                plans,info=planner.joint(arm,joint_target(arm,msg.name,msg.position))
            elif kind=='body_pose':
                expected=snapshot['config']['body_frame']
                if msg.header.frame_id!=expected:raise ValueError(f'Expected frame {expected}; no TF conversion')
                # The torso model always uses mm, independently of arm SDK units.
                plans,info=planner.body_pose(pose_matrix(msg.pose,.001))
            else:
                if msg.velocity or msg.effort: raise ValueError('Position targets only')
                plans={3:[body_target(msg.name,msg.position)]}
                info={}
            planner.check_cancel()
            if not plans:
                self.status('completed',reason='Already at target')
                return
            if not self.action.wait_for_server(timeout_sec=2.): raise RuntimeError('Driver execution action unavailable')
            goal=ExecuteTrajectory.Goal(session_id=self.session,task_id=self.task_id)
            latest=self.rpc('snapshot',snapshot_payload)
            # Confirm the driver has seen this task's heartbeat before submitting an action.
            heartbeat_deadline=time.monotonic()+3.
            while latest.get('heartbeat_task_id')!=self.task_id:
                planner.check_cancel()
                if time.monotonic()>heartbeat_deadline:raise TimeoutError('Driver did not confirm control heartbeat')
                self.heartbeat()
                time.sleep(.02)
                latest=self.rpc('snapshot',snapshot_payload)
            if latest['session_id']!=snapshot['session_id']: raise ValueError('Driver session changed; replan')
            goal.start_state.name=JOINT_NAMES+BODY_JOINT_NAMES
            goal.start_state.position=[math.radians(q) for q in snapshot['joints'][0]+snapshot['joints'][1]+snapshot['body']]
            goal.start_state.header.stamp.sec=latest['stamp']['sec']
            goal.start_state.header.stamp.nanosec=latest['stamp']['nanosec']
            for obj,path in plans.items():
                names=BODY_JOINT_NAMES if obj==3 else JOINT_NAMES[obj*7:(obj+1)*7]
                trajectory=JointTrajectory(joint_names=names)
                for index,q in enumerate(path):
                    point=JointTrajectoryPoint(positions=[math.radians(v) for v in q])
                    point.time_from_start.sec=index//50
                    point.time_from_start.nanosec=(index%50)*20000000
                    trajectory.points.append(point)
                goal.trajectories.append(trajectory)
                settings=snapshot['controller']['body']['set'] if obj==3 else planner.cartesian_rate_limits[obj]
                goal.controller_ratios.extend([settings['vel_ratio'],settings['acc_ratio']])
            goal.line_objects=lines
            goal.line_starts=[as_pose(snapshot['matrices'][a],planner.c.sdk_length_to_m) for a in lines]
            goal.line_goals=[as_pose(goals[a],planner.c.sdk_length_to_m) for a in lines]
            self.plan_pub.publish(String(data=json.dumps({'task_id':self.task_id,'session_id':self.session,
                'plans':{str(a):q for a,q in plans.items()},'starts':snapshot['matrices'],'goals':goals,
                'config':snapshot['config'],'plan_info':info,'line_objects':lines})))
            planner.check_cancel()
            submitted=True
            handle=self.wait(self.action.send_goal_async(goal))
            if not handle.accepted: raise ValueError('Driver rejected execution goal')
            submitted=True
            self.status('running')
            # Continue heartbeat while the task worker waits; the timer is in another callback group.
            result=self.wait(handle.get_result_async(),timeout=planner.c.timeout+10.).result
            self.status('completed' if result.success else 'fault',reason=result.message,terminal=True)
        except Exception as exc:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
                if self.context.ok():self.stop_pub.publish(Empty())
            elif submitted:
                if self.context.ok():self.stop_pub.publish(Empty())
            self.status('stopped' if self.cancel.is_set() else ('rejected' if not submitted else 'fault'),reason=str(exc),terminal=True)
        finally:
            with self.lock:
                self.accept_after=self.get_clock().now().nanoseconds
                self.busy=False

    def stop(self,_=None):
        self.cancel.set()
        if self.context.ok():self.stop_pub.publish(Empty())

    def reset(self,obj=None):
        with self.lock:
            if self.busy:
                self.status('rejected',reason='Stop and wait for task termination before reset')
                return
            self.busy=True
            self.recovering=True
            self.task_id=uuid.uuid4().hex
            self.cancel.clear()
        def recover():
            try:
                self.rpc('snapshot')
                self.rpc('reset' if obj is None else 'object_reset',{} if obj is None else {'object':obj})
            except Exception as exc: self.status('fault',reason=str(exc))
            finally:
                with self.lock:
                    self.accept_after=self.get_clock().now().nanoseconds
                    self.recovering=False
                    self.busy=False
        self.worker.submit(recover)

    def close(self):
        self.closing.set()
        if self.busy: self.stop()
        self.worker.shutdown(wait=True)


def main(args=None):
    rclpy.init(args=args)
    node=None
    executor=MultiThreadedExecutor(num_threads=3)
    try:
        node=Control()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        if node: node.close()
        executor.shutdown()
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
