"""Aggregate raw state, diagnose path errors and own all motion recording."""
import json
from collections import OrderedDict
import math
import time
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Empty
from .records import save_record

# Pure geometry lives here, independently of the driver safety implementation.
def segment_error(matrix,begin,end,scale):
    vector=[end[i][3]-begin[i][3] for i in range(3)]
    offset=[matrix[i][3]-begin[i][3] for i in range(3)]
    length=sum(v*v for v in vector)
    t=min(1.,max(0.,sum(a*b for a,b in zip(vector,offset))/length)) if length>1e-12 else 0.
    delta=[(a-t*b)*scale for a,b in zip(offset,vector)]
    return math.sqrt(sum(x*x for x in delta)),delta


def quaternion(m):
    # Largest-component formulation, including rotations near 180 degrees.
    candidates=[1+m[0][0]+m[1][1]+m[2][2],1+m[0][0]-m[1][1]-m[2][2],
                1-m[0][0]+m[1][1]-m[2][2],1-m[0][0]-m[1][1]+m[2][2]]
    k=max(range(4),key=candidates.__getitem__)
    s=2*math.sqrt(max(candidates[k],0.))
    if s<1e-12: raise ValueError('Invalid FK rotation')
    if k==0:q=[(m[2][1]-m[1][2])/s,(m[0][2]-m[2][0])/s,(m[1][0]-m[0][1])/s,s/4]
    elif k==1:q=[s/4,(m[0][1]+m[1][0])/s,(m[0][2]+m[2][0])/s,(m[2][1]-m[1][2])/s]
    elif k==2:q=[(m[0][1]+m[1][0])/s,s/4,(m[1][2]+m[2][1])/s,(m[0][2]-m[2][0])/s]
    else:q=[(m[0][2]+m[2][0])/s,(m[1][2]+m[2][1])/s,s/4,(m[1][0]-m[0][1])/s]
    norm=math.sqrt(sum(v*v for v in q))
    return [v/norm for v in q]


class Feedback(Node):
    def __init__(self):
        super().__init__('gento_feedback')
        self.declare_parameter('trajectory_log_dir','~/.ros/gento_trajectories')
        self.trace=None
        self.task_statuses=OrderedDict()
        self.last_raw=None
        self.last_receive=time.monotonic()
        self.warned=False
        self.writer=ThreadPoolExecutor(max_workers=1,thread_name_prefix='feedback-record')
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        prefix='/gento_feedback'
        self.output_publishers={}
        for suffix in ('joint_states','body/joint_states','head/joint_states','full_joint_states'):
            self.output_publishers[suffix]=self.create_publisher(JointState,f'{prefix}/{suffix}',1)
        for suffix in ('left/tcp_pose','right/tcp_pose','body/tcp_pose'):
            self.output_publishers[suffix]=self.create_publisher(PoseStamped,f'{prefix}/{suffix}',1)
        for suffix in ('status','joint_errors'):
            self.output_publishers[suffix]=self.create_publisher(String,f'{prefix}/{suffix}',latched)
        self.stop_pub=self.create_publisher(Empty,'/gento_driver/stop',1)
        self.create_subscription(String,'/gento_driver/raw_state',self.on_raw,1)
        self.create_subscription(String,'/gento_control/plan',self.on_plan,latched)
        self.create_subscription(String,'/gento_driver/status',self.on_status,latched)
        self.create_subscription(String,'/gento_control/status',self.on_control_status,latched)
        self.timer=self.create_timer(.1,self.maintenance)

    def publish(self,suffix,message):
        self.output_publishers[suffix].publish(message)

    def on_plan(self,msg):
        try:
            plan=json.loads(msg.data)
            if self.trace: self.flush('superseded')
            self.trace={'schema_version':2,'created_ns':time.time_ns(),'plan':plan,'samples':[],
                        'dropped_samples':0,'terminal':None,'received_at':time.monotonic(),'running':False,'executing':False}
            cached=self.task_statuses.get((plan['session_id'],plan['task_id']),{})
            for source in ('driver','control'):
                if source in cached:self.apply_task_status(*cached[source],source)
        except Exception as exc: self.get_logger().error(f'Invalid plan record: {exc}')

    def apply_task_status(self,value,received_at,source):
        if not self.trace:return
        plan=self.trace['plan']
        if (value.get('session_id'),value.get('task_id'))!=(plan['session_id'],plan['task_id']):return
        state=value.get('state')
        if source=='driver':
            if state in ('validating','uploading','running'):self.trace['running']=True
            if state=='running':self.trace['executing']=True
            if state in ('completed','fault','stopped'):
                self.trace['terminal']=(received_at+.2,value)
        elif value.get('terminal') and not self.trace['running'] and not self.trace['terminal']:
            self.trace['terminal']=(received_at+.2,value)

    def remember_status(self,msg,source):
        self.publish('status',msg)
        try:
            value=json.loads(msg.data)
            session,task=value.get('session_id'),value.get('task_id')
            if not isinstance(session,str) or not session or not isinstance(task,str) or not task:return
            key=(session,task)
            received_at=time.monotonic()
            cached=self.task_statuses.setdefault(key,{})
            cached[source]=(value,received_at)
            self.task_statuses.move_to_end(key)
            while len(self.task_statuses)>128:self.task_statuses.popitem(last=False)
            self.apply_task_status(value,received_at,source)
        except (ValueError,KeyError,TypeError,AttributeError):pass

    def on_status(self,msg):
        self.remember_status(msg,'driver')

    def on_control_status(self,msg):
        self.remember_status(msg,'control')

    def on_raw(self,msg):
        try:
            raw=json.loads(msg.data)
            self.last_raw=raw
            self.last_receive=time.monotonic()
            self.warned=False
            stamp=raw['stamp']
            def joints(names,positions):
                message=JointState(name=names,position=[math.radians(q) for q in positions])
                message.header.stamp.sec=stamp['sec'];message.header.stamp.nanosec=stamp['nanosec']
                return message
            arms=[f'{s}_joint_{i}' for s in ('left','right') for i in range(1,8)]
            body=[f'body_joint_{i}' for i in range(1,7)]
            head=[f'head_joint_{i}' for i in range(1,len(raw['head'])+1)]
            self.publish('joint_states',joints(arms,raw['joints'][0]+raw['joints'][1]))
            self.publish('body/joint_states',joints(body,raw['body']))
            self.publish('head/joint_states',joints(head,raw['head']))
            self.publish('full_joint_states',joints(head+body+arms,raw['head']+raw['body']+raw['joints'][0]+raw['joints'][1]))
            scale=raw['config']['sdk_length_to_m']
            for arm,side in enumerate(('left','right')):
                pose=PoseStamped()
                pose.header.stamp.sec=stamp['sec'];pose.header.stamp.nanosec=stamp['nanosec']
                pose.header.frame_id=raw['config'][f'{side}_frame']
                matrix=raw['matrices'][arm]
                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=[matrix[i][3]*scale for i in range(3)]
                pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=quaternion(matrix)
                self.publish(f'{side}/tcp_pose',pose)
            if raw.get('body_matrix') is not None:
                pose=PoseStamped()
                pose.header.stamp.sec=stamp['sec'];pose.header.stamp.nanosec=stamp['nanosec']
                pose.header.frame_id=raw['config']['body_frame']
                matrix=raw['body_matrix']
                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=[matrix[i][3]*.001 for i in range(3)]
                pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=quaternion(matrix)
                self.publish('body/tcp_pose',pose)
            errors={'available':True,'objects':raw['objects'],'stamp':stamp,
                    'fault_objects':[o['object_id'] for o in raw['objects'] if o['error_code'] or o['state']==100]}
            self.publish('joint_errors',String(data=json.dumps(errors)))
            if self.trace and raw['session_id']==self.trace['plan']['session_id'] and (raw.get('active_task')==self.trace['plan']['task_id'] or
                    (self.trace['terminal'] and raw.get('active_task') is None)):
                sample={k:raw[k] for k in ('stamp','sequence','joints','body','head','matrices','controller')}
                if 'body_matrix' in raw:sample['body_matrix']=raw['body_matrix']
                sample['tracking']=raw.get('tracking',[])
                sample['line_errors']={}
                for arm in self.trace['plan'].get('line_objects',[]):
                    error,vector=segment_error(raw['matrices'][arm],self.trace['plan']['starts'][arm],self.trace['plan']['goals'][arm],scale)
                    sample['line_errors'][str(arm)]={'distance_m':error,'vector_m':vector}
                    tracking=sample['tracking'][arm] if len(sample['tracking'])>arm else {}
                    command_matrix=tracking.get('command_matrix') if tracking.get('available') else None
                    if command_matrix is not None:
                        commanded,_=segment_error(command_matrix,self.trace['plan']['starts'][arm],self.trace['plan']['goals'][arm],scale)
                        sample['line_errors'][str(arm)]['command_distance_m']=commanded
                        sample['line_errors'][str(arm)]['tcp_following_error_m']=math.sqrt(sum(
                            ((raw['matrices'][arm][i][3]-command_matrix[i][3])*scale)**2 for i in range(3)))
                    if error>raw['config']['cartesian_path_tolerance'] and not self.trace['terminal']:
                        self.stop_pub.publish(Empty())
                if len(self.trace['samples'])<20000:self.trace['samples'].append(sample)
                else:self.trace['dropped_samples']+=1
                self.trace['last_sample']=sample
        except Exception as exc:
            self.get_logger().error(f'Feedback decoding/diagnosis failed: {exc}')

    def maintenance(self):
        if self.trace and self.trace['terminal'] and time.monotonic()>=self.trace['terminal'][0]:
            self.flush(self.trace['terminal'][1]['state'])
        elif self.trace and not self.trace['running'] and time.monotonic()-self.trace['received_at']>10:
            self.flush('not_started')
        timeout=max(1.,3./self.last_raw['config'].get('publish_rate',20.)) if self.last_raw else 1.
        if self.last_raw and time.monotonic()-self.last_receive>timeout and not self.warned:
            self.warned=True
            self.publish('joint_errors',String(data=json.dumps({'available':False,'reason':'Driver feedback timeout'})))
            if self.trace and self.trace['executing'] and not self.trace['terminal']:self.stop_pub.publish(Empty())

    def flush(self,outcome):
        record=self.trace
        self.trace=None
        if not record:return
        # A confirmed terminal event takes precedence over rollover or shutdown.
        record['outcome']=record['terminal'][1]['state'] if record['terminal'] else outcome
        outcome=record['outcome']
        record.pop('received_at',None)
        directory=self.get_parameter('trajectory_log_dir').value
        if not directory:return
        future=self.writer.submit(save_record,directory,record)
        def saved(done):
            try:
                path=done.result()
                if outcome in ('fault','stopped'):self.get_logger().info('Trajectory diagnostic: '+path)
            except Exception as exc:self.get_logger().error(f'Trajectory log failed: {exc}')
        future.add_done_callback(saved)

    def close(self):
        self.timer.cancel()
        self.flush('feedback_shutdown')
        self.writer.shutdown(wait=True)


def main(args=None):
    rclpy.init(args=args)
    node=None
    try:
        node=Feedback()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):pass
    finally:
        if node:node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
