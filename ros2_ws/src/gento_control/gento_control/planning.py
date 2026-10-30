"""Motion planning lives here; SDK computations are read-only RPC calls."""
import math
from types import SimpleNamespace
from .cartesian import prepare_cartesian
from .conversions import finite


def validate_target(start,target,limits,max_delta):
    if len(target)!=7: raise ValueError('Expected seven joints')
    finite(target)
    for index,(a,b,(lo,hi)) in enumerate(zip(start,target,limits)):
        if not lo<=b<=hi or abs(b-a)>max_delta:
            raise ValueError(f'joint{index+1} exceeds limits/max_delta')


class Planner:
    def __init__(self,snapshot,rpc,cancel):
        self.snapshot=snapshot
        self.rpc=rpc
        self.cancel=cancel
        self.c=SimpleNamespace(**snapshot['config'])
        self.robot=SimpleNamespace(max_point=5000)
        self.h=SimpleNamespace(validate_target=validate_target)
        self.limits=snapshot['limits']
        self.cartesian_rate_limits={int(a):v for a,v in snapshot.get('controller_limits',{}).items()}
        self.on_retime=None

    def check_cancel(self):
        if self.cancel.is_set(): raise ValueError('Planning cancelled')

    def read_cartesian_rate_limits(self,moving):
        return {a:self.cartesian_rate_limits[a] for a in moving}

    def solve(self,matrices,starts,moving=(0,1)):
        self.check_cancel()
        result=self.rpc('ik',{'matrices':matrices,'starts':starts,'moving':list(moving)})
        self.check_cancel()
        return result

    def cartesian(self,arm,matrix):
        goals=[m for m in self.snapshot['matrices']]
        goals[arm]=matrix
        targets,plans,info=prepare_cartesian(self,self.snapshot['joints'],self.snapshot['matrices'],goals)
        return {a:[list(points[i*7:(i+1)*7]) for i in range(count)] for a,(points,count) in plans.items()},goals,info

    def body_pose(self,matrix):
        self.check_cancel()
        result=self.rpc('body_ik',{'matrix':matrix,'reference':self.snapshot['body']})
        self.check_cancel()
        if not result['success'] or result['joints_deg'] is None:
            raise ValueError('Body IK returned no feasible solution')
        joints=result['joints_deg']
        if len(joints)!=6:raise ValueError('Body IK must return six joints')
        finite(joints)
        for i,(q,start,(lo,hi)) in enumerate(zip(joints,self.snapshot['body'],self.snapshot['body_limits'])):
            if not lo<=q<=hi or not start-self.c.max_delta<=q<=start+self.c.max_delta:
                raise ValueError(f'Body joint{i+1} exceeds limits/max_delta')
        return {3:[joints]},{'body_target_matrix':matrix,'body_ik':result}

    def joint(self,arm,target):
        start=self.snapshot['joints'][arm]
        validate_target(start,target,self.limits[arm],self.c.max_delta)
        limits=self.cartesian_rate_limits[arm]
        duration=max([1.]+[1.875*abs(b-a)/v for a,b,v in zip(start,target,limits['speed'])]
                     +[math.sqrt(5.774*abs(b-a)/v) for a,b,v in zip(start,target,limits['acceleration'])]) * 1.02
        steps=math.ceil(duration*50)
        if steps+1>5000 or steps/50+1>=self.c.timeout: raise ValueError('Joint trajectory exceeds timeout/capacity')
        path=[]
        for index in range(steps+1):
            self.check_cancel()
            t=index/steps
            s=t*t*t*(10+t*(-15+6*t))
            path.append([a+s*(b-a) for a,b in zip(start,target)])
        return {arm:path},{'planned_duration_s':steps/50}
