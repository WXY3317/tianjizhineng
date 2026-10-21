# 维护躯干六关节和头部两关节的限位与插值状态，供虚拟控制器推进运动。
"""Bounded torso/head joint interpolation; no vendor hardware commands."""
from pathlib import Path
import time

import numpy as np
import yaml


class Body:
    def __init__(self, model_root, now=time.monotonic):
        config=yaml.safe_load((Path(model_root)/'config/body.yaml').read_text())
        self.joints=config['joints']
        self.names=[j['name'] for j in self.joints]
        self.limits=np.array([j['range'] for j in self.joints])
        self.q=np.array([j['initial'] for j in self.joints],dtype=float)
        self.target=self.q.copy()
        self.velocity=np.zeros_like(self.q)
        self.now=now
        self.move=None

    def set_target(self, names, positions, speed=.5, acceleration=.5):
        if (not names or len(set(names))!=len(names) or len(names)!=len(positions)
                or not set(names)<=set(self.names)):
            raise ValueError('Provide unique known torso/head joint names and matching positions')
        target=self.target.copy()
        for name,value in zip(names,positions):target[self.names.index(name)]=value
        if (not np.isfinite(target).all() or np.any(target<self.limits[:,0])
                or np.any(target>self.limits[:,1])):
            raise ValueError('Torso/head target must be finite and inside source joint limits')
        distance=float(np.max(np.abs(target-self.q)))
        duration=max(.5,1.875*distance/speed, float(np.sqrt(5.774*distance/acceleration)))
        self.target=target.copy()
        self.move=(self.now(),duration,self.q.copy(),target)

    def tick(self):
        if self.move is None:return
        started,duration,start,target=self.move
        u=min(1.,max(0.,(self.now()-started)/duration))
        self.q=start+(target-start)*(10*u**3-15*u**4+6*u**5)
        self.velocity=(target-start)*(30*u**2-60*u**3+30*u**4)/duration
        if u==1.:self.move=None
