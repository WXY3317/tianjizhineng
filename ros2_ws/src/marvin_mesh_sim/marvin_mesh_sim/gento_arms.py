"""Pure joint interpolation for the virtual Gento arm controller (no old SDK)."""
import time
import numpy as np

from marvin_sim.kinematics import INITIAL_DEG
from marvin_sim.local_address import local_address
from marvin_sim.controller import VirtualController


class GentoArms:
    port=3721
    feedback_port=3721
    _start=VirtualController._start
    stop=VirtualController.stop

    def __init__(self,kine,sim_ip='',now=time.monotonic):
        self.kine,self.now=kine,now;self.address=local_address(sim_ip)
        self.q=np.radians(INITIAL_DEG).ravel();self.v=np.zeros(14)
        self.target=self.q.copy();self.moves=[None,None];self.modes=[0,0]
        self.ratios=[[10.,10.],[10.,10.]]
        self.received=0;self.rejected=0;self.last_error=''

    def tick(self):
        for arm,move in enumerate(self.moves):
            if move is None:continue
            started,duration,start,goal=move
            u=np.clip((self.now()-started)/duration,0.,1.)
            s=slice(arm*7,arm*7+7)
            self.q[s]=start+(goal-start)*(10*u**3-15*u**4+6*u**5)
            self.v[s]=(goal-start)*(30*u**2-60*u**3+30*u**4)/duration
            if u>=1.:self.moves[arm]=None;self.v[s]=0.

    def close(self):pass
