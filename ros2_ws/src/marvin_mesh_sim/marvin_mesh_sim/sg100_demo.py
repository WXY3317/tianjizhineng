"""Move SG100 fingers via original vendor SDK APIs and inspect SDK feedback."""
import argparse
import time

import numpy as np
from ament_index_python.packages import get_package_share_directory

from .sg100_client import SG100Client


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk-root',required=True)
    parser.add_argument('--sim-ip',default='')
    parser.add_argument('--hand',choices=['L','R'],default='L')
    args=parser.parse_args()
    client=SG100Client(args.sdk_root,get_package_share_directory('marvin_mesh_sim'),args.sim_ip)
    try:
        deadline=time.monotonic()+3
        while True:
            try:state=client.read();break
            except RuntimeError:
                if time.monotonic()>deadline:raise
                time.sleep(.01)
        goal=np.degrees(state[args.hand]);goal[8]=min(goal[8]+5.,30.)
        # Public vendor naming: FINGER_RING consists of Joint 9,10,11.
        # Validate first using the same bounds as the simulation model.
        bounds=np.degrees(client.limits[args.hand])
        if np.any(goal<bounds[:,0]) or np.any(goal>bounds[:,1]):raise ValueError('Demo target outside model limits')
        client.devices[args.hand].SetJointPositions(goal)
        client.flush()  # transport actual SDK-produced CANFD fragments to local palm
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            actual=np.degrees(client.read()[args.hand])
            if np.max(np.abs(actual-goal))<1e-3:
                print(f'Original SG100 SDK {args.hand}: Joint9 (FINGER_RING) reached {actual[8]:.3f} degrees')
                return
            time.sleep(.01)
        raise RuntimeError('Original SG100 feedback did not reach target')
    finally:client.close()
