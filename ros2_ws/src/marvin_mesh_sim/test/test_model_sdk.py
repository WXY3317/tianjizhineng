import hashlib
import json
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from marvin_sim.kinematics import INITIAL_DEG,pose_error
from marvin_sim.controller import VirtualController
from marvin_sim.local_address import local_address
from marvin_mesh_sim.kinematics import MeshKinematics
from marvin_mesh_sim.hands import Hands,closed_posture
from marvin_mesh_sim.sdk import MeshSimulationSDK


ROOT=Path(__file__).resolve().parents[4]
PACKAGE=ROOT/'ros2_ws/src/marvin_mesh_sim'


@pytest.fixture(scope='module')
def kine():
    return MeshKinematics(ROOT/'tianji_armSDK',PACKAGE)


def urdf_fk(root,base,tip,values):
    joints={j.find('child').get('link'):j for j in root.findall('joint')}
    chain=[]
    while tip!=base:
        j=joints[tip];chain.append(j);tip=j.find('parent').get('link')
    result=np.eye(4)
    for j in reversed(chain):
        o=j.find('origin')
        r,p,y=map(float,o.get('rpy','0 0 0').split())
        cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
        t=np.eye(4)
        t[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                   [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
        t[:3,3]=list(map(float,o.get('xyz','0 0 0').split()))
        result=result@t
        if j.get('type')=='revolute':
            axis=np.array(list(map(float,j.find('axis').get('xyz').split())))
            axis/=np.linalg.norm(axis)
            x,y,z=axis;k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
            angle=values.get(j.get('name'),0.)
            t=np.eye(4);t[:3,:3]=np.eye(3)+np.sin(angle)*k+(1-np.cos(angle))*(k@k)
            result=result@t
    return result


def test_model_tree_assets_and_joint_counts():
    root=ET.parse(PACKAGE/'urdf/marvin_mesh_hands.urdf').getroot()
    links={v.get('name') for v in root.findall('link')}
    children={v.find('child').get('link') for v in root.findall('joint')}
    assert links-children=={'world'}
    moving=[j for j in root.findall('joint') if j.get('type')=='revolute']
    assert len(moving)==36
    assert len([j for j in moving if j.get('name').startswith('Joint')])==14
    report=json.loads((PACKAGE/'config/asset_report.json').read_text())
    assert report['mesh_count']+report['empty_mesh_count']==38
    referenced=root.findall('.//geometry/mesh')
    assert len(referenced)==report['mesh_count']
    for asset,entry in report['assets'].items():
        if entry['status']=='mesh':
            path=PACKAGE/'meshes'/Path(asset).relative_to('assets/meshes')
            assert path.stat().st_size>84
            assert hashlib.sha256(path.read_bytes()).hexdigest()==entry['sha256']
        else:
            assert 'MJCF collision box' in entry['fallback']
    for j in moving:
        urdf_fk(root,'world',j.find('child').get('link'),{})


@pytest.mark.parametrize('arm,side',[(0,'L'),(1,'R')])
def test_actual_sdk_matches_generated_model_and_ik(kine,arm,side):
    root=ET.parse(PACKAGE/'urdf/marvin_mesh_hands.urdf').getroot()
    rng=np.random.default_rng(42+arm)
    for index in range(15):
        q=np.radians(INITIAL_DEG[arm])+rng.uniform(-.08,.08,7)
        actual=kine.fk(arm,q)
        expected=urdf_fk(root,f'Base_{side}',f'flange_{side}',
                         dict(zip([f'Joint{i}_{side}' for i in range(1,8)],q)))
        dp,dr=pose_error(actual,expected)
        assert dp<1e-6
        assert dr<2e-5
        if index<3:
            solved=kine.ik(arm,actual,np.radians(INITIAL_DEG[arm]))
            dp,dr=pose_error(kine.fk(arm,solved),actual)
            assert dp<1e-5 and dr<1e-4
    assert abs(kine.fk(arm,[0.]*7)[2,3]-.615)<1e-7


def test_hand_limits_atomicity_and_interpolation():
    clock=[0.]
    hands=Hands(PACKAGE,local_address(),port=0,now=lambda:clock[0])
    try:
        target=closed_posture(hands.names['L'])
        hands.set_target('L',target)
        clock[0]=.5;hands.tick()
        assert np.max(np.abs(hands.q['L']))>0
        before=hands.target['L'].copy()
        for bad in ([float('nan')]*11,[100.]*11,[0.]*10):
            with pytest.raises(ValueError):hands.set_target('L',bad)
            assert np.array_equal(hands.target['L'],before)
        clock[0]=3.;hands.tick()
        assert np.allclose(hands.q['L'],target)
        assert np.array_equal(hands.q['R'],np.zeros(11))
    finally:hands.close()


def test_original_sdk_arm_and_hand_api_end_to_end(kine):
    controller=VirtualController(kine)
    hands=Hands(PACKAGE,controller.address)
    stop=threading.Event();errors=[]
    def run():
        try:
            while not stop.wait(.002):controller.tick();hands.tick()
        except BaseException as exc:errors.append(exc)
    thread=threading.Thread(target=run);thread.start()
    try:
        with MeshSimulationSDK(ROOT/'tianji_armSDK',PACKAGE) as sdk:
            goals={}
            for arm in ('A','B'):
                q=sdk.get_joint_positions(arm);q[0]+=2
                target=sdk.fk(arm,q)
                goals[arm]=sdk.move_pose(arm,target)
            target_hand=closed_posture(hands.names['L'])
            sdk.set_hand_joints('L',target_hand)
            end=time.monotonic()+5
            while time.monotonic()<end:
                error=max(np.max(np.abs(np.array(sdk.get_joint_positions(a))-q)) for a,q in goals.items())
                state=sdk.get_hand_state()
                if error<1e-3 and not state['L']['moving']:break
                time.sleep(.02)
            else:pytest.fail('SDK arm/hand feedback timeout')
            assert np.max(np.abs(np.array(state['L']['positions_rad'])-target_hand))<1e-8
            with pytest.raises(ValueError):sdk.set_hand_joints('R',[999.]*11)
            sdk.stop()
            assert all(not h['moving'] for h in sdk.get_hand_state().values())
            assert sdk.robot.SDK_version()>0
    finally:
        stop.set();thread.join(timeout=2);controller.close();hands.close()
    assert not errors
