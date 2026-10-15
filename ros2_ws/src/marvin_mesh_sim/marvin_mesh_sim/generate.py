"""Build a portable arm/hand URDF from existing MJCF and audited STL assets."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from marvin_sim.kinematics import quaternion_matrix


def numbers(value):
    return [float(v) for v in value.split()]


def text(value):
    return ' '.join(f'{float(v):.12g}' for v in value)


def rotation(element, snap=False):
    q = numbers(element.get('quat', '1 0 0 0'))
    matrix = quaternion_matrix(q[1:]+q[:1])
    ideal = np.round(matrix)
    if snap:
        if (np.max(np.abs(ideal-matrix)) > 5e-5
                or not np.array_equal(ideal.T @ ideal, np.eye(3))
                or np.linalg.det(ideal) < .99):
            raise ValueError('Arm rotation differs from a nominal quarter turn; recalibrate SDK mapping')
        matrix = ideal
    return matrix


def rpy(matrix):
    pitch = math.atan2(-matrix[2,0], math.hypot(matrix[0,0], matrix[1,0]))
    if abs(math.cos(pitch)) < 1e-8:
        return [0., pitch, math.atan2(-matrix[0,1], matrix[1,1])]
    return [math.atan2(matrix[2,1], matrix[2,2]), pitch, math.atan2(matrix[1,0], matrix[0,0])]


def origin(parent, element=None, snap=False):
    xyz = element.get('pos', '0 0 0') if element is not None else '0 0 0'
    angles = rpy(rotation(element, snap)) if element is not None else [0,0,0]
    ET.SubElement(parent, 'origin', xyz=xyz, rpy=text(angles))


def audit_stl(path):
    raw = path.read_bytes()
    if not raw:
        return {'status': 'empty', 'bytes': 0}
    if len(raw) >= 84:
        triangles, = struct.unpack_from('<I', raw, 80)
        if triangles and len(raw) == 84+50*triangles:
            dtype = np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
            vertices = np.frombuffer(raw, dtype=dtype, count=triangles, offset=84)['vertices']
        else:
            vertices = None
    else:
        vertices = None
    if vertices is None:
        try:
            vertices = np.array([numbers(line.strip()[7:]) for line in raw.decode('ascii').splitlines()
                                 if line.strip().startswith('vertex ')])
            triangles = len(vertices)//3
        except (UnicodeError, ValueError) as exc:
            raise ValueError(f'Invalid STL: {path}') from exc
    if not triangles or not np.isfinite(vertices).all():
        raise ValueError(f'Invalid STL triangles: {path}')
    return {'status': 'mesh', 'bytes': len(raw), 'triangles': triangles,
            'bounds_m': [vertices.reshape(-1,3).min(0).tolist(), vertices.reshape(-1,3).max(0).tolist()],
            'sha256': hashlib.sha256(raw).hexdigest()}


def generate(source_root, destination, sdk_root):
    source_root, destination, sdk_root = map(Path, (source_root, destination, sdk_root))
    source = ET.parse(source_root/'luna_m6_robot.xml').getroot()
    asset_paths = {m.get('name'): m.get('file') for m in source.findall('asset/mesh')}
    for sub in ('urdf', 'config', 'meshes'):
        (destination/sub).mkdir(parents=True, exist_ok=True)
    robot = ET.Element('robot', name='marvin_mesh_dual_arm_hands')
    ET.SubElement(robot, 'link', name='world')
    fixture = ET.SubElement(robot, 'link', name='base_link')
    visual = ET.SubElement(fixture, 'visual')
    ET.SubElement(visual, 'origin', xyz='0 0 -0.35')
    ET.SubElement(ET.SubElement(visual, 'geometry'), 'cylinder', radius='.11', length='.7')
    ET.SubElement(ET.SubElement(visual, 'material', name='fixture'), 'color', rgba='.22 .25 .3 1')
    weld = ET.SubElement(robot, 'joint', name='world_to_fixture', type='fixed')
    ET.SubElement(weld, 'parent', link='world'); ET.SubElement(weld, 'child', link='base_link')
    ET.SubElement(weld, 'origin', xyz='0 0 .7', rpy='0 0 0')
    manifest, hands, joint_limits, corrections = {}, {}, {}, {}

    def body_link(body, name):
        link = ET.SubElement(robot, 'link', name=name)
        inertial = body.find('inertial')
        if inertial is not None:
            target = ET.SubElement(link, 'inertial')
            origin(target, inertial)
            ET.SubElement(target, 'mass', value=inertial.get('mass'))
            values = numbers(inertial.get('fullinertia'))
            ET.SubElement(target, 'inertia', **dict(zip(
                ('ixx','iyy','izz','ixy','ixz','iyz'), map(str, values))))
        geom = next((g for g in body.findall('geom') if g.get('type') == 'mesh'), None)
        if geom is not None:
            asset = asset_paths[geom.get('mesh')]
            path = source_root/asset
            report = audit_stl(path)
            manifest[asset] = report
            collision = next((g for g in body.findall('geom') if g.get('type') == 'box'), None)
            view = ET.SubElement(link, 'visual')
            if report['status'] == 'mesh':
                relative = Path(asset).relative_to('assets/meshes')
                target = destination/'meshes'/relative
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(path, target)
                origin(view, geom)
                ET.SubElement(ET.SubElement(view, 'geometry'), 'mesh',
                              filename=f'package://marvin_mesh_sim/meshes/{relative}')
                color = '.65 .70 .76 1' if 'Arm_' in geom.get('mesh') else '.6 .62 .66 1'
            else:
                if collision is None:
                    raise ValueError(f'Empty mesh without MJCF fallback geometry: {path}')
                origin(view, collision)
                ET.SubElement(ET.SubElement(view, 'geometry'), 'box',
                              size=text(2*np.array(numbers(collision.get('size')))))
                color = '.83 .52 .16 1'
                report['fallback'] = 'MJCF collision box (real STL missing)'
            ET.SubElement(ET.SubElement(view, 'material', name=f'mat_{name}'), 'color', rgba=color)
            if collision is not None:
                hit = ET.SubElement(link, 'collision')
                origin(hit, collision)
                ET.SubElement(ET.SubElement(hit, 'geometry'), 'box',
                              size=text(2*np.array(numbers(collision.get('size')))))
        return link

    def connect(parent, child, element, joint_name, source_joint=None, snap=False):
        j = ET.SubElement(robot, 'joint', name=joint_name,
                          type='revolute' if source_joint is not None else 'fixed')
        ET.SubElement(j, 'parent', link=parent); ET.SubElement(j, 'child', link=child)
        origin(j, element, snap)
        if source_joint is not None:
            lo, hi = numbers(source_joint.get('range'))
            ET.SubElement(j, 'axis', xyz=source_joint.get('axis', '0 0 1'))
            ET.SubElement(j, 'limit', lower=str(lo), upper=str(hi),
                          velocity=str(math.pi if child.startswith('Arm_') else 2.),
                          effort=str(108. if child.startswith('Arm_') else 2.))
        return j

    def add_hand(body, parent, side):
        name = body.get('name')
        body_link(body, name)
        j = body.find('joint')
        joint_name = j.get('name') if j is not None else f'{parent}_to_{name}'
        connect(parent, name, body, joint_name, j)
        if j is not None:
            hands[side]['joints'].append({'name': joint_name, 'range': numbers(j.get('range'))})
        for child in body.findall('body'):
            add_hand(child, name, side)

    for side in ('L', 'R'):
        shoulder = source.find(f'.//body[@name="Arm_{side}1_Link"]')
        ET.SubElement(robot, 'link', name=f'Base_{side}')
        connect('base_link', f'Base_{side}', shoulder, f'mount_{side}', snap=True)
        for i in range(1,8):
            body = source.find(f'.//body[@name="Arm_{side}{i}_Link"]')
            name = body.get('name')
            body_link(body, name)
            element = body if i > 1 else None
            parent = f'Arm_{side}{i-1}_Link' if i > 1 else f'Base_{side}'
            connect(parent, name, element, f'Joint{i}_{side}', body.find('joint'), snap=i>1)
            if i > 1:
                old, new = rotation(body), rotation(body, True)
                corrections[name] = float(math.acos(float(np.clip((np.trace(old.T@new)-1)/2,-1,1))))
            lo, hi = numbers(body.find('joint').get('range'))
            joint_limits[f'joint{i}'] = {'limit': {'lower':lo,'upper':hi,'velocity':math.pi,'effort':108.}}
        ee = source.find(f'.//body[@name="{side.lower()}_ee"]')
        ET.SubElement(robot, 'link', name=f'flange_{side}')
        connect(f'Arm_{side}7_Link', f'flange_{side}', ee, f'flange_joint_{side}')
        hands[side] = {'joints': []}
        add_hand(source.find(f'.//body[@name="{side}_hand_base"]'), f'flange_{side}', side)

    # Type 1017 uses the SDK's CCS geometric lengths. The source shoulder frame
    # is at joint 1, so L1=0; source upper/lower arms are 264 mm, flange 87 mm.
    template = (sdk_root/'CommonConfig/ccs_m6_40.MvKDCfg').read_text().splitlines()
    for arm in range(2):
        offset = 13*arm
        for row, value in ((1,0.), (3,264.), (5,264.), (8,87.)):
            values = template[offset+row].split(',')
            values[2] = str(value)
            template[offset+row] = ','.join(values)
        for i in range(7):
            row = template[offset+1+i].split(',')
            limits = joint_limits[f'joint{i+1}']['limit']
            row[4:6] = [str(math.degrees(limits['lower'])),str(math.degrees(limits['upper']))]
            # This derived vendor file is for kinematics only. Do not copy the
            # unrelated stock model's mass/CoM/inertia into this model.
            row[8:18] = ['0']*10
            template[offset+1+i] = ','.join(row)
        # No vendor wrist-collision calibration exists for these assets. Use
        # rectangular wrist joint bounds; do not reuse unrelated fitted curves.
        for i, bound in enumerate((90.,90.,-90.,-90.)):
            template[offset+9+i] = f'0,0,{bound},'
    (destination/'config/mesh_arms.MvKDCfg').write_text('\n'.join(template)+'\n')
    tool = [[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1]]
    (destination/'config/sdk_geometry.yaml').write_text(yaml.safe_dump({
        'type':1017,'dimensions_mm':{'shoulder_offset':0,'upper_arm':264,'forearm':264,'flange':87},
        'flange_tool_rotation':tool, 'arm_rotations': 'MJCF quarter-turn nominalization, <5e-5 matrix entry error',
        'wrist_coupling':'not calibrated; only rectangular joint limits'}, sort_keys=False))
    (destination/'config/joint_limits.yaml').write_text(yaml.safe_dump(joint_limits,sort_keys=False))
    (destination/'config/hands.yaml').write_text(yaml.safe_dump(hands,sort_keys=False))
    report = {'source': 'mjcf/luna_m6_robot.xml', 'units':'metres', 'arm_dof':14, 'hand_dof':22,
              'assets':manifest, 'nominalized_arm_rotation_errors_rad':corrections,
              'mesh_count':sum(v['status']=='mesh' for v in manifest.values()),
              'empty_mesh_count':sum(v['status']=='empty' for v in manifest.values())}
    (destination/'config/asset_report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    ET.indent(robot, space='  ')
    ET.ElementTree(robot).write(destination/'urdf/marvin_mesh_hands.urdf',encoding='utf-8',xml_declaration=True)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-root',required=True);p.add_argument('--destination',required=True)
    p.add_argument('--sdk-root',required=True)
    args=p.parse_args()
    report=generate(args.source_root,args.destination,args.sdk_root)
    print(json.dumps({k:report[k] for k in ('arm_dof','hand_dof','mesh_count','empty_mesh_count')},indent=2))
