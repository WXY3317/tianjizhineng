import json
from pathlib import Path

import pytest

from scene.arm_positions import load_arm_positions


def test_config_supports_new_position_id(tmp_path):
    source = Path(__file__).parents[1] / 'config' / 'arm_positions.json'
    data = json.loads(source.read_text(encoding='utf-8'))
    data['2'] = {'name': 'service', 'left': [0.1] * 7, 'right': [-0.1] * 7}
    path = tmp_path / 'positions.json'
    path.write_text(json.dumps(data), encoding='utf-8')
    positions = load_arm_positions(path)
    assert positions[2] == 'service'
    assert positions['service', 'left'] == (0.1,) * 7
    assert positions['service', 'right'] == (-0.1,) * 7


def test_config_rejects_incomplete_position(tmp_path):
    path = tmp_path / 'positions.json'
    path.write_text(json.dumps({'1': {'name': 'pre', 'left': [0] * 7}}), encoding='utf-8')
    with pytest.raises(ValueError, match='right needs seven finite radians'):
        load_arm_positions(path)


def test_body_pose_ptp_position_is_loaded():
    source = Path(__file__).parents[1] / 'config' / 'arm_positions.json'
    positions = load_arm_positions(source)
    assert positions[2] == 'body_photo'
    xyz, xyzw = positions['body_photo', 'body_pose_ptp']
    assert xyz == (-0.05, 0.0, 0.6)
    assert xyzw == (0.0, 0.38941332986322064, 0.0, 0.9210631132147452)


def test_body_joint_home_has_six_joints():
    source = Path(__file__).parents[1] / 'config' / 'arm_positions.json'
    positions = load_arm_positions(source)
    assert positions[3] == 'body_home'
    assert positions['body_home', 'body_joints'] == (0.,) * 6
