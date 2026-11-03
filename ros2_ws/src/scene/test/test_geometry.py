import math

import pytest

from scene.geometry import parse_targets, route_targets, transform_target


SAMPLE = '''{
position1:{"x": 0.12, "y":-0.08, "z":0.39},
position2:{"x":0.0,"y":0.0,"z":0.0},
orientation:{"x":0.0,"y":0.0,"z":0.0,"w":1.0}
}'''


def test_pictured_payload_retains_two_distinct_targets():
    targets = parse_targets(SAMPLE)
    assert targets[0][0] == (.12, -.08, .39)
    assert targets[1][0] == (0., 0., 0.)
    assert [side for side, _ in route_targets(targets)] == ['right']


def test_targets_route_by_y_and_keep_order():
    targets = parse_targets('''{"position1":{"x":0,"y":0.2,"z":0.3},
        "position2":{"x":0,"y":-0.1,"z":0.4},
        "orientation":{"x":0,"y":0,"z":0,"w":1}}''')
    assert [side for side, _ in route_targets(targets)] == ['left', 'right']
    with pytest.raises(ValueError, match='nonzero Y'):
        route_targets([((.1, 0., .3), targets[0][1])])


def test_transform_uses_body_joint_six_rotation_and_translation():
    target = parse_targets(SAMPLE)[0]
    q = (0., 0., math.sqrt(.5), math.sqrt(.5))
    position, orientation = transform_target((.2, .3, .765), q, target)
    assert position == pytest.approx((.28, .42, 1.155))
    assert orientation == pytest.approx(q)


@pytest.mark.parametrize('payload', [
    '{position1:{"x":0,"y":0,"z":0},orientation:{"x":0,"y":0,"z":0,"w":0}}',
    '{position1:{"x":0,"y":0,"z":NaN},orientation:{"x":0,"y":0,"z":0,"w":1}}',
    '{position1:{"x":0,"y":0,"z":0},orientation:{"x":0,"y":0,"z":0,"w":1},extra:1}',
])
def test_invalid_vision_pose_is_rejected(payload):
    with pytest.raises(ValueError):
        parse_targets(payload)
