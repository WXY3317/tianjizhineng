import math

import pytest

from scene.geometry import (flange_target_from_tip, joint5_restore_waypoints,
                            parse_targets, parse_vision_message, rotate_vector,
                            route_targets, tool_z_rotation_matrix, tool_z_rotations,
                            undo_tool_z_matrix, undo_tool_z_rotation,
                            workpiece_pose_from_flange)


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


def test_labeled_front_and_back_route_before_motion():
    front = '''{"label":"无遮挡正面","position1":{"x":0.12,"y":-0.08,"z":0.39},
        "position2":{"x":0.12,"y":-0.08,"z":0.35},
        "orientation":{"x":0,"y":0,"z":0,"w":1}}'''
    label, targets = parse_vision_message(front)
    assert [side for side, _ in route_targets(targets, label=label)] == ['left', 'left']
    back = front.replace('无遮挡正面', '无遮挡背面')
    label, targets = parse_vision_message(back)
    assert [side for side, _ in route_targets(targets, label=label)] == ['right', 'right']
    with pytest.raises(ValueError, match='0.05'):
        route_targets([((.12, .049, .39), targets[0][1])], label='无遮挡背面')
    with pytest.raises(ValueError, match='Unsupported grasp label'):
        route_targets(targets, label='无遮挡侧面')
    with pytest.raises(ValueError, match='label'):
        parse_vision_message(front.replace('"label":"无遮挡正面",', ''))


def test_tool_tip_target_is_shifted_along_rotated_local_z():
    q = (0., math.sqrt(.5), 0., math.sqrt(.5))  # Local +Z points along body +X.
    flange, orientation = flange_target_from_tip((1., 2., 3.), q, .20)
    assert flange == pytest.approx((.8, 2., 3.))
    assert orientation == pytest.approx(q)
    flange_negative, _ = flange_target_from_tip((1., 2., 3.), q, -.20)
    assert flange_negative == pytest.approx((1.2, 2., 3.))


def test_attached_workpiece_pose_reverses_selected_tool_z_rotation():
    requested = (0., 0., 0., 1.)
    tip = (.5, .2, .6)
    flange, _ = flange_target_from_tip(tip, requested, .2)
    selected = list(tool_z_rotations(requested))[1]
    assert selected[0] == 15.
    position, orientation = workpiece_pose_from_flange(flange, selected[1], .2, selected[0])
    assert position == pytest.approx(tip)
    assert orientation == pytest.approx(requested)
    moved_position, moved_orientation = workpiece_pose_from_flange(
        (.6, .2, .4), selected[1], .2, selected[0])
    assert moved_position == pytest.approx((.6, .2, .6))
    assert moved_orientation == pytest.approx(requested)
    tilted = (0., math.sqrt(.5), 0., math.sqrt(.5))
    tilted_position, tilted_orientation = workpiece_pose_from_flange(
        (.2, .3, .4), tilted, .2, 0.)
    assert tilted_position == pytest.approx((.4, .3, .4))
    assert tilted_orientation == pytest.approx(tilted)


def test_tool_z_candidates_keep_tip_position_and_try_small_angles_first():
    original = (.93310935, -.34575016, .09466474, .02832574)
    candidates = list(tool_z_rotations(original))
    assert [angle for angle, _ in candidates[:5]] == [0., 15., -15., 30., -30.]
    assert candidates[-1][0] == 180.
    assert len({round(angle, 6) for angle, _ in candidates}) == len(candidates)
    tip = (.6313, .17275, .29167)
    flange, _ = flange_target_from_tip(tip, original, .20)
    for _, q in candidates:
        assert rotate_vector(q, (0, 0, 1)) == pytest.approx(rotate_vector(original, (0, 0, 1)), abs=1e-7)
        candidate_flange, _ = flange_target_from_tip(tip, q, .20)
        assert candidate_flange == pytest.approx(flange, abs=1e-7)


def test_joint5_restore_is_split_and_checked_before_motion():
    waypoints = joint5_restore_waypoints(0., 127.18532124600992)
    assert len(waypoints) == 2
    assert waypoints[0] == pytest.approx(-math.radians(127.18532124600992) / 2)
    assert waypoints[-1] == pytest.approx(-math.radians(127.18532124600992))
    with pytest.raises(ValueError, match='limits'):
        joint5_restore_waypoints(-2., 127.18532124600992)


def test_undo_tool_z_rotation_uses_current_orientation():
    current_c = (0., 0., 1., 0.)
    returned = undo_tool_z_rotation(current_c, 90.)
    assert returned == pytest.approx((0., 0., math.sqrt(.5), math.sqrt(.5)))
    assert rotate_vector(returned, (0., 0., 1.)) == pytest.approx(
        rotate_vector(current_c, (0., 0., 1.)))
    assert undo_tool_z_matrix(current_c, tool_z_rotation_matrix(90.)) == pytest.approx(returned)
    with pytest.raises(ValueError, match='local Z'):
        undo_tool_z_matrix(current_c, ((1., 0., 0.), (0., 0., -1.), (0., 1., 0.)))


@pytest.mark.parametrize('payload', [
    '{position1:{"x":0,"y":0,"z":0},orientation:{"x":0,"y":0,"z":0,"w":0}}',
    '{position1:{"x":0,"y":0,"z":NaN},orientation:{"x":0,"y":0,"z":0,"w":1}}',
    '{position1:{"x":0,"y":0,"z":0},orientation:{"x":0,"y":0,"z":0,"w":1},extra:1}',
])
def test_invalid_vision_pose_is_rejected(payload):
    with pytest.raises(ValueError):
        parse_targets(payload)
