"""Check Gazebo/world alignment and replacement of obstacle poses."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from user_package.circuit_scene import load_world_scene, Pose2D  # noqa: E402

SOURCE = Path(__file__).resolve().parents[3]
WORLD = SOURCE / 'custom_start/worlds/limo_obstacle_world.world'


def test_saved_world_poses_and_visual_geometry():
    scene = load_world_scene(WORLD)
    assert not scene.warnings
    assert scene.ground.texture.is_file()
    assert set(scene.models) == {'walls', 'unit_box', 'unit_box_clone'}
    assert scene.models['unit_box'].pose == Pose2D(1.0061, 0.541182, -0.514761)
    assert scene.models['unit_box_clone'].pose == Pose2D(-2.05024, -1.26486, -0.001635)
    assert len(scene.models['walls'].shapes) == 4
    points = scene.models['unit_box'].shapes[0].points
    assert math.dist(points[0], points[1]) == pytest.approx(1.0)


def test_box_uv_orientation_matches_gazebo_top_face():
    scene = load_world_scene(WORLD)
    a, b, c = scene.ground.texture_corners()
    x, y = scene.ground.size
    assert a == pytest.approx((x / 2, y / 2))
    assert b == pytest.approx((x / 2, -y / 2))
    assert c == pytest.approx((-x / 2, y / 2))
    # A live model pose replaces the initial pose and rotates the entire texture.
    a, b, c = scene.ground.texture_corners(Pose2D(2, 3, math.pi / 2))
    assert a == pytest.approx((2 - y / 2, 3 + x / 2))


def test_live_pose_replaces_saved_pose_and_missing_models_disappear():
    scene = load_world_scene(WORLD)
    name, points, _ = next(scene.footprints({'unit_box': Pose2D(4, 2, math.pi / 2)}))
    assert name == 'unit_box'
    assert len(list(scene.footprints({'unit_box': Pose2D()}))) == 1
    assert points[0] == pytest.approx((4.5, 1.5))
    assert list(scene.footprints({})) == []
    assert scene.unknown_models({'limo': Pose2D(), 'new_obstacle': Pose2D()}) == ['new_obstacle']


def test_included_models_and_relative_visual_poses(tmp_path):
    models = tmp_path / 'models'
    (models / 'obstacle').mkdir(parents=True)
    (models / 'obstacle/model.sdf').write_text('''<sdf version="1.6">
      <model name="original"><pose>99 99 0 0 0 0</pose><link name="link">
      <pose>1 0 0 0 0 0</pose><visual name="visual"><pose>0 2 0 0 0 0</pose>
      <geometry><box><size>2 4 1</size></box></geometry></visual></link></model></sdf>''')
    # Keep the circuit include, replacing other objects with a renamed/rotated model.
    world = tmp_path / 'test.world'
    world.write_text('''<sdf version="1.6"><world name="default">
      <include><uri>model://plane_test</uri></include>
      <include><uri>model://obstacle</uri><name>custom_name</name>
      <pose>3 4 0 0 0 1.5707963267948966</pose></include>
      <model name="limo"><link name="base"><visual name="v"><geometry>
      <box><size>1 1 1</size></box></geometry></visual></link></model>
      </world></sdf>''')
    scene = load_world_scene(world, [models, SOURCE / 'custom_start/models'])
    assert set(scene.models) == {'custom_name'}
    name, points, color = next(scene.footprints())
    assert name == 'custom_name'
    # Local centre (1,2) is rotated and translated to world (1,5).
    assert tuple(sum(p[i] for p in points) / 4 for i in (0, 1)) == pytest.approx((1, 5))
    assert math.dist(points[0], points[1]) == pytest.approx(2)


def test_quaternion_normalization_and_validation():
    pose = SimpleNamespace(position=SimpleNamespace(x=1, y=2), orientation=SimpleNamespace(
        x=0, y=0, z=2 * math.sin(math.pi / 4), w=2 * math.cos(math.pi / 4)))
    converted = Pose2D.from_message(pose)
    assert converted.x == 1
    assert converted.yaw == pytest.approx(math.pi / 2)
    pose.orientation.z = pose.orientation.w = 0
    with pytest.raises(ValueError):
        Pose2D.from_message(pose)
    pose.orientation.w = math.nan
    with pytest.raises(ValueError):
        Pose2D.from_message(pose)


def test_live_snapshot_loads_new_geometry_and_ignores_robot(tmp_path):
    from user_package.circuit_scene import load_scene_sdf, world_sdf_from_response
    import json
    original = load_world_scene(WORLD)
    sdf = '''<sdf version="1.7"><world name="default">
      <model name="limo"><pose>0 0 0 0.4 0.2 0</pose></model>
      <model name="new_box"><pose>1 2 0 0 0 0</pose><link name="link">
      <visual name="v"><geometry><box><size>2 3 1</size></box></geometry>
      </visual></link></model>
      <state world_name="default"><model name="limo"><pose>0 0 0 1 2 3</pose></model>
      <model name="new_box"><pose>5 6 0 0 0 0</pose></model></state>
      </world></sdf>'''
    decoded = world_sdf_from_response('data: ' + json.dumps(sdf))
    scene = load_scene_sdf(decoded, WORLD, ground=original.ground)
    assert scene.ground is original.ground
    assert set(scene.models) == {'new_box'}
    assert scene.models['new_box'].pose == Pose2D(5, 6, 0)
    _, points, _ = next(scene.footprints())
    assert math.dist(points[0], points[1]) == pytest.approx(2)
    assert not scene.warnings
    assert scene.world_name == 'default'


def test_gazebo_response_octals_and_rejection():
    from user_package.circuit_scene import world_sdf_from_response
    assert world_sdf_from_response(r'data: "\303\250\n"') == 'è\n'
    with pytest.raises(ValueError):
        world_sdf_from_response('No response received')
    with pytest.raises(ValueError):
        world_sdf_from_response('data: "world" unexpected trailing output')


def test_link_states_select_exact_robot_link_and_handle_removal():
    from user_package.circuit_scene import pose_from_link_states

    def message_pose(x, y, yaw):
        return SimpleNamespace(position=SimpleNamespace(x=x, y=y), orientation=SimpleNamespace(
            x=0, y=0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))
    msg = SimpleNamespace(name=['other::base_link', 'limo::base_link', 'limo::laser_link'],
                          pose=[message_pose(99, 99, 0), message_pose(1, 2, math.pi / 2),
                                message_pose(10, 20, 0)])
    selected = pose_from_link_states(msg, 'limo::base_link')
    assert (selected.x, selected.y, selected.yaw) == pytest.approx((1, 2, math.pi / 2))
    assert pose_from_link_states(msg, 'missing::base_link') is None
    msg.name = ['limo::base_link', 'limo::base_link', 'limo::laser_link']
    with pytest.raises(ValueError):
        pose_from_link_states(msg, 'limo::base_link')
    msg.name = []
    with pytest.raises(ValueError):
        pose_from_link_states(msg, 'limo::base_link')
