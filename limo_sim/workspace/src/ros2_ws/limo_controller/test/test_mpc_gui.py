"""Exercise native geometry, inspection and live/stale ROS behavior offscreen."""

from dataclasses import replace
import math
import os
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest

pytest.importorskip('PyQt5')
pytest.importorskip('rclpy')
from PyQt5.QtCore import QPointF  # noqa: E402
from PyQt5.QtWidgets import QApplication, QGraphicsPixmapItem  # noqa: E402
import rclpy  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from limo_controller.mpc_preview.gui import MpcView, MpcWindow, scene_point  # noqa: E402
from limo_controller.mpc_preview.node import MpcPreview  # noqa: E402
from limo_controller.mpc_preview.render import RenderOptions, world_to_base  # noqa: E402
from test_mpc_preview_render import sample_frame  # noqa: E402
from test_mpc_preview_node import debug_message  # noqa: E402


@pytest.fixture(scope='module')
def app():
    instance = QApplication.instance() or QApplication([])
    yield instance


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '193')
    rclpy.init()
    instance = MpcPreview(gui=True)
    instance.set_parameters([Parameter('use_sim_time', value=True)])
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def test_costmap_transform_preserves_cell_centers_with_both_rotations(app):
    frame = sample_frame()
    frame.robot_pose = (1.2, -0.4, -0.7)
    frame.costmap_origin = (-0.8, 0.2, 0.6)
    view = MpcView(RenderOptions())
    view.update_scene((frame, '', None, (), None))
    item = next(i for i in view.scene().items() if isinstance(i, QGraphicsPixmapItem))
    x, y = 7.5, 12.5
    ox, oy, yaw = frame.costmap_origin
    wx = ox + frame.costmap_resolution * (math.cos(yaw) * x - math.sin(yaw) * y)
    wy = oy + frame.costmap_resolution * (math.sin(yaw) * x + math.cos(yaw) * y)
    expected = scene_point(world_to_base((wx, wy), frame.robot_pose))
    actual = item.mapToScene(QPointF(x, y))
    assert actual.x() == pytest.approx(expected.x())
    assert actual.y() == pytest.approx(expected.y())
    view.close()


def test_selected_candidate_remains_visible_when_other_candidates_are_hidden(app):
    frame = sample_frame()
    view = MpcView(RenderOptions())
    view.update_scene((frame, '', None, (), None), {'candidates': False})
    trajectories = [i for i in view.scene().items() if i.toolTip()]
    assert len(trajectories) == 1
    assert trajectories[0].toolTip().startswith('ID 4 |')
    view.inspected_id = 1
    view.update_scene(view.snapshot)
    assert len([i for i in view.scene().items() if i.toolTip()]) == 2
    view.close()


def test_gui_needs_no_image_subscriber_and_clears_stale_predictions(app, node):
    assert node.publisher is None
    assert node.timer is None
    node._snapshot(debug_message(sample_frame()))
    assert node.scene_snapshot()[0] is not None
    window = MpcWindow(node)
    window.refresh()
    assert window.table.rowCount() == 5
    node.received_at = time.monotonic() - 5
    window.last_draw = 0
    window.refresh()
    assert window.table.rowCount() == 0
    assert 'stale' in window.status.text()
    window.close()


def test_freeze_preserves_cycle_while_callbacks_continue_and_resume_uses_latest(app, node):
    node._snapshot(debug_message(sample_frame()))
    window = MpcWindow(node)
    window.refresh()
    frozen = window.view.snapshot[0]
    window.set_frozen(True)
    node._snapshot(debug_message(replace(sample_frame(), command_velocity=0.123)))
    window.last_draw = 0
    window.refresh()
    assert window.view.snapshot[0] is frozen
    assert node.frame.command_velocity == pytest.approx(0.123)
    window.set_frozen(False)
    window.refresh()
    assert window.view.snapshot[0] is node.frame
    assert '0.123' in window.metrics.text()
    window.close()


def test_candidate_inspection_and_pan_survive_new_telemetry(app, node):
    node._snapshot(debug_message(sample_frame()))
    window = MpcWindow(node)
    window.refresh()
    window.table.selectRow(1)
    assert window.view.inspected_id == 1
    window.view.scale(1.5, 1.5)
    transform = window.view.transform()
    node._snapshot(debug_message(sample_frame()))
    window.last_draw = 0
    window.refresh()
    assert window.view.inspected_id == 1
    assert window.view.transform() == transform
    assert window.table.currentRow() == 1
    window.close()
