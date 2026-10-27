# Copyright 2026 Giulio Cataldo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Verify explicit path execution through the control GUI service."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
import rclpy  # noqa: E402
from rclpy.task import Future  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402

from limo_controller.path_executor import PathExecutor  # noqa: E402
from limo_controller.chassis_safety import ChassisSafety  # noqa: E402


def test_start_control_sends_the_stored_path():
    path = Path()
    path.poses.append(PoseStamped())
    sent = []
    states = []
    bridge = SimpleNamespace(
        goal_pending=False,
        waiting_for_server=False,
        arrival_pending=False,
        control_path=path,
        active_control_goal_handle=None,
        follow_path_client=SimpleNamespace(
            wait_for_server=lambda timeout_sec: True),
        control_requested=False,
        control_paused=False,
        _send_control_goal=lambda: sent.append(path) or True,
        _publish_control_state=states.append,
        _reset_arrival=lambda: None,
    )
    request = SimpleNamespace(data=True)
    response = SimpleNamespace(success=False, message='')

    result = PathExecutor._set_control_active(
        bridge, request, response)

    assert result.success is True
    assert result.message == 'Control start requested.'
    assert bridge.control_requested is True
    assert bridge.control_paused is False
    assert sent == [path]


def test_execution_path_uses_latest_tf_without_mutating_stored_path():
    path = Path()
    path.header.stamp.sec = 10
    pose = PoseStamped()
    pose.header.stamp.sec = 10
    path.poses.append(pose)

    execution = PathExecutor._path_for_execution(path)

    assert execution is not path
    assert execution.header.stamp.sec == 10
    assert execution.header.stamp.nanosec == 0
    assert execution.poses[0].header.stamp.sec == 0
    assert execution.poses[0].header.stamp.nanosec == 0
    assert path.header.stamp.sec == 10
    assert path.poses[0].header.stamp.sec == 10


def test_follow_path_goal_supports_foxy_action_definition():
    path = Path()

    goal = PathExecutor._build_follow_path_goal(
        path,
        'FollowPath',
        'goal_checker',
    )

    assert goal.path == path
    assert goal.controller_id == 'FollowPath'
    if hasattr(goal, 'goal_checker_id'):
        assert goal.goal_checker_id == 'goal_checker'


@pytest.fixture
def executor():
    rclpy.init()
    node = PathExecutor()
    node.follow_path_client.wait_for_server = Mock(return_value=True)
    node.follow_path_client.send_goal_async = Mock(return_value=Future())
    node.control_status_publisher = Mock()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def ready_path(executor):
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp.sec = 42
    path.poses = [PoseStamped()]
    executor._path_callback(path)
    return path


def start(executor):
    return executor._set_control_active(
        SetBool.Request(data=True), SetBool.Response())


def accept_pending_goal(executor):
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = Future()
    executor.follow_path_client.send_goal_async.return_value.set_result(handle)
    return handle


def test_receiving_a_path_waits_for_start_and_sends_only_once(executor):
    path = ready_path(executor)
    executor.follow_path_client.send_goal_async.assert_not_called()
    assert start(executor).success
    assert start(executor).success
    executor.follow_path_client.send_goal_async.assert_called_once()
    sent = executor.follow_path_client.send_goal_async.call_args[0][0].path
    assert len(sent.poses) == 1
    assert sent.header.frame_id == 'map'
    assert sent.header.stamp.sec == 42
    assert path.header.stamp.sec == 42
    assert executor.control_path.header.stamp.sec == 42


@pytest.mark.parametrize('operation', ['abort', 'pause', 'invalidate'])
def test_stop_before_acceptance_cancels_the_pending_goal(executor, operation):
    ready_path(executor)
    assert start(executor).success
    if operation == 'abort':
        executor._set_control_active(SetBool.Request(data=False), SetBool.Response())
    elif operation == 'pause':
        executor._set_control_enabled(SetBool.Request(data=False), SetBool.Response())
    else:
        executor._path_callback(Path())
    handle = accept_pending_goal(executor)
    handle.cancel_goal_async.assert_called_once()
    assert not executor.control_status_publisher.publish.call_args[0][0].data.startswith(
        'ACTIVE:')


def test_replacing_an_active_path_cancels_and_requires_another_start(executor):
    from action_msgs.msg import GoalStatus
    ready_path(executor)
    assert start(executor).success
    handle = accept_pending_goal(executor)
    ready_path(executor)
    handle.cancel_goal_async.assert_called_once()
    assert not executor.control_requested
    assert not start(executor).success  # Old action has not finished canceling.
    handle.get_result_async.return_value.set_result(
        SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    executor.follow_path_client.send_goal_async.assert_called_once()
    executor.follow_path_client.send_goal_async.return_value = Future()
    assert start(executor).success
    assert executor.follow_path_client.send_goal_async.call_count == 2


def test_unavailable_controller_queues_start_until_server_is_ready(executor):
    ready_path(executor)
    executor.follow_path_client.wait_for_server.return_value = False
    assert start(executor).success
    assert executor.control_requested
    assert executor.waiting_for_server
    executor.follow_path_client.send_goal_async.assert_not_called()

    executor.follow_path_client.wait_for_server.return_value = True
    executor._retry_waiting_control_goal()

    assert not executor.waiting_for_server
    assert executor.goal_pending
    executor.follow_path_client.send_goal_async.assert_called_once()


def test_abort_removes_a_queued_start(executor):
    ready_path(executor)
    executor.follow_path_client.wait_for_server.return_value = False
    assert start(executor).success

    executor._set_control_active(
        SetBool.Request(data=False), SetBool.Response())
    executor.follow_path_client.wait_for_server.return_value = True
    executor._retry_waiting_control_goal()

    assert not executor.waiting_for_server
    assert not executor.control_requested
    executor.follow_path_client.send_goal_async.assert_not_called()


def test_send_exception_returns_failure_without_killing_services(executor):
    ready_path(executor)
    executor.follow_path_client.send_goal_async.side_effect = RuntimeError('transport failed')
    assert not start(executor).success
    assert not executor.goal_pending
    assert not executor.control_requested


def require_chassis(executor, mode=1):
    executor.chassis_safety = ChassisSafety(enabled=True)
    executor.chassis_safety.update(SimpleNamespace(
        vehicle_state=0, error_code=0, control_mode=mode, motion_mode=2))


@pytest.mark.parametrize('problem', ['missing', 'stale', 'phone', 'fault'])
def test_real_start_never_sends_action_without_safe_chassis(executor, problem):
    ready_path(executor)
    require_chassis(executor)
    if problem == 'missing':
        executor.chassis_safety.status = None
    elif problem == 'stale':
        executor.chassis_safety.received_at -= 2.0
    elif problem == 'phone':
        executor.chassis_safety.status.control_mode = 2
    else:
        executor.chassis_safety.status.error_code = 4
    response = start(executor)
    assert not response.success
    assert response.message == executor.safety_stop_reason
    assert not executor.control_requested
    executor.follow_path_client.send_goal_async.assert_not_called()


def test_real_start_accepts_command_mode_with_stale_motion_mode_reporting(executor):
    ready_path(executor)
    require_chassis(executor)
    assert start(executor).success
    executor.follow_path_client.send_goal_async.assert_called_once()


@pytest.mark.parametrize('pending', [False, True])
def test_mode_loss_cancels_control_and_never_restarts_automatically(executor, pending):
    from action_msgs.msg import GoalStatus
    ready_path(executor)
    require_chassis(executor)
    assert start(executor).success
    handle = None if pending else accept_pending_goal(executor)
    executor._chassis_callback(SimpleNamespace(
        vehicle_state=0, error_code=0, control_mode=2))
    assert not executor.control_requested
    if pending:
        handle = accept_pending_goal(executor)
    handle.cancel_goal_async.assert_called_once()
    reason = executor.safety_stop_reason
    handle.get_result_async.return_value.set_result(
        SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert executor.control_status_publisher.publish.call_args[0][0].data == (
        'ERROR: ' + reason)
    executor._chassis_callback(SimpleNamespace(
        vehicle_state=0, error_code=0, control_mode=1))
    executor._retry_waiting_control_goal()
    assert not executor.control_requested
    executor.follow_path_client.send_goal_async.assert_called_once()


def test_stale_chassis_watchdog_cancels_active_control(executor):
    ready_path(executor)
    require_chassis(executor)
    assert start(executor).success
    handle = accept_pending_goal(executor)
    executor.chassis_safety.received_at -= 2.0
    executor._monitor_chassis()
    handle.cancel_goal_async.assert_called_once()
    assert not executor.control_requested
    assert 'stale' in executor.safety_stop_reason


def test_resume_rechecks_chassis_even_if_cancellation_is_pending(executor):
    ready_path(executor)
    require_chassis(executor)
    assert start(executor).success
    handle = accept_pending_goal(executor)
    executor._set_control_enabled(SetBool.Request(data=False), SetBool.Response())
    executor.chassis_safety.status.control_mode = 2
    response = executor._set_control_enabled(
        SetBool.Request(data=True), SetBool.Response())
    assert not response.success
    assert not executor.control_requested
    assert handle.cancel_goal_async.call_count == 2
    executor.follow_path_client.send_goal_async.assert_called_once()


@pytest.fixture
def tube_executor(executor):
    executor.require_tube_start = True
    executor.tube_check_client = Mock()
    executor.tube_check_client.wait_for_service.return_value = True
    executor.tube_check_client.call_async.return_value = Future()
    return executor


def test_tube_start_waits_for_full_footprint_approval_before_sending_action(tube_executor):
    ready_path(tube_executor)
    assert start(tube_executor).success
    assert start(tube_executor).success
    tube_executor.follow_path_client.send_goal_async.assert_not_called()
    tube_executor.tube_check_client.call_async.assert_called_once()
    tube_executor.tube_check_client.call_async.return_value.set_result(
        SimpleNamespace(success=True, message='Entire footprint inside tube'))
    tube_executor.follow_path_client.send_goal_async.assert_called_once()
    assert tube_executor.goal_pending


def test_tube_start_rejects_outside_footprint_without_sending_action(tube_executor):
    ready_path(tube_executor)
    assert start(tube_executor).success
    tube_executor.tube_check_client.call_async.return_value.set_result(
        SimpleNamespace(success=False, message='START blocked: footprint outside tube'))
    tube_executor.follow_path_client.send_goal_async.assert_not_called()
    assert not tube_executor.control_requested
    assert 'outside tube' in tube_executor.control_status_publisher.publish.call_args[0][0].data


@pytest.mark.parametrize('operation', ['abort', 'pause', 'replace'])
def test_pending_tube_approval_is_invalidated_by_stop_or_new_path(tube_executor, operation):
    ready_path(tube_executor)
    assert start(tube_executor).success
    old_future = tube_executor.tube_check_future
    old_generation = tube_executor.path_generation
    if operation == 'abort':
        tube_executor._set_control_active(SetBool.Request(data=False), SetBool.Response())
    elif operation == 'pause':
        tube_executor._set_control_enabled(SetBool.Request(data=False), SetBool.Response())
    else:
        ready_path(tube_executor)
    tube_executor._tube_check_done(old_future, old_generation)
    tube_executor.follow_path_client.send_goal_async.assert_not_called()
    assert tube_executor.tube_check_future is None


def test_tube_guard_timeout_fails_closed(tube_executor):
    import time
    ready_path(tube_executor)
    assert start(tube_executor).success
    tube_executor.tube_check_started = time.monotonic() - 3.0
    tube_executor._retry_waiting_control_goal()
    tube_executor.follow_path_client.send_goal_async.assert_not_called()
    assert not tube_executor.control_requested
    assert 'timed out' in tube_executor.control_status_publisher.publish.call_args[0][0].data


def test_resume_performs_a_fresh_tube_check(tube_executor):
    ready_path(tube_executor)
    assert start(tube_executor).success
    tube_executor._set_control_enabled(SetBool.Request(data=False), SetBool.Response())
    fresh = Future()
    tube_executor.tube_check_client.call_async.return_value = fresh
    resumed = tube_executor._set_control_enabled(
        SetBool.Request(data=True), SetBool.Response())
    assert resumed.success
    tube_executor.follow_path_client.send_goal_async.assert_not_called()
    assert tube_executor.tube_check_future is fresh
    fresh.set_result(SimpleNamespace(success=True, message='Footprint inside tube'))
    tube_executor.follow_path_client.send_goal_async.assert_called_once()


@pytest.fixture
def arriving_executor(executor, monkeypatch):
    from rclpy.time import Time
    path = ready_path(executor)
    path.poses[-1].pose.position.x = 10.0
    path.poses[-1].pose.position.y = 5.0
    executor.control_path = path
    clock = Mock()
    clock.now.return_value = Time(nanoseconds=0)
    monkeypatch.setattr(executor, 'get_clock', lambda: clock)
    executor.tf_buffer.lookup_transform = Mock()
    assert start(executor).success
    executor.test_goal_handle = accept_pending_goal(executor)
    return executor


def arrival_sample(executor, now, *, stamp=None, x=10.1, linear=0.0, angular=0.0):
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.time import Time
    executor.get_clock().now.return_value = Time(nanoseconds=round(now * 1e9))
    odometry = Odometry()
    odometry.header.stamp = Time(nanoseconds=round(
        (now if stamp is None else stamp) * 1e9)).to_msg()
    # The odometry pose is deliberately in a different frame: use TF for the goal distance.
    odometry.pose.pose.position.x = -100.0
    odometry.twist.twist.linear.x = linear
    odometry.twist.twist.angular.z = angular
    transform = TransformStamped()
    transform.transform.translation.x = x
    transform.transform.translation.y = 5.0
    executor.tf_buffer.lookup_transform.return_value = transform
    executor._arrival_odometry(odometry)


def control_status(executor):
    return executor.control_status_publisher.publish.call_args[0][0].data


@pytest.mark.parametrize('late_status', ['STATUS_CANCELED', 'STATUS_SUCCEEDED', 'STATUS_ABORTED'])
def test_measured_arrival_cancels_control_and_latches_success_despite_late_action_result(
        arriving_executor, late_status):
    from action_msgs.msg import GoalStatus
    node = arriving_executor
    for index in range(20):
        arrival_sample(node, index / 10)
        assert not control_status(node).startswith('SUCCESS:')
    arrival_sample(node, 2.0)
    assert control_status(node).startswith('SUCCESS:')
    assert not node.control_requested
    assert not node.control_paused
    node.test_goal_handle.cancel_goal_async.assert_called_once()
    node.test_goal_handle.get_result_async.return_value.set_result(
        SimpleNamespace(status=getattr(GoalStatus, late_status)))
    assert control_status(node).startswith('SUCCESS:')
    node._retry_waiting_control_goal()
    node.follow_path_client.send_goal_async.assert_called_once()
    ready_path(node)
    assert control_status(node).startswith('READY:')
    assert node.success_generation is None


@pytest.mark.parametrize('problem', ['moving', 'rotating', 'outside', 'stale', 'tf', 'nan'])
def test_arrival_dwell_resets_on_motion_outside_circle_or_missing_measurements(
        arriving_executor, problem):
    from tf2_ros import TransformException
    node = arriving_executor
    for index in range(11):
        arrival_sample(node, index / 10)
    values = {}
    if problem == 'moving':
        values['linear'] = 0.02
    elif problem == 'rotating':
        values['angular'] = 0.1
    elif problem == 'outside':
        values['x'] = 10.21
    elif problem == 'stale':
        values['stamp'] = 0.0
    elif problem == 'tf':
        node.tf_buffer.lookup_transform.side_effect = TransformException('missing TF')
    else:
        values['linear'] = float('nan')
    arrival_sample(node, 1.1, **values)
    node.tf_buffer.lookup_transform.side_effect = None
    for index in range(12, 32):
        arrival_sample(node, index / 10)
        assert not control_status(node).startswith('SUCCESS:')
    arrival_sample(node, 3.2)
    assert control_status(node).startswith('SUCCESS:')


def test_nav2_completion_waits_for_measured_stop_instead_of_returning_to_ready(arriving_executor):
    from action_msgs.msg import GoalStatus
    node = arriving_executor
    node.test_goal_handle.get_result_async.return_value.set_result(
        SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert node.arrival_pending
    assert node.control_requested
    assert control_status(node).startswith('ACTIVE:')
    assert start(node).success  # Repeated START cannot restart a completed Nav2 action.
    assert node._set_control_enabled(SetBool.Request(data=True), SetBool.Response()).success
    node.follow_path_client.send_goal_async.assert_called_once()
    for index in range(21):
        arrival_sample(node, index / 10)
    assert control_status(node).startswith('SUCCESS:')
    node.test_goal_handle.cancel_goal_async.assert_not_called()


@pytest.mark.parametrize('operation', ['pause', 'abort', 'replace'])
def test_pause_abort_and_new_path_discard_partial_arrival(arriving_executor, operation):
    node = arriving_executor
    for index in range(11):
        arrival_sample(node, index / 10)
    if operation == 'pause':
        node._set_control_enabled(SetBool.Request(data=False), SetBool.Response())
    elif operation == 'abort':
        node._set_control_active(SetBool.Request(data=False), SetBool.Response())
    else:
        ready_path(node)
    for index in range(12, 35):
        arrival_sample(node, index / 10)
    assert not control_status(node).startswith('SUCCESS:')
    assert node.arrival_monitor.started_at is None


def test_stationary_robot_does_not_report_success_before_start(executor):
    from nav_msgs.msg import Odometry
    ready_path(executor)
    executor._arrival_odometry(Odometry())
    assert control_status(executor).startswith('READY:')


def test_controller_failure_cannot_turn_into_success_from_idle_measurements(arriving_executor):
    from action_msgs.msg import GoalStatus
    node = arriving_executor
    node.test_goal_handle.get_result_async.return_value.set_result(
        SimpleNamespace(status=GoalStatus.STATUS_ABORTED))
    for index in range(25):
        arrival_sample(node, index / 10)
    assert control_status(node).startswith('ERROR:')
    assert not node.control_requested
