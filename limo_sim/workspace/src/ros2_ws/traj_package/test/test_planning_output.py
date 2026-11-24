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

"""Exercise planning output without a controller process."""

import copy
from itertools import islice
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip('rclpy')
from action_msgs.msg import GoalStatus  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
import rclpy  # noqa: E402

from traj_package.rviz_goal_bridge import GoalCandidate, RvizGoalBridge  # noqa: E402


@pytest.fixture
def bridge():
    rclpy.init()
    node = RvizGoalBridge()
    node.path_publisher = Mock()
    node.visualization_path_publisher = Mock()
    node.planning_status_publisher = Mock()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_success_publishes_path_without_controller(bridge):
    path = Path()
    path.poses = [PoseStamped()]
    candidate = GoalCandidate(PoseStamped(), 0.0, 0.0, 0.0)
    future = Mock()
    future.result.return_value = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(path=path),
    )
    bridge._result_callback(future, None, 0, candidate, candidate)
    bridge.path_publisher.publish.assert_called_once_with(path)
    assert bridge.planning_status_publisher.publish.call_args[0][0].data == (
        'READY: path contains 1 poses')


def test_new_goal_invalidates_path_even_when_planner_is_unavailable(bridge):
    bridge.planning_log = 'Previous search log\n'
    bridge.compute_path_client.wait_for_server = Mock(return_value=False)
    bridge.tf_buffer.can_transform = Mock(return_value=True)
    old_goal = Mock()
    bridge.active_planning_goal_handle = old_goal
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    bridge._goal_callback(pose)
    assert bridge.path_publisher.publish.call_args[0][0].poses == []
    assert bridge.visualization_path_publisher.publish.call_args[0][0].poses == []
    assert bridge.visualization_path_publisher.publish.call_args[0][0].header.frame_id == 'map'
    old_goal.cancel_goal_async.assert_called_once()
    assert bridge.search_generation == 1
    assert bridge.planning_status_publisher.publish.call_args[0][0].data == (
        'ERROR: planner server unavailable')
    assert 'Previous search log' not in bridge.planning_log
    assert 'NEW SEARCH #1' in bridge.planning_log
    assert 'planner server unavailable' in bridge.planning_log


def test_missing_localization_explains_how_to_activate_planning(bridge):
    bridge.compute_path_client.wait_for_server = Mock(return_value=False)
    bridge.tf_buffer.can_transform = Mock(return_value=False)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    bridge._goal_callback(pose)
    assert bridge.path_publisher.publish.call_args[0][0].poses == []
    assert bridge.planning_status_publisher.publish.call_args[0][0].data == (
        'ERROR: localization unavailable; set 2D Pose Estimate in RViz')


def test_late_planning_result_cannot_restore_an_invalidated_path(bridge):
    bridge.search_generation = 1
    bridge._result_callback(Mock(), None, 0, None, None)
    bridge.path_publisher.publish.assert_not_called()


def test_foxy_service_sends_real_and_virtual_starts(bridge):
    bridge.use_explicit_start_service = True
    bridge.supports_explicit_start = False
    bridge.explicit_start_client.call_async = Mock()
    real = PoseStamped()
    real.header.frame_id = 'map'
    virtual = PoseStamped()
    virtual.header.frame_id = 'map'
    virtual.pose.position.x = 0.5
    bridge.real_start = real
    bridge.start_candidates = [GoalCandidate(virtual, 0.5, 0.0, 0.5)]
    bridge.candidates = [GoalCandidate(PoseStamped(), 0.0, 0.0, 0.0)]
    bridge.planning_pairs = [(0, 0)]
    bridge._send_next_candidate(bridge.search_generation)
    request = bridge.explicit_start_client.call_async.call_args[0][0]
    assert request.real_start.pose.position.x == 0.0
    assert request.start.pose.position.x == 0.5


def test_service_success_publishes_bound_endpoints_and_ignores_stale_results(bridge):
    bridge.adjusted_goal_publisher = Mock()
    bridge.adjusted_start_publisher = Mock()
    path = Path()
    path.header.stamp.sec = 42
    path.header.stamp.nanosec = 7
    path.poses = [PoseStamped(), PoseStamped(), PoseStamped()]
    start = PoseStamped()
    start.pose.position.x = 2.0
    candidate = GoalCandidate(start, 0.0, 0.0, 0.0)
    future = Mock()
    future.result.return_value = SimpleNamespace(
        success=True, path=path)
    bridge._explicit_start_result(future, 0, candidate, candidate)
    adjusted = bridge.adjusted_goal_publisher.publish.call_args[0][0]
    assert adjusted.header.stamp == path.header.stamp
    selected_start = bridge.adjusted_start_publisher.publish.call_args[0][0]
    assert selected_start.header.stamp == path.header.stamp
    assert selected_start.pose == start.pose
    bridge.path_publisher.publish.assert_called_once_with(path)
    bridge.visualization_path_publisher.publish.assert_called_once_with(path)
    bridge.search_generation = 1
    stale = Mock()
    bridge._explicit_start_result(stale, 0, candidate, candidate)
    stale.result.assert_not_called()


@pytest.mark.parametrize('planning_start', [None, PoseStamped()])
def test_selected_start_falls_back_to_candidate_without_a_valid_planner_pose(
        bridge, planning_start):
    path = Path()
    path.header.frame_id = 'map'
    path.poses = [PoseStamped(), PoseStamped(), PoseStamped()]
    start = PoseStamped()
    start.pose.position.x = 0.5
    candidate = GoalCandidate(start, 0.0, 0.0, 0.0)
    bridge.adjusted_start_publisher = Mock()
    bridge._publish_success(candidate, candidate, path, planning_start)
    selected = bridge.adjusted_start_publisher.publish.call_args[0][0]
    assert selected.pose == start.pose


def prepare_logged_search(bridge, count):
    bridge.use_explicit_start_service = True
    start = PoseStamped()
    start.header.frame_id = 'map'
    start.pose.position.x = 1.0
    bridge._set_pose_yaw(start, 1.5707963267948966)
    bridge.real_start = start
    bridge.start_candidates = [GoalCandidate(start, 0.0, 0.0, 0.0)]
    bridge.candidates = []
    for index in range(count):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 2.0 + 0.1 * index
        goal.pose.orientation.w = 1.0
        bridge.candidates.append(GoalCandidate(goal, 0.1 * index, 0.0, 0.1 * index))
    bridge.planning_pairs = [(0, index) for index in range(count)]
    bridge.planning_log_publisher = Mock()


def test_attempt_log_records_each_pair_and_failure_until_success(bridge):
    prepare_logged_search(bridge, 3)
    planned = Path()
    planned.poses = [PoseStamped(), PoseStamped()]
    responses = [
        SimpleNamespace(success=False, path=Path(), error='Virtual start or goal is occupied'),
        SimpleNamespace(success=False, path=Path(), error='SMAC found no path'),
        SimpleNamespace(success=True, path=planned, error=''),
    ]
    futures = [Mock() for _ in responses]
    for future, response in zip(futures, responses):
        future.result.return_value = response
    bridge.explicit_start_client.call_async = Mock(side_effect=futures)
    bridge._send_next_candidate(bridge.search_generation)
    assert 'ATTEMPT 1/3' in bridge.planning_log
    assert 'Planning in progress' in bridge.planning_log
    for future in futures:
        future.add_done_callback.call_args[0][0](future)
    transcript = bridge.planning_log
    assert transcript.count('ATTEMPT ') == 3
    assert transcript.count('  Start: ') == 3
    assert transcript.count('  Goal: ') == 3
    assert 'yaw=+90.0° [frame=map]' in transcript
    assert 'x=+2.000 m' in transcript
    assert 'x=+2.100 m' in transcript
    assert 'x=+2.200 m' in transcript
    assert 'FAILED: Virtual start or goal is occupied' in transcript
    assert 'FAILED: SMAC found no path' in transcript
    assert transcript.count('SUCCESS:') == 1
    assert 'Actual start:' in transcript
    assert transcript.index('ATTEMPT 1/3') < transcript.index('FAILED:')
    assert transcript.index('ATTEMPT 3/3') < transcript.index('SUCCESS:')
    assert bridge.explicit_start_client.call_async.call_count == 3
    bridge.path_publisher.publish.assert_called_once_with(planned)
    assert bridge.planning_log_publisher.publish.call_args[0][0].data == transcript
    bridge.search_generation += 1
    stale = Mock()
    bridge._explicit_start_result(stale, bridge.search_generation - 1, None, None)
    stale.result.assert_not_called()
    assert bridge.planning_log == transcript


def test_exhausted_search_logs_failure_and_final_summary(bridge):
    prepare_logged_search(bridge, 1)
    future = Mock()
    future.result.return_value = SimpleNamespace(
        success=False, path=Path(), error='SMAC found no path')
    bridge.explicit_start_client.call_async = Mock(return_value=future)
    bridge._send_next_candidate(bridge.search_generation)
    future.add_done_callback.call_args[0][0](future)
    assert 'FAILED: SMAC found no path' in bridge.planning_log
    assert 'ERROR: no feasible planning start/goal pair found' in bridge.planning_log
    assert 'SUCCESS:' not in bridge.planning_log


@pytest.mark.parametrize('case,reason', [
    ('reject', 'Planner rejected the request'),
    ('send_error', 'Failed to send planning request: transport error'),
    ('result_error', 'Failed to receive planning result: transport error'),
    ('empty', 'Planner returned an empty path'),
    ('abort', 'ABORTED (6); planner provided no details'),
])
def test_action_failure_reasons_are_logged_without_inventing_diagnostics(bridge, case, reason):
    bridge._send_next_candidate = Mock()
    candidate = GoalCandidate(PoseStamped(), 0.0, 0.0, 0.0)
    future = Mock()
    if case in ('send_error', 'result_error'):
        future.result.side_effect = RuntimeError('transport error')
    elif case == 'reject':
        future.result.return_value = SimpleNamespace(accepted=False)
    else:
        future.result.return_value = SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED if case == 'empty' else GoalStatus.STATUS_ABORTED,
            result=SimpleNamespace(path=Path()))
    if case in ('reject', 'send_error'):
        bridge._goal_response_callback(future, 0, candidate, candidate)
    else:
        bridge._result_callback(future, None, 0, candidate, candidate)
    assert 'FAILED: ' in bridge.planning_log
    assert reason in bridge.planning_log
    bridge._send_next_candidate.assert_called_once_with(0)


def test_synchronous_service_error_is_logged_as_a_failed_attempt(bridge):
    prepare_logged_search(bridge, 1)
    bridge.explicit_start_client.call_async = Mock(side_effect=RuntimeError('service unavailable'))
    bridge._send_next_candidate(bridge.search_generation)
    assert 'ATTEMPT 1/1' in bridge.planning_log
    assert 'FAILED: Failed to send service request: service unavailable' in bridge.planning_log
    assert 'ERROR:' in bridge.planning_log


def test_failed_pairs_preserve_diagonal_order_and_attempt_budget(bridge):
    prepare_logged_search(bridge, 3)
    other_pose = copy.deepcopy(bridge.real_start)
    other_pose.pose.position.x = 1.5
    bridge.start_candidates.append(GoalCandidate(other_pose, 0.5, 0.0, 0.5))
    bridge.max_planning_attempts = 3
    bridge.planning_pair_limit = 3
    bridge.planning_pairs = bridge._build_planning_pairs(2, 3)
    planned = Path()
    planned.poses = [PoseStamped(), PoseStamped()]
    responses = [
        SimpleNamespace(success=False, path=Path(), error='Virtual start is occupied'),
        SimpleNamespace(success=False, path=Path(), error='SMAC found no path'),
        SimpleNamespace(success=True, path=planned, error=''),
    ]
    futures = [Mock() for _ in responses]
    for future, response in zip(futures, responses):
        future.result.return_value = response
    bridge.explicit_start_client.call_async = Mock(side_effect=futures)
    bridge._send_next_candidate(bridge.search_generation)
    for future in futures:
        future.add_done_callback.call_args[0][0](future)
    requests = [call[0][0] for call in bridge.explicit_start_client.call_async.call_args_list]
    assert [request.start.pose.position.x for request in requests] == [1.0, 1.0, 1.5]
    assert [request.goal.pose.position.x for request in requests] == [2.0, 2.1, 2.0]
    assert bridge.planning_attempts == 3
    assert bridge.candidate_index == 3
    assert bridge.planning_log.count('  Robot start: ') == 3
    assert 'ATTEMPT 3/3' in bridge.planning_log
    assert 'SUCCESS:' in bridge.planning_log
    assert 'ATTEMPT 4/' not in bridge.planning_log


def test_reaching_attempt_budget_finishes_without_sending_more_requests(bridge):
    prepare_logged_search(bridge, 2)
    bridge.planning_attempts = 1
    bridge.planning_pair_limit = 1
    bridge.explicit_start_client.call_async = Mock()
    bridge._send_next_candidate(bridge.search_generation)
    bridge.explicit_start_client.call_async.assert_not_called()
    assert bridge.planning_attempts == 1
    assert 'ERROR:' in bridge.planning_log


def test_pair_search_is_lazy_and_keeps_diagonal_order(bridge):
    pairs = bridge._build_planning_pairs(1000000, 1000000)
    assert iter(pairs) is pairs
    assert list(islice(pairs, 6)) == [(0, 0), (0, 1), (1, 0), (0, 2), (1, 1), (2, 0)]
    assert list(bridge._build_planning_pairs(0, 10)) == []
    assert list(bridge._build_planning_pairs(1, 2)) == [(0, 0), (0, 1)]


def test_missing_planning_start_does_not_invent_pose_diagnostics(bridge):
    bridge._log_planning_start(SimpleNamespace(planning_start=PoseStamped()))
    assert bridge.planning_log == ''


def test_virtual_start_success_uses_quantized_pose(bridge):
    prepare_logged_search(bridge, 1)
    requested_start = copy.deepcopy(bridge.real_start)
    requested_start.pose.position.y = 0.212
    bridge.start_candidates[0] = GoalCandidate(requested_start, 0.212, 0.0, 0.212)
    quantized = copy.deepcopy(requested_start)
    quantized.pose.position.y = 0.225
    planned = Path()
    planned.header.frame_id = 'map'
    planned.header.stamp.sec = 42
    planned.poses = [quantized, bridge.candidates[0].pose]
    response = SimpleNamespace(
        success=True, path=planned, error='', planning_start=quantized)
    future = Mock()
    future.result.return_value = response
    bridge.explicit_start_client.call_async = Mock(return_value=future)
    bridge.adjusted_start_publisher = Mock()
    bridge._send_next_candidate(bridge.search_generation)
    future.add_done_callback.call_args[0][0](future)
    actual_start = bridge.adjusted_start_publisher.publish.call_args[0][0]
    assert actual_start.pose.position.y == 0.225
    assert actual_start.header.stamp == planned.header.stamp
    assert 'Quantized virtual start:' in bridge.planning_log
    assert 'SUCCESS:' in bridge.planning_log


@pytest.mark.parametrize('cost,expected', [(0, True), (98, True), (99, False), (100, False), (-1, False)])
def test_point_footprint_checks_containing_cell_away_from_center(cost, expected):
    from nav_msgs.msg import OccupancyGrid
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = 1.0
    grid.info.width = grid.info.height = 3
    grid.info.origin.orientation.w = 1.0
    grid.data = [100] * 9
    grid.data[4] = cost
    node = SimpleNamespace(costmap=grid, footprint_half_length=0.0,
                           footprint_half_width=0.0, collision_cost_threshold=99,
                           _quaternion_to_yaw=RvizGoalBridge._quaternion_to_yaw,
                           _world_to_grid_continuous=RvizGoalBridge._world_to_grid_continuous)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.orientation.w = 1.0
    pose.pose.position.x, pose.pose.position.y = 1.01, 1.99
    assert RvizGoalBridge._is_footprint_free(node, pose) is expected


def test_point_footprint_rotated_grid_and_bounds():
    import math
    from nav_msgs.msg import OccupancyGrid
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = 1.0
    grid.info.width = grid.info.height = 3
    grid.info.origin.position.x, grid.info.origin.position.y = 10.0, 20.0
    grid.info.origin.orientation.z = math.sin(math.pi / 4)
    grid.info.origin.orientation.w = math.cos(math.pi / 4)
    grid.data = [0] * 9
    node = SimpleNamespace(costmap=grid, footprint_half_length=0.0,
                           footprint_half_width=0.0, collision_cost_threshold=99,
                           _quaternion_to_yaw=RvizGoalBridge._quaternion_to_yaw,
                           _world_to_grid_continuous=RvizGoalBridge._world_to_grid_continuous)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.orientation.w = 1.0
    pose.pose.position.x, pose.pose.position.y = 8.01, 21.01
    assert RvizGoalBridge._is_footprint_free(node, pose)
    pose.pose.position.x, pose.pose.position.y = 10.1, 21.01
    assert not RvizGoalBridge._is_footprint_free(node, pose)


@pytest.mark.parametrize('heading', [0.0, 1.5707963267948966, 0.7853981633974483])
@pytest.mark.parametrize('map_heading', [0.0, 1.5707963267948966])
@pytest.mark.parametrize('blocked_cost', [None, 100, -1])
def test_zero_length_ten_cm_width_checks_rotated_segment(heading, map_heading, blocked_cost):
    import math
    from nav_msgs.msg import OccupancyGrid
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = 0.01
    grid.info.width = grid.info.height = 40
    grid.info.origin.position.x = 10.0
    grid.info.origin.position.y = 20.0
    grid.info.origin.orientation.z = math.sin(map_heading / 2)
    grid.info.origin.orientation.w = math.cos(map_heading / 2)
    grid.data = [0] * 1600
    if blocked_cost is not None:
        x = 0.205 - math.sin(heading) * 0.045
        y = 0.205 + math.cos(heading) * 0.045
        grid.data[math.floor(y / 0.01) * 40 + math.floor(x / 0.01)] = blocked_cost
    else:
        # An obstacle ahead of the segment must not add longitudinal extent.
        x = 0.205 + math.cos(heading) * 0.04
        y = 0.205 + math.sin(heading) * 0.04
        grid.data[math.floor(y / 0.01) * 40 + math.floor(x / 0.01)] = 100
    node = SimpleNamespace(costmap=grid, footprint_half_length=0.0,
                           footprint_half_width=0.05, collision_cost_threshold=99,
                           _quaternion_to_yaw=RvizGoalBridge._quaternion_to_yaw,
                           _world_to_grid_continuous=RvizGoalBridge._world_to_grid_continuous)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = 10 + 0.205 * (math.cos(map_heading) - math.sin(map_heading))
    pose.pose.position.y = 20 + 0.205 * (math.sin(map_heading) + math.cos(map_heading))
    pose.pose.orientation.z = math.sin((heading + map_heading) / 2)
    pose.pose.orientation.w = math.cos((heading + map_heading) / 2)
    assert RvizGoalBridge._is_footprint_free(node, pose) is (blocked_cost is None)
    pose.pose.position.x = 100.0
    assert not RvizGoalBridge._is_footprint_free(node, pose)
