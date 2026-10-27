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
    bridge.planning_log = 'Log della ricerca precedente\n'
    bridge.rejected_starts.add(123)
    bridge.compute_path_client.wait_for_server = Mock(return_value=False)
    bridge.tf_buffer.can_transform = Mock(return_value=True)
    old_goal = Mock()
    bridge.active_planning_goal_handle = old_goal
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    bridge._goal_callback(pose)
    assert bridge.path_publisher.publish.call_args[0][0].poses == []
    old_goal.cancel_goal_async.assert_called_once()
    assert bridge.search_generation == 1
    assert bridge.planning_status_publisher.publish.call_args[0][0].data == (
        'ERROR: planner server unavailable')
    assert 'Log della ricerca precedente' not in bridge.planning_log
    assert 'NUOVA RICERCA #1' in bridge.planning_log
    assert 'planner server unavailable' in bridge.planning_log
    assert not bridge.rejected_starts


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


def test_service_success_publishes_bound_metadata_and_ignores_stale_results(bridge):
    bridge.connector_publisher = Mock()
    bridge.adjusted_goal_publisher = Mock()
    bridge.adjusted_start_publisher = Mock()
    path = Path()
    path.header.stamp.sec = 42
    path.header.stamp.nanosec = 7
    path.poses = [PoseStamped(), PoseStamped()]
    path.poses[1].pose.position.x = 2.0
    candidate = GoalCandidate(PoseStamped(), 0.0, 0.0, 0.0)
    future = Mock()
    future.result.return_value = SimpleNamespace(
        success=True, path=path, connector_end_index=1)
    bridge._explicit_start_result(future, 0, candidate, candidate)
    metadata = bridge.connector_publisher.publish.call_args[0][0]
    assert metadata.path == path
    assert metadata.end_index == 1
    adjusted = bridge.adjusted_goal_publisher.publish.call_args[0][0]
    assert adjusted.header.stamp == path.header.stamp
    selected_start = bridge.adjusted_start_publisher.publish.call_args[0][0]
    assert selected_start.header.stamp == path.header.stamp
    assert selected_start.pose == path.poses[1].pose
    bridge.path_publisher.publish.assert_called_once_with(path)
    bridge.search_generation = 1
    stale = Mock()
    bridge._explicit_start_result(stale, 0, candidate, candidate)
    stale.result.assert_not_called()


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
        SimpleNamespace(success=False, path=Path(), error='Initial Dubins connector exceeds 2 m'),
        SimpleNamespace(success=True, path=planned, error='', connector_end_index=0),
    ]
    futures = [Mock() for _ in responses]
    for future, response in zip(futures, responses):
        future.result.return_value = response
    bridge.explicit_start_client.call_async = Mock(side_effect=futures)
    bridge._send_next_candidate(bridge.search_generation)
    assert 'TENTATIVO 1/3' in bridge.planning_log
    assert 'Calcolo in corso' in bridge.planning_log
    for future in futures:
        future.add_done_callback.call_args[0][0](future)
    transcript = bridge.planning_log
    assert transcript.count('TENTATIVO ') == 3
    assert transcript.count('  Start: ') == 3
    assert transcript.count('  Arrivo: ') == 3
    assert 'yaw=+90.0° [frame=map]' in transcript
    assert 'x=+2.000 m' in transcript
    assert 'x=+2.100 m' in transcript
    assert 'x=+2.200 m' in transcript
    assert 'FALLITO: Virtual start or goal is occupied' in transcript
    assert 'FALLITO: Initial Dubins connector exceeds 2 m' in transcript
    assert transcript.count('SUCCESSO:') == 1
    assert 'Start effettivo:' in transcript
    assert transcript.index('TENTATIVO 1/3') < transcript.index('FALLITO:')
    assert transcript.index('TENTATIVO 3/3') < transcript.index('SUCCESSO:')
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
    assert 'FALLITO: SMAC found no path' in bridge.planning_log
    assert 'ERRORE: no feasible planning start/goal pair found' in bridge.planning_log
    assert 'SUCCESSO:' not in bridge.planning_log


@pytest.mark.parametrize('case,reason', [
    ('reject', 'Richiesta rifiutata dal planner'),
    ('send_error', 'Invio al planner fallito: transport error'),
    ('result_error', 'Ricezione del risultato fallita: transport error'),
    ('empty', 'Il planner ha restituito un percorso vuoto'),
    ('abort', 'ABORTED (6); dettaglio non fornito dal planner'),
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
    assert 'FALLITO: ' in bridge.planning_log
    assert reason in bridge.planning_log
    bridge._send_next_candidate.assert_called_once_with(0)


def test_synchronous_service_error_is_logged_as_a_failed_attempt(bridge):
    prepare_logged_search(bridge, 1)
    bridge.explicit_start_client.call_async = Mock(side_effect=RuntimeError('service unavailable'))
    bridge._send_next_candidate(bridge.search_generation)
    assert 'TENTATIVO 1/1' in bridge.planning_log
    assert 'FALLITO: Invio al servizio fallito: service unavailable' in bridge.planning_log
    assert 'ERRORE:' in bridge.planning_log


def test_rejected_connector_skips_same_start_and_refills_attempt_budget(bridge):
    prepare_logged_search(bridge, 3)
    other_pose = copy.deepcopy(bridge.real_start)
    other_pose.pose.position.x = 1.5
    bridge.start_candidates.append(GoalCandidate(other_pose, 0.5, 0.0, 0.5))
    bridge.max_planning_attempts = 3
    bridge.planning_pair_limit = 3
    bridge.planning_pairs = bridge._build_planning_pairs(2, 3)
    quantized = copy.deepcopy(bridge.real_start)
    quantized.pose.position.x = 1.025
    planned = Path()
    planned.poses = [PoseStamped(), PoseStamped()]
    responses = [
        SimpleNamespace(
            success=False, path=Path(), error='Initial Dubins connector exceeds limit',
            start_rejected=True, connector_checked=True, connector_required=True,
            planning_start=quantized, connector_length=4.5, connector_max_length=2.0),
        SimpleNamespace(
            success=False, path=Path(), error='SMAC found no path', start_rejected=False),
        SimpleNamespace(success=True, path=planned, error='', connector_end_index=0),
    ]
    futures = [Mock() for _ in responses]
    for future, response in zip(futures, responses):
        future.result.return_value = response
    bridge.explicit_start_client.call_async = Mock(side_effect=futures)
    bridge._send_next_candidate(bridge.search_generation)
    for future in futures:
        future.add_done_callback.call_args[0][0](future)
    requests = [call[0][0] for call in bridge.explicit_start_client.call_async.call_args_list]
    assert [request.start.pose.position.x for request in requests] == [1.0, 1.5, 1.5]
    assert [request.goal.pose.position.x for request in requests] == [2.0, 2.0, 2.1]
    assert bridge.planning_attempts == 3
    assert bridge.candidate_index == 5  # Two cached pairs were skipped, without requests.
    assert id(bridge.start_candidates[0]) in bridge.rejected_starts
    assert id(bridge.start_candidates[1]) not in bridge.rejected_starts
    assert bridge.planning_log.count('  Start reale: ') == 3
    assert 'Start virtuale quantizzato: x=+1.025 m' in bridge.planning_log
    assert 'Lunghezza raccordo: 4.500 m' in bridge.planning_log
    assert 'Limite raccordo: 2.000 m' in bridge.planning_log
    assert 'SMAC non eseguito per questo tentativo' in bridge.planning_log
    assert 'TENTATIVO 3/3' in bridge.planning_log
    assert 'SUCCESSO:' in bridge.planning_log
    assert 'TENTATIVO 4/' not in bridge.planning_log


def test_all_cached_starts_finish_without_sending_more_requests(bridge):
    prepare_logged_search(bridge, 2)
    bridge.rejected_starts.add(id(bridge.start_candidates[0]))
    bridge.explicit_start_client.call_async = Mock()
    bridge._send_next_candidate(bridge.search_generation)
    bridge.explicit_start_client.call_async.assert_not_called()
    assert bridge.planning_attempts == 0
    assert 'ERRORE:' in bridge.planning_log


def test_pair_search_is_lazy_and_keeps_diagonal_order(bridge):
    pairs = bridge._build_planning_pairs(1000000, 1000000)
    assert iter(pairs) is pairs
    assert list(islice(pairs, 6)) == [(0, 0), (0, 1), (1, 0), (0, 2), (1, 1), (2, 0)]
    assert list(bridge._build_planning_pairs(0, 10)) == []
    assert list(bridge._build_planning_pairs(1, 2)) == [(0, 0), (0, 1)]


def test_exact_start_diagnostic_reports_zero_connector_length(bridge):
    bridge._log_connector_diagnostics(SimpleNamespace(
        planning_start=PoseStamped(), connector_checked=True, connector_required=False,
        connector_length=0.0, connector_max_length=2.0))
    assert 'Lunghezza raccordo: 0.000 m' in bridge.planning_log
    assert 'Raccordo non necessario: start originale.' in bridge.planning_log
