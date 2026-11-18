"""Behavior checks for the Python translations; no ROS installation required."""
from copy import deepcopy
import importlib
import math
from pathlib import Path as FilePath
import unittest
from unittest.mock import patch

from ...include.python_ver.sampling_mpc import MpcConfig, MpcControl, MpcState, MpcFamily
from ...include.python_ver.ros_compat import (
    Costmap2D, Costmap2DROS, ObstacleLayer, LocalNode, Header, Point, Pose, Pose2D, PoseStamped,
    Path, Path2D, Twist, Twist2D, Trajectory2D, IllegalTrajectoryException,
    NoLegalTrajectoriesException, PlannerException,
)
from ...include.python_ver.mpc_timing import MpcTimingSample, CriticPrepareTiming
from ...include.python_ver.mpc_timing_scope import MpcTimingScope
from .sampling_mpc import SamplingMpc
from .ackermann_kinematics_critic import commandRespectsAckermann
from .obstacle_guidance import ObstacleGuidance
from .mppi_cost_critics import MppiObstacleCritic, MppiPathCritic
from .mpc_debug import MpcDebugPublisher
from .mpc_timing import MpcTimingPublisher
from .ackermann_mpc_controller import AckermannMPCController
from . import controller_server_main as server


def small_config(**overrides):
    values = dict(time_steps=10, control_segments=2, batch_size=23, velocity_samples=3, curvature_samples=7)
    values.update(overrides)
    return MpcConfig(**values)


def line_path():
    header = Header(frame_id='map')
    return Path(header, [PoseStamped(deepcopy(header), Pose(Point(i * 0.02, 0))) for i in range(31)])


def grid_host():
    return Costmap2DROS(Costmap2D(80, 80, 0.05, origin_x=-2, origin_y=-2))


class TranslationTests(unittest.TestCase):
    def test_every_original_module_has_an_importable_translation(self):
        root = FilePath(__file__).resolve().parents[2]
        for source in (root / 'src').glob('*.cpp'):
            importlib.import_module('limo_dwb_critics.src.python_ver.' + source.stem)
        for header in (root / 'include/limo_dwb_critics').glob('*.hpp'):
            importlib.import_module('limo_dwb_critics.include.python_ver.' + header.stem)

    def test_ackermann_constraints(self):
        check = lambda x, y, w: commandRespectsAckermann(x, y, w, 0.462, 1e-6, 1e-3)
        self.assertTrue(check(0.5, 0, 1.0))
        self.assertTrue(check(-0.5, 0, -1.0))
        self.assertFalse(check(0.2, 0.01, 0.1))
        self.assertFalse(check(0, 0, 0.5))
        self.assertFalse(check(0.2, 0, 0.5))

    def test_invalid_parameters_are_rejected(self):
        for changes in ({'dt': 0}, {'time_steps': 1}, {'steering_std': math.nan},
                        {'structured_actions': True}, {'startup_enabled': True}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                SamplingMpc(small_config(**changes))

    def test_dataclass_lists_do_not_share_storage(self):
        a, b = MpcConfig(), MpcConfig()
        a.block_fractions.append(1)
        self.assertEqual(len(b.block_fractions), 3)

    def test_rollout_copies_initial_and_integrates_straight_motion(self):
        mpc = SamplingMpc(small_config(rear_axle_to_base=0))
        initial = MpcState(control=MpcControl(0.2, 0))
        rollout = mpc.rollout(initial, [MpcControl(0.2, 0) for _ in range(10)])
        self.assertAlmostEqual(rollout.states[-1].x, 0.2 * 10 * mpc.config().dt)
        self.assertEqual(initial.x, 0)
        self.assertIsNot(rollout.states[0], rollout.states[1])
        self.assertEqual(rollout.effort_cost, 0)

    def test_reverse_brakes_to_zero_before_changing_direction(self):
        mpc = SamplingMpc(small_config())
        current = MpcControl(0.02, 0)
        stopped = mpc.advance(current, MpcControl(-0.1, 0))
        self.assertEqual(stopped.velocity, 0)
        self.assertLess(mpc.advance(stopped, MpcControl(-0.1, 0)).velocity, 0)

    def test_every_step_obeys_actuation_limits(self):
        mpc = SamplingMpc(small_config())
        c = mpc.config()
        rollout = mpc.rollout(MpcState(), [MpcControl(10, 10) for _ in range(c.time_steps)])
        for a, b in zip(rollout.states, rollout.states[1:]):
            self.assertLessEqual(abs(b.control.velocity - a.control.velocity), c.acceleration * c.dt + 1e-12)
            self.assertLessEqual(abs(b.control.steering - a.control.steering), c.steering_rate * c.dt + 1e-12)
            self.assertLessEqual(abs(mpc.yawRate(b.control) - mpc.yawRate(a.control)), c.yaw_acceleration * c.dt + 1e-12)
            self.assertLessEqual(abs(mpc.yawRate(b.control)), c.max_yaw_rate + 1e-12)

    def test_reverse_distance_cost_is_not_averaged_over_horizon(self):
        c = small_config(reverse_distance_weight=2)
        mpc = SamplingMpc(c)
        result = mpc.rollout(MpcState(control=MpcControl(-0.1, 0)), [MpcControl(-0.1, 0) for _ in range(c.time_steps)])
        self.assertAlmostEqual(result.effort_cost, 2 * 0.1 * c.dt * c.time_steps)

    def test_failed_search_clears_warm_start(self):
        mpc = SamplingMpc(small_config())
        mpc.solve(MpcState(), lambda r, limit: 0)
        self.assertTrue(mpc.hasWarmStart())
        solution = mpc.solve(MpcState(), lambda r, limit: math.inf)
        self.assertTrue(math.isinf(solution.cost))
        self.assertFalse(mpc.hasWarmStart())

    def test_evaluator_exception_clears_warm_start(self):
        mpc = SamplingMpc(small_config())
        mpc.solve(MpcState(), lambda r, limit: 0)
        def fail(rollout, limit):
            raise RuntimeError('environment failed')
        with self.assertRaises(RuntimeError):
            mpc.solve(MpcState(), fail)
        self.assertFalse(mpc.hasWarmStart())

    def test_observer_exception_does_not_change_selection(self):
        mpc = SamplingMpc(small_config())
        def fail(*args):
            raise RuntimeError('telemetry failed')
        solution = mpc.solve(MpcState(), lambda r, limit: 0, observer=fail)
        self.assertTrue(math.isfinite(solution.cost))
        self.assertEqual(solution.generated_count, mpc.config().batch_size)

    def test_duplicate_candidates_are_removed_before_observation(self):
        c = small_config(deduplicate=True)
        observed = []
        solution = SamplingMpc(c).solve(MpcState(), lambda r, limit: 1,
            observer=lambda r, family, identifier, cost: observed.append(identifier))
        self.assertGreater(solution.duplicate_count, 0)
        self.assertEqual(solution.generated_count + solution.duplicate_count, c.batch_size)
        self.assertEqual(len(observed), solution.generated_count)

    def test_structured_and_startup_families(self):
        c = small_config(time_steps=12, control_segments=3, structured_actions=True,
                         min_velocity=0, action_variants=1, startup_enabled=True, batch_size=23)
        families = []
        solution = SamplingMpc(c).solve(MpcState(), lambda r, limit: 1,
            observer=lambda r, family, identifier, cost: families.append(family))
        self.assertEqual(families.count(MpcFamily.Structured), 9)
        self.assertEqual(families.count(MpcFamily.Monophase), 9)
        self.assertEqual(families.count(MpcFamily.Startup), 3)
        self.assertEqual(solution.generated_count, 23)

    def test_guidance_does_not_cut_blocked_diagonal_corners(self):
        costs = [254] * 25
        costs[6] = costs[12] = 0
        guide = ObstacleGuidance()
        guide.update(5, 5, 0.1, costs, 0, 0)
        self.assertTrue(guide.setTarget(2, 2))
        self.assertTrue(math.isinf(guide.distance(1, 1)))

    def test_guidance_subcell_progress_and_updates(self):
        guide = ObstacleGuidance()
        guide.update(40, 40, 0.05, [0] * 1600, 0.2, 0)
        self.assertTrue(guide.setTarget(30, 20))
        self.assertAlmostEqual(guide.distance(10, 20), 1.0)
        self.assertAlmostEqual(guide.distance(10.1, 20), 0.995)
        guide.update(40, 40, 0.05, [0] * 1600, 0.2, 0)
        self.assertTrue(math.isinf(guide.distance(10, 20)))
        with self.assertRaises(ValueError):
            guide.update(0, 1, 0.1, [], 0, 0)

    def test_obstacle_cost_normalization_ignores_shared_initial_center(self):
        host = grid_host()
        host.getCostmap().costs[:] = [127] * 6400
        critic = MppiObstacleCritic(costmap_ros=host)
        critic.onInit()
        critic.prepare(Pose2D(), Twist2D(), Pose2D(2, 0), Path2D())
        trajectory = Trajectory2D(poses=[Pose2D(), Pose2D(0.1, 0), Pose2D(0.2, 0)])
        self.assertAlmostEqual(critic.scoreTrajectory(trajectory), 0.5)

    def test_full_footprint_finds_interior_obstacle_and_between_pose_collision(self):
        host = grid_host()
        critic = MppiObstacleCritic(costmap_ros=host)
        critic.full_footprint_check_ = True
        host.getCostmap().setCost(41, 40, 254)  # Inside the footprint, away from its center.
        critic.prepare(Pose2D(), Twist2D(), Pose2D(2, 0), Path2D())
        with self.assertRaises(IllegalTrajectoryException):
            critic.scoreTrajectory(Trajectory2D(poses=[Pose2D(), Pose2D(0.4, 0)]))
        host.getCostmap().costs[:] = [0] * 6400
        host.getCostmap().setCost(44, 40, 254)  # Between the two prediction poses.
        critic.prepare(Pose2D(), Twist2D(), Pose2D(2, 0), Path2D())
        with self.assertRaises(IllegalTrajectoryException):
            critic.scoreTrajectory(Trajectory2D(poses=[Pose2D(), Pose2D(0.6, 0)]))

    def test_unknown_center_is_rejected(self):
        host = grid_host()
        host.getCostmap().setCost(40, 40, 255)
        critic = MppiObstacleCritic(costmap_ros=host)
        critic.onInit()
        critic.prepare(Pose2D(), Twist2D(), Pose2D(2, 0), Path2D())
        with self.assertRaises(IllegalTrajectoryException):
            critic.scoreTrajectory(Trajectory2D(poses=[Pose2D(), Pose2D(0.2, 0)]))

    def test_path_cost_prefers_reference_and_allows_configured_reverse(self):
        host = grid_host()
        critic = MppiPathCritic(costmap_ros=host)
        critic.onInit()
        path = Path2D(poses=[Pose2D(i * 0.05, 0) for i in range(40)])
        self.assertTrue(critic.prepare(Pose2D(), Twist2D(), path.poses[-1], path))
        on_path = Trajectory2D(poses=[Pose2D(), Pose2D(0.5, 0)])
        off_path = Trajectory2D(poses=[Pose2D(), Pose2D(0.5, 0.4)])
        self.assertLess(critic.scoreTrajectory(on_path), critic.scoreTrajectory(off_path))
        on_path.velocity.x = -0.1
        self.assertTrue(math.isfinite(critic.scoreTrajectory(on_path)))

    def test_live_obstacle_activates_path_guidance_and_clear_map_restores_alignment(self):
        host = grid_host()
        critic = MppiPathCritic(costmap_ros=host)
        critic.onInit()
        critic.guidance_enabled_ = True
        path = Path2D(poses=[Pose2D(i * 0.05, 0) for i in range(38)])
        critic.prepare(Pose2D(), Twist2D(), path.poses[-1], path)
        self.assertFalse(critic.detouring())
        grid = host.getCostmap()
        for y in range(38, 43):
            for x in range(48, 53):
                grid.setCost(x, y, 254)
        critic.prepare(Pose2D(), Twist2D(), path.poses[-1], path)
        self.assertTrue(critic.detouring())
        grid.costs[:] = [0] * (80 * 80)
        critic.prepare(Pose2D(), Twist2D(), path.poses[-1], path)
        self.assertFalse(critic.detouring())

    def test_smac_mode_requires_an_explicit_native_planner_adapter(self):
        node = LocalNode({'FollowPath.MppiPath.ObstacleGuidance.enabled': True,
                          'FollowPath.MppiPath.ObstacleGuidance.use_smac': True})
        critic = MppiPathCritic(node=node, name='MppiPath', costmap_ros=grid_host())
        with self.assertRaisesRegex(ValueError, 'injected Nav2'):
            critic.onInit()

    def test_only_marked_laser_cells_trigger_guidance_with_the_five_centimeter_margin(self):
        host = grid_host()
        laser = ObstacleLayer(80, 80, .05, origin_x=-1.75, origin_y=-1.85, costs=[255]*6400)
        host.plugins.append(laser)
        node = LocalNode({'FollowPath.MppiPath.ObstacleGuidance.enabled': True,
                          'FollowPath.MppiPath.ObstacleGuidance.trigger_layer': 'obstacle_layer',
                          'FollowPath.MppiPath.ObstacleGuidance.trigger_margin': .05})
        critic = MppiPathCritic(node=node, name='MppiPath', costmap_ros=host)
        critic.onInit()
        path = Path2D(poses=[Pose2D(i*.05, 0) for i in range(38)])
        grid = host.getCostmap()
        grid.setCost(*grid.worldToMap(.825, .025), 254)
        critic.prepare(Pose2D(), Twist2D(), path.poses[-1], path)
        self.assertFalse(critic.guidancePlanCalled())
        grid.costs[:] = [0]*6400
        laser.costs[:] = [0]*6400
        laser.setCost(*laser.worldToMap(.825, .175), 254)
        grid.setCost(*grid.worldToMap(.825, .175), 254)
        critic.prepare(Pose2D(), Twist2D(), path.poses[-1], path)
        self.assertTrue(critic.detouring())
        laser.enabled = False
        critic.reset()
        critic.prepare(Pose2D(), Twist2D(), path.poses[-1], path)
        self.assertFalse(critic.guidancePlanCalled())

    def test_missing_sensor_layer_does_not_fall_back_to_the_master(self):
        node = LocalNode({'FollowPath.MppiPath.ObstacleGuidance.enabled': True,
                          'FollowPath.MppiPath.ObstacleGuidance.trigger_layer': 'obstacle_layer'})
        critic = MppiPathCritic(node=node, name='MppiPath', costmap_ros=grid_host())
        with self.assertRaisesRegex(ValueError, 'must name an ObstacleLayer'):
            critic.onInit()

    def test_injected_smac_receives_arrival_yaw_and_exports_its_reference(self):
        class Planner:
            def createPlan(self, start, goal):
                self.goal = goal
                return Path(deepcopy(start.header), [start, goal])
        planner, host = Planner(), grid_host()
        node = LocalNode({'FollowPath.MppiPath.ObstacleGuidance.enabled': True,
                          'FollowPath.MppiPath.ObstacleGuidance.use_smac': True})
        critic = MppiPathCritic(node=node, name='MppiPath', costmap_ros=host, smac_planner=planner)
        critic.onInit()
        host.getCostmap().setCost(50, 43, 254)
        reference = Path2D(poses=[Pose2D(i * .05, 0, .4) for i in range(38)])
        critic.prepare(Pose2D(), Twist2D(), reference.poses[-1], reference)
        self.assertTrue(critic.detouring())
        self.assertAlmostEqual(planner.goal.pose.orientation.z, math.sin(.2))
        self.assertTrue(critic.guidanceReference(Pose2D()))
        self.assertTrue(math.isinf(critic.guidance_.distance(40, 40)))
        critic.reset()
        self.assertEqual(critic.guidanceReference(Pose2D()), [])

    def test_dijkstra_preview_route_keeps_field_and_respects_blocked_corners(self):
        guide = ObstacleGuidance()
        costs = [0] * 36
        costs[1 * 6 + 2] = 254
        guide.update(6, 6, .1, costs, 0, 0)
        self.assertTrue(guide.setTarget(4, 4))
        before = list(guide.distances_)
        route = guide.route(1, 1, 0)
        self.assertEqual(route[-1], (4, 4))
        for (x, y), (nx, ny) in zip(route, route[1:]):
            self.assertTrue(guide.traversable(nx, ny))
            if nx != x and ny != y:
                self.assertTrue(guide.traversable(nx, y) and guide.traversable(x, ny))
        self.assertEqual(before, guide.distances_)

    def test_timing_windows_count_only_called_critics(self):
        clock = [0.0]
        node = LocalNode({'FollowPath.MPC.Timing.enabled': True})
        pub = MpcTimingPublisher(node, 'FollowPath', 50, now=lambda: clock[0])
        pub.setCriticNames(['first', 'second'])
        pub.activate()
        sample = MpcTimingSample(cycle_ms=60, success=False,
                                 critic_prepare=[CriticPrepareTiming(2, 1, True), CriticPrepareTiming()])
        pub.finish(sample)
        clock[0] = 1.1
        pub.finish(sample)
        report = pub.publisher_.messages[-1].status[0]
        values = {v.key: v.value for v in report.values}
        self.assertEqual(values['cycles'], '2')
        self.assertEqual(values['deadline_misses'], '2')
        self.assertEqual(values['critic_prepare/first_calls'], '2')
        self.assertEqual(values['critic_prepare/second_calls'], '0')
        self.assertEqual(report.level, 1)

    def test_timing_scope_records_exceptions_and_preserves_exception(self):
        sample = MpcTimingSample()
        with self.assertRaisesRegex(RuntimeError, 'original'):
            with MpcTimingScope(sample, 'solve_ms'):
                raise RuntimeError('original')
        self.assertGreaterEqual(sample.solve_ms, 0)

    def test_debug_snapshot_copies_live_map_and_includes_winner(self):
        node = LocalNode({'FollowPath.MPC.Debug.enabled': True})
        host = grid_host()
        c = small_config()
        pub = MpcDebugPublisher(node, 'FollowPath', host, c)
        pub.activate()
        observer = pub.begin(MpcState())
        self.assertIsNotNone(observer)
        host.getCostmap().setCost(0, 0, 254)
        solution = SamplingMpc(c).solve(MpcState(), lambda r, limit: 1, observer=observer)
        pub.finish(solution)
        frame = pub.publisher_.messages[-1]
        self.assertEqual(frame.costmap.data[0], 0)
        self.assertTrue(any(c.candidate_id == solution.candidate_id for c in frame.candidates))

    def test_controller_runs_lifecycle_and_records_success(self):
        node = LocalNode({'FollowPath.MPC.time_steps': 10, 'FollowPath.MPC.control_segments': 2,
            'FollowPath.MPC.batch_size': 23, 'FollowPath.MPC.velocity_samples': 3,
            'FollowPath.MPC.curvature_samples': 7, 'FollowPath.MPC.Timing.enabled': True})
        controller = AckermannMPCController()
        controller.configure(node, 'FollowPath', None, grid_host())
        controller.setPlan(line_path())
        controller.activate()
        cmd = controller.computeVelocityCommands(PoseStamped(Header(frame_id='map')), Twist())
        self.assertTrue(math.isfinite(cmd.twist.linear.x))
        self.assertTrue(controller.timing_sample_.success)
        self.assertTrue(controller.have_command_)
        self.assertEqual(controller.timing_sample_.generated_count, 23)
        controller.deactivate()
        self.assertFalse(controller.have_command_)
        controller.cleanup()
        self.assertIsNone(controller.mpc_)
        self.assertFalse(node.subscriptions)

    def test_controller_failed_search_resets_prediction(self):
        controller = AckermannMPCController()
        node, host = LocalNode(), grid_host()
        controller.configure(node, 'FollowPath', None, host)
        controller.setPlan(line_path())
        host.getCostmap().costs[:] = [254] * 6400
        with self.assertRaises(NoLegalTrajectoriesException):
            controller.computeVelocityCommands(PoseStamped(Header(frame_id='map')), Twist())
        self.assertFalse(controller.have_command_)
        self.assertFalse(controller.mpc_.hasWarmStart())

    def test_controller_server_cleans_up_before_context_shutdown(self):
        events = []
        class Node:
            state = server.PRIMARY_STATE_ACTIVE
            def get_logger(self): return LocalNode().get_logger()
            def get_current_state(self): return self.state
            def deactivate(self):
                events.append('deactivate')
                self.state = server.PRIMARY_STATE_INACTIVE
                return self.state
            def cleanup(self):
                events.append('cleanup')
                self.state = server.PRIMARY_STATE_UNCONFIGURED
                return self.state
        class Executor:
            def add_node(self, node): events.append('add')
            def spin_once(self, timeout_sec): server.request_stop(None)
            def remove_node(self, node): events.append('remove')
        class Backend:
            def init(self, args): events.append('init')
            def uninstall_signal_handlers(self): return None
            def ok(self): return True
            def create_controller_server(self): return Node()
            def create_executor(self): return Executor()
            def shutdown(self): events.append('shutdown')
        with patch.dict('os.environ', {'LIMO_FAST_EXIT': '0'}):
            self.assertEqual(server.main([], Backend()), 0)
        self.assertEqual(events, ['init', 'add', 'remove', 'deactivate', 'cleanup', 'shutdown'])


if __name__ == '__main__':
    unittest.main()
