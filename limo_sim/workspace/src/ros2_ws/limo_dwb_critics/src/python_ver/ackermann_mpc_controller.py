"""Translation of ackermann_mpc_controller.cpp.

Algorithms and state transitions are Python; external DWB work is delegated to
an explicit host. The default DwbAdapter runs locally without ROS transport.
Python classes cannot be exported through pluginlib's C++ class ABI.
"""
from collections import Counter
from copy import deepcopy
import math
import threading
import time
from ...include.python_ver.ackermann_mpc_controller import AckermannMPCParameters
from ...include.python_ver.sampling_mpc import MpcConfig, MpcControl, MpcState
from ...include.python_ver.mpc_timing import MpcTimingSample, CriticPrepareTiming
from ...include.python_ver.mpc_timing_scope import MpcTimingScope
from ...include.python_ver.nav2_compat import PathProgressTracker, pathPosePoints
from ...include.python_ver.ros_compat import (
    Path, Path2D, Pose2D, Pose2DStamped, Header, Twist, Twist2D, TwistStamped,
    Trajectory2D, TrajectoryScore, CriticScore, PlannerException,
    IllegalTrajectoryException, NoLegalTrajectoriesException, read_parameter,
    poseToPose2D, pathToPath2D, clamp, wrappedAngle,
)
from .sampling_mpc import SamplingMpc
from .mppi_cost_critics import MppiObstacleCritic, MppiPathCritic
from .ackermann_kinematics_critic import AckermannKinematicsCritic
from .mpc_debug import MpcDebugPublisher
from .mpc_timing import MpcTimingPublisher
from .mpc_prepare_timing import compute_velocity_commands_2d
from .dwb_adapter import DwbAdapter


class AckermannMPCController:
    def __init__(self, critics=None, dwb_adapter=None):
        self.__dict__.update(vars(AckermannMPCParameters()))
        self.dwb_ = dwb_adapter if dwb_adapter is not None else DwbAdapter()
        self.critics_ = list(critics) if critics is not None else [
            AckermannKinematicsCritic(name='AckermannKinematics'),
            MppiObstacleCritic(name='MppiObstacle'), MppiPathCritic(name='MppiPath')]
        self.mpc_ = self.debug_publisher_ = self.timing_publisher_ = self.tube_layer_ = None
        self.previous_command_ = MpcControl()
        self.tracking_plan_, self.global_plan_, self.complete_plan_ = Path2D(), Path2D(), Path()
        self.tracking_points_, self.progress_tracker_ = [], PathProgressTracker()
        self.plan_mutex_ = threading.RLock()
        self.timing_sample_, self.timing_cycle_start_ = MpcTimingSample(), 0.0

    def configure(self, node, name, tf, costmap_ros):
        self.dwb_.configure(self, node, name, tf, costmap_ros)
        for critic in self.critics_:
            if isinstance(critic, MppiPathCritic):
                critic.setTrackedReference(True)

        def read_double(suffix, value):
            return read_parameter(node, name + '.' + suffix, value)

        def read_int(suffix, value):
            result = read_parameter(node, name + '.MPC.' + suffix, value)
            if not isinstance(result, int) or isinstance(result, bool) or result < 0 or result > 10000:
                raise ValueError('MPC integer parameter out of range: ' + name + '.MPC.' + suffix)
            return result

        def read_bool(suffix, value):
            return read_parameter(node, name + '.MPC.' + suffix, value)

        c = MpcConfig()
        c.dt = read_double('MPC.model_dt', c.dt)
        for key in ('time_steps', 'control_segments', 'batch_size', 'velocity_samples', 'curvature_samples'):
            setattr(c, key, read_int(key, getattr(c, key)))
        c.structured_actions, c.deduplicate = read_bool('structured_actions', False), read_bool('deduplicate', False)
        c.action_variants, c.action_noise = read_int('action_variants', 3), read_double('MPC.action_noise', 0.20)
        c.startup_enabled = read_bool('Startup.enabled', c.startup_enabled)
        c.startup_max_initial_speed = read_double('MPC.Startup.max_initial_speed', c.startup_max_initial_speed)
        c.startup_target_speed = read_double('MPC.Startup.target_speed', c.startup_target_speed)
        c.startup_acceleration = read_double('MPC.Startup.acceleration', c.startup_acceleration)
        c.block_fractions = read_parameter(node, name + '.MPC.block_fractions', c.block_fractions)
        self.tube_required_ = read_bool('Tube.enabled', False)
        self.progress_weight_ = read_double('MPC.progress_weight', 2.0)
        self.progress_distance_ = read_double('MPC.progress_distance', 1.0)
        self.terminal_heading_weight_ = read_double('MPC.terminal_heading_weight', 1.0)
        if (not math.isfinite(self.progress_weight_) or self.progress_weight_ < 0 or
                not math.isfinite(self.progress_distance_) or self.progress_distance_ <= 0 or
                not math.isfinite(self.terminal_heading_weight_) or self.terminal_heading_weight_ < 0 or
                (c.structured_actions and not self.tube_required_)):
            raise ValueError('Structured MPC requires a tube and valid terminal weights')
        if self.tube_required_:
            full_collision_critic = False
            for critic in self.critics_:
                if isinstance(critic, MppiObstacleCritic):
                    full_collision_critic = critic.enforcesFullFootprint() and critic.getScale() > 0
            if not full_collision_critic:
                raise ValueError('Tube MPC requires a full-footprint obstacle critic with positive scale')
            for layer in costmap_ros.getLayeredCostmap().getPlugins():
                if all(hasattr(layer, method) for method in ('matchesPath', 'containsFootprint', 'projectProgress', 'tubeRadius')):
                    self.tube_layer_ = layer
            if self.tube_layer_ is None:
                raise ValueError('MPC.Tube requires a PathDistanceLayer')
        c.min_velocity = read_double('min_vel_x', c.min_velocity)
        c.max_velocity = read_double('max_vel_x', c.max_velocity)
        c.max_yaw_rate = read_double('max_vel_theta', c.max_yaw_rate)
        c.acceleration = read_double('acc_lim_x', c.acceleration)
        c.deceleration = -read_double('decel_lim_x', -c.deceleration)
        angular_acceleration = read_double('acc_lim_theta', c.yaw_acceleration)
        angular_deceleration = -read_double('decel_lim_theta', -c.yaw_acceleration)
        if not math.isfinite(angular_acceleration) or not math.isfinite(angular_deceleration):
            raise ValueError('Non-finite angular acceleration limits')
        c.yaw_acceleration = min(angular_acceleration, angular_deceleration)
        for key, parameter in (
            ('wheelbase', 'MPC.wheelbase'), ('rear_axle_to_base', 'MPC.rear_axle_to_base'),
            ('min_turning_radius', 'AckermannKinematics.min_turning_radius'), ('steering_rate', 'MPC.max_steering_rate'),
            ('velocity_std', 'MPC.velocity_std'), ('steering_std', 'MPC.steering_std'),
            ('acceleration_weight', 'MPC.acceleration_weight'), ('steering_weight', 'MPC.steering_weight'),
            ('steering_command_weight', 'MPC.steering_command_weight'), ('steering_rate_change_weight', 'MPC.steering_rate_change_weight'),
            ('reverse_distance_weight', 'MPC.reverse_distance_weight'), ('direction_change_weight', 'MPC.direction_change_weight')):
            setattr(c, key, read_double(parameter, getattr(c, key)))
        self.steering_feedback_min_velocity_ = read_double('MPC.steering_feedback_min_velocity', self.steering_feedback_min_velocity_)
        if not math.isfinite(self.steering_feedback_min_velocity_) or self.steering_feedback_min_velocity_ <= 0:
            raise ValueError('MPC.steering_feedback_min_velocity must be positive')
        feedback = read_parameter(node, name + '.MPC.feedback', 'OPEN_LOOP')
        if feedback not in ('OPEN_LOOP', 'CLOSED_LOOP'):
            raise ValueError('MPC.feedback must be OPEN_LOOP or CLOSED_LOOP')
        self.open_loop_ = feedback == 'OPEN_LOOP'
        frequency = read_parameter(node, 'controller_frequency', 20.0)
        if not math.isfinite(frequency) or frequency <= 0 or abs(c.dt * frequency - 1.0) > 1e-6:
            raise ValueError('MPC.model_dt must equal 1 / controller_frequency')
        self.mpc_ = SamplingMpc(c)
        self.debug_publisher_ = MpcDebugPublisher(node, name, costmap_ros, c)
        self.timing_publisher_ = MpcTimingPublisher(node, name, c.dt * 1000.0)
        self.timing_publisher_.setCriticNames([critic.getName() for critic in self.critics_])
        self.timing_sample_.critic_prepare = [CriticPrepareTiming() for _ in self.critics_]
        for critic in self.critics_:
            if isinstance(critic, MppiPathCritic):
                critic.setTimingEnabled(self.timing_publisher_.enabled())
        self.resetPrediction()

    # To reset the MPC memory in case of reference path change
    def resetPrediction(self):
        if self.mpc_ is not None:
            self.mpc_.reset()
        self.have_time_ = self.have_command_ = False
        self.previous_command_ = MpcControl()

    def controlTimeNs(self):
        return time.monotonic_ns()

    def setPlan(self, path):
        with self.plan_mutex_:
            self.complete_plan_ = deepcopy(path)    # Copy of the complete path
            self.resetPrediction()
            self.tube_start_pending_ = True         # To verify if the robot is in the tube
            self.setTrackingPlan(path)

    def setTrackingPlan(self, path):
        # path contains (x, y, z) and orientations in quaternions
        plan = pathToPath2D(path)                   # Converts path into 2D
        points = pathPosePoints(plan.poses)         # For each point we have (x, y, cumulative_distance, theta)
        if points != self.tracking_points_ or plan.header.frame_id != self.tracking_plan_.header.frame_id:
            self.progress_tracker_.reset()          # Reset in case of plan change
        self.tracking_plan_, self.tracking_points_ = plan, points
        self.dwb_.setPlan(self, path)               # Reset local critics and sets up the local planner

    def transformGlobalPlan(self, pose):
        reference = self.dwb_.transformPose(self, pose, self.tracking_plan_.header.frame_id)    # Transform pose from odom to map (probably)
        projection = self.progress_tracker_.update(self.tracking_points_, reference.pose)       # Checks the tracker in order to understand the progress inside the path
        if projection is None:
            raise PlannerException('Reference has no valid path segments')
        self.global_plan_ = deepcopy(self.tracking_plan_)
        self.global_plan_.poses = self.global_plan_.poses[projection.segment:]  # Cut of the global trajectory
        return self.dwb_.transformGlobalPlan(self, pose)

    def prepareGlobalPlan(self, pose):
        return self.dwb_.prepareGlobalPlan(self, pose)  # Function used to get the local path and the final goal

    def scoreTrajectory(self, trajectory, best_score=-1.0):
        return self.dwb_.scoreTrajectory(self, trajectory, best_score)

    def baseComputeVelocityCommands2D(self, pose, velocity, results=None):
        return compute_velocity_commands_2d(self, pose, velocity, results, delegate=False)

    def computeVelocityCommands(self, pose, velocity, results=None):
        # Python dispatch replaces C++'s geometry-msg and nav-2d overloads.
        if isinstance(pose.pose, Pose2D):
            return compute_velocity_commands_2d(self, pose, velocity, results)
        with self.plan_mutex_:
            enabled = self.timing_publisher_ is not None and self.timing_publisher_.enabled()
            cpu_start = MpcTimingScope.threadCpuMilliseconds() if enabled else 0.0
            if enabled:
                self.timing_sample_.reset()
                self.timing_cycle_start_ = time.monotonic()
                self.timing_core_started_, self.timing_sample_active_ = False, True
                for critic in self.critics_:
                    if isinstance(critic, MppiPathCritic):
                        critic.resetTiming()

            def record_timing(success):
                self.timing_sample_active_ = False
                if not enabled:
                    return
                s = self.timing_sample_
                s.cycle_ms = (time.monotonic() - self.timing_cycle_start_) * 1000.0
                s.thread_cpu_ms = MpcTimingScope.threadCpuMilliseconds() - cpu_start
                if not self.timing_core_started_:
                    s.prepare_ms = s.cycle_ms
                s.success = success
                for critic in self.critics_:
                    if isinstance(critic, MppiPathCritic):
                        s.guidance_grid_ms += critic.guidanceGridMs()
                        s.grid_updated = s.grid_updated or critic.gridUpdated()
                        s.detouring = s.detouring or (critic.gridUpdated() and critic.detouring())
                self.timing_publisher_.finish(s)
            try:
                planar_pose = Pose2DStamped(deepcopy(pose.header), poseToPose2D(pose.pose))
                # External DWB's 3D wrapper normally converts these messages.
                command = compute_velocity_commands_2d(self, planar_pose,
                    Twist2D(velocity.linear.x, velocity.linear.y, velocity.angular.z), results)
                output = TwistStamped(deepcopy(command.header), Twist())
                output.twist.linear.x, output.twist.linear.y = command.velocity.x, command.velocity.y
                output.twist.angular.z = command.velocity.theta
                record_timing(True)
                return output
            except Exception:
                record_timing(False)
                raise

    def deactivate(self):
        if self.debug_publisher_ is not None: self.debug_publisher_.deactivate()
        if self.timing_publisher_ is not None: self.timing_publisher_.deactivate()
        self.resetPrediction()
        self.dwb_.deactivate(self)

    def cleanup(self):
        self.complete_plan_, self.tracking_plan_, self.tracking_points_ = Path(), Path2D(), []
        self.progress_tracker_.reset()
        self.debug_publisher_ = self.timing_publisher_ = self.tube_layer_ = None
        self.timing_sample_active_ = False
        self.resetPrediction()
        self.mpc_ = None
        self.dwb_.cleanup(self)

    def activate(self):
        self.dwb_.activate(self)
        if self.debug_publisher_ is not None: self.debug_publisher_.activate()
        if self.timing_publisher_ is not None: self.timing_publisher_.activate()

    # Converts MPC prediction into a format that can be evaluated by DWB critics
    def toTrajectory(self, rollout):
        first = rollout.states[1].control
        return Trajectory2D(Twist2D(first.velocity, 0.0, self.mpc_.yawRate(first)),
            [Pose2D(s.x, s.y, s.yaw) for s in rollout.states],
            [t * self.mpc_.config_.dt for t in range(len(rollout.states))])

    def coreScoringAlgorithm(self, pose, velocity, results=None):
        enabled = self.timing_publisher_ is not None and self.timing_publisher_.enabled()
        sample = self.timing_sample_ if enabled else None
        if enabled:
            self.timing_core_started_ = True
            sample.prepare_ms = (time.monotonic() - self.timing_cycle_start_) * 1000.0
        if self.mpc_ is None:
            raise PlannerException('Ackermann MPC is not configured')
        if not all(math.isfinite(v) for v in (velocity.x, velocity.y, velocity.theta)):
            self.resetPrediction()
            raise PlannerException('Non-finite odometry velocity')
        c = self.mpc_.config()
        start_progress = 0.0
        if self.tube_required_:
            if self.tube_layer_ is None or not self.tube_layer_.matchesPath(self.complete_plan_):
                self.resetPrediction()
                raise PlannerException('Path tube is not ready for this reference')
            if self.tube_start_pending_:
                if not self.tube_layer_.containsFootprint(pose):
                    self.resetPrediction()
                    raise PlannerException('START requires the entire footprint inside the tube')
                self.tube_start_pending_ = False
            tracked = self.progress_tracker_.progress()
            minimum = tracked - 0.10 if self.progress_tracker_.valid() else -1e30
            maximum = tracked + 0.10 if self.progress_tracker_.valid() else 1e30
            projection = self.tube_layer_.projectProgress(pose, minimum, maximum, tracked)
            if projection is None:
                raise PlannerException('Tube reference has no valid path segments')
            start_progress, start_heading = projection

        def terminal_cost(rollout):
            if not self.tube_required_:
                return 0.0
            state = rollout.states[-1]
            terminal = Pose2D(state.x, state.y, state.yaw)
            if not self.tube_layer_.containsFootprint(terminal):
                raise IllegalTrajectoryException('TerminalTube', 'Final footprint outside tube')
            projection = self.tube_layer_.projectProgress(terminal, start_progress - 0.05,
                start_progress + c.max_velocity * c.dt * c.time_steps + self.tube_layer_.tubeRadius(), start_progress)
            if projection is None:
                raise IllegalTrajectoryException('TerminalTube', 'No forward rejoin segment')
            progress, heading = projection
            angle = wrappedAngle(state.yaw - heading)
            return (self.progress_weight_ * max(0.0, self.progress_distance_ - (progress - start_progress)) +
                    self.terminal_heading_weight_ * angle * angle)

        now, ros_now, shift = self.controlTimeNs(), self.node_.now(), 1
        if hasattr(ros_now, 'nanoseconds'):
            ros_now = ros_now.nanoseconds
        if self.have_time_:
            elapsed = (now - self.last_time_ns_) * 1e-9
            if elapsed < 0 or elapsed > 3 * c.dt or ros_now < self.last_ros_time_ns_:
                self.resetPrediction()
            else:
                shift = max(1, math.floor(elapsed / c.dt + 0.5))
        self.last_time_ns_, self.last_ros_time_ns_, self.have_time_ = now, ros_now, True
        initial = MpcState(pose.x, pose.y, pose.theta, MpcControl(velocity.x, 0.0))
        max_angle = math.atan(c.wheelbase / c.min_turning_radius)
        initial.control.steering = (clamp(math.atan(c.wheelbase * velocity.theta / velocity.x), -max_angle, max_angle)
            if abs(velocity.x) >= self.steering_feedback_min_velocity_ else
            self.previous_command_.steering if self.have_command_ else 0.0)
        if self.open_loop_ and self.have_command_:
            initial.control = deepcopy(self.previous_command_)
        with MpcTimingScope(sample, 'debug_ms'):
            preview = self.debug_publisher_.begin(initial) if self.debug_publisher_ is not None else None

        def timed_observer(rollout, family, identifier, cost):
            with MpcTimingScope(sample, 'debug_ms'):
                preview(rollout, family, identifier, cost)
        observer = timed_observer if enabled and preview is not None else preview
        rejections, legal_count = Counter(), 0
        best_debug, worst_debug = math.inf, -1.0

        # To assign the next command to \cmd_vel
        def evaluate(rollout, remaining):
            # 'remaining' is a variable used to preemptively interrupt useless cost computations
            nonlocal best_debug, worst_debug, legal_count
            with MpcTimingScope(sample, 'scoring_ms'):
                if enabled: sample.evaluated_count += 1
                trajectory = self.toTrajectory(rollout)
                try:
                    terminal = terminal_cost(rollout)
                    score = self.scoreTrajectory(trajectory, remaining if math.isfinite(remaining) else -1.0)
                    score.scores.append(CriticScore('MpcTerminalTube', terminal, 1.0))
                    score.total += terminal
                    legal_count += 1
                    environment_cost = score.total
                    score.scores.append(CriticScore('MpcEffort', rollout.effort_cost, 1.0))
                    score.total += rollout.effort_cost
                    if results is not None:
                        if score.total < best_debug:
                            best_debug, results.best_index = score.total, len(results.twists)
                        if score.total > worst_debug:
                            worst_debug, results.worst_index = score.total, len(results.twists)
                        results.twists.append(score)
                    return environment_cost
                except IllegalTrajectoryException as error:
                    if enabled: sample.rejected_count += 1
                    rejections[(error.getCriticName(), str(error))] += 1
                    if results is not None:
                        results.twists.append(TrajectoryScore(trajectory, [CriticScore(error.getCriticName(), -1.0)], -1.0))
                    return math.inf
        try:
            with MpcTimingScope(sample, 'solve_ms'):
                solution = self.mpc_.solve(initial, evaluate, shift, observer)
            if enabled: sample.generated_count = solution.generated_count
            if not math.isfinite(solution.cost):
                if self.debug_publisher_ is not None:
                    with MpcTimingScope(sample, 'debug_ms'):
                        self.debug_publisher_.finish(solution)
                if self.debug_trajectory_details_:
                    self.node_.get_logger().error('No legal trajectories; legal=%d; rejections=%s', legal_count, dict(rejections))
                raise NoLegalTrajectoriesException(str(dict(rejections)))
            # Recheck the winner with every critic, without short-circuiting.
            with MpcTimingScope(sample, 'scoring_ms'):
                best = self.scoreTrajectory(self.toTrajectory(solution.rollout), -1.0)
                terminal = terminal_cost(solution.rollout)
                best.scores.extend([CriticScore('MpcTerminalTube', terminal, 1.0),
                                    CriticScore('MpcEffort', solution.rollout.effort_cost, 1.0)])
                best.total += terminal + solution.rollout.effort_cost
            self.previous_command_, self.have_command_ = deepcopy(solution.rollout.states[1].control), True
            if self.debug_publisher_ is not None:
                with MpcTimingScope(sample, 'debug_ms'):
                    self.debug_publisher_.finish(solution)
            return best
        except Exception:
            self.resetPrediction()
            raise
