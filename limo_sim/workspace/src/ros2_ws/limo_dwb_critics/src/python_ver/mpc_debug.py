"""Translation of mpc_debug.cpp: bounded telemetry with independent snapshots."""
from copy import deepcopy
import math
import time
from ...include.python_ver.mpc_debug import MpcCandidate, MpcDebug
from ...include.python_ver.ros_compat import Point, Pose2D, read_parameter


class MpcDebugPublisher:
    def __init__(self, node, name, costmap, config):
        self.node_, self.costmap_, self.config_ = node, costmap, deepcopy(config)
        self.publisher_ = None
        self.totals_, self.seen_ = [0] * 8, [0] * 8
        self.samples_per_family_, self.pose_stride_ = 6, 2
        self.period_ns_ = self.last_publish_ns_ = 0
        self.collecting_, self.frame_ = False, MpcDebug()
        prefix = name + '.MPC.Debug.'
        if not read_parameter(node, prefix + 'enabled', False):
            return
        topic = read_parameter(node, prefix + 'topic', '/limo/control/mpc_debug')
        rate = read_parameter(node, prefix + 'publish_rate', 4.0)
        samples = read_parameter(node, prefix + 'samples_per_family', 6)
        stride = read_parameter(node, prefix + 'pose_stride', min(2, config.time_steps))
        if (not math.isfinite(rate) or not 0 < rate <= 100 or not 1 <= samples <= 128 or
                not 1 <= stride <= config.time_steps or not topic):
            raise ValueError('Invalid MPC.Debug rate, sample count, pose stride or topic')
        self.samples_per_family_, self.pose_stride_ = int(samples), int(stride)
        self.period_ns_ = int(1e9 / rate)
        seeds = config.velocity_samples * config.curvature_samples
        sampled = config.batch_size - seeds - 2
        broad = (sampled + 3) // 4
        self.totals_ = ([1, 1, 0, 0, 0, 9 * config.action_variants, 9 * config.action_variants,
                         3 if config.startup_enabled else 0] if config.structured_actions else
                        [1, 1, seeds, sampled - broad, broad, 0, 0, 0])
        self.publisher_ = node.create_publisher(topic, qos=1)

    def activate(self):
        if self.publisher_ is not None:
            self.publisher_.on_activate()

    def deactivate(self):
        self.collecting_, self.frame_, self.last_publish_ns_ = False, MpcDebug(), 0
        if self.publisher_ is not None:
            self.publisher_.on_deactivate()

    def begin(self, initial):
        self.collecting_ = False
        if self.publisher_ is None or self.publisher_.get_subscription_count() == 0:
            return None
        now = time.monotonic_ns()
        if self.last_publish_ns_ and now - self.last_publish_ns_ < self.period_ns_:
            return None
        try:
            self.frame_ = MpcDebug()
            f, c = self.frame_, self.config_
            f.header.stamp, f.header.frame_id = self.node_.now(), self.costmap_.getGlobalFrameID()
            f.base_frame, f.robot_pose = self.costmap_.getBaseFrameID(), Pose2D(initial.x, initial.y, initial.yaw)
            f.initial_velocity, f.initial_steering = initial.control.velocity, initial.control.steering
            f.wheelbase, f.rear_axle_to_base, f.model_dt = c.wheelbase, c.rear_axle_to_base, c.dt
            f.time_steps, f.generated_count, f.selected_id = c.time_steps, c.batch_size, -1
            f.footprint.points = [Point(p.x, p.y) for p in self.costmap_.getRobotFootprint()]
            grid = self.costmap_.getCostmap()
            f.costmap.header = deepcopy(f.header)
            meta = f.costmap.metadata
            meta.update_time, meta.layer = f.header.stamp, 'controller_costmap'
            meta.resolution = grid.getResolution()
            meta.size_x, meta.size_y = grid.getSizeInCellsX(), grid.getSizeInCellsY()
            meta.origin.position = Point(grid.getOriginX(), grid.getOriginY())
            meta.origin.orientation.w = 1.0
            f.costmap.data = list(grid.getCharMap())
            self.seen_, self.last_publish_ns_, self.collecting_ = [0] * 8, now, True
        except Exception as error:
            self.node_.get_logger().warning('MPC preview capture failed: %s', error)
            return None

        def observer(rollout, family, identifier, cost):
            if not self.collecting_:
                return
            try:
                f = int(family)
                if not 0 <= f < 8:
                    raise IndexError('Invalid MPC candidate family')
                index = self.seen_[f]
                self.seen_[f] += 1
                keep = min(self.samples_per_family_, self.totals_[f])
                for k in range(keep):
                    representative = self.totals_[f] // 2 if keep == 1 else math.floor(k * (self.totals_[f] - 1.0) / (keep - 1) + 0.5)
                    if index == representative:
                        self.frame_.candidates.append(self.candidate(rollout, family, identifier, cost))
                        break
            except Exception as error:
                self.collecting_ = False
                self.node_.get_logger().warning('MPC preview sampling failed: %s', error)
        return observer

    def candidate(self, rollout, family, identifier, cost):
        status = MpcCandidate.COST_PRUNED if math.isnan(cost) else MpcCandidate.SCORED if math.isfinite(cost) else MpcCandidate.REJECTED
        return MpcCandidate(identifier, int(family), cost, status,
            [Pose2D(s.x, s.y, s.yaw) for t, s in enumerate(rollout.states)
             if t % self.pose_stride_ == 0 or t + 1 == len(rollout.states)])

    def finish(self, solution):
        if not self.collecting_:
            return
        self.collecting_ = False
        try:
            f = self.frame_
            f.generated_count = solution.generated_count
            if math.isfinite(solution.cost):
                f.selected_id = solution.candidate_id
                command = solution.rollout.states[1].control
                f.command_velocity, f.command_steering = command.velocity, command.steering
                winner = self.candidate(solution.rollout, solution.family, solution.candidate_id, solution.cost)
                for index, existing in enumerate(f.candidates):
                    if existing.candidate_id == winner.candidate_id:
                        f.candidates[index] = winner
                        break
                else:
                    f.candidates.append(winner)
            self.publisher_.publish(f)
        except Exception as error:
            self.node_.get_logger().warning('MPC preview publication failed: %s', error)
