"""Translation of sampling_mpc.cpp: finite-sample nonlinear shooting MPC.

The fixed seed is retained. Python's random distributions differ from C++'s
standard library, so perturbed candidates are reproducible but not bit-identical.
"""
from copy import deepcopy
import math
import random
from ...include.python_ver.sampling_mpc import (
    MpcConfig, MpcControl, MpcState, MpcRollout, MpcFamily, MpcObserver, MpcSolution,
)
from ...include.python_ver.ros_compat import clamp


def finiteControl(control):
    return math.isfinite(control.velocity) and math.isfinite(control.steering)


def approach(value, target, amount):
    return value + clamp(target - value, -amount, amount)


def motionDirection(velocity):
    return 1 if velocity > 0.001 else -1 if velocity < -0.001 else 0


class SamplingMpc:
    def __init__(self, config=None):
        self.config_ = deepcopy(config if config is not None else MpcConfig())
        self.config_.validate()
        self.reset()

    def config(self):
        # A copy gives callers the same protection as C++'s const reference.
        return deepcopy(self.config_)

    def reset(self):
        self.previous_targets_ = []
        self.previous_steering_rate_ = 0.0
        self.previous_direction_ = 0

    def hasWarmStart(self):
        return bool(self.previous_targets_)

    def yawRate(self, control):
        return control.velocity * math.tan(control.steering) / self.config_.wheelbase

    def boundedTarget(self, target):
        c = self.config_
        velocity = clamp(target.velocity, c.min_velocity, c.max_velocity)
        curvature = 1.0 / c.min_turning_radius
        if abs(velocity) > 1e-9:
            curvature = min(curvature, c.max_yaw_rate / abs(velocity))
        angle = math.atan(c.wheelbase * curvature)
        # The LIMO driver requests centered steering when commanded to stop.
        return MpcControl(velocity, 0.0 if velocity == 0.0 else clamp(target.steering, -angle, angle))

    def advance(self, current, requested):
        if not finiteControl(current) or not finiteControl(requested):
            raise ValueError("Non-finite MPC control")
        c = self.config_
        target = self.boundedTarget(requested)
        # Brake to zero before reversing; do not jump across zero in one step.
        desired = 0.0 if current.velocity * target.velocity < 0 else target.velocity
        acceleration = c.acceleration if abs(desired) > abs(current.velocity) else c.deceleration
        next_v = approach(current.velocity, desired, acceleration * c.dt)
        next_s = approach(current.steering, target.steering, c.steering_rate * c.dt)
        dv, ds = next_v - current.velocity, next_s - current.steering
        tangent = math.tan(max(abs(current.steering), abs(next_s)))
        derivative = (abs(dv) * tangent + max(abs(current.velocity), abs(next_v)) *
                      (1 + tangent * tangent) * abs(ds)) / c.wheelbase
        fraction = min(1.0, c.yaw_acceleration * c.dt / derivative) if derivative > 0 else 1.0

        def interpolate(f):
            return MpcControl(current.velocity + f * dv, current.steering + f * ds)

        yaw_limit = max(c.max_yaw_rate, abs(self.yawRate(current)))
        if abs(self.yawRate(interpolate(fraction))) > yaw_limit:
            low, high = 0.0, fraction
            for _ in range(40):
                mid = (low + high) * 0.5
                if abs(self.yawRate(interpolate(mid))) <= yaw_limit:
                    low = mid
                else:
                    high = mid
            fraction = low
        return interpolate(fraction)

    def rollout(self, initial, targets, previous_steering_rate=0.0, previous_direction=0):
        c = self.config_
        if (len(targets) != c.time_steps or
                not all(math.isfinite(v) for v in (initial.x, initial.y, initial.yaw, previous_steering_rate)) or
                not finiteControl(initial.control) or not -1 <= previous_direction <= 1 or
                abs(initial.control.steering) > math.atan(c.wheelbase / c.min_turning_radius) + 1e-9):
            raise ValueError("Invalid MPC initial state or sequence length")
        result = MpcRollout(states=[deepcopy(initial)], targets=deepcopy(targets))
        state = deepcopy(initial)
        direction = motionDirection(initial.control.velocity) or previous_direction
        reverse_distance, changes = 0.0, 0
        for target in targets:
            control = self.advance(state.control, target)
            reverse_distance += max(0.0, -control.velocity) * c.dt
            next_direction = motionDirection(control.velocity)
            if next_direction:
                changes += bool(direction and next_direction != direction)
                direction = next_direction
            a = (control.velocity - state.control.velocity) / (c.dt * max(c.acceleration, c.deceleration))
            s = (control.steering - state.control.steering) / (c.dt * c.steering_rate)
            result.effort_cost += c.acceleration_weight * a * a + c.steering_weight * s * s
            # Integrate the rear-axle arc, then translate back to base_link.
            yaw_delta = self.yawRate(control) * c.dt
            half = yaw_delta * 0.5
            sinc = 1.0 if abs(half) < 1e-9 else math.sin(half) / half
            distance = control.velocity * c.dt * sinc
            next_yaw = state.yaw + yaw_delta
            state.x += distance * math.cos(state.yaw + half) + c.rear_axle_to_base * (math.cos(next_yaw) - math.cos(state.yaw))
            state.y += distance * math.sin(state.yaw + half) + c.rear_axle_to_base * (math.sin(next_yaw) - math.sin(state.yaw))
            state.yaw, state.control = next_yaw, control
            # C++ push_back copies a value; Python needs an explicit copy here.
            result.states.append(deepcopy(state))
        result.effort_cost /= len(targets)
        result.effort_cost += c.reverse_distance_weight * reverse_distance + c.direction_change_weight * changes
        first_rate = (result.states[1].control.steering - initial.control.steering) / (c.dt * c.steering_rate)
        rate_change = first_rate - previous_steering_rate / c.steering_rate
        result.effort_cost += (c.steering_command_weight * first_rate ** 2 +
                               c.steering_rate_change_weight * rate_change ** 2)
        return result

    def solve(self, initial, environment_cost, shift_steps=1, observer=None):
        if not isinstance(shift_steps, int) or shift_steps < 0:
            raise ValueError("shift_steps must be a nonnegative integer")
        c = self.config_
        count = c.time_steps
        nominal = [self.boundedTarget(initial.control) for _ in range(count)]
        if len(self.previous_targets_) == count:
            nominal = [deepcopy(self.previous_targets_[min(t + min(shift_steps, count), count - 1)])
                       for t in range(count)]
        previous_rate = self.previous_steering_rate_
        previous_direction = self.previous_direction_ or motionDirection(initial.control.velocity)
        self.reset()  # A failing evaluator cannot leave an old warm start behind.
        best = MpcSolution()
        candidate_id = duplicate_count = 0
        unique_controls, unique_targets = set(), set()

        def sequence_key(sequence):
            return tuple((control.velocity, control.steering) for control in sequence)

        def consider(targets, family):
            nonlocal candidate_id, duplicate_count, best
            target_key = sequence_key(targets)
            if c.deduplicate and target_key in unique_targets:
                duplicate_count += 1
                return
            prediction = self.rollout(initial, targets, previous_rate, previous_direction)
            if c.deduplicate:
                control_key = sequence_key([s.control for s in prediction.states[1:]])
                if control_key in unique_controls:
                    duplicate_count += 1
                    return
                unique_controls.add(control_key)
                unique_targets.add(target_key)
            identifier = candidate_id
            candidate_id += 1
            remaining = best.cost - prediction.effort_cost

            def observe(total):
                if observer is not None:
                    try:
                        observer(prediction, family, identifier, total)
                    except Exception:
                        pass  # Telemetry failures cannot invalidate a command.

            if remaining < 0.0:
                observe(math.nan)
                return
            cost = environment_cost(prediction, remaining)
            valid = math.isfinite(cost) and cost >= 0.0
            total = cost + prediction.effort_cost if valid else math.inf
            observe(total)
            if valid and total < best.cost:
                best = MpcSolution(prediction, total, family, identifier)

        consider([MpcControl() for _ in range(count)], MpcFamily.Braking)
        consider(nominal, MpcFamily.Nominal)
        rng = random.Random(42)
        if c.structured_actions:
            monophase = []
            gain = lambda: rng.uniform(1.0 - c.action_noise, 1.0)
            for longitudinal in (-1, 0, 1):
                for lateral in (-1, 0, 1):
                    for variant in range(c.action_variants):
                        targets = [MpcControl() for _ in range(count)]
                        control = deepcopy(initial.control)
                        max_angle = math.atan(c.wheelbase / c.min_turning_radius)
                        strength = (variant + 1.0) / c.action_variants * gain()
                        first_angle = clamp(initial.control.steering + lateral * max_angle * strength, -max_angle, max_angle)
                        begin, elapsed, first_acceleration = 0, 0.0, 0.0
                        for segment in range(c.control_segments):
                            elapsed += c.block_fractions[segment]
                            end = count if segment + 1 == c.control_segments else math.floor(elapsed * count + 0.5)
                            phase = 3 * segment // c.control_segments
                            angle = -first_angle if phase == 1 else first_angle
                            a_mode = longitudinal if phase == 0 else 0
                            a = a_mode * (c.deceleration if a_mode < 0 else c.acceleration) * gain()
                            if segment == 0:
                                first_acceleration = a
                            for t in range(begin, end):
                                targets[t] = self.boundedTarget(MpcControl(control.velocity + a * c.dt, angle))
                                control = self.advance(control, targets[t])
                            begin = end
                        consider(targets, MpcFamily.Structured)
                        mono, control = [], deepcopy(initial.control)
                        for _ in range(count):
                            target = self.boundedTarget(MpcControl(control.velocity + first_acceleration * c.dt, first_angle))
                            mono.append(target)
                            control = self.advance(control, target)
                        monophase.append(mono)
            for targets in monophase:
                consider(targets, MpcFamily.Monophase)
            if c.startup_enabled and abs(initial.control.velocity) <= c.startup_max_initial_speed:
                max_angle = math.atan(c.wheelbase / c.min_turning_radius)
                for direction in (-1, 0, 1):
                    targets, control = [], deepcopy(initial.control)
                    for _ in range(count):
                        target = self.boundedTarget(MpcControl(approach(control.velocity, c.startup_target_speed,
                            c.startup_acceleration * c.dt), direction * max_angle))
                        targets.append(target)
                        control = self.advance(control, target)
                    consider(targets, MpcFamily.Startup)
        else:
            for v in range(c.velocity_samples):
                velocity = c.min_velocity + (c.max_velocity - c.min_velocity) * v / (c.velocity_samples - 1)
                for k in range(c.curvature_samples):
                    curvature = (-1.0 + 2.0 * k / (c.curvature_samples - 1)) / c.min_turning_radius
                    target = self.boundedTarget(MpcControl(velocity, math.atan(c.wheelbase * curvature)))
                    consider([deepcopy(target) for _ in range(count)], MpcFamily.ConstantCurvature)
            remaining = c.batch_size - 2 - c.velocity_samples * c.curvature_samples
            for sample in range(remaining):
                targets = [MpcControl() for _ in range(count)]
                for segment in range(c.control_segments):
                    begin, end = segment * count // c.control_segments, (segment + 1) * count // c.control_segments
                    velocity_noise = c.velocity_std * rng.gauss(0, 1)
                    steering_noise = c.steering_std * rng.gauss(0, 1)
                    broad = self.boundedTarget(MpcControl(rng.uniform(c.min_velocity, c.max_velocity),
                        math.atan(c.wheelbase * rng.uniform(-1 / c.min_turning_radius, 1 / c.min_turning_radius))))
                    for t in range(begin, end):
                        targets[t] = deepcopy(broad) if sample % 4 == 0 else self.boundedTarget(MpcControl(
                            nominal[t].velocity + velocity_noise, nominal[t].steering + steering_noise))
                consider(targets, MpcFamily.BroadExploration if sample % 4 == 0 else MpcFamily.PerturbedNominal)
        best.generated_count, best.duplicate_count = candidate_id, duplicate_count
        if math.isfinite(best.cost):
            self.previous_targets_ = deepcopy(best.rollout.targets)
            self.previous_steering_rate_ = (best.rollout.states[1].control.steering - initial.control.steering) / c.dt
            self.previous_direction_ = motionDirection(best.rollout.states[1].control.velocity) or previous_direction
        return best
