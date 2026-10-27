#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

#include "limo_dwb_critics/sampling_mpc.hpp"

using limo_dwb_critics::MpcConfig;
using limo_dwb_critics::MpcControl;
using limo_dwb_critics::MpcRollout;
using limo_dwb_critics::MpcState;
using limo_dwb_critics::SamplingMpc;

TEST(SamplingMpc, ThreeSlowStartsRampFromRestAndHoldAbsoluteSteeringReferences)
{
  for (const double dt : {0.05, 0.10}) {
    MpcConfig config;
    config.structured_actions = true;
    config.startup_enabled = true;
    config.min_velocity = 0.0;
    config.control_segments = 3;
    config.batch_size = 59;
    config.dt = dt;
    config.time_steps = static_cast<int>(4.0 / dt);
    config.steering_rate = 0.5;
    for (const double speed : {0.0, 0.005, config.startup_max_initial_speed}) {
      MpcState initial;
      initial.control.velocity = speed;
      initial.control.steering = 0.1;
      SamplingMpc model(config);
      std::vector<MpcRollout> starts;
      model.solve(initial, [](const MpcRollout &, double) {return 1000.0;}, 1,
        [&](const MpcRollout & r, limo_dwb_critics::MpcFamily family, std::size_t, double) {
          if (family == limo_dwb_critics::MpcFamily::Startup) {starts.push_back(r);}
        });
      ASSERT_EQ(starts.size(), 3u);
      const double max_angle = std::atan(config.wheelbase / config.min_turning_radius);
      for (std::size_t i = 0; i < starts.size(); ++i) {
        const auto & r = starts[i];
        for (std::size_t t = 0; t < r.targets.size(); ++t) {
          EXPECT_NEAR(r.targets[t].steering, (static_cast<int>(i) - 1) * max_angle, 1e-12);
          const auto & a = r.states[t].control;
          const auto & b = r.states[t + 1].control;
          EXPECT_LE(std::abs(b.velocity - a.velocity), config.startup_acceleration * dt + 1e-12);
          EXPECT_LE(std::abs(b.steering - a.steering), config.steering_rate * dt + 1e-12);
          EXPECT_GE(b.velocity, 0.0);
          EXPECT_LE(b.velocity, std::max(speed, config.startup_target_speed) + 1e-12);
          EXPECT_LE(std::abs(model.yawRate(b)), config.max_yaw_rate + 1e-12);
        }
        EXPECT_NEAR(r.states.back().control.velocity, config.startup_target_speed, 1e-12);
        EXPECT_GT(r.states.back().x, initial.x);
        EXPECT_GT(r.effort_cost, 0.0);
      }
    }
    MpcState moving;
    moving.control.velocity = config.startup_max_initial_speed + 0.001;
    SamplingMpc model(config);
    int starts = 0;
    model.solve(moving, [](const MpcRollout &, double) {return 1000.0;}, 1,
      [&](const MpcRollout &, limo_dwb_critics::MpcFamily family, std::size_t, double) {
        starts += family == limo_dwb_critics::MpcFamily::Startup;
      });
    EXPECT_EQ(starts, 0);
  }
}

TEST(SamplingMpc, SlowStartEscapesStationaryChoiceWhenFastPredictionsAreBlocked)
{
  MpcConfig config;
  config.structured_actions = true;
  config.deduplicate = true;
  config.min_velocity = 0.0;
  config.control_segments = 3;
  config.time_steps = 80;
  config.steering_rate = 0.5;
  config.batch_size = 56;
  const auto objective = [](const MpcRollout & r, double) {
      for (const auto & s : r.states) {
        if (std::hypot(s.x, s.y) > 0.20) {return std::numeric_limits<double>::infinity();}
      }
      return 10.0 * std::pow(0.15 - r.states.back().x, 2);
    };
  SamplingMpc baseline(config);
  auto stopped = baseline.solve(MpcState(), objective);
  ASSERT_TRUE(std::isfinite(stopped.cost));
  EXPECT_DOUBLE_EQ(stopped.rollout.states[1].control.velocity, 0.0);
  config.startup_enabled = true;
  config.batch_size = 59;
  SamplingMpc updated(config);
  auto result = updated.solve(MpcState(), objective);
  ASSERT_TRUE(std::isfinite(result.cost));
  EXPECT_EQ(result.family, limo_dwb_critics::MpcFamily::Startup);
  EXPECT_GT(result.rollout.states[1].control.velocity, 0.0);
  EXPECT_GT(result.rollout.states.back().x, 0.05);
  EXPECT_LT(result.cost, stopped.cost);
  EXPECT_EQ(result.generated_count + result.duplicate_count, 59u);
  auto blocked = updated.solve(MpcState(), [](const MpcRollout &, double) {
      return std::numeric_limits<double>::infinity();
    });
  EXPECT_FALSE(std::isfinite(blocked.cost));
  EXPECT_FALSE(updated.hasWarmStart());
}

TEST(SamplingMpc, StartupConfigurationRequiresValidLimitsAndItsThreeProposalBudget)
{
  MpcConfig valid;
  valid.structured_actions = true;
  valid.startup_enabled = true;
  valid.min_velocity = 0.0;
  valid.control_segments = 3;
  valid.batch_size = 59;
  EXPECT_NO_THROW(valid.validate());
  for (int failure = 0; failure < 6; ++failure) {
    auto config = valid;
    if (failure == 0) {config.batch_size = 56;}
    if (failure == 1) {config.startup_target_speed = 0.0;}
    if (failure == 2) {config.startup_acceleration = config.acceleration + 0.1;}
    if (failure == 3) {config.startup_max_initial_speed = config.startup_target_speed / 2;}
    if (failure == 4) {config.startup_acceleration = std::numeric_limits<double>::quiet_NaN();}
    if (failure == 5) {config.structured_actions = false;}
    EXPECT_THROW(config.validate(), std::invalid_argument);
  }
}

TEST(SamplingMpc, RejectsInvalidConfigurationAndState)
{
  MpcConfig config;
  config.steering_rate = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(SamplingMpc model(config), std::invalid_argument);
  config = MpcConfig();
  config.batch_size = 2;
  EXPECT_THROW(SamplingMpc model(config), std::invalid_argument);
  config = MpcConfig();
  config.control_segments = config.time_steps + 1;
  EXPECT_THROW(SamplingMpc model(config), std::invalid_argument);
  config = MpcConfig();
  config.steering_command_weight = -0.1;
  EXPECT_THROW(SamplingMpc model(config), std::invalid_argument);
  config = MpcConfig();
  config.steering_rate_change_weight = std::numeric_limits<double>::infinity();
  EXPECT_THROW(SamplingMpc model(config), std::invalid_argument);
  SamplingMpc model;
  MpcState state;
  state.x = std::numeric_limits<double>::infinity();
  EXPECT_THROW(model.rollout(state, std::vector<MpcControl>(50)), std::invalid_argument);
  EXPECT_THROW(model.rollout(MpcState(), {}), std::invalid_argument);
}

TEST(SamplingMpc, DiagnosticObserverLabelsAllFamiliesWithoutChangingTheWinner)
{
  MpcConfig config;
  config.dt = 0.1;
  config.time_steps = 25;
  config.batch_size = 128;
  config.velocity_samples = 6;
  config.curvature_samples = 11;
  SamplingMpc baseline(config), observed(config);
  const auto objective = [](const MpcRollout & rollout, double) {
      return 1000.0 + 100.0 * std::pow(rollout.states.back().x - 0.8, 2);
    };
  const auto expected = baseline.solve(MpcState(), objective);
  std::vector<double> costs;
  std::vector<limo_dwb_critics::MpcFamily> families;
  const auto result = observed.solve(MpcState(), objective, 1,
    [&](const MpcRollout &, limo_dwb_critics::MpcFamily family, std::size_t id, double cost) {
      EXPECT_EQ(id, costs.size());
      costs.push_back(cost);
      families.push_back(family);
    });
  ASSERT_EQ(costs.size(), 128U);
  EXPECT_EQ(families[0], limo_dwb_critics::MpcFamily::Braking);
  EXPECT_EQ(families[1], limo_dwb_critics::MpcFamily::Nominal);
  std::vector<int> counts(5);
  for (auto family : families) {++counts[static_cast<std::size_t>(family)];}
  EXPECT_EQ(counts, std::vector<int>({1, 1, 66, 45, 15}));
  EXPECT_DOUBLE_EQ(result.cost, expected.cost);
  ASSERT_LT(result.candidate_id, costs.size());
  EXPECT_DOUBLE_EQ(costs[result.candidate_id], result.cost);
  EXPECT_EQ(families[result.candidate_id], result.family);
  ASSERT_EQ(result.rollout.states.size(), expected.rollout.states.size());
  for (std::size_t t = 0; t < result.rollout.states.size(); ++t) {
    EXPECT_DOUBLE_EQ(result.rollout.states[t].x, expected.rollout.states[t].x);
    EXPECT_DOUBLE_EQ(result.rollout.states[t].control.steering,
      expected.rollout.states[t].control.steering);
  }
  observed.reset();
  const auto throwing = observed.solve(MpcState(), objective, 1,
    [](const MpcRollout &, limo_dwb_critics::MpcFamily, std::size_t, double) {
      throw std::runtime_error("optional diagnostic consumer failed");
    });
  EXPECT_DOUBLE_EQ(throwing.cost, expected.cost);
}

TEST(SamplingMpc, ImmediateSteeringCostDoesNotShrinkWithHorizon)
{
  for (int steps : {20, 50, 100}) {
    MpcConfig config;
    config.time_steps = steps;
    config.acceleration_weight = 0.0;
    config.steering_weight = 0.0;
    SamplingMpc model(config);
    MpcState initial;
    initial.control.velocity = 0.3;
    const auto continuing = model.rollout(
      initial, std::vector<MpcControl>(steps, {0.3, 0.2}), config.steering_rate);
    const auto reversing = model.rollout(
      initial, std::vector<MpcControl>(steps, {0.3, -0.2}), config.steering_rate);
    EXPECT_NEAR(continuing.effort_cost, config.steering_command_weight, 1e-12);
    EXPECT_NEAR(reversing.effort_cost,
      config.steering_command_weight + 4.0 * config.steering_rate_change_weight, 1e-12);
  }
}

TEST(SamplingMpc, PenalizesReverseDistanceAndDirectionChangesAcrossZeroVelocity)
{
  MpcConfig config;
  config.acceleration_weight = 0.0;
  config.steering_weight = 0.0;
  config.steering_command_weight = 0.0;
  config.steering_rate_change_weight = 0.0;
  config.reverse_distance_weight = 5.0;
  config.direction_change_weight = 1.0;
  SamplingMpc model(config);
  MpcState initial;
  initial.control.velocity = 0.1;
  const auto prediction = model.rollout(
    initial, std::vector<MpcControl>(config.time_steps, {-0.1, 0.0}));
  double backwards = 0.0;
  for (std::size_t t = 1; t < prediction.states.size(); ++t) {
    backwards += std::max(0.0, -prediction.states[t].control.velocity) * config.dt;
  }
  EXPECT_GT(backwards, 0.0);
  EXPECT_NEAR(prediction.effort_cost, 5.0 * backwards + 1.0, 1e-12);
  const auto stopped = model.rollout(
    initial, std::vector<MpcControl>(config.time_steps));
  EXPECT_DOUBLE_EQ(stopped.effort_cost, 0.0);
  config.reverse_distance_weight = -1.0;
  EXPECT_THROW(SamplingMpc invalid(config), std::invalid_argument);
  config.reverse_distance_weight = 0.0;
  config.direction_change_weight = std::numeric_limits<double>::infinity();
  EXPECT_THROW(SamplingMpc invalid(config), std::invalid_argument);
}

TEST(SamplingMpc, RemembersForwardDirectionWhileStoppedAndAllowsResetOrNecessaryReverse)
{
  MpcConfig config;
  config.acceleration_weight = 0.0;
  config.steering_weight = 0.0;
  config.steering_command_weight = 0.0;
  config.steering_rate_change_weight = 0.0;
  config.direction_change_weight = 2.0;
  SamplingMpc model(config);
  MpcState initial;
  initial.control.velocity = 0.01;
  const auto stop = model.solve(initial, [](const MpcRollout & rollout, double) {
      return std::abs(rollout.states[1].control.velocity) < 1e-9 ?
             0.0 : std::numeric_limits<double>::infinity();
    });
  ASSERT_TRUE(std::isfinite(stop.cost));
  const auto slight_reverse_preference = [](const MpcRollout & rollout, double) {
      return 2.0 + 5.0 * rollout.states.back().x;
    };
  auto stopped = stop.rollout.states[1];
  const auto stable = model.solve(stopped, slight_reverse_preference);
  EXPECT_GE(stable.rollout.states[1].control.velocity, 0.0);
  // Reverse is a soft penalty: a blocked forward passage can still require it.
  const auto necessary = model.solve(stopped, [](const MpcRollout & rollout, double) {
      return rollout.states.back().x < -0.05 ?
             1.0 : std::numeric_limits<double>::infinity();
    });
  ASSERT_TRUE(std::isfinite(necessary.cost));
  EXPECT_LT(necessary.rollout.states[1].control.velocity, 0.0);
  model.reset();
  const auto reset = model.solve(stopped, slight_reverse_preference);
  EXPECT_LT(reset.rollout.states[1].control.velocity, 0.0);
}

TEST(SamplingMpc, ResetReproducesSearchWithoutStaleSteeringHistory)
{
  SamplingMpc model;
  const auto objective = [](const MpcRollout & rollout, double) {
      const auto & end = rollout.states.back();
      return 10.0 + 100.0 * std::pow(end.x - 0.8, 2) + 100.0 * std::pow(end.y - 0.3, 2);
    };
  const auto first = model.solve(MpcState(), objective);
  ASSERT_TRUE(std::isfinite(first.cost));
  ASSERT_GT(first.rollout.states[1].control.steering, 0.0);
  model.solve(first.rollout.states[1], objective);
  model.reset();
  const auto restarted = model.solve(MpcState(), objective);
  ASSERT_DOUBLE_EQ(first.cost, restarted.cost);
  for (std::size_t t = 0; t < first.rollout.targets.size(); ++t) {
    EXPECT_DOUBLE_EQ(first.rollout.targets[t].velocity, restarted.rollout.targets[t].velocity);
    EXPECT_DOUBLE_EQ(first.rollout.targets[t].steering, restarted.rollout.targets[t].steering);
  }
}

TEST(SamplingMpc, SteeringRegularizationReducesChatterNearNoisyObstacle)
{
  struct Metrics
  {
    double progress{0.0};
    double variation{0.0};
    double rate_change{0.0};
    double clearance{10.0};
  };
  const auto run = [](bool smooth) {
      MpcConfig config;
      config.steering_rate = 0.5;
      config.steering_std = 0.10;
      if (!smooth) {
        config.steering_command_weight = 0.0;
        config.steering_rate_change_weight = 0.0;
      }
      SamplingMpc model(config);
      MpcState current;
      current.control.velocity = 0.3;
      Metrics metrics;
      double previous_increment = 0.0;
      // Synthetic model ablation: identical noise/samples/rate limits, only
      // the two command regularizers differ. This is not a Gazebo test.
      for (int cycle = 0; cycle < 140; ++cycle) {
        const double obstacle_y = 0.30 + 0.012 * std::sin(2.1 * cycle);
        const auto solution = model.solve(current, [&](const MpcRollout & rollout, double) {
            double cost = 0.0;
            for (const auto & state : rollout.states) {
              const double distance = std::hypot(state.x - 0.9, state.y - obstacle_y);
              if (distance < 0.25) {
                return std::numeric_limits<double>::infinity();
              }
              cost += 10.0 * std::hypot(state.y, 0.25 * state.yaw) +
                3.0 * std::exp(-std::max(0.0, distance - 0.25) / 0.15);
            }
            const auto & end = rollout.states.back();
            return cost / rollout.states.size() +
              5.0 * std::hypot(end.x - std::min(2.5, current.x + 1.25), end.y);
          });
        EXPECT_TRUE(std::isfinite(solution.cost));
        if (!std::isfinite(solution.cost)) {
          break;
        }
        const auto next = solution.rollout.states[1];
        const double increment = next.control.steering - current.control.steering;
        metrics.variation += std::abs(increment);
        metrics.rate_change += std::abs(increment - previous_increment);
        previous_increment = increment;
        current = next;
        metrics.clearance = std::min(metrics.clearance,
          std::hypot(current.x - 0.9, current.y - obstacle_y));
      }
      metrics.progress = current.x;
      return metrics;
    };
  const auto baseline = run(false);
  const auto smooth = run(true);
  EXPECT_GT(smooth.progress, 2.0);
  EXPECT_GT(smooth.progress, 0.9 * baseline.progress);
  EXPECT_GE(smooth.clearance, 0.25);
  EXPECT_LT(smooth.variation, 0.6 * baseline.variation);
  EXPECT_LT(smooth.rate_change, 0.6 * baseline.rate_change);
  RecordProperty("steering_variation_ratio", smooth.variation / baseline.variation);
  RecordProperty("steering_rate_change_ratio", smooth.rate_change / baseline.rate_change);
}

TEST(SamplingMpc, IntegratesKnownRearAxleCircleAndBaseOffset)
{
  SamplingMpc model;
  const auto & config = model.config();
  for (double speed : {-0.1, 0.4}) {
    MpcState initial;
    initial.control = {speed, std::atan(config.wheelbase / 0.7)};
    const auto result = model.rollout(
      initial, std::vector<MpcControl>(config.time_steps, initial.control));
    const double yaw = speed * config.dt * config.time_steps / 0.7;
    EXPECT_NEAR(result.states.back().yaw, yaw, 1e-10);
    EXPECT_NEAR(result.states.back().x,
      0.7 * std::sin(yaw) + config.rear_axle_to_base * (std::cos(yaw) - 1.0), 1e-10);
    EXPECT_NEAR(result.states.back().y,
      0.7 * (1.0 - std::cos(yaw)) + config.rear_axle_to_base * std::sin(yaw), 1e-10);
  }
}

TEST(SamplingMpc, EnforcesPhysicalLimitsAcrossTimeVaryingSequences)
{
  SamplingMpc model;
  const auto & config = model.config();
  std::mt19937 random(123);
  std::uniform_real_distribution<double> speed(-0.4, 0.9);
  std::uniform_real_distribution<double> steering(-0.8, 0.8);
  for (int sample = 0; sample < 100; ++sample) {
    std::vector<MpcControl> targets;
    for (int t = 0; t < config.time_steps; ++t) {
      targets.push_back({speed(random), steering(random)});
    }
    MpcState initial;
    initial.control = {sample % 2 == 0 ? 0.4 : -0.08, 0.3};
    const auto result = model.rollout(initial, targets);
    ASSERT_EQ(result.states.size(), targets.size() + 1);
    for (std::size_t t = 1; t < result.states.size(); ++t) {
      const auto & before = result.states[t - 1].control;
      const auto & after = result.states[t].control;
      EXPECT_LE(std::abs(std::tan(after.steering) / config.wheelbase),
        1.0 / config.min_turning_radius + 1e-10);
      EXPECT_LE(std::abs(after.steering - before.steering),
        config.steering_rate * config.dt + 1e-10);
      EXPECT_LE(std::abs(after.velocity - before.velocity),
        std::max(config.acceleration, config.deceleration) * config.dt + 1e-10);
      EXPECT_LE(std::abs(model.yawRate(after) - model.yawRate(before)),
        config.yaw_acceleration * config.dt + 1e-10);
      EXPECT_LE(std::abs(model.yawRate(after)), config.max_yaw_rate + 1e-10);
      EXPECT_GE(after.velocity, config.min_velocity - 1e-10);
      EXPECT_LE(after.velocity, config.max_velocity + 1e-10);
      EXPECT_GE(before.velocity * after.velocity, -1e-12);
      if (after.velocity == 0.0) {
        EXPECT_DOUBLE_EQ(model.yawRate(after), 0.0);
      }
    }
  }
}

TEST(SamplingMpc, BrakesBeforeReversingAndRecoversFromOverspeed)
{
  MpcConfig config;
  config.acceleration = 0.4;
  config.deceleration = 0.8;
  SamplingMpc model(config);
  MpcState initial;
  initial.control.velocity = 0.12;
  const auto reverse = model.rollout(
    initial, std::vector<MpcControl>(config.time_steps, {-0.1, 0.0}));
  EXPECT_NEAR(reverse.states[1].control.velocity, 0.08, 1e-12);
  bool stopped = false;
  for (const auto & state : reverse.states) {
    stopped = stopped || std::abs(state.control.velocity) < 1e-12;
    if (state.control.velocity < 0.0) {
      EXPECT_TRUE(stopped);
    }
  }
  EXPECT_NEAR(reverse.states.back().control.velocity, -0.1, 1e-12);
  initial.control.velocity = 0.7;
  const auto brake = model.rollout(initial, std::vector<MpcControl>(config.time_steps));
  EXPECT_NEAR(brake.states[1].control.velocity, 0.66, 1e-12);
  EXPECT_DOUBLE_EQ(brake.states.back().control.velocity, 0.0);
}

TEST(SamplingMpc, ExploresChangingSteeringAndReturnsReachableFirstCommand)
{
  SamplingMpc model;
  bool changing_sequence = false;
  int evaluations = 0;
  const auto solution = model.solve(MpcState(), [&](const MpcRollout & rollout, double) {
      ++evaluations;
      for (std::size_t t = 1; t < rollout.targets.size(); ++t) {
        changing_sequence = changing_sequence ||
          rollout.targets[t].steering != rollout.targets[t - 1].steering;
      }
      const auto & end = rollout.states.back();
      return 10.0 + 100.0 * std::pow(end.x - 0.8, 2) + 100.0 * std::pow(end.y - 0.2, 2);
    });
  EXPECT_TRUE(changing_sequence);
  EXPECT_EQ(evaluations, model.config().batch_size);
  ASSERT_TRUE(std::isfinite(solution.cost));
  EXPECT_TRUE(model.hasWarmStart());
  EXPECT_GT(solution.rollout.states.back().x, 0.5);
  EXPECT_LE(solution.rollout.states[1].control.velocity,
    model.config().acceleration * model.config().dt + 1e-12);
}

TEST(SamplingMpc, ShiftsWinnerAndClearsWarmStartAfterFailure)
{
  SamplingMpc model;
  const auto objective = [](const MpcRollout & rollout, double) {
      return 10.0 + 100.0 * std::pow(rollout.states.back().x - 0.8, 2);
    };
  const auto first = model.solve(MpcState(), objective);
  ASSERT_TRUE(std::isfinite(first.cost));
  int index = 0;
  model.solve(first.rollout.states[1], [&](const MpcRollout & rollout, double) {
      if (index == 1) {  // Candidate 0 brakes; candidate 1 is the shifted winner.
        for (std::size_t t = 0; t < rollout.targets.size(); ++t) {
          const auto & expected = first.rollout.targets[
            std::min(t + 2, first.rollout.targets.size() - 1)];
          EXPECT_DOUBLE_EQ(rollout.targets[t].velocity, expected.velocity);
          EXPECT_DOUBLE_EQ(rollout.targets[t].steering, expected.steering);
        }
      }
      ++index;
      return objective(rollout, 0.0);
    }, 2);
  EXPECT_TRUE(model.hasWarmStart());
  const auto failed = model.solve(MpcState(), [](const MpcRollout &, double) {
      return std::numeric_limits<double>::infinity();
    });
  EXPECT_FALSE(std::isfinite(failed.cost));
  EXPECT_FALSE(model.hasWarmStart());
  model.solve(MpcState(), objective);
  EXPECT_THROW(model.solve(MpcState(), [](const MpcRollout &, double) -> double {
      throw std::runtime_error("evaluation failure");
    }), std::runtime_error);
  EXPECT_FALSE(model.hasWarmStart());
  model.solve(MpcState(), objective);
  model.reset();
  EXPECT_FALSE(model.hasWarmStart());
}

TEST(SamplingMpc, SelectsBrakingWhenMovingPredictionsCollide)
{
  SamplingMpc model;
  MpcState initial;
  initial.control.velocity = 0.3;
  const auto solution = model.solve(initial, [](const MpcRollout & rollout, double) {
      for (const auto & state : rollout.states) {
        if (state.x >= 0.05) {
          return std::numeric_limits<double>::infinity();
        }
      }
      return 100.0 * std::abs(rollout.states.back().control.velocity);
    });
  ASSERT_TRUE(std::isfinite(solution.cost));
  EXPECT_LT(solution.rollout.states[1].control.velocity, initial.control.velocity);
  EXPECT_NEAR(solution.rollout.states.back().control.velocity, 0.0, 1e-12);
}

TEST(SamplingMpc, RecedingHorizonTracksAnSReference)
{
  MpcConfig config;
  config.steering_rate = 0.5;
  config.steering_std = 0.10;
  SamplingMpc model(config);
  MpcState current;
  const auto reference_y = [](double x) {return 0.18 * std::sin(3.0 * x);};
  const auto reference_yaw = [](double x) {return std::atan(0.54 * std::cos(3.0 * x));};
  double maximum_error = 0.0;
  bool positive_steering = false;
  bool negative_steering = false;
  // Synthetic closed-loop model test, not Gazebo or a hardware validation.
  for (int cycle = 0; cycle < 120; ++cycle) {
    const auto solution = model.solve(current, [&](const MpcRollout & rollout, double) {
        double cost = 0.0;
        for (const auto & state : rollout.states) {
          cost += 30.0 * std::pow(state.y - reference_y(state.x), 2) +
            2.0 * std::pow(state.yaw - reference_yaw(state.x), 2) +
            4.0 * std::pow(state.control.velocity - 0.3, 2);
        }
        return cost;
      });
    ASSERT_TRUE(std::isfinite(solution.cost));
    current = solution.rollout.states[1];
    maximum_error = std::max(maximum_error, std::abs(current.y - reference_y(current.x)));
    positive_steering = positive_steering || current.control.steering > 0.05;
    negative_steering = negative_steering || current.control.steering < -0.05;
  }
  EXPECT_GT(current.x, 1.0);
  EXPECT_LT(maximum_error, 0.15);
  EXPECT_TRUE(positive_steering);
  EXPECT_TRUE(negative_steering);
}

TEST(SamplingMpc, StructuredSearchCoversNineActionsWithThreeRepeatableBlockVariants)
{
  MpcConfig config;
  config.structured_actions = true;
  config.min_velocity = 0.0;
  config.control_segments = 3;
  config.batch_size = 56;
  SamplingMpc first(config), second(config);
  MpcState initial;
  initial.control.velocity = 0.2;
  std::vector<MpcRollout> candidates;
  std::vector<int> families(7);
  const auto objective = [](const MpcRollout & rollout, double) {
      return 1000.0 + std::pow(rollout.states.back().x - 1.0, 2);
    };
  const auto result = first.solve(initial, objective, 1,
    [&](const MpcRollout & rollout, limo_dwb_critics::MpcFamily family,
    std::size_t id, double) {
      EXPECT_EQ(id, candidates.size());
      candidates.push_back(rollout);
      ++families[static_cast<std::size_t>(family)];
    });
  ASSERT_EQ(candidates.size(), 56u);
  EXPECT_EQ(families, std::vector<int>({1, 1, 0, 0, 0, 27, 27}));
  bool countersteer = false;
  for (std::size_t i = 2; i < 29; ++i) {
    const int longitudinal = static_cast<int>((i - 2) / 9) - 1;
    const int lateral = static_cast<int>(((i - 2) / 3) % 3) - 1;
    const auto & next = candidates[i].states[1].control;
    EXPECT_GE(next.velocity, 0.0);
    EXPECT_LE(next.velocity, config.max_velocity);
    EXPECT_GE((next.velocity - initial.control.velocity) * longitudinal, -1e-12);
    if (!longitudinal) {EXPECT_DOUBLE_EQ(next.velocity, initial.control.velocity);}
    EXPECT_GE(next.steering * lateral, -1e-12);
    bool left = false, right = false;
    for (const auto & state : candidates[i].states) {
      EXPECT_GE(state.control.velocity, 0.0);
      EXPECT_LE(state.control.velocity, config.max_velocity);
      EXPECT_LE(std::abs(first.yawRate(state.control)), config.max_yaw_rate + 1e-9);
      left = left || state.control.steering > 0.01;
      right = right || state.control.steering < -0.01;
    }
    countersteer = countersteer || (left && right);
  }
  EXPECT_TRUE(countersteer);
  const auto repeated = second.solve(initial, objective);
  EXPECT_DOUBLE_EQ(result.cost, repeated.cost);
  ASSERT_EQ(result.rollout.targets.size(), repeated.rollout.targets.size());
  for (std::size_t i = 0; i < result.rollout.targets.size(); ++i) {
    EXPECT_DOUBLE_EQ(result.rollout.targets[i].velocity, repeated.rollout.targets[i].velocity);
    EXPECT_DOUBLE_EQ(result.rollout.targets[i].steering, repeated.rollout.targets[i].steering);
  }
}

TEST(SamplingMpc, StructuredBudgetRequiresForwardOnlyAndClearsFailedWarmStart)
{
  MpcConfig config;
  config.structured_actions = true;
  config.batch_size = 56;
  EXPECT_THROW(SamplingMpc model(config), std::invalid_argument);
  config.min_velocity = 0.0;
  config.batch_size = 28;
  EXPECT_THROW(SamplingMpc model(config), std::invalid_argument);
  config.batch_size = 56;
  config.control_segments = 3;
  SamplingMpc model(config);
  model.solve(MpcState(), [](const MpcRollout &, double) {return 1000.0;});
  ASSERT_TRUE(model.hasWarmStart());
  auto result = model.solve(MpcState(), [](const MpcRollout &, double) {
      return std::numeric_limits<double>::infinity();
    });
  EXPECT_FALSE(std::isfinite(result.cost));
  EXPECT_TRUE(result.rollout.states.empty());
  EXPECT_FALSE(model.hasWarmStart());
}

TEST(SamplingMpc, MonophaseVariantsKeepTheSameInitialActionsWithoutPhaseSwitches)
{
  MpcConfig config;
  config.structured_actions = true;
  config.min_velocity = 0.0;
  config.max_velocity = 0.4;
  config.dt = 0.1;
  config.time_steps = 40;
  config.control_segments = 3;
  config.batch_size = 56;
  config.steering_rate = 0.5;
  SamplingMpc model(config);
  MpcState initial;
  initial.control.velocity = 0.2;
  std::vector<MpcRollout> blocks, mono;
  int legacy_arcs = 0;
  model.solve(initial, [](const MpcRollout &, double) {return 10000.0;}, 1,
    [&](const MpcRollout & rollout, limo_dwb_critics::MpcFamily family, std::size_t, double) {
      if (family == limo_dwb_critics::MpcFamily::Structured) {blocks.push_back(rollout);}
      if (family == limo_dwb_critics::MpcFamily::Monophase) {mono.push_back(rollout);}
      if (family == limo_dwb_critics::MpcFamily::ConstantCurvature) {++legacy_arcs;}
    });
  ASSERT_EQ(blocks.size(), 27u);
  ASSERT_EQ(mono.size(), 27u);
  EXPECT_EQ(legacy_arcs, 0);
  for (std::size_t i = 0; i < mono.size(); ++i) {
    const int longitudinal = static_cast<int>(i / 9) - 1;
    const int lateral = static_cast<int>((i / 3) % 3) - 1;
    const auto & rollout = mono[i];
    const double angle = rollout.targets.front().steering;
    // Corresponding multi/monophase samples share the same first-block inputs.
    for (std::size_t t = 0; t < 10; ++t) {
      EXPECT_DOUBLE_EQ(rollout.targets[t].velocity, blocks[i].targets[t].velocity);
      EXPECT_DOUBLE_EQ(rollout.targets[t].steering, blocks[i].targets[t].steering);
    }
    for (std::size_t t = 0; t < rollout.targets.size(); ++t) {
      const auto & target = rollout.targets[t];
      EXPECT_DOUBLE_EQ(target.steering, target.velocity == 0.0 ? 0.0 : angle);
      const auto & previous = rollout.states[t].control;
      const auto & next = rollout.states[t + 1].control;
      EXPECT_GE(next.velocity, 0.0);
      EXPECT_LE(next.velocity, config.max_velocity);
      EXPECT_GE((next.velocity - previous.velocity) * longitudinal, -1e-12);
      if (!longitudinal) {EXPECT_DOUBLE_EQ(next.velocity, initial.control.velocity);}
      EXPECT_GE(next.steering * lateral, -1e-12);
      EXPECT_LE(std::abs(next.steering - previous.steering), config.steering_rate * config.dt + 1e-12);
      EXPECT_LE(std::abs(model.yawRate(next)), config.max_yaw_rate + 1e-12);
    }
  }
}

TEST(SamplingMpc, DeduplicationRemovesStationaryAndSaturatedDuplicatesWithoutChangingSolution)
{
  MpcConfig config;
  config.structured_actions = true;
  config.min_velocity = 0.0;
  config.max_velocity = 0.4;
  config.control_segments = 3;
  config.time_steps = 40;
  config.dt = 0.1;
  config.batch_size = 56;
  config.steering_rate = 0.5;
  for (const double speed : {0.0, 0.2, 0.4}) {
    MpcState initial;
    initial.control.velocity = speed;
    int baseline_calls = 0, unique_calls = 0;
    SamplingMpc baseline(config);
    const auto objective = [](const MpcRollout & rollout) {
        double cost = 1000.0 + std::pow(rollout.states.back().x - 0.8, 2);
        for (const auto & state : rollout.states) {cost += state.y * state.y;}
        return cost;
      };
    const auto expected = baseline.solve(initial, [&](const MpcRollout & rollout, double) {
        ++baseline_calls; return objective(rollout);
      });
    auto optimized_config = config;
    optimized_config.deduplicate = true;
    SamplingMpc optimized(optimized_config);
    std::size_t observed = 0;
    const auto actual = optimized.solve(initial, [&](const MpcRollout & rollout, double) {
        ++unique_calls; return objective(rollout);
      }, 1, [&](const MpcRollout &, limo_dwb_critics::MpcFamily, std::size_t id, double) {
        EXPECT_EQ(id, observed++);
      });
    EXPECT_EQ(baseline_calls, 56);
    EXPECT_LT(unique_calls, baseline_calls);
    EXPECT_EQ(actual.generated_count, observed);
    EXPECT_EQ(actual.generated_count + actual.duplicate_count, 56u);
    if (speed == 0.0) {EXPECT_EQ(actual.generated_count, 16u);}
    EXPECT_DOUBLE_EQ(actual.cost, expected.cost);
    ASSERT_EQ(actual.rollout.states.size(), expected.rollout.states.size());
    for (std::size_t t = 0; t < actual.rollout.states.size(); ++t) {
      EXPECT_DOUBLE_EQ(actual.rollout.states[t].x, expected.rollout.states[t].x);
      EXPECT_DOUBLE_EQ(actual.rollout.states[t].y, expected.rollout.states[t].y);
      EXPECT_DOUBLE_EQ(actual.rollout.states[t].yaw, expected.rollout.states[t].yaw);
      EXPECT_DOUBLE_EQ(actual.rollout.states[t].control.velocity, expected.rollout.states[t].control.velocity);
      EXPECT_DOUBLE_EQ(actual.rollout.states[t].control.steering, expected.rollout.states[t].control.steering);
    }
    std::cout << "MPC dedup, initial v=" << speed << ": " << baseline_calls
              << " proposals -> " << unique_calls << " unique cost evaluations\n";
  }
}

TEST(SamplingMpc, DeduplicationPreservesDifferentFuturesWithIdenticalFirstCommands)
{
  MpcConfig config;
  config.structured_actions = true;
  config.deduplicate = true;
  config.min_velocity = 0.0;
  config.action_noise = 0.0;
  config.control_segments = 3;
  config.batch_size = 56;
  SamplingMpc model(config);
  std::vector<MpcControl> first_commands;
  const auto result = model.solve(MpcState(), [](const MpcRollout &, double) {return 1000.0;}, 1,
    [&](const MpcRollout & rollout, limo_dwb_critics::MpcFamily, std::size_t, double) {
      first_commands.push_back(rollout.states[1].control);
    });
  EXPECT_EQ(result.generated_count, 14u);
  bool shared_first = false;
  for (std::size_t i = 0; i < first_commands.size(); ++i) {
    for (std::size_t j = i + 1; j < first_commands.size(); ++j) {
      shared_first = shared_first || (first_commands[i].velocity == first_commands[j].velocity &&
        first_commands[i].steering == first_commands[j].steering);
    }
  }
  EXPECT_TRUE(shared_first);
  EXPECT_EQ(result.generated_count + result.duplicate_count, 56u);
}
