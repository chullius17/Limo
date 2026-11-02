#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "limo_dwb_critics/mpc_timing.hpp"

namespace limo_dwb_critics
{
// Inspect the same report used by the ROS publisher, without relying on DDS
// discovery delays or sleeps to advance a reporting interval.
class MpcTimingPublisherTestPeer
{
public:
  static diagnostic_msgs::msg::DiagnosticArray report(
    const MpcTimingPublisher & timing, MpcTimingPublisher::Clock::time_point now)
  {
    return timing.report(now);
  }
  static std::uint64_t cycles(const MpcTimingPublisher & timing) {return timing.window_.cycles;}
  static bool active(const MpcTimingPublisher & timing) {return timing.active_;}
  static const void * criticStorage(const MpcTimingPublisher & timing)
  {
    return timing.critic_windows_.data();
  }
};
}  // namespace limo_dwb_critics

namespace
{
using limo_dwb_critics::MpcTimingPublisher;
using limo_dwb_critics::MpcTimingPublisherTestPeer;
using limo_dwb_critics::MpcTimingSample;
using limo_dwb_critics::CriticPrepareTiming;

double value(const diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key)
{
  for (const auto & field : status.values) {
    if (field.key == key) {return std::stod(field.value);}
  }
  ADD_FAILURE() << "Missing timing diagnostic field: " << key;
  return -1.0;
}

class MpcTimingTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
  nav2_util::LifecycleNode::SharedPtr node(std::vector<rclcpp::Parameter> parameters = {})
  {
    auto options = rclcpp::NodeOptions().parameter_overrides(parameters);
    return std::make_shared<rclcpp_lifecycle::LifecycleNode>("mpc_timing_test", options);
  }
  nav2_util::LifecycleNode::SharedPtr enabledNode(
    std::vector<rclcpp::Parameter> parameters = {})
  {
    parameters.emplace_back("FollowPath.MPC.Timing.enabled", true);
    return node(parameters);
  }
};

TEST_F(MpcTimingTest, DisabledCollectorNeverReadsClockOrAccumulates)
{
  int clock_calls = 0;
  MpcTimingPublisher timing(node(), "FollowPath", 100.0, [&]() {
      ++clock_calls;
      return MpcTimingPublisher::Clock::now();
    });
  EXPECT_FALSE(timing.enabled());
  timing.activate();
  timing.finish(MpcTimingSample{});
  timing.deactivate();
  EXPECT_EQ(clock_calls, 0);
  EXPECT_EQ(MpcTimingPublisherTestPeer::cycles(timing), 0U);
  EXPECT_FALSE(MpcTimingPublisherTestPeer::active(timing));
}

TEST_F(MpcTimingTest, AggregatesActualCycleResultsAndStrictDeadlineBoundary)
{
  auto now = MpcTimingPublisher::Clock::time_point(std::chrono::seconds(1));
  auto lifecycle_node = enabledNode();
  MpcTimingPublisher timing(lifecycle_node, "FollowPath", 100.0, [&]() {return now;});
  ASSERT_TRUE(timing.enabled());
  timing.activate();
  MpcTimingSample sample;
  sample.success = true;
  sample.cycle_ms = 100.0;
  sample.thread_cpu_ms = 60.0;
  sample.prepare_ms = 10.0;
  sample.guidance_grid_ms = 4.0;
  sample.solve_ms = 80.0;
  sample.scoring_ms = 70.0;
  sample.debug_ms = 5.0;
  sample.plan_prepare_ms = 2.0;
  sample.plan_prepare_cpu_ms = 1.0;
  sample.costmap_lock_wait_ms = 6.0;
  sample.critics_prepare_ms = 3.0;
  sample.critics_prepare_cpu_ms = 2.5;
  sample.debrief_ms = 0.4;
  sample.dwb_publish_ms = 0.6;
  sample.generated_count = 59;
  sample.evaluated_count = 55;
  sample.rejected_count = 3;
  sample.grid_updated = true;
  timing.finish(sample);
  now += std::chrono::milliseconds(500);
  sample.success = false;
  sample.detouring = true;
  sample.grid_updated = false;
  sample.cycle_ms = 120.0;
  sample.thread_cpu_ms = 80.0;
  sample.plan_prepare_ms = 4.0;
  sample.plan_prepare_cpu_ms = 3.0;
  sample.costmap_lock_wait_ms = 10.0;
  sample.critics_prepare_ms = 7.0;
  sample.critics_prepare_cpu_ms = 3.5;
  sample.debrief_ms = 0.8;
  sample.dwb_publish_ms = 1.0;
  sample.generated_count = 30;
  sample.evaluated_count = 25;
  sample.rejected_count = 25;
  timing.finish(sample);
  const auto report = MpcTimingPublisherTestPeer::report(timing, now);
  ASSERT_EQ(report.status.size(), 1U);
  const auto & status = report.status.front();
  EXPECT_EQ(status.name, "FollowPath/mpc_timing");
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_DOUBLE_EQ(value(status, "cycles"), 2.0);
  EXPECT_DOUBLE_EQ(value(status, "failures"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "deadline_misses"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "candidate_generated"), 89.0);
  EXPECT_DOUBLE_EQ(value(status, "candidate_evaluated"), 80.0);
  EXPECT_DOUBLE_EQ(value(status, "candidate_rejected"), 28.0);
  EXPECT_DOUBLE_EQ(value(status, "detour_cycles"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "grid_updates"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "budget_ms"), 100.0);
  EXPECT_DOUBLE_EQ(value(status, "window_ms"), 500.0);
  EXPECT_DOUBLE_EQ(value(status, "cycle_ms_mean"), 110.0);
  EXPECT_DOUBLE_EQ(value(status, "cycle_ms_max"), 120.0);
  EXPECT_DOUBLE_EQ(value(status, "thread_cpu_ms_mean"), 70.0);
  EXPECT_DOUBLE_EQ(value(status, "thread_cpu_ms_max"), 80.0);
  EXPECT_DOUBLE_EQ(value(status, "prepare_ms_mean"), 10.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_grid_ms_mean"), 4.0);
  EXPECT_DOUBLE_EQ(value(status, "solve_ms_mean"), 80.0);
  EXPECT_DOUBLE_EQ(value(status, "scoring_ms_mean"), 70.0);
  EXPECT_DOUBLE_EQ(value(status, "debug_ms_mean"), 5.0);
  EXPECT_DOUBLE_EQ(value(status, "plan_prepare_ms_mean"), 3.0);
  EXPECT_DOUBLE_EQ(value(status, "plan_prepare_ms_max"), 4.0);
  EXPECT_DOUBLE_EQ(value(status, "plan_prepare_cpu_ms_mean"), 2.0);
  EXPECT_DOUBLE_EQ(value(status, "plan_prepare_cpu_ms_max"), 3.0);
  EXPECT_DOUBLE_EQ(value(status, "costmap_lock_wait_ms_mean"), 8.0);
  EXPECT_DOUBLE_EQ(value(status, "costmap_lock_wait_ms_max"), 10.0);
  EXPECT_DOUBLE_EQ(value(status, "critics_prepare_ms_mean"), 5.0);
  EXPECT_DOUBLE_EQ(value(status, "critics_prepare_ms_max"), 7.0);
  EXPECT_DOUBLE_EQ(value(status, "critics_prepare_cpu_ms_mean"), 3.0);
  EXPECT_DOUBLE_EQ(value(status, "critics_prepare_cpu_ms_max"), 3.5);
  EXPECT_DOUBLE_EQ(value(status, "debrief_ms_mean"), 0.6);
  EXPECT_DOUBLE_EQ(value(status, "debrief_ms_max"), 0.8);
  EXPECT_DOUBLE_EQ(value(status, "dwb_publish_ms_mean"), 0.8);
  EXPECT_DOUBLE_EQ(value(status, "dwb_publish_ms_max"), 1.0);
}

TEST_F(MpcTimingTest, PerCriticMeansUseOnlyActualCallsIncludingPartialFailedCycles)
{
  const auto now = MpcTimingPublisher::Clock::time_point(std::chrono::seconds(1));
  MpcTimingPublisher timing(enabledNode(), "FollowPath", 100.0, [&]() {return now;});
  timing.setCriticNames({"Obstacle", "Path"});
  timing.activate();
  MpcTimingSample sample;
  sample.success = true;
  sample.critic_prepare = {{12.0, 2.0, true}, {0.5, 0.1, true}};
  timing.finish(sample);
  // Model prepare throwing in the first critic: the second was never called.
  sample.success = false;
  sample.critic_prepare = {{8.0, 1.0, true}, {1000.0, 1000.0, false}};
  timing.finish(sample);
  // A cycle that failed before any prepare must not dilute either mean.
  sample.reset();
  timing.finish(sample);
  const auto status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "cycles"), 3.0);
  EXPECT_DOUBLE_EQ(value(status, "failures"), 2.0);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Obstacle_calls"), 2.0);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Obstacle_ms_mean"), 10.0);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Obstacle_ms_max"), 12.0);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare_cpu/Obstacle_ms_mean"), 1.5);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare_cpu/Obstacle_ms_max"), 2.0);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Path_calls"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Path_ms_mean"), 0.5);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Path_ms_max"), 0.5);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare_cpu/Path_ms_mean"), 0.1);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare_cpu/Path_ms_max"), 0.1);
}

TEST_F(MpcTimingTest, SampleResetPreservesPreparedStorageAndClearsAllValues)
{
  MpcTimingSample sample;
  sample.cycle_ms = sample.thread_cpu_ms = sample.prepare_ms = sample.guidance_grid_ms =
    sample.solve_ms = sample.scoring_ms = sample.debug_ms = 5.0;
  sample.plan_prepare_ms = sample.plan_prepare_cpu_ms = sample.costmap_lock_wait_ms =
    sample.critics_prepare_ms = sample.critics_prepare_cpu_ms = sample.debrief_ms =
    sample.dwb_publish_ms = 6.0;
  sample.generated_count = sample.evaluated_count = sample.rejected_count = 10;
  sample.success = sample.detouring = sample.grid_updated = true;
  sample.critic_prepare.reserve(8);
  sample.critic_prepare.assign(3, CriticPrepareTiming{5.0, 2.0, true});
  const auto capacity = sample.critic_prepare.capacity();
  const auto storage = sample.critic_prepare.data();
  for (int i = 0; i < 100; ++i) {
    sample.reset();
    EXPECT_EQ(sample.critic_prepare.size(), 3U);
    EXPECT_EQ(sample.critic_prepare.capacity(), capacity);
    EXPECT_EQ(sample.critic_prepare.data(), storage);
  }
  const std::array<double, 14> measurements{{sample.cycle_ms, sample.thread_cpu_ms,
    sample.prepare_ms, sample.guidance_grid_ms, sample.solve_ms, sample.scoring_ms, sample.debug_ms,
    sample.plan_prepare_ms, sample.plan_prepare_cpu_ms, sample.costmap_lock_wait_ms,
    sample.critics_prepare_ms, sample.critics_prepare_cpu_ms, sample.debrief_ms, sample.dwb_publish_ms}};
  for (const auto measurement : measurements) {EXPECT_DOUBLE_EQ(measurement, 0.0);}
  EXPECT_EQ(sample.generated_count, 0U);
  EXPECT_EQ(sample.evaluated_count, 0U);
  EXPECT_EQ(sample.rejected_count, 0U);
  EXPECT_FALSE(sample.success);
  EXPECT_FALSE(sample.detouring);
  EXPECT_FALSE(sample.grid_updated);
  for (const auto & critic : sample.critic_prepare) {
    EXPECT_DOUBLE_EQ(critic.prepare_ms, 0.0);
    EXPECT_DOUBLE_EQ(critic.prepare_cpu_ms, 0.0);
    EXPECT_FALSE(critic.called);
  }
}

TEST_F(MpcTimingTest, UnexpectedCriticSizeKeepsCycleTotalsAndDoesNotUsePartialIdentity)
{
  const auto now = MpcTimingPublisher::Clock::time_point(std::chrono::seconds(1));
  MpcTimingPublisher timing(enabledNode(), "FollowPath", 100.0, [&]() {return now;});
  timing.setCriticNames({"Obstacle", "Path"});
  timing.activate();
  MpcTimingSample sample;
  sample.cycle_ms = 42.0;
  sample.critics_prepare_ms = 5.0;
  sample.critic_prepare = {{5.0, 1.0, true}};
  EXPECT_NO_THROW(timing.finish(sample));
  auto status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "cycles"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "failures"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "cycle_ms_mean"), 42.0);
  EXPECT_DOUBLE_EQ(value(status, "critics_prepare_ms_mean"), 5.0);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Obstacle_calls"), 0.0);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Path_calls"), 0.0);
  sample.critic_prepare = {{2.0, 1.0, true}, {3.0, 1.5, true}, {100.0, 50.0, true}};
  EXPECT_NO_THROW(timing.finish(sample));
  status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "cycles"), 2.0);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Obstacle_calls"), 0.0);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Path_calls"), 0.0);
}

TEST_F(MpcTimingTest, PerCriticWindowResetsAfterReportsLifecycleAndDiagnosticExceptions)
{
  auto now = MpcTimingPublisher::Clock::time_point(std::chrono::seconds(1));
  bool fail_clock = false;
  MpcTimingPublisher timing(enabledNode(), "FollowPath", 100.0, [&]() {
      if (fail_clock) {throw std::runtime_error("diagnostic clock failed");}
      return now;
    });
  timing.setCriticNames({"Path"});
  const auto storage = MpcTimingPublisherTestPeer::criticStorage(timing);
  timing.activate();
  MpcTimingSample sample;
  sample.success = true;
  sample.critic_prepare = {{100.0, 50.0, true}};
  timing.finish(sample);
  fail_clock = true;
  EXPECT_NO_THROW(timing.finish(sample));
  EXPECT_EQ(MpcTimingPublisherTestPeer::cycles(timing), 0U);
  fail_clock = false;
  sample.critic_prepare.front() = {2.0, 1.0, true};
  timing.finish(sample);
  auto status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Path_calls"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Path_ms_mean"), 2.0);
  now += std::chrono::seconds(1);
  timing.finish(sample);
  status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "cycles"), 0.0);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Path_calls"), 0.0);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Path_ms_max"), 0.0);
  EXPECT_EQ(MpcTimingPublisherTestPeer::criticStorage(timing), storage);
  timing.finish(sample);
  timing.deactivate();
  timing.activate();
  sample.critic_prepare.front() = {4.0, 3.0, true};
  timing.finish(sample);
  status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Path_calls"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Path_ms_mean"), 4.0);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare_cpu/Path_ms_mean"), 3.0);
  EXPECT_EQ(MpcTimingPublisherTestPeer::criticStorage(timing), storage);
}

TEST_F(MpcTimingTest, RejectsAmbiguousCriticNamesWithoutReplacingConfiguration)
{
  const auto now = MpcTimingPublisher::Clock::time_point(std::chrono::seconds(1));
  MpcTimingPublisher timing(enabledNode(), "FollowPath", 100.0, [&]() {return now;});
  timing.setCriticNames({"Obstacle", "Path"});
  EXPECT_THROW(timing.setCriticNames({"", "Path"}), std::invalid_argument);
  EXPECT_THROW(timing.setCriticNames({"Path", "Path"}), std::invalid_argument);
  timing.activate();
  EXPECT_THROW(timing.setCriticNames({"NewIdentity"}), std::logic_error);
  MpcTimingSample sample;
  sample.critic_prepare = {{1.0, 0.5, true}, {3.0, 2.0, true}};
  timing.finish(sample);
  const auto status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Obstacle_ms_mean"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "critic_prepare/Path_ms_mean"), 3.0);
}

TEST_F(MpcTimingTest, SteadyReportingBoundaryAndLifecycleDiscardOldWindow)
{
  auto now = MpcTimingPublisher::Clock::time_point(std::chrono::seconds(1));
  auto lifecycle_node = enabledNode();
  MpcTimingPublisher timing(lifecycle_node, "FollowPath", 100.0, [&]() {return now;});
  MpcTimingSample sample;
  sample.success = true;
  // Configured publishers must remain silent before activation.
  timing.finish(sample);
  EXPECT_EQ(MpcTimingPublisherTestPeer::cycles(timing), 0U);
  timing.activate();
  timing.finish(sample);
  now += std::chrono::milliseconds(999);
  timing.finish(sample);
  EXPECT_EQ(MpcTimingPublisherTestPeer::cycles(timing), 2U);
  now += std::chrono::milliseconds(1);
  timing.finish(sample);
  // A report publishes the complete window at one second, then starts fresh.
  EXPECT_EQ(MpcTimingPublisherTestPeer::cycles(timing), 0U);
  timing.finish(sample);
  EXPECT_EQ(MpcTimingPublisherTestPeer::cycles(timing), 1U);
  timing.deactivate();
  EXPECT_EQ(MpcTimingPublisherTestPeer::cycles(timing), 0U);
  EXPECT_FALSE(MpcTimingPublisherTestPeer::active(timing));
  now += std::chrono::seconds(10);
  timing.finish(sample);
  EXPECT_EQ(MpcTimingPublisherTestPeer::cycles(timing), 0U);
  timing.activate();
  timing.finish(sample);
  EXPECT_EQ(MpcTimingPublisherTestPeer::cycles(timing), 1U);
}

TEST_F(MpcTimingTest, TelemetryFailureCannotEscapeIntoControlLoop)
{
  MpcTimingPublisher timing(enabledNode(), "FollowPath", 100.0, []()
    -> MpcTimingPublisher::Clock::time_point {throw std::runtime_error("clock failed");});
  timing.activate();
  EXPECT_NO_THROW(timing.finish(MpcTimingSample{}));
  EXPECT_EQ(MpcTimingPublisherTestPeer::cycles(timing), 0U);
}

TEST_F(MpcTimingTest, RejectsInvalidConfiguration)
{
  for (const double rate : {0.0, -1.0, 10.1,
      std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
  {
    auto lifecycle_node = enabledNode({rclcpp::Parameter("FollowPath.MPC.Timing.publish_rate", rate)});
    EXPECT_THROW(MpcTimingPublisher(lifecycle_node, "FollowPath", 100.0), std::invalid_argument);
  }
  auto empty_topic = enabledNode({rclcpp::Parameter("FollowPath.MPC.Timing.topic", "")});
  EXPECT_THROW(MpcTimingPublisher(empty_topic, "FollowPath", 100.0), std::invalid_argument);
  auto lifecycle_node = enabledNode();
  EXPECT_THROW(MpcTimingPublisher(lifecycle_node, "FollowPath", 0.0), std::invalid_argument);
  EXPECT_THROW(MpcTimingPublisher(lifecycle_node, "FollowPath", 100.0,
      MpcTimingPublisher::ClockNow{}), std::invalid_argument);
}
}  // namespace
