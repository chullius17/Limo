#include <gtest/gtest.h>

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

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

class TimingCsvDirectory
{
public:
  TimingCsvDirectory()
  {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    path = std::filesystem::path("/tmp") /
      ("limo_mpc_timing_test_" + std::to_string(getpid()) + "_" + std::to_string(stamp));
    if (!std::filesystem::create_directory(path)) {
      throw std::runtime_error("Cannot create timing test directory");
    }
  }
  ~TimingCsvDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }
  std::vector<std::filesystem::path> files() const
  {
    std::vector<std::filesystem::path> result;
    for (const auto & entry : std::filesystem::directory_iterator(path)) {
      if (entry.is_regular_file() && entry.path().extension() == ".csv") {
        result.push_back(entry.path());
      }
    }
    std::sort(result.begin(), result.end());
    return result;
  }
  std::filesystem::path path;
};

using CsvRows = std::vector<std::vector<std::string>>;

CsvRows readCsv(const std::filesystem::path & path)
{
  std::ifstream input(path);
  if (!input) {throw std::runtime_error("Cannot read timing CSV");}
  CsvRows rows;
  std::string line;
  while (std::getline(input, line)) {
    std::istringstream fields(line);
    rows.emplace_back();
    std::string field;
    while (std::getline(fields, field, ',')) {rows.back().push_back(field);}
  }
  return rows;
}

const std::vector<std::string> csv_header{
  "session", "cycle", "ros_time_ns", "steady_time_ns", "backend", "plan_called", "plan_failed",
  "detouring", "success", "budget_ms", "deadline_missed", "grid_updated", "grid_ms", "plan_ms",
  "cycle_ms", "cpu_ms", "prepare_ms", "solve_ms", "scoring_ms", "debug_ms", "generated",
  "evaluated", "rejected", "plan_start_steady_ns", "plan_epoch", "replan_hz",
  "plan_interval_valid", "plan_interval_ms", "rate_limit_violation", "cadence_reset",
  "plan_async", "plan_running", "plan_discarded"};

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

TEST_F(MpcTimingTest, DisabledCollectorDoesNotCreateConfiguredCsvDirectory)
{
  TimingCsvDirectory directory;
  const auto unused = directory.path / "disabled";
  MpcTimingPublisher timing(node({rclcpp::Parameter(
        "FollowPath.MPC.Timing.csv_directory", unused.string())}), "FollowPath", 50.0);
  timing.activate();
  timing.finish(MpcTimingSample{});
  timing.deactivate();
  EXPECT_FALSE(timing.enabled());
  EXPECT_FALSE(std::filesystem::exists(unused));
  EXPECT_TRUE(directory.files().empty());
}

TEST_F(MpcTimingTest, CsvRecordsIdleActiveFailedAndReactivatedCycles)
{
  TimingCsvDirectory directory;
  auto now = MpcTimingPublisher::Clock::time_point(std::chrono::seconds(10));
  MpcTimingPublisher timing(enabledNode({rclcpp::Parameter(
        "FollowPath.MPC.Timing.csv_directory", directory.path.string())}),
    "FollowPath", 50.0, [&]() {return now;});
  const auto files = directory.files();
  ASSERT_EQ(files.size(), 1U);
  ASSERT_EQ(readCsv(files.front()).size(), 1U);
  EXPECT_EQ(readCsv(files.front()).front(), csv_header);
  MpcTimingSample sample;
  sample.success = true;
  sample.cycle_ms = 50.0;
  // A configured, inactive collector must not write rows.
  timing.finish(sample);
  timing.activate();
  timing.finish(sample);
  now += std::chrono::milliseconds(10);
  sample.guidance_smac = sample.guidance_plan_called = sample.detouring = sample.grid_updated = true;
  sample.guidance_plan_ms = 12.25;
  sample.guidance_grid_ms = 0.5;
  sample.cycle_ms = 60.0;
  sample.thread_cpu_ms = 30.0;
  sample.prepare_ms = 15.0;
  sample.solve_ms = 40.0;
  sample.scoring_ms = 35.0;
  sample.debug_ms = 2.0;
  sample.generated_count = 10;
  sample.evaluated_count = 8;
  sample.rejected_count = 2;
  timing.finish(sample);
  now += std::chrono::milliseconds(10);
  sample.guidance_plan_failed = true;
  sample.success = false;
  sample.guidance_plan_ms = 6.75;
  sample.cycle_ms = 20.0;
  timing.finish(sample);
  timing.deactivate();
  timing.finish(sample);
  auto rows = readCsv(files.front());
  ASSERT_EQ(rows.size(), 4U);
  EXPECT_EQ(rows.front(), csv_header);
  for (std::size_t row = 1; row < rows.size(); ++row) {
    ASSERT_EQ(rows[row].size(), csv_header.size());
    for (std::size_t column = 0; column < rows[row].size(); ++column) {
      if (column == 4) {continue;}
      const auto number = std::stod(rows[row][column]);
      EXPECT_TRUE(std::isfinite(number)) << csv_header[column];
      EXPECT_GE(number, 0.0) << csv_header[column];
    }
    EXPECT_EQ(std::stoull(rows[row][0]), 1U);
    EXPECT_EQ(std::stoull(rows[row][1]), row);
    EXPECT_GT(std::stoll(rows[row][2]), 0);
    EXPECT_EQ(std::stoll(rows[row][3]),
      10000000000LL + static_cast<long long>(row - 1) * 10000000LL);
    EXPECT_DOUBLE_EQ(std::stod(rows[row][9]), 50.0);
  }
  EXPECT_EQ(rows[1][4], "DIJKSTRA");
  EXPECT_EQ(rows[1][5], "0");
  EXPECT_EQ(rows[1][7], "0");
  EXPECT_EQ(rows[1][8], "1");
  EXPECT_EQ(rows[1][10], "0");  // Equal to budget is not a missed deadline.
  EXPECT_EQ(rows[2][4], "SMAC");
  EXPECT_EQ(rows[2][5], "1");
  EXPECT_EQ(rows[2][6], "0");
  EXPECT_EQ(rows[2][7], "1");
  EXPECT_EQ(rows[2][8], "1");
  EXPECT_EQ(rows[2][10], "1");
  EXPECT_EQ(rows[2][11], "1");
  const std::array<double, 11> active_values{{0.5, 12.25, 60.0, 30.0, 15.0,
    40.0, 35.0, 2.0, 10.0, 8.0, 2.0}};
  for (std::size_t i = 0; i < active_values.size(); ++i) {
    EXPECT_DOUBLE_EQ(std::stod(rows[2][12 + i]), active_values[i]);
  }
  EXPECT_EQ(rows[3][4], "SMAC");
  EXPECT_EQ(rows[3][5], "1");
  EXPECT_EQ(rows[3][6], "1");
  EXPECT_EQ(rows[3][8], "0");
  EXPECT_DOUBLE_EQ(std::stod(rows[3][13]), 6.75);
  now += std::chrono::milliseconds(10);
  sample.reset();
  sample.success = sample.guidance_plan_called = true;
  sample.guidance_plan_ms = 1.25;
  timing.activate();
  timing.finish(sample);
  timing.deactivate();
  rows = readCsv(files.front());
  ASSERT_EQ(rows.size(), 5U);
  ASSERT_EQ(rows.back().size(), csv_header.size());
  EXPECT_EQ(rows.back()[0], "2");
  EXPECT_EQ(rows.back()[1], "4");
  EXPECT_EQ(rows.back()[4], "DIJKSTRA");
  EXPECT_EQ(rows.back()[5], "1");
  EXPECT_DOUBLE_EQ(std::stod(rows.back()[13]), 1.25);
}

TEST_F(MpcTimingTest, CadenceUsesActualSearchStartsRatherThanUnequalFinishDurations)
{
  auto now = MpcTimingPublisher::Clock::time_point(
    std::chrono::seconds(10) + std::chrono::milliseconds(400));
  MpcTimingPublisher timing(enabledNode(), "FollowPath", 1000.0, [&]() {return now;});
  timing.activate();
  MpcTimingSample sample;
  sample.success = sample.guidance_smac = sample.guidance_plan_called = sample.detouring = true;
  sample.guidance_replan_hz = 2.0;
  sample.guidance_plan_epoch = 1;
  sample.guidance_plan_start_steady_ns = 10000000000LL;
  sample.guidance_plan_ms = 400.0;
  timing.finish(sample);
  now += std::chrono::milliseconds(110);  // Finish spacing differs from start spacing.
  sample.guidance_plan_start_steady_ns += 500000000LL;
  sample.guidance_plan_ms = 10.0;
  timing.finish(sample);
  auto status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "guidance_replan_hz"), 2.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_min_interval_ms"), 500.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_observed_hz"), 2.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_interval_ms_min"), 500.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_interval_ms_mean"), 500.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_interval_ms_max"), 500.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_intervals"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_rate_limit_violations"), 0.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_cadence_resets"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_ms_mean"), 205.0);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  sample.guidance_plan_start_steady_ns += 499999999LL;  // One nanosecond tolerance.
  timing.finish(sample);
  sample.guidance_plan_start_steady_ns += 499999998LL;
  timing.finish(sample);
  status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "guidance_rate_limit_violations"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_intervals"), 3.0);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(status.message, "Guidance planning rate limit exceeded");
}

TEST_F(MpcTimingTest, AsyncWorkerCompletionsDoNotConsumeControllerCycleBudget)
{
  const auto now = MpcTimingPublisher::Clock::time_point(std::chrono::seconds(10));
  MpcTimingPublisher timing(enabledNode(), "FollowPath", 50.0, [&]() {return now;});
  timing.activate();
  MpcTimingSample sample;
  sample.success = sample.guidance_smac = sample.guidance_async = true;
  sample.guidance_plan_running = sample.detouring = true;
  sample.cycle_ms = 8.0;
  sample.guidance_replan_hz = 2.0;
  sample.guidance_plan_epoch = 7;
  timing.finish(sample);
  timing.finish(sample);  // An in-flight worker is not a second attempt.
  sample.success = sample.detouring = false;
  sample.grid_updated = true;
  timing.finish(sample);  // Waiting with a zero command remains independent.
  sample.success = sample.detouring = sample.guidance_plan_called = true;
  sample.guidance_plan_running = false;
  sample.guidance_plan_discarded = true;
  sample.guidance_plan_ms = 240.0;
  sample.guidance_plan_start_steady_ns = 10000000000LL;
  timing.finish(sample);  // A consumed stale result still records its elapsed time.
  sample.guidance_plan_called = false;
  sample.guidance_plan_running = true;
  timing.finish(sample);  // Leftover result flags must never count another completion.
  auto status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "cycles"), 5.0);
  EXPECT_DOUBLE_EQ(value(status, "cycle_ms_mean"), 8.0);
  EXPECT_DOUBLE_EQ(value(status, "cycle_ms_max"), 8.0);
  EXPECT_DOUBLE_EQ(value(status, "deadline_misses"), 0.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_calls"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_failures"), 0.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_ms_mean"), 240.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_ms_max"), 240.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_async_cycles"), 5.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_running_cycles"), 4.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_async_completions"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_discarded"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_waiting_cycles"), 1.0);

  sample.guidance_plan_called = sample.guidance_plan_failed = true;
  sample.guidance_plan_running = sample.guidance_plan_discarded = false;
  sample.guidance_plan_ms = 120.0;
  sample.guidance_plan_start_steady_ns += 500000000LL;
  timing.finish(sample);  // A failed worker can leave a valid cached route active.
  sample.guidance_plan_called = false;
  sample.guidance_plan_running = true;
  timing.finish(sample);
  status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_calls"), 2.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_failures"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_async_completions"), 2.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_discarded"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_ms_mean"), 180.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_interval_ms_mean"), 500.0);
  EXPECT_DOUBLE_EQ(value(status, "deadline_misses"), 0.0);
  EXPECT_DOUBLE_EQ(value(status, "failures"), 1.0);
}

TEST_F(MpcTimingTest, AsyncCsvUsesConsumedResultEpochAfterIdleControllerEpochChanges)
{
  TimingCsvDirectory directory;
  const auto now = MpcTimingPublisher::Clock::time_point(std::chrono::seconds(10));
  MpcTimingPublisher timing(enabledNode({rclcpp::Parameter(
        "FollowPath.MPC.Timing.csv_directory", directory.path.string())}),
    "FollowPath", 50.0, [&]() {return now;});
  timing.activate();
  MpcTimingSample sample;
  sample.success = sample.guidance_smac = sample.guidance_async = true;
  sample.guidance_plan_called = true;
  sample.guidance_plan_epoch = 7;
  sample.guidance_plan_start_steady_ns = 10000000000LL;
  sample.guidance_plan_ms = 200.0;
  sample.guidance_replan_hz = 2.0;
  sample.cycle_ms = 8.0;
  timing.finish(sample);
  sample.guidance_plan_called = false;
  sample.guidance_plan_running = true;
  sample.guidance_plan_epoch = 8;
  sample.guidance_plan_start_steady_ns = 0;
  sample.guidance_plan_ms = 0.0;
  timing.finish(sample);  // Current reference changed while an older job was running.
  sample.guidance_plan_called = sample.guidance_plan_discarded = true;
  sample.guidance_plan_running = false;
  sample.guidance_plan_epoch = 7;
  sample.guidance_plan_start_steady_ns = 10500000000LL;
  sample.guidance_plan_ms = 150.0;
  timing.finish(sample);  // Its result retains the actual start and old epoch.
  sample.guidance_plan_discarded = false;
  sample.guidance_plan_epoch = 8;
  sample.guidance_plan_start_steady_ns = 10600000000LL;
  timing.finish(sample);  // The first completed job from the new epoch is exempt.
  const auto status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_calls"), 3.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_intervals"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_interval_ms_mean"), 500.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_rate_limit_violations"), 0.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_cadence_resets"), 2.0);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  timing.deactivate();
  const auto files = directory.files();
  ASSERT_EQ(files.size(), 1U);
  const auto rows = readCsv(files.front());
  ASSERT_EQ(rows.size(), 5U);
  EXPECT_EQ(rows.front(), csv_header);
  for (std::size_t row = 1; row < rows.size(); ++row) {
    ASSERT_EQ(rows[row].size(), csv_header.size());
    EXPECT_EQ(rows[row][30], "1");
    EXPECT_EQ(rows[row][10], "0");
  }
  EXPECT_EQ(rows[1][31], "0");
  EXPECT_EQ(rows[1][32], "0");
  EXPECT_EQ(rows[2][5], "0");
  EXPECT_EQ(rows[2][23], "0");
  EXPECT_EQ(rows[2][24], "8");
  EXPECT_EQ(rows[2][31], "1");
  EXPECT_EQ(rows[2][32], "0");
  EXPECT_EQ(rows[3][5], "1");
  EXPECT_EQ(rows[3][6], "0");
  EXPECT_EQ(rows[3][13], "150");
  EXPECT_EQ(rows[3][23], "10500000000");
  EXPECT_EQ(rows[3][24], "7");
  EXPECT_EQ(rows[3][26], "1");
  EXPECT_EQ(rows[3][27], "500");
  EXPECT_EQ(rows[3][31], "0");
  EXPECT_EQ(rows[3][32], "1");
  EXPECT_EQ(rows[4][24], "8");
  EXPECT_EQ(rows[4][26], "0");
  EXPECT_EQ(rows[4][29], "1");
}

TEST_F(MpcTimingTest, CadenceSurvivesReportWindowsButResetsForEpochBackendAndLifecycle)
{
  auto now = MpcTimingPublisher::Clock::time_point(std::chrono::seconds(10));
  MpcTimingPublisher timing(enabledNode(), "FollowPath", 50.0, [&]() {return now;});
  timing.activate();
  MpcTimingSample sample;
  sample.success = sample.guidance_smac = sample.guidance_plan_called = true;
  sample.guidance_replan_hz = 2.0;
  sample.guidance_plan_epoch = 1;
  sample.guidance_plan_start_steady_ns = 10000000000LL;
  timing.finish(sample);
  now += std::chrono::seconds(1);
  sample.guidance_plan_start_steady_ns += 1000000000LL;
  timing.finish(sample);  // Publishes and clears aggregation, retaining start history.
  EXPECT_EQ(MpcTimingPublisherTestPeer::cycles(timing), 0U);
  now += std::chrono::milliseconds(100);
  sample.guidance_plan_start_steady_ns += 100000000LL;
  timing.finish(sample);
  auto status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_intervals"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_interval_ms_mean"), 100.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_rate_limit_violations"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_cadence_resets"), 0.0);

  ++sample.guidance_plan_epoch;
  sample.guidance_plan_start_steady_ns += 1;
  timing.finish(sample);  // New detour may search immediately.
  sample.guidance_smac = false;
  sample.guidance_replan_hz = 0.0;
  sample.guidance_plan_epoch = 0;
  sample.guidance_plan_start_steady_ns += 1;
  timing.finish(sample);  // Backend switch also starts a separate cadence.
  sample.guidance_plan_start_steady_ns += 10000000LL;
  timing.finish(sample);  // Unlimited Dijkstra may run after ten milliseconds.
  status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_intervals"), 2.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_rate_limit_violations"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_cadence_resets"), 2.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_replan_hz"), 0.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_min_interval_ms"), 0.0);
  timing.deactivate();
  timing.activate();
  sample.guidance_plan_start_steady_ns += 1;
  timing.finish(sample);
  status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_intervals"), 0.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_cadence_resets"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_rate_limit_violations"), 0.0);
}

TEST_F(MpcTimingTest, CachedWaitingAndLegacySamplesCannotCreateSearchIntervals)
{
  const auto now = MpcTimingPublisher::Clock::time_point(std::chrono::seconds(10));
  MpcTimingPublisher timing(enabledNode(), "FollowPath", 50.0, [&]() {return now;});
  timing.activate();
  MpcTimingSample sample;
  sample.success = sample.guidance_smac = sample.guidance_plan_called = true;
  sample.guidance_replan_hz = 2.0;
  sample.guidance_plan_epoch = 1;
  sample.guidance_plan_start_steady_ns = 10000000000LL;
  timing.finish(sample);
  sample.guidance_plan_called = false;
  sample.detouring = true;
  timing.finish(sample);  // Cached successful detour.
  sample.success = false;
  sample.grid_updated = true;
  timing.finish(sample);  // A failed MPC solve with an active route is not a guidance wait.
  sample.detouring = false;
  sample.grid_updated = true;
  timing.finish(sample);  // Grid checked, no search, no valid route to follow.
  sample.grid_updated = false;
  timing.finish(sample);  // Generic failure is not counted as a guidance wait.
  sample.success = sample.guidance_plan_called = true;
  sample.guidance_plan_start_steady_ns += 500000000LL;
  timing.finish(sample);
  auto status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_intervals"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_interval_ms_mean"), 500.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_cache_cycles"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_waiting_cycles"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_rate_limit_violations"), 0.0);
  sample.guidance_plan_start_steady_ns = 0;  // A legacy, untimestamped actual search.
  timing.finish(sample);
  sample.guidance_plan_start_steady_ns = 10510000000LL;
  timing.finish(sample);  // Missing attempt prevents bridging an invalid interval.
  status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_intervals"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_cadence_resets"), 2.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_rate_limit_violations"), 0.0);
}

TEST_F(MpcTimingTest, CsvSeparatesSearchCadenceFromCachedCyclesAndImmediateEpochStarts)
{
  TimingCsvDirectory directory;
  auto now = MpcTimingPublisher::Clock::time_point(std::chrono::seconds(10));
  MpcTimingPublisher timing(enabledNode({rclcpp::Parameter(
        "FollowPath.MPC.Timing.csv_directory", directory.path.string())}),
    "FollowPath", 50.0, [&]() {return now;});
  const auto files = directory.files();
  ASSERT_EQ(files.size(), 1U);
  timing.activate();
  MpcTimingSample sample;
  sample.success = sample.guidance_smac = sample.guidance_plan_called = sample.detouring = true;
  sample.guidance_replan_hz = 2.0;
  sample.guidance_plan_epoch = 7;
  sample.guidance_plan_start_steady_ns = 10000000000LL;
  timing.finish(sample);
  sample.guidance_plan_called = false;
  timing.finish(sample);
  sample.guidance_plan_called = true;
  sample.guidance_plan_start_steady_ns += 500000000LL;
  timing.finish(sample);
  sample.guidance_plan_failed = true;
  sample.guidance_plan_start_steady_ns += 100000000LL;
  timing.finish(sample);
  ++sample.guidance_plan_epoch;
  sample.guidance_plan_start_steady_ns += 1;
  timing.finish(sample);
  sample.guidance_plan_start_steady_ns = 0;
  timing.finish(sample);
  timing.deactivate();
  timing.activate();
  sample.guidance_plan_start_steady_ns = 11000000000LL;
  timing.finish(sample);
  timing.deactivate();
  const auto rows = readCsv(files.front());
  ASSERT_EQ(rows.size(), 8U);
  EXPECT_EQ(rows.front(), csv_header);
  for (std::size_t row = 1; row < rows.size(); ++row) {
    ASSERT_EQ(rows[row].size(), csv_header.size());
    EXPECT_EQ(rows[row][25], "2");
  }
  EXPECT_EQ(rows[1][23], "10000000000");
  EXPECT_EQ(rows[1][24], "7");
  EXPECT_EQ(rows[1][26], "0");
  EXPECT_EQ(rows[1][27], "0");
  EXPECT_EQ(rows[1][28], "0");
  EXPECT_EQ(rows[1][29], "1");
  EXPECT_EQ(rows[2][5], "0");
  EXPECT_EQ(rows[2][26], "0");
  EXPECT_EQ(rows[2][27], "0");
  EXPECT_EQ(rows[2][28], "0");
  EXPECT_EQ(rows[2][29], "0");
  EXPECT_EQ(rows[3][26], "1");
  EXPECT_EQ(rows[3][27], "500");
  EXPECT_EQ(rows[3][28], "0");
  EXPECT_EQ(rows[4][6], "1");  // Failed searches count in cadence too.
  EXPECT_EQ(rows[4][27], "100");
  EXPECT_EQ(rows[4][28], "1");
  EXPECT_EQ(rows[5][24], "8");
  EXPECT_EQ(rows[5][26], "0");
  EXPECT_EQ(rows[5][28], "0");
  EXPECT_EQ(rows[5][29], "1");
  EXPECT_EQ(rows[6][23], "0");
  EXPECT_EQ(rows[6][26], "0");
  EXPECT_EQ(rows[6][29], "0");
  EXPECT_EQ(rows[7][0], "2");
  EXPECT_EQ(rows[7][26], "0");
  EXPECT_EQ(rows[7][29], "1");
}

TEST_F(MpcTimingTest, CsvFlushesDuringActiveSessionAfterOneSteadySecond)
{
  TimingCsvDirectory directory;
  auto now = MpcTimingPublisher::Clock::time_point(std::chrono::seconds(10));
  MpcTimingPublisher timing(enabledNode({rclcpp::Parameter(
        "FollowPath.MPC.Timing.csv_directory", directory.path.string())}),
    "FollowPath", 50.0, [&]() {return now;});
  const auto files = directory.files();
  ASSERT_EQ(files.size(), 1U);
  timing.activate();
  MpcTimingSample sample;
  sample.success = true;
  timing.finish(sample);
  now += std::chrono::seconds(1);
  timing.finish(sample);
  // Read while the publisher remains active: no destructor/deactivation flush.
  const auto rows = readCsv(files.front());
  ASSERT_EQ(rows.size(), 3U);
  EXPECT_EQ(rows.front(), csv_header);
  timing.deactivate();
}

TEST_F(MpcTimingTest, CsvPublishersUseSeparateFiles)
{
  TimingCsvDirectory directory;
  const auto parameters = std::vector<rclcpp::Parameter>{rclcpp::Parameter(
      "FollowPath.MPC.Timing.csv_directory", directory.path.string())};
  MpcTimingPublisher first(enabledNode(parameters), "FollowPath", 50.0);
  MpcTimingPublisher second(enabledNode(parameters), "FollowPath", 50.0);
  const auto files = directory.files();
  ASSERT_EQ(files.size(), 2U);
  EXPECT_NE(files[0], files[1]);
  for (const auto & file : files) {
    EXPECT_EQ(file.filename().string().find("mpc_timing_"), 0U);
    const auto rows = readCsv(file);
    ASSERT_EQ(rows.size(), 1U);
    EXPECT_EQ(rows.front(), csv_header);
  }
}

TEST_F(MpcTimingTest, InvalidCsvDirectoryDoesNotDisableCycleDiagnostics)
{
  TimingCsvDirectory directory;
  const auto not_a_directory = directory.path / "regular_file";
  {
    std::ofstream file(not_a_directory);
    file << "keep this file";
  }
  auto now = MpcTimingPublisher::Clock::time_point(std::chrono::seconds(10));
  std::unique_ptr<MpcTimingPublisher> timing;
  EXPECT_NO_THROW(timing = std::make_unique<MpcTimingPublisher>(
      enabledNode({rclcpp::Parameter(
          "FollowPath.MPC.Timing.csv_directory", not_a_directory.string())}),
      "FollowPath", 50.0, [&]() {return now;}));
  ASSERT_TRUE(timing);
  EXPECT_TRUE(timing->enabled());
  timing->activate();
  MpcTimingSample sample;
  sample.success = true;
  sample.cycle_ms = 12.0;
  EXPECT_NO_THROW(timing->finish(sample));
  const auto status = MpcTimingPublisherTestPeer::report(*timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "cycles"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "cycle_ms_mean"), 12.0);
  EXPECT_TRUE(directory.files().empty());
  EXPECT_TRUE(std::filesystem::is_regular_file(not_a_directory));
  timing->deactivate();
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

TEST_F(MpcTimingTest, GuidanceSearchMeansCountOnlyAttemptsIncludingFailedSearches)
{
  const auto now = MpcTimingPublisher::Clock::time_point(std::chrono::seconds(1));
  MpcTimingPublisher timing(enabledNode(), "FollowPath", 50.0, [&]() {return now;});
  timing.activate();
  MpcTimingSample sample;
  sample.success = true;
  sample.guidance_smac = true;
  sample.guidance_plan_called = true;
  sample.guidance_plan_ms = 12.0;
  timing.finish(sample);
  sample.guidance_plan_ms = 8.0;
  sample.guidance_plan_failed = true;
  sample.success = false;
  timing.finish(sample);
  sample.guidance_plan_called = false;
  sample.guidance_plan_ms = 1000.0;
  timing.finish(sample);
  const auto status = MpcTimingPublisherTestPeer::report(timing, now).status.front();
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_calls"), 2.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_failures"), 1.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_ms_mean"), 10.0);
  EXPECT_DOUBLE_EQ(value(status, "guidance_plan_ms_max"), 12.0);
  bool identified = false;
  for (const auto & field : status.values) {
    if (field.key == "guidance_backend") {identified = field.value == "SMAC";}
  }
  EXPECT_TRUE(identified);
  sample.reset();
  EXPECT_DOUBLE_EQ(sample.guidance_plan_ms, 0.0);
  EXPECT_FALSE(sample.guidance_plan_called || sample.guidance_plan_failed || sample.guidance_smac);
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
  sample.guidance_plan_ms = sample.guidance_replan_hz = 2.0;
  sample.guidance_plan_start_steady_ns = 123456789LL;
  sample.guidance_plan_epoch = 9;
  sample.guidance_plan_called = sample.guidance_plan_failed = sample.guidance_smac = true;
  sample.guidance_async = sample.guidance_plan_running = sample.guidance_plan_discarded = true;
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
  EXPECT_DOUBLE_EQ(sample.guidance_plan_ms, 0.0);
  EXPECT_DOUBLE_EQ(sample.guidance_replan_hz, 0.0);
  EXPECT_EQ(sample.guidance_plan_start_steady_ns, 0);
  EXPECT_EQ(sample.guidance_plan_epoch, 0U);
  EXPECT_FALSE(sample.guidance_plan_called || sample.guidance_plan_failed || sample.guidance_smac);
  EXPECT_FALSE(sample.guidance_async || sample.guidance_plan_running || sample.guidance_plan_discarded);
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
