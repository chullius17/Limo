#ifndef LIMO_DWB_CRITICS__MPC_TIMING_HPP_
#define LIMO_DWB_CRITICS__MPC_TIMING_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <fstream>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace limo_dwb_critics
{

struct CriticPrepareTiming
{
  double prepare_ms{0.0};
  double prepare_cpu_ms{0.0};
  bool called{false};
};

struct MpcTimingSample
{
  double cycle_ms{0.0};
  double thread_cpu_ms{0.0};
  double prepare_ms{0.0};
  double guidance_grid_ms{0.0};
  double guidance_plan_ms{0.0};
  bool guidance_plan_called{false};
  bool guidance_plan_failed{false};
  bool guidance_smac{false};
  // In async mode a call records a consumed worker completion, including an
  // ignored stale result. Its elapsed time is independent of cycle_ms.
  bool guidance_async{false};
  bool guidance_plan_running{false};
  bool guidance_plan_discarded{false};
  std::int64_t guidance_plan_start_steady_ns{0};
  std::uint64_t guidance_plan_epoch{0};
  double guidance_replan_hz{0.0};
  double solve_ms{0.0};
  double scoring_ms{0.0};
  double debug_ms{0.0};
  double plan_prepare_ms{0.0};
  double plan_prepare_cpu_ms{0.0};
  double costmap_lock_wait_ms{0.0};
  double critics_prepare_ms{0.0};
  double critics_prepare_cpu_ms{0.0};
  double debrief_ms{0.0};
  double dwb_publish_ms{0.0};
  std::vector<CriticPrepareTiming> critic_prepare;
  std::size_t generated_count{0};
  std::size_t evaluated_count{0};
  std::size_t rejected_count{0};
  bool success{false};
  bool detouring{false};
  bool grid_updated{false};
  // Keep the storage configured once for the fixed critic list.
  void reset() noexcept;
};

// Serial control-loop collector: no timers, background threads or per-candidate
// messages. A steady clock controls reporting even when ROS time is paused.
class MpcTimingPublisher
{
public:
  using Clock = std::chrono::steady_clock;
  using ClockNow = std::function<Clock::time_point()>;

  MpcTimingPublisher(
    const nav2_util::LifecycleNode::SharedPtr & node, const std::string & name,
    double budget_ms, ClockNow now = Clock::now);
  bool enabled() const noexcept {return static_cast<bool>(publisher_);}
  void setCriticNames(const std::vector<std::string> & names);
  void activate();
  void deactivate();
  void finish(const MpcTimingSample & sample) noexcept;

private:
  friend class MpcTimingPublisherTestPeer;
  struct Metric
  {
    double sum{0.0};
    double maximum{0.0};
    double minimum{0.0};
    std::uint64_t count{0};
    void add(double value) noexcept;
    double mean() const noexcept {return count ? sum / count : 0.0;}
  };
  struct Window
  {
    std::array<Metric, 14> times{};
    std::uint64_t cycles{0};
    std::uint64_t failures{0};
    std::uint64_t deadline_misses{0};
    std::uint64_t generated{0};
    std::uint64_t evaluated{0};
    std::uint64_t rejected{0};
    std::uint64_t detour_cycles{0};
    std::uint64_t grid_updates{0};
    Metric guidance_plan;
    std::uint64_t guidance_plan_failures{0};
    std::uint64_t guidance_async_cycles{0};
    std::uint64_t guidance_plan_running_cycles{0};
    std::uint64_t guidance_async_completions{0};
    std::uint64_t guidance_plan_discarded{0};
    bool guidance_smac{false};
    double guidance_replan_hz{0.0};
    Metric guidance_plan_interval;
    std::uint64_t guidance_rate_limit_violations{0};
    std::uint64_t guidance_cadence_resets{0};
    std::uint64_t guidance_cache_cycles{0};
    std::uint64_t guidance_waiting_cycles{0};
  };
  struct Cadence
  {
    bool interval_valid{false};
    double interval_ms{0.0};
    bool rate_limit_violation{false};
    bool reset{false};
  };
  struct PlanHistory
  {
    std::int64_t start_ns{0};
    std::uint64_t epoch{0};
    bool smac{false};
    bool valid{false};
  };
  struct CriticWindow
  {
    Metric prepare;
    Metric prepare_cpu;
    std::uint64_t calls{0};
  };
  diagnostic_msgs::msg::DiagnosticArray report(Clock::time_point now) const;
  void reset() noexcept;
  void openCsv(const std::string & directory) noexcept;
  Cadence cadence(const MpcTimingSample & sample) noexcept;
  void writeCsv(
    const MpcTimingSample & sample, Clock::time_point now, const Cadence & cadence) noexcept;
  void flushCsv() noexcept;

  nav2_util::LifecycleNode::SharedPtr node_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  std::string name_;
  ClockNow now_;
  std::chrono::duration<double> period_{};
  Clock::time_point window_start_{};
  Window window_{};
  PlanHistory plan_history_{};
  std::vector<std::string> critic_names_;
  std::vector<CriticWindow> critic_windows_;
  double budget_ms_;
  bool log_summary_{false};
  bool active_{false};
  std::ofstream csv_;
  std::string csv_path_;
  std::uint64_t csv_session_{0};
  std::uint64_t csv_cycle_{0};
  Clock::time_point csv_last_flush_{};
};

}  // namespace limo_dwb_critics
#endif  // LIMO_DWB_CRITICS__MPC_TIMING_HPP_
