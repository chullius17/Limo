#ifndef LIMO_DWB_CRITICS__MPC_TIMING_HPP_
#define LIMO_DWB_CRITICS__MPC_TIMING_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
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
  };
  struct CriticWindow
  {
    Metric prepare;
    Metric prepare_cpu;
    std::uint64_t calls{0};
  };
  diagnostic_msgs::msg::DiagnosticArray report(Clock::time_point now) const;
  void reset() noexcept;

  nav2_util::LifecycleNode::SharedPtr node_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  std::string name_;
  ClockNow now_;
  std::chrono::duration<double> period_{};
  Clock::time_point window_start_{};
  Window window_{};
  std::vector<std::string> critic_names_;
  std::vector<CriticWindow> critic_windows_;
  double budget_ms_;
  bool log_summary_{false};
  bool active_{false};
};

}  // namespace limo_dwb_critics
#endif  // LIMO_DWB_CRITICS__MPC_TIMING_HPP_
