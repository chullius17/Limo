#include "limo_dwb_critics/mpc_timing.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace limo_dwb_critics
{

void MpcTimingSample::reset() noexcept
{
  cycle_ms = thread_cpu_ms = prepare_ms = guidance_grid_ms = solve_ms = scoring_ms = debug_ms = 0.0;
  plan_prepare_ms = plan_prepare_cpu_ms = costmap_lock_wait_ms = critics_prepare_ms =
    critics_prepare_cpu_ms = debrief_ms = dwb_publish_ms = 0.0;
  generated_count = evaluated_count = rejected_count = 0;
  success = detouring = grid_updated = false;
  for (auto & critic : critic_prepare) {critic = CriticPrepareTiming{};}
}

MpcTimingPublisher::MpcTimingPublisher(
  const nav2_util::LifecycleNode::SharedPtr & node, const std::string & name,
  double budget_ms, ClockNow now)
: node_(node), name_(name), now_(std::move(now)), budget_ms_(budget_ms)
{
  const auto prefix = name + ".MPC.Timing.";
  const auto read = [&](const std::string & key, const rclcpp::ParameterValue & value) {
      if (!node_->has_parameter(prefix + key)) {
        node_->declare_parameter(prefix + key, value);
      }
      return node_->get_parameter(prefix + key);
    };
  const bool enabled = read("enabled", rclcpp::ParameterValue(false)).as_bool();
  const double rate = read("publish_rate", rclcpp::ParameterValue(1.0)).as_double();
  const auto topic = read("topic", rclcpp::ParameterValue(
        std::string("/limo/control/mpc_timing"))).as_string();
  log_summary_ = read("log_summary", rclcpp::ParameterValue(false)).as_bool();
  if (!std::isfinite(rate) || rate <= 0.0 || rate > 10.0 || topic.empty() ||
    !std::isfinite(budget_ms_) || budget_ms_ <= 0.0 || !now_)
  {
    throw std::invalid_argument("Invalid MPC.Timing publish_rate, topic, clock or cycle budget");
  }
  period_ = std::chrono::duration<double>(1.0 / rate);
  if (enabled) {
    publisher_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      topic, rclcpp::QoS(1).best_effort());
  }
}

void MpcTimingPublisher::Metric::add(double value) noexcept
{
  // Keep unavailable clock measurements from contaminating the entire window.
  if (!std::isfinite(value) || value < 0.0) {return;}
  sum += value;
  maximum = std::max(maximum, value);
  ++count;
}

void MpcTimingPublisher::setCriticNames(const std::vector<std::string> & names)
{
  if (active_) {throw std::logic_error("Cannot change MPC timing critic names while active");}
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (names[i].empty() ||
      std::find(names.begin(), names.begin() + i, names[i]) != names.begin() + i)
    {throw std::invalid_argument("MPC timing critic names must be nonempty and unique");}
  }
  // Prepare both allocations before replacing either identity or window storage.
  auto configured_names = names;
  std::vector<CriticWindow> configured_windows(names.size());
  critic_names_.swap(configured_names);
  critic_windows_.swap(configured_windows);
  reset();
}

void MpcTimingPublisher::reset() noexcept
{
  window_ = Window{};
  for (auto & critic : critic_windows_) {critic = CriticWindow{};}
  window_start_ = Clock::time_point{};
}

void MpcTimingPublisher::activate()
{
  reset();
  if (publisher_) {
    publisher_->on_activate();
    active_ = true;
  }
}

void MpcTimingPublisher::deactivate()
{
  active_ = false;
  reset();
  if (publisher_) {publisher_->on_deactivate();}
}

diagnostic_msgs::msg::DiagnosticArray MpcTimingPublisher::report(Clock::time_point now) const
{
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = node_->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name_ + "/mpc_timing";
  status.hardware_id = node_->get_name();
  status.level = window_.failures || window_.deadline_misses ?
    diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = window_.failures ? "MPC cycles failed" :
    (window_.deadline_misses ? "MPC cycle budget exceeded" : "MPC within cycle budget");
  const auto add = [&](const std::string & key, const auto value) {
      diagnostic_msgs::msg::KeyValue field;
      field.key = key;
      field.value = std::to_string(value);
      status.values.push_back(std::move(field));
    };
  add("cycles", window_.cycles);
  add("failures", window_.failures);
  add("deadline_misses", window_.deadline_misses);
  add("candidate_generated", window_.generated);
  add("candidate_evaluated", window_.evaluated);
  add("candidate_rejected", window_.rejected);
  add("detour_cycles", window_.detour_cycles);
  add("grid_updates", window_.grid_updates);
  add("budget_ms", budget_ms_);
  add("window_ms", std::chrono::duration<double, std::milli>(now - window_start_).count());
  static const std::array<const char *, 14> names{{
    "cycle_ms", "thread_cpu_ms", "prepare_ms", "guidance_grid_ms", "solve_ms",
    "scoring_ms", "debug_ms", "plan_prepare_ms", "plan_prepare_cpu_ms", "costmap_lock_wait_ms",
    "critics_prepare_ms", "critics_prepare_cpu_ms", "debrief_ms", "dwb_publish_ms"}};
  for (std::size_t i = 0; i < names.size(); ++i) {
    add(std::string(names[i]) + "_mean", window_.times[i].mean());
    add(std::string(names[i]) + "_max", window_.times[i].maximum);
  }
  for (std::size_t i = 0; i < critic_names_.size(); ++i) {
    const auto wall_prefix = "critic_prepare/" + critic_names_[i];
    const auto cpu_prefix = "critic_prepare_cpu/" + critic_names_[i];
    const auto & critic = critic_windows_[i];
    add(wall_prefix + "_ms_mean", critic.prepare.mean());
    add(wall_prefix + "_ms_max", critic.prepare.maximum);
    add(cpu_prefix + "_ms_mean", critic.prepare_cpu.mean());
    add(cpu_prefix + "_ms_max", critic.prepare_cpu.maximum);
    add(wall_prefix + "_calls", critic.calls);
  }
  message.status.push_back(std::move(status));
  return message;
}

void MpcTimingPublisher::finish(const MpcTimingSample & sample) noexcept
{
  if (!publisher_ || !active_) {return;}
  try {
    const auto now = now_();
    if (window_.cycles == 0) {window_start_ = now;}
    const std::array<double, 14> times{{sample.cycle_ms, sample.thread_cpu_ms,
      sample.prepare_ms, sample.guidance_grid_ms, sample.solve_ms,
      sample.scoring_ms, sample.debug_ms, sample.plan_prepare_ms, sample.plan_prepare_cpu_ms,
      sample.costmap_lock_wait_ms, sample.critics_prepare_ms, sample.critics_prepare_cpu_ms,
      sample.debrief_ms, sample.dwb_publish_ms}};
    for (std::size_t i = 0; i < times.size(); ++i) {window_.times[i].add(times[i]);}
    // A diagnostic size mismatch must neither index past the configured list
    // nor discard the cycle totals that are still useful for diagnosis.
    if (sample.critic_prepare.size() == critic_windows_.size()) {
      for (std::size_t i = 0; i < critic_windows_.size(); ++i) {
        const auto & input = sample.critic_prepare[i];
        if (!input.called) {continue;}
        auto & output = critic_windows_[i];
        output.prepare.add(input.prepare_ms);
        output.prepare_cpu.add(input.prepare_cpu_ms);
        ++output.calls;
      }
    }
    ++window_.cycles;
    window_.failures += !sample.success;
    window_.deadline_misses += sample.cycle_ms > budget_ms_;
    window_.generated += sample.generated_count;
    window_.evaluated += sample.evaluated_count;
    window_.rejected += sample.rejected_count;
    window_.detour_cycles += sample.detouring;
    window_.grid_updates += sample.grid_updated;
    if (now - window_start_ < period_) {return;}
    publisher_->publish(report(now));
    if (log_summary_) {
      RCLCPP_INFO(node_->get_logger(),
        "MPC timing: cycles=%llu wall mean/max=%.2f/%.2f ms CPU=%.2f ms "
        "solve=%.2f scoring=%.2f prepare=%.2f grid=%.2f debug=%.2f ms "
        "lock=%.2f plan=%.2f critics=%.2f ms "
        "overruns=%llu/%llu failures=%llu",
        static_cast<unsigned long long>(window_.cycles),
        window_.times[0].mean(), window_.times[0].maximum, window_.times[1].mean(),
        window_.times[4].mean(), window_.times[5].mean(), window_.times[2].mean(),
        window_.times[3].mean(), window_.times[6].mean(),
        window_.times[9].mean(), window_.times[7].mean(), window_.times[10].mean(),
        static_cast<unsigned long long>(window_.deadline_misses),
        static_cast<unsigned long long>(window_.cycles),
        static_cast<unsigned long long>(window_.failures));
    }
    reset();
  } catch (...) {
    // Diagnostics must never interrupt or replace a control result/exception.
    reset();
  }
}

}  // namespace limo_dwb_critics
