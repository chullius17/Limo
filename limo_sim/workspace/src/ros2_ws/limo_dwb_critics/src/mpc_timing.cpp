#include "limo_dwb_critics/mpc_timing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <locale>
#include <stdexcept>
#include <utility>
#include <unistd.h>

namespace limo_dwb_critics
{

void MpcTimingSample::reset() noexcept
{
  cycle_ms = thread_cpu_ms = prepare_ms = guidance_grid_ms = solve_ms = scoring_ms = debug_ms = 0.0;
  plan_prepare_ms = plan_prepare_cpu_ms = costmap_lock_wait_ms = critics_prepare_ms =
    critics_prepare_cpu_ms = debrief_ms = dwb_publish_ms = 0.0;
  generated_count = evaluated_count = rejected_count = 0;
  success = detouring = grid_updated = false;
  guidance_plan_ms = 0.0;
  guidance_plan_called = guidance_plan_failed = guidance_smac = false;
  guidance_async = guidance_plan_running = guidance_plan_discarded = false;
  guidance_plan_start_steady_ns = 0;
  guidance_plan_epoch = 0;
  guidance_replan_hz = 0.0;
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
  const auto csv_directory = read("csv_directory", rclcpp::ParameterValue(std::string())).as_string();
  if (!std::isfinite(rate) || rate <= 0.0 || rate > 10.0 || topic.empty() ||
    !std::isfinite(budget_ms_) || budget_ms_ <= 0.0 || !now_)
  {
    throw std::invalid_argument("Invalid MPC.Timing publish_rate, topic, clock or cycle budget");
  }
  period_ = std::chrono::duration<double>(1.0 / rate);
  if (enabled) {
    publisher_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      topic, rclcpp::QoS(1).best_effort());
    if (!csv_directory.empty()) {openCsv(csv_directory);}
  }
}

void MpcTimingPublisher::openCsv(const std::string & directory) noexcept
{
  try {
    auto expanded = directory;
    if (expanded == "~" || expanded.compare(0, 2, "~/") == 0) {
      const char * home_directory = std::getenv("HOME");
      if (!home_directory) {throw std::runtime_error("HOME is unavailable");}
      expanded.replace(0, 1, home_directory);
    }
    std::filesystem::create_directories(expanded);
    const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    csv_path_ = (std::filesystem::path(expanded) /
      ("mpc_timing_" + std::to_string(stamp) + "_" + std::to_string(getpid()) + ".csv")).string();
    csv_.imbue(std::locale::classic());
    csv_.open(csv_path_);
    if (!csv_) {throw std::runtime_error("cannot open " + csv_path_);}
    csv_ << "session,cycle,ros_time_ns,steady_time_ns,backend,plan_called,plan_failed,"
      "detouring,success,budget_ms,deadline_missed,grid_updated,grid_ms,plan_ms,"
      "cycle_ms,cpu_ms,prepare_ms,solve_ms,scoring_ms,debug_ms,generated,evaluated,rejected,"
      "plan_start_steady_ns,plan_epoch,replan_hz,plan_interval_valid,plan_interval_ms,"
      "rate_limit_violation,cadence_reset,plan_async,plan_running,plan_discarded\n";
    csv_ << std::setprecision(9);
    csv_.flush();
    if (!csv_) {throw std::runtime_error("cannot write " + csv_path_);}
    RCLCPP_INFO(node_->get_logger(), "MPC timing CSV: %s", csv_path_.c_str());
  } catch (const std::exception & error) {
    csv_.close();
    RCLCPP_WARN(node_->get_logger(), "MPC timing CSV disabled: %s", error.what());
  }
}

void MpcTimingPublisher::flushCsv() noexcept
{
  if (!csv_.is_open()) {return;}
  csv_.flush();
  if (!csv_) {
    csv_.close();
    RCLCPP_WARN(node_->get_logger(), "MPC timing CSV disabled after a write error: %s", csv_path_.c_str());
  }
}

MpcTimingPublisher::Cadence MpcTimingPublisher::cadence(const MpcTimingSample & sample) noexcept
{
  Cadence result;
  // Epochs, backend switches and lifecycle transitions delimit reported
  // intervals, while reporting windows retain history. These boundaries do not
  // exempt searches from the planner's persistent start-to-start rate limit.
  // Async idle rows describe the current controller epoch, while a later
  // consumed result can still belong to an older epoch. Only that completion
  // delimits the worker's measured start sequence.
  if (plan_history_.valid && (plan_history_.smac != sample.guidance_smac ||
    ((!sample.guidance_async || sample.guidance_plan_called) &&
    plan_history_.epoch != sample.guidance_plan_epoch)))
  {
    plan_history_.valid = false;
  }
  if (!sample.guidance_plan_called) {return result;}
  const auto start = sample.guidance_plan_start_steady_ns;
  if (start <= 0) {
    // Legacy callers cannot supply a meaningful start-to-start interval. Do not
    // bridge over an unmeasured attempt with the following measured one.
    plan_history_.valid = false;
    return result;
  }
  if (plan_history_.valid && start >= plan_history_.start_ns) {
    const auto interval_ns = start - plan_history_.start_ns;
    result.interval_valid = true;
    result.interval_ms = static_cast<double>(interval_ns) / 1e6;
    if (std::isfinite(sample.guidance_replan_hz) && sample.guidance_replan_hz > 0.0) {
      // Permit a nanosecond of representation/rounding error at the boundary.
      result.rate_limit_violation = static_cast<double>(interval_ns) + 1.0 <
        1e9 / sample.guidance_replan_hz;
    }
  } else {
    result.reset = true;
  }
  plan_history_ = {start, sample.guidance_plan_epoch, sample.guidance_smac, true};
  return result;
}

void MpcTimingPublisher::writeCsv(
  const MpcTimingSample & sample, Clock::time_point now, const Cadence & cadence) noexcept
{
  if (!csv_.is_open()) {return;}
  try {
    csv_ << csv_session_ << ',' << ++csv_cycle_ << ',' << node_->now().nanoseconds() << ',' <<
      std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() << ',' <<
      (sample.guidance_smac ? "SMAC" : "DIJKSTRA") << ',' << sample.guidance_plan_called << ',' <<
      sample.guidance_plan_failed << ',' << sample.detouring << ',' << sample.success << ',' <<
      budget_ms_ << ',' << (sample.cycle_ms > budget_ms_) << ',' << sample.grid_updated << ',' <<
      sample.guidance_grid_ms << ',' << sample.guidance_plan_ms << ',' << sample.cycle_ms << ',' <<
      sample.thread_cpu_ms << ',' << sample.prepare_ms << ',' << sample.solve_ms << ',' <<
      sample.scoring_ms << ',' << sample.debug_ms << ',' << sample.generated_count << ',' <<
      sample.evaluated_count << ',' << sample.rejected_count << ',' <<
      sample.guidance_plan_start_steady_ns << ',' << sample.guidance_plan_epoch << ',' <<
      window_.guidance_replan_hz << ',' << cadence.interval_valid << ',' <<
      cadence.interval_ms << ',' << cadence.rate_limit_violation << ',' << cadence.reset << ',' <<
      sample.guidance_async << ',' << sample.guidance_plan_running << ',' <<
      sample.guidance_plan_discarded << '\n';
    if (!csv_ || now - csv_last_flush_ >= std::chrono::seconds(1)) {
      flushCsv();
      csv_last_flush_ = now;
    }
  } catch (...) {
    csv_.close();
    RCLCPP_WARN(node_->get_logger(), "MPC timing CSV disabled after a logging error: %s", csv_path_.c_str());
  }
}

void MpcTimingPublisher::Metric::add(double value) noexcept
{
  // Keep unavailable clock measurements from contaminating the entire window.
  if (!std::isfinite(value) || value < 0.0) {return;}
  sum += value;
  maximum = std::max(maximum, value);
  minimum = count ? std::min(minimum, value) : value;
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
  plan_history_ = PlanHistory{};
  if (publisher_) {
    publisher_->on_activate();
    active_ = true;
    if (csv_.is_open()) {
      ++csv_session_;
      csv_last_flush_ = now_();
    }
  }
}

void MpcTimingPublisher::deactivate()
{
  active_ = false;
  flushCsv();
  reset();
  plan_history_ = PlanHistory{};
  if (publisher_) {publisher_->on_deactivate();}
}

diagnostic_msgs::msg::DiagnosticArray MpcTimingPublisher::report(Clock::time_point now) const
{
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = node_->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name_ + "/mpc_timing";
  status.hardware_id = node_->get_name();
  status.level = window_.failures || window_.deadline_misses ||
    window_.guidance_rate_limit_violations ?
    diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = window_.guidance_rate_limit_violations ? "Guidance planning rate limit exceeded" :
    window_.failures ? "MPC cycles failed" :
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
  status.values.push_back(diagnostic_msgs::msg::KeyValue());
  status.values.back().key = "guidance_backend";
  status.values.back().value = window_.guidance_smac ? "SMAC" : "DIJKSTRA";
  add("guidance_plan_calls", window_.guidance_plan.count);
  add("guidance_plan_failures", window_.guidance_plan_failures);
  add("guidance_async_cycles", window_.guidance_async_cycles);
  add("guidance_plan_running_cycles", window_.guidance_plan_running_cycles);
  add("guidance_async_completions", window_.guidance_async_completions);
  add("guidance_plan_discarded", window_.guidance_plan_discarded);
  add("guidance_plan_ms_mean", window_.guidance_plan.mean());
  add("guidance_plan_ms_max", window_.guidance_plan.maximum);
  add("guidance_replan_hz", window_.guidance_replan_hz);
  add("guidance_min_interval_ms", window_.guidance_replan_hz > 0.0 ?
    1000.0 / window_.guidance_replan_hz : 0.0);
  add("guidance_observed_hz", window_.guidance_plan_interval.sum > 0.0 ?
    1000.0 / window_.guidance_plan_interval.mean() : 0.0);
  add("guidance_plan_intervals", window_.guidance_plan_interval.count);
  add("guidance_plan_interval_ms_min", window_.guidance_plan_interval.minimum);
  add("guidance_plan_interval_ms_mean", window_.guidance_plan_interval.mean());
  add("guidance_plan_interval_ms_max", window_.guidance_plan_interval.maximum);
  add("guidance_rate_limit_violations", window_.guidance_rate_limit_violations);
  add("guidance_cadence_resets", window_.guidance_cadence_resets);
  add("guidance_cache_cycles", window_.guidance_cache_cycles);
  add("guidance_waiting_cycles", window_.guidance_waiting_cycles);
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
    window_.guidance_smac = sample.guidance_smac;
    window_.guidance_replan_hz = std::isfinite(sample.guidance_replan_hz) &&
      sample.guidance_replan_hz > 0.0 ? sample.guidance_replan_hz : 0.0;
    const auto plan_cadence = cadence(sample);
    if (plan_cadence.interval_valid) {
      window_.guidance_plan_interval.add(plan_cadence.interval_ms);
    }
    window_.guidance_rate_limit_violations += plan_cadence.rate_limit_violation;
    window_.guidance_cadence_resets += plan_cadence.reset;
    window_.guidance_async_cycles += sample.guidance_async;
    window_.guidance_plan_running_cycles += sample.guidance_async && sample.guidance_plan_running;
    window_.guidance_cache_cycles += sample.detouring &&
      !sample.guidance_plan_called && sample.success;
    // Count a guidance wait only without an active route. A failed MPC solve
    // while following a valid cache is a separate controller failure.
    window_.guidance_waiting_cycles += sample.guidance_smac && sample.grid_updated &&
      !sample.guidance_plan_called && !sample.success && !sample.detouring;
    if (sample.guidance_plan_called) {
      window_.guidance_plan.add(sample.guidance_plan_ms);
      window_.guidance_plan_failures += sample.guidance_plan_failed;
      window_.guidance_async_completions += sample.guidance_async;
      window_.guidance_plan_discarded += sample.guidance_plan_discarded;
    }
    writeCsv(sample, now, plan_cadence);
    if (now - window_start_ < period_) {return;}
    publisher_->publish(report(now));
    if (window_.guidance_rate_limit_violations) {
      RCLCPP_WARN(node_->get_logger(),
        "Guidance rate limit exceeded: backend=%s limit=%.3f Hz min_interval=%.3f ms "
        "observed_min=%.3f ms violations=%llu intervals=%llu",
        window_.guidance_smac ? "SMAC" : "DIJKSTRA", window_.guidance_replan_hz,
        window_.guidance_replan_hz > 0.0 ? 1000.0 / window_.guidance_replan_hz : 0.0,
        window_.guidance_plan_interval.minimum,
        static_cast<unsigned long long>(window_.guidance_rate_limit_violations),
        static_cast<unsigned long long>(window_.guidance_plan_interval.count));
    }
    if (log_summary_) {
      RCLCPP_INFO(node_->get_logger(),
        "MPC timing: cycles=%llu wall mean/max=%.2f/%.2f ms CPU=%.2f ms "
        "solve=%.2f scoring=%.2f prepare=%.2f grid=%.2f debug=%.2f ms "
        "lock=%.2f plan=%.2f critics=%.2f ms guidance=%s search=%.2f/%.2f ms "
        "guidance_calls=%llu guidance_failures=%llu detour_cycles=%llu "
        "async_cycles=%llu in_flight_cycles=%llu async_completions=%llu discarded=%llu "
        "limit=%.3f Hz min_interval=%.3f ms observed=%.3f Hz "
        "interval_min/mean/max=%.3f/%.3f/%.3f ms intervals=%llu "
        "rate_violations=%llu cadence_resets=%llu cache_cycles=%llu waiting_cycles=%llu "
        "overruns=%llu/%llu failures=%llu",
        static_cast<unsigned long long>(window_.cycles),
        window_.times[0].mean(), window_.times[0].maximum, window_.times[1].mean(),
        window_.times[4].mean(), window_.times[5].mean(), window_.times[2].mean(),
        window_.times[3].mean(), window_.times[6].mean(),
        window_.times[9].mean(), window_.times[7].mean(), window_.times[10].mean(),
        window_.guidance_smac ? "SMAC" : "DIJKSTRA",
        window_.guidance_plan.mean(), window_.guidance_plan.maximum,
        static_cast<unsigned long long>(window_.guidance_plan.count),
        static_cast<unsigned long long>(window_.guidance_plan_failures),
        static_cast<unsigned long long>(window_.detour_cycles),
        static_cast<unsigned long long>(window_.guidance_async_cycles),
        static_cast<unsigned long long>(window_.guidance_plan_running_cycles),
        static_cast<unsigned long long>(window_.guidance_async_completions),
        static_cast<unsigned long long>(window_.guidance_plan_discarded),
        window_.guidance_replan_hz,
        window_.guidance_replan_hz > 0.0 ? 1000.0 / window_.guidance_replan_hz : 0.0,
        window_.guidance_plan_interval.sum > 0.0 ?
        1000.0 / window_.guidance_plan_interval.mean() : 0.0,
        window_.guidance_plan_interval.minimum, window_.guidance_plan_interval.mean(),
        window_.guidance_plan_interval.maximum,
        static_cast<unsigned long long>(window_.guidance_plan_interval.count),
        static_cast<unsigned long long>(window_.guidance_rate_limit_violations),
        static_cast<unsigned long long>(window_.guidance_cadence_resets),
        static_cast<unsigned long long>(window_.guidance_cache_cycles),
        static_cast<unsigned long long>(window_.guidance_waiting_cycles),
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
