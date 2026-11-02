#ifndef LIMO_DWB_CRITICS__MPC_TIMING_SCOPE_HPP_
#define LIMO_DWB_CRITICS__MPC_TIMING_SCOPE_HPP_

#include <chrono>
#include <ctime>
#include <limits>

namespace limo_dwb_critics
{
// Stack-only timing that also accounts for rejected trajectories/exceptions.
// A null accumulator performs no clock reads.
class MpcTimingScope
{
public:
  explicit MpcTimingScope(double * milliseconds, double * cpu_milliseconds = nullptr) noexcept
  : milliseconds_(milliseconds), cpu_milliseconds_(cpu_milliseconds),
    start_(milliseconds ? Clock::now() : Clock::time_point()),
    cpu_start_(cpu_milliseconds ? threadCpuMilliseconds() : 0.0) {}
  ~MpcTimingScope() noexcept
  {
    if (milliseconds_) {
      *milliseconds_ += std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
    }
    if (cpu_milliseconds_) {
      *cpu_milliseconds_ += threadCpuMilliseconds() - cpu_start_;
    }
  }
  static double threadCpuMilliseconds() noexcept
  {
    timespec value{};
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return value.tv_sec * 1000.0 + value.tv_nsec * 1e-6;
  }
  MpcTimingScope(const MpcTimingScope &) = delete;
  MpcTimingScope & operator=(const MpcTimingScope &) = delete;

private:
  using Clock = std::chrono::steady_clock;
  double * milliseconds_;
  double * cpu_milliseconds_;
  Clock::time_point start_;
  double cpu_start_;
};
}  // namespace limo_dwb_critics
#endif  // LIMO_DWB_CRITICS__MPC_TIMING_SCOPE_HPP_
