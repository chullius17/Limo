#include "limo_dwb_critics/special_avoidance/smac_worker.hpp"

#include <stdexcept>
#include <utility>

namespace limo_dwb_critics
{

SmacPlannerWorker::SmacPlannerWorker(Planner planner)
: planner_(std::move(planner))
{
  if (!planner_) {throw std::invalid_argument("SMAC worker requires a planner");}
  thread_ = std::thread([this]() {run();});
}

SmacPlannerWorker::~SmacPlannerWorker()
{
  stop();
}

bool SmacPlannerWorker::submit(SmacPlanRequest request)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || busy_ || result_) {return false;}
    request_ = std::move(request);
    busy_ = true;
  }
  wake_.notify_one();
  return true;
}

std::optional<SmacPlanResult> SmacPlannerWorker::takeResult()
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = std::move(result_);
  result_.reset();
  return result;
}

bool SmacPlannerWorker::busy() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return busy_;
}

void SmacPlannerWorker::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
    request_.reset();
  }
  wake_.notify_one();
  if (thread_.joinable()) {thread_.join();}
}

void SmacPlannerWorker::run()
{
  for (;;) {
    SmacPlanRequest request;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [this]() {return stopping_ || request_.has_value();});
      if (stopping_) {busy_ = false; return;}
      request = std::move(*request_);
      request_.reset();
    }

    // The planner and its potentially slow native search never hold the
    // handoff mutex. The control loop only exchanges request/result ownership.
    const auto started = std::chrono::steady_clock::now();
    SmacPlanResult result;
    try {
      result = planner_(request);
    } catch (const std::exception & error) {
      result.failure_reason = "planner_exception";
      result.detail = error.what();
    } catch (...) {
      result.failure_reason = "planner_exception";
      result.detail = "Unknown planner exception";
    }
    result.finished = std::chrono::steady_clock::now();
    result.started = started;
    result.duration_ms = std::chrono::duration<double, std::milli>(
      result.finished - result.started).count();
    result.generation = request.generation;
    result.epoch = request.epoch;
    result.goal = request.goal;
    result.target_progress = request.target_progress;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      result_ = std::move(result);
      busy_ = false;
    }
  }
}

}  // namespace limo_dwb_critics
