#ifndef LIMO_DWB_CRITICS__SPECIAL_AVOIDANCE__SMAC_WORKER_HPP_
#define LIMO_DWB_CRITICS__SPECIAL_AVOIDANCE__SMAC_WORKER_HPP_

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace limo_dwb_critics
{

struct SmacPlanRequest
{
  std::uint64_t generation{0};
  std::uint64_t epoch{0};
  geometry_msgs::msg::Pose2D start;
  geometry_msgs::msg::Pose2D goal;
  double target_progress{0.0};
  std::shared_ptr<const nav2_costmap_2d::Costmap2D> map;
  std::vector<geometry_msgs::msg::Point> footprint;
  double clearance_radius{0.0};
  double clearance_margin{0.0};
  double cost_weight{0.0};
  std::string frame;
  builtin_interfaces::msg::Time stamp;
};

struct SmacPlanResult
{
  std::uint64_t generation{0};
  std::uint64_t epoch{0};
  geometry_msgs::msg::Pose2D goal;
  double target_progress{0.0};
  bool planned{false};
  std::vector<geometry_msgs::msg::Pose2D> path;
  std::vector<double> lengths;
  std::string failure_reason;
  std::string detail;
  std::chrono::steady_clock::time_point started;
  std::chrono::steady_clock::time_point finished;
  double duration_ms{0.0};
};

// One accepted request owns its immutable map until planning finishes. A result
// must be consumed before another request can start; requests never form a queue.
class SmacPlannerWorker
{
public:
  using Planner = std::function<SmacPlanResult(const SmacPlanRequest &)>;

  explicit SmacPlannerWorker(Planner planner);
  ~SmacPlannerWorker();
  SmacPlannerWorker(const SmacPlannerWorker &) = delete;
  SmacPlannerWorker & operator=(const SmacPlannerWorker &) = delete;

  bool submit(SmacPlanRequest request);
  std::optional<SmacPlanResult> takeResult();
  bool busy() const;
  // Lifecycle operation only: waits for an executing native planner to finish.
  void stop();

private:
  void run();

  Planner planner_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  bool stopping_{false};
  bool busy_{false};
  std::optional<SmacPlanRequest> request_;
  std::optional<SmacPlanResult> result_;
  std::thread thread_;
};

}  // namespace limo_dwb_critics

#endif  // LIMO_DWB_CRITICS__SPECIAL_AVOIDANCE__SMAC_WORKER_HPP_
