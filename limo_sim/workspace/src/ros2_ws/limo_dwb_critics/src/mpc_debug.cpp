#include "limo_dwb_critics/mpc_debug.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace limo_dwb_critics
{

MpcDebugPublisher::MpcDebugPublisher(
  const nav2_util::LifecycleNode::SharedPtr & node, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap, const MpcConfig & config)
: node_(node), costmap_(std::move(costmap)), config_(config)
{
  const auto prefix = name + ".MPC.Debug.";
  const auto read = [&](const std::string & key, const rclcpp::ParameterValue & value) {
      if (!node_->has_parameter(prefix + key)) {
        node_->declare_parameter(prefix + key, value);
      }
      return node_->get_parameter(prefix + key);
    };
  const bool enabled = read("enabled", rclcpp::ParameterValue(false)).as_bool();
  if (!enabled) {return;}
  const auto topic = read("topic", rclcpp::ParameterValue(
        std::string("/limo/control/mpc_debug"))).as_string();
  const double rate = read("publish_rate", rclcpp::ParameterValue(4.0)).as_double();
  const auto samples = read("samples_per_family", rclcpp::ParameterValue(6)).as_int();
  const auto stride = read("pose_stride", rclcpp::ParameterValue(
        std::min(2, config_.time_steps))).as_int();
  if (!std::isfinite(rate) || rate <= 0.0 || rate > 100.0 ||
    samples < 1 || samples > 128 || stride < 1 || stride > config_.time_steps || topic.empty())
  {
    throw std::invalid_argument("Invalid MPC.Debug rate, sample count, pose stride or topic");
  }
  samples_per_family_ = static_cast<int>(samples);
  pose_stride_ = static_cast<int>(stride);
  period_ns_ = static_cast<std::int64_t>(1e9 / rate);
  const int seeds = config_.velocity_samples * config_.curvature_samples;
  const int sampled = config_.batch_size - seeds - 2;
  const int broad = (sampled + 3) / 4;
  totals_ = {1, 1, seeds, sampled - broad, broad};
  publisher_ = node_->create_publisher<limo_interfaces::msg::MpcDebug>(
    topic, rclcpp::QoS(1).best_effort());
}

void MpcDebugPublisher::activate()
{
  if (publisher_) {publisher_->on_activate();}
}

void MpcDebugPublisher::deactivate()
{
  collecting_ = false;
  frame_ = limo_interfaces::msg::MpcDebug();
  last_publish_ns_ = 0;
  if (publisher_) {publisher_->on_deactivate();}
}

MpcObserver MpcDebugPublisher::begin(const MpcState & initial)
{
  collecting_ = false;
  if (!publisher_ || publisher_->get_subscription_count() == 0) {return {};}
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  if (last_publish_ns_ && now - last_publish_ns_ < period_ns_) {return {};}
  try {
    frame_ = limo_interfaces::msg::MpcDebug();
    frame_.header.stamp = node_->now();
    frame_.header.frame_id = costmap_->getGlobalFrameID();
    frame_.base_frame = costmap_->getBaseFrameID();
    frame_.robot_pose.x = initial.x;
    frame_.robot_pose.y = initial.y;
    frame_.robot_pose.theta = initial.yaw;
    frame_.initial_velocity = initial.control.velocity;
    frame_.initial_steering = initial.control.steering;
    frame_.wheelbase = config_.wheelbase;
    frame_.rear_axle_to_base = config_.rear_axle_to_base;
    frame_.model_dt = config_.dt;
    frame_.time_steps = config_.time_steps;
    frame_.generated_count = config_.batch_size;
    frame_.selected_id = -1;
    for (const auto & point : costmap_->getRobotFootprint()) {
      geometry_msgs::msg::Point32 vertex;
      vertex.x = point.x;
      vertex.y = point.y;
      frame_.footprint.points.push_back(vertex);
    }
    // DWB calls coreScoringAlgorithm while holding the costmap mutex. Capture
    // its raw bytes (0..255), not a separately timed OccupancyGrid topic.
    auto * map = costmap_->getCostmap();
    frame_.costmap.header = frame_.header;
    auto & meta = frame_.costmap.metadata;
    meta.update_time = frame_.header.stamp;
    meta.layer = "controller_costmap";
    meta.resolution = map->getResolution();
    meta.size_x = map->getSizeInCellsX();
    meta.size_y = map->getSizeInCellsY();
    meta.origin.position.x = map->getOriginX();
    meta.origin.position.y = map->getOriginY();
    meta.origin.orientation.w = 1.0;
    frame_.costmap.data.assign(
      map->getCharMap(), map->getCharMap() + meta.size_x * meta.size_y);
    seen_.fill(0);
    last_publish_ns_ = now;
    collecting_ = true;
  } catch (const std::exception & error) {
    RCLCPP_WARN(node_->get_logger(), "MPC preview capture failed: %s", error.what());
    return {};
  }
  return [this](const MpcRollout & rollout, MpcFamily family, std::size_t id, double cost) {
      if (!collecting_) {return;}
      try {
        const auto f = static_cast<std::size_t>(family);
        const int index = seen_.at(f)++;
        const int keep = std::min(samples_per_family_, totals_.at(f));
        bool selected = false;
        for (int k = 0; k < keep; ++k) {
          const int representative = keep == 1 ? totals_[f] / 2 :
            static_cast<int>(std::lround(k * (totals_[f] - 1.0) / (keep - 1)));
          selected = selected || index == representative;
        }
        if (selected) {frame_.candidates.push_back(candidate(rollout, family, id, cost));}
      } catch (const std::exception & error) {
        collecting_ = false;
        RCLCPP_WARN(node_->get_logger(), "MPC preview sampling failed: %s", error.what());
      }
    };
}

limo_interfaces::msg::MpcCandidate MpcDebugPublisher::candidate(
  const MpcRollout & rollout, MpcFamily family, std::size_t id, double cost) const
{
  limo_interfaces::msg::MpcCandidate msg;
  msg.candidate_id = id;
  msg.family = static_cast<std::uint8_t>(family);
  msg.total_cost = cost;
  msg.score_status = std::isnan(cost) ? msg.COST_PRUNED :
    (std::isfinite(cost) ? msg.SCORED : msg.REJECTED);
  for (std::size_t t = 0; t < rollout.states.size(); ++t) {
    if (t % pose_stride_ && t + 1 != rollout.states.size()) {continue;}
    const auto & state = rollout.states[t];
    geometry_msgs::msg::Pose2D pose;
    pose.x = state.x;
    pose.y = state.y;
    pose.theta = state.yaw;
    msg.poses.push_back(pose);
  }
  return msg;
}

void MpcDebugPublisher::finish(const MpcSolution & solution)
{
  if (!collecting_) {return;}
  collecting_ = false;
  try {
    if (std::isfinite(solution.cost)) {
      frame_.selected_id = solution.candidate_id;
      const auto & command = solution.rollout.states.at(1).control;
      frame_.command_velocity = command.velocity;
      frame_.command_steering = command.steering;
      const auto winner = candidate(
        solution.rollout, solution.family, solution.candidate_id, solution.cost);
      auto found = std::find_if(frame_.candidates.begin(), frame_.candidates.end(),
        [&](const auto & value) {return value.candidate_id == winner.candidate_id;});
      if (found == frame_.candidates.end()) {
        frame_.candidates.push_back(winner);
      } else {
        *found = winner;
      }
    }
    publisher_->publish(frame_);
  } catch (const std::exception & error) {
    RCLCPP_WARN(node_->get_logger(), "MPC preview publication failed: %s", error.what());
  }
}

}  // namespace limo_dwb_critics
