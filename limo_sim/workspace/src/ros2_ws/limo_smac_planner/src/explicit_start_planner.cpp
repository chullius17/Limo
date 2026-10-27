#include <cmath>
#include <mutex>
#include <stdexcept>
#include "limo_interfaces/srv/compute_path_with_start.hpp"
#include "limo_smac_planner/dubins_connector.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

namespace limo_smac_planner
{
class ExplicitStartPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    clock_ = node->get_clock();
    costmap_ = costmap;
    const auto read = [&](const std::string & key, double value) {
        nav2_util::declare_parameter_if_not_declared(
          node, name + "." + key,
          rclcpp::ParameterValue(value));
        const double result = node->get_parameter(name + "." + key).as_double();
        if (!std::isfinite(result) || result <= 0.0) {
          throw std::invalid_argument("Invalid connector parameter: " + key);
        }
        return result;
      };
    radius_ = read("minimum_turning_radius", 0.7);
    step_ = read("start_connector_sample_step", 0.02);
    max_length_ = read("start_connector_max_length", 2.0);
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".connect_adjusted_start", rclcpp::ParameterValue(false));
    connect_adjusted_start_ = node->get_parameter(name + ".connect_adjusted_start").as_bool();
    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::GlobalPlanner>>(
      "nav2_core", "nav2_core::GlobalPlanner");
    planner_ = loader_->createSharedInstance("smac_planner/SmacPlanner");
    planner_->configure(node, name, tf, costmap);
    angle_bins_ = node->get_parameter(name + ".angle_quantization_bins").as_int();
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".explicit_start_service",
      rclcpp::ParameterValue("/compute_path_with_start"));
    service_ = node->create_service<limo_interfaces::srv::ComputePathWithStart>(
      node->get_parameter(name + ".explicit_start_service").as_string(),
      [this](const std::shared_ptr<limo_interfaces::srv::ComputePathWithStart::Request> request,
      std::shared_ptr<limo_interfaces::srv::ComputePathWithStart::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        response->connector_max_length = max_length_;
        try {
          if (!active_) {throw std::runtime_error("Planner is inactive");}
          std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(
            *costmap_->getCostmap()->getMutex());
          const auto frame = costmap_->getGlobalFrameID();
          auto * map = costmap_->getCostmap();
          const auto validate = [&](const geometry_msgs::msg::PoseStamped & pose, bool check_cell,
          bool require_on_map = true) {
            if (pose.header.frame_id != frame) {
              throw std::runtime_error("Explicit-start poses must use the global costmap frame");
            }
            const auto & q = pose.pose.orientation;
            if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
            !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
            !std::isfinite(q.w) || q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w < 1e-12)
            {
              throw std::runtime_error("Invalid explicit-start pose coordinates or quaternion");
            }
            if (!require_on_map) {return;}
            unsigned int x, y;
            if (!map->worldToMap(pose.pose.position.x, pose.pose.position.y, x, y)) {
              throw std::runtime_error("Explicit-start pose is outside the global costmap");
            }
            if (check_cell && map->getCost(x, y) >=
            nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            {
              throw std::runtime_error("Virtual start or goal is occupied");
            }
          };
          validate(request->real_start, false, connect_adjusted_start_);
          validate(request->start, true);
          auto & virtual_start = response->planning_start;
          virtual_start = request->start;
          unsigned int x, y;
          map->worldToMap(virtual_start.pose.position.x, virtual_start.pose.position.y, x, y);
          map->mapToWorld(x, y, virtual_start.pose.position.x, virtual_start.pose.position.y);
          const double bin = 2.0 * std::acos(-1.0) / angle_bins_;
          double heading = tf2::getYaw(virtual_start.pose.orientation);
          if (heading < 0.0) {heading += 2.0 * std::acos(-1.0);}
          tf2::Quaternion orientation;
          orientation.setRPY(0.0, 0.0, std::floor(heading / bin) * bin);
          virtual_start.pose.orientation = tf2::toMsg(orientation);
          const double translation = std::hypot(
            request->real_start.pose.position.x - request->start.pose.position.x,
            request->real_start.pose.position.y - request->start.pose.position.y);
          const double yaw = tf2::getYaw(request->real_start.pose.orientation) -
          tf2::getYaw(request->start.pose.orientation);
          const bool adjusted = translation > 1e-6 ||
          std::abs(std::atan2(std::sin(yaw), std::cos(yaw))) > 1e-6;
          response->connector_required = adjusted && connect_adjusted_start_;
          response->connector_checked = !adjusted || response->connector_required;
          nav_msgs::msg::Path connector;
          if (response->connector_required) {
            // Check and sample the prefix BEFORE spending time in SMAC. As
            // before, only this real-to-virtual prefix ignores collisions.
            try {
              connector = dubinsConnector(
                request->real_start, virtual_start,
                radius_, step_, max_length_, &response->connector_length);
            } catch (const DubinsLengthError &) {
              response->start_rejected = true;
              throw;
            }
          }
          if (adjusted) {
            nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map);
            const double footprint_cost = checker.footprintCostAtPose(
              virtual_start.pose.position.x, virtual_start.pose.position.y,
              tf2::getYaw(virtual_start.pose.orientation), costmap_->getRobotFootprint());
            if (footprint_cost < 0.0 || footprint_cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
              throw std::runtime_error("Quantized virtual start footprint is occupied");
            }
          }
          validate(request->goal, true);
          auto path = planner_->createPlan(request->start, request->goal);
          if (path.poses.empty()) {throw std::runtime_error("SMAC found no path");}
          if (adjusted) {
            // Foxy backtrace omits the initial node. Restore its grid/yaw pose
            // at the join, then keep the original collision-checked SMAC suffix.
            path.poses.insert(path.poses.begin(), virtual_start);
          }
          if (response->connector_required) {
            response->connector_end_index = connector.poses.size() - 1;
            connector.poses.insert(connector.poses.end(), path.poses.begin() + 1, path.poses.end());
            path = std::move(connector);
          }
          const auto now = clock_->now();
          last_plan_stamp_ = std::max(last_plan_stamp_ + 1, now.nanoseconds());
          path.header.stamp = rclcpp::Time(last_plan_stamp_, now.get_clock_type());
          for (auto & pose : path.poses) {pose.header = path.header;}
          response->path = std::move(path);
          response->success = true;
        } catch (const std::exception & error) {
          response->error = error.what();
        }
      });
  }
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return planner_->createPlan(start, goal);
  }
  void activate() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    planner_->activate();
    active_ = true;
  }
  void deactivate() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    planner_->deactivate();
  }
  void cleanup() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    service_.reset();
    // Foxy SMAC cleanup dereferences its optional downsampler even when disabled.
    // Destruction releases its resources without that unsafe cleanup call.
    planner_.reset();
    loader_.reset();
    costmap_.reset();
    clock_.reset();
    active_ = false;
  }

private:
  std::mutex mutex_;
  bool active_{false};
  bool connect_adjusted_start_{false};
  double radius_{0.7}, step_{0.02}, max_length_{2.0};
  int angle_bins_{72};
  rclcpp::Clock::SharedPtr clock_;
  std::int64_t last_plan_stamp_{0};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GlobalPlanner>> loader_;
  nav2_core::GlobalPlanner::Ptr planner_;
  rclcpp::Service<limo_interfaces::srv::ComputePathWithStart>::SharedPtr service_;
};
}
PLUGINLIB_EXPORT_CLASS(limo_smac_planner::ExplicitStartPlanner, nav2_core::GlobalPlanner)
