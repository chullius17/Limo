#include "limo_inflation/local_lane_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace limo_inflation
{

void LocalLaneLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("obstacle_cost_threshold", rclcpp::ParameterValue(1));
  declareParameter("high_cost_expansion_radius", rclcpp::ParameterValue(0.0));
  declareParameter("high_cost_threshold", rclcpp::ParameterValue(200));
  declareParameter("high_cost_expansion_cost", rclcpp::ParameterValue(252));
  declareParameter("robot_width", rclcpp::ParameterValue(0.22));
  declareParameter("safety_margin", rclcpp::ParameterValue(0.05));
  declareParameter("distance_tolerance", rclcpp::ParameterValue(0.02));
  declareParameter("near_max_cost", rclcpp::ParameterValue(120));
  declareParameter("near_cost_sigma", rclcpp::ParameterValue(0.0));
  declareParameter("near_band_start", rclcpp::ParameterValue(0.0));
  declareParameter("near_band_width", rclcpp::ParameterValue(0.0));
  declareParameter("near_band_ramp", rclcpp::ParameterValue(0.025));
  declareParameter("near_band_cost", rclcpp::ParameterValue(100));
  declareParameter("inter_lane_peak_cost", rclcpp::ParameterValue(100));
  declareParameter("inter_lane_plateau_ratio", rclcpp::ParameterValue(0.0));
  declareParameter("far_cost_slope", rclcpp::ParameterValue(10.0));
  declareParameter("far_max_cost", rclcpp::ParameterValue(8));

  double robot_width;
  double safety_margin;
  int near_cost;
  int near_band_cost;
  int inter_lane_cost;
  int far_cost;
  node_->get_parameter(name_ + ".enabled", enabled_);
  node_->get_parameter(name_ + ".obstacle_cost_threshold", obstacle_cost_threshold_);
  node_->get_parameter(name_ + ".high_cost_expansion_radius", high_cost_expansion_radius_);
  node_->get_parameter(name_ + ".high_cost_threshold", high_cost_threshold_);
  node_->get_parameter(name_ + ".high_cost_expansion_cost", high_cost_expansion_cost_);
  node_->get_parameter(name_ + ".robot_width", robot_width);
  node_->get_parameter(name_ + ".safety_margin", safety_margin);
  node_->get_parameter(name_ + ".distance_tolerance", profile_.tolerance);
  node_->get_parameter(name_ + ".near_max_cost", near_cost);
  node_->get_parameter(name_ + ".near_cost_sigma", profile_.near_sigma);
  node_->get_parameter(name_ + ".near_band_start", profile_.near_band_start);
  node_->get_parameter(name_ + ".near_band_width", profile_.near_band_width);
  node_->get_parameter(name_ + ".near_band_ramp", profile_.near_band_ramp);
  node_->get_parameter(name_ + ".near_band_cost", near_band_cost);
  node_->get_parameter(name_ + ".inter_lane_peak_cost", inter_lane_cost);
  node_->get_parameter(name_ + ".inter_lane_plateau_ratio", profile_.inter_lane_plateau_ratio);
  node_->get_parameter(name_ + ".far_cost_slope", profile_.far_cost_slope);
  node_->get_parameter(name_ + ".far_max_cost", far_cost);
  profile_.target_distance = 0.5 * robot_width + safety_margin;
  declareParameter("second_lane_distance", rclcpp::ParameterValue(1.5 * robot_width));
  node_->get_parameter(name_ + ".second_lane_distance", profile_.second_lane_distance);

  if (obstacle_cost_threshold_ < 1 || obstacle_cost_threshold_ > 254 ||
    !std::isfinite(high_cost_expansion_radius_) || high_cost_expansion_radius_ < 0.0 ||
    high_cost_threshold_ < 1 || high_cost_threshold_ > 254 ||
    high_cost_expansion_cost_ < 0 || high_cost_expansion_cost_ > 252 ||
    !std::isfinite(profile_.near_sigma) || profile_.near_sigma < 0.0 ||
    !std::isfinite(profile_.near_band_start) || profile_.near_band_start < 0.0 ||
    !std::isfinite(profile_.near_band_width) || profile_.near_band_width < 0.0 ||
    !std::isfinite(profile_.near_band_ramp) || profile_.near_band_ramp < 0.0 ||
    near_band_cost < 0 || near_band_cost > 252 ||
    (profile_.near_band_width > 0.0 &&
    profile_.near_band_start + profile_.near_band_width + profile_.near_band_ramp >
    profile_.target_distance - profile_.tolerance + 1e-9) ||
    !std::isfinite(profile_.inter_lane_plateau_ratio) ||
    profile_.inter_lane_plateau_ratio < 0.0 || profile_.inter_lane_plateau_ratio >= 1.0 ||
    !std::isfinite(robot_width) || robot_width <= 0.0 ||
    !std::isfinite(safety_margin) || safety_margin < 0.0 ||
    !std::isfinite(profile_.target_distance) ||
    !std::isfinite(profile_.second_lane_distance) ||
    !std::isfinite(profile_.tolerance) || profile_.tolerance < 0.0 ||
    profile_.tolerance >= profile_.target_distance ||
    profile_.second_lane_distance - profile_.tolerance <=
    profile_.target_distance + profile_.tolerance ||
    !std::isfinite(profile_.far_cost_slope) || profile_.far_cost_slope < 0.0 ||
    near_cost < 0 || near_cost > 252 || inter_lane_cost < 0 || inter_lane_cost > 252 ||
    far_cost < 0 || far_cost > 252)
  {
    throw std::invalid_argument("Invalid local lane inflation parameters");
  }
  profile_.near_max_cost = static_cast<std::uint8_t>(near_cost);
  profile_.near_band_cost = static_cast<std::uint8_t>(near_band_cost);
  profile_.inter_lane_peak_cost = static_cast<std::uint8_t>(inter_lane_cost);
  profile_.far_max_cost = static_cast<std::uint8_t>(far_cost);
  current_ = true;
  RCLCPP_INFO(
    node_->get_logger(), "Local lane inflation: seeds >= %d, lanes %.3f/%.3f +/- %.3f m",
    obstacle_cost_threshold_, profile_.target_distance,
    profile_.second_lane_distance, profile_.tolerance);
}

void LocalLaneLayer::updateBounds(
  double, double, double,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }
  const auto * map = layered_costmap_->getCostmap();
  *min_x = std::min(*min_x, map->getOriginX());
  *min_y = std::min(*min_y, map->getOriginY());
  *max_x = std::max(*max_x, map->getOriginX() + map->getSizeInMetersX());
  *max_y = std::max(*max_y, map->getOriginY() + map->getSizeInMetersY());
}

void LocalLaneLayer::rebuildField(const nav2_costmap_2d::Costmap2D & source)
{
  width_ = source.getSizeInCellsX();
  height_ = source.getSizeInCellsY();
  resolution_ = source.getResolution();
  const auto size = static_cast<std::size_t>(width_) * height_;
  source_costs_.assign(source.getCharMap(), source.getCharMap() + size);
  lane_costs_.assign(size, 0u);
  if (!size) {
    return;
  }
  cv::Mat high_mask(height_, width_, CV_8UC1, cv::Scalar(255));
  cv::Mat soft_mask(height_, width_, CV_8UC1, cv::Scalar(255));
  bool has_high = false;
  bool has_soft = false;
  for (unsigned int y = 0; y < height_; ++y) {
    auto * high_row = high_mask.ptr<std::uint8_t>(y);
    auto * soft_row = soft_mask.ptr<std::uint8_t>(y);
    for (unsigned int x = 0; x < width_; ++x) {
      const auto cost = source_costs_[y * width_ + x];
      if (cost == nav2_costmap_2d::NO_INFORMATION || cost < obstacle_cost_threshold_) {
        continue;
      }
      if (cost >= high_cost_threshold_) {
        high_row[x] = 0;
        has_high = true;
      } else {
        soft_row[x] = 0;
        has_soft = true;
      }
    }
  }
  cv::Mat high_distances;
  cv::Mat soft_distances;
  if (has_high) {
    cv::distanceTransform(high_mask, high_distances, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  }
  if (has_soft) {
    cv::distanceTransform(soft_mask, soft_distances, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  }
  const double first_lower = profile_.target_distance - profile_.tolerance;
  for (unsigned int y = 0; y < height_; ++y) {
    for (unsigned int x = 0; x < width_; ++x) {
      const auto index = y * width_ + x;
      const auto source_cost = source_costs_[index];
      // Reserved critical, lethal and unknown source values remain untouched.
      if (source_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        continue;
      }
      const bool high_source = source_cost >= obstacle_cost_threshold_ &&
        source_cost >= high_cost_threshold_;
      const bool soft_source = source_cost >= obstacle_cost_threshold_ &&
        source_cost < high_cost_threshold_;
      int high_cost = 0;
      if (has_high) {
        const double distance = high_distances.at<float>(y, x) * resolution_;
        high_cost = computeBorderCost(distance, profile_);
        if (high_cost_expansion_radius_ > 0.0) {
          int expansion_cost = 0;
          if (distance <= high_cost_expansion_radius_ + 1e-9) {
            expansion_cost = high_cost_expansion_cost_;
          } else if (profile_.near_sigma > 0.0) {
            expansion_cost = computeNearCost(
              distance - high_cost_expansion_radius_,
              first_lower - high_cost_expansion_radius_,
              static_cast<std::uint8_t>(high_cost_expansion_cost_), profile_.near_sigma);
          }
          high_cost = std::max(high_cost, expansion_cost);
        }
      }
      // Yellow/soft points have their own monotone repulsion field. They do
      // not replace the distance source used to construct the two lane wells.
      int soft_cost = high_source ? 0 : source_cost;
      if (has_soft && !soft_source) {
        soft_cost = std::max(soft_cost, static_cast<int>(computeNearCost(
              soft_distances.at<float>(y, x) * resolution_, first_lower,
              profile_.near_max_cost, profile_.near_sigma)));
      }
      if (high_source) {
        high_cost = std::max(high_cost, static_cast<int>(source_cost));
      }
      // Add independent fields once, never per update. Saturation cannot create
      // artificial lethal (254) or unknown (255) cells from soft repulsion.
      lane_costs_[index] = static_cast<std::uint8_t>(std::min(252, high_cost + soft_cost));
    }
  }
}

void LocalLaneLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  const auto size = static_cast<std::size_t>(master_grid.getSizeInCellsX()) *
    master_grid.getSizeInCellsY();
  if (width_ != master_grid.getSizeInCellsX() || height_ != master_grid.getSizeInCellsY() ||
    resolution_ != master_grid.getResolution() || source_costs_.size() != size ||
    !std::equal(source_costs_.begin(), source_costs_.end(), master_grid.getCharMap()))
  {
    rebuildField(master_grid);
  }
  for (int y = std::max(0, min_j); y < std::min(static_cast<int>(height_), max_j); ++y) {
    for (int x = std::max(0, min_i); x < std::min(static_cast<int>(width_), max_i); ++x) {
      const auto old_cost = master_grid.getCost(x, y);
      if (old_cost != nav2_costmap_2d::NO_INFORMATION) {
        master_grid.setCost(x, y, std::max(old_cost, lane_costs_[y * width_ + x]));
      }
    }
  }
  current_ = true;
}

void LocalLaneLayer::reset()
{
  source_costs_.clear();
  lane_costs_.clear();
  current_ = false;
}

}  // namespace limo_inflation

PLUGINLIB_EXPORT_CLASS(limo_inflation::LocalLaneLayer, nav2_costmap_2d::Layer)
