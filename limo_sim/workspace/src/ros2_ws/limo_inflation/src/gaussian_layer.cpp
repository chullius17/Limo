#include "limo_inflation/gaussian_layer.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace limo_inflation
{

void GaussianLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("sigma_x", rclcpp::ParameterValue(0.02));
  declareParameter("sigma_y", rclcpp::ParameterValue(0.02));
  declareParameter("covariance_xy", rclcpp::ParameterValue(0.0));
  declareParameter("obstacle_cost_threshold", rclcpp::ParameterValue(1));
  declareParameter("broad_enabled", rclcpp::ParameterValue(false));
  declareParameter("broad_sigma", rclcpp::ParameterValue(0.15));
  declareParameter("broad_peak_cost", rclcpp::ParameterValue(100));
  node_->get_parameter(name_ + ".enabled", enabled_);
  node_->get_parameter(name_ + ".sigma_x", sigma_x_);
  node_->get_parameter(name_ + ".sigma_y", sigma_y_);
  node_->get_parameter(name_ + ".covariance_xy", covariance_xy_);
  node_->get_parameter(name_ + ".obstacle_cost_threshold", obstacle_cost_threshold_);
  node_->get_parameter(name_ + ".broad_enabled", broad_enabled_);
  node_->get_parameter(name_ + ".broad_sigma", broad_sigma_);
  node_->get_parameter(name_ + ".broad_peak_cost", broad_peak_cost_);
  const double product = sigma_x_ * sigma_y_;
  if (!std::isfinite(sigma_x_) || !std::isfinite(sigma_y_) ||
    sigma_x_ <= 0.0 || sigma_y_ <= 0.0 ||
    !std::isfinite(sigma_x_ * sigma_x_) || !std::isfinite(sigma_y_ * sigma_y_) ||
    !std::isfinite(product) || product <= 0.0 ||
    !std::isfinite(covariance_xy_) || std::abs(covariance_xy_) >= product ||
    obstacle_cost_threshold_ < 1 || obstacle_cost_threshold_ > 254)
  {
    throw std::invalid_argument("Gaussian covariance must be finite and positive definite");
  }
  correlation_ = covariance_xy_ / product;
  if (!std::isfinite(broad_sigma_) || broad_sigma_ <= 0.0 ||
    !std::isfinite(broad_sigma_ * broad_sigma_) ||
    broad_peak_cost_ < 1 || broad_peak_cost_ > 252)
  {
    throw std::invalid_argument("Invalid broad Gaussian sigma or peak cost (1..252)");
  }
  current_ = true;
  RCLCPP_INFO(
    node_->get_logger(),
    "Gaussian inflation: sigma=%.3f/%.3f m, cov_xy=%.6f m^2, merge=MAX, threshold=%d",
    sigma_x_, sigma_y_, covariance_xy_, obstacle_cost_threshold_);
  if (broad_enabled_) {
    RCLCPP_INFO(
      node_->get_logger(), "Broad Gaussian: sigma=%.3f m, common peak=%d, merge=MAX",
      broad_sigma_, broad_peak_cost_);
  }
  // Read-only, on-demand snapshot: no extra publication or per-cycle map copy.
  sources_service_ = node_->create_service<nav2_msgs::srv::GetCostmap>(
    "~/" + name_ + "/get_sources",
    [this](const std::shared_ptr<nav2_msgs::srv::GetCostmap::Request>,
    std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> response) {
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
        *layered_costmap_->getCostmap()->getMutex());
      auto & snapshot = response->map;
      snapshot.header.stamp = source_stamp_;
      snapshot.header.frame_id = layered_costmap_->getGlobalFrameID();
      auto & meta = snapshot.metadata;
      meta.update_time = source_stamp_;
      meta.layer = name_ + "/sources_before_inflation";
      meta.size_x = source_costs_.empty() ? 0 : width_;
      meta.size_y = source_costs_.empty() ? 0 : height_;
      meta.resolution = resolution_;
      meta.origin.position.x = source_origin_x_;
      meta.origin.position.y = source_origin_y_;
      meta.origin.orientation.w = 1.0;
      snapshot.data = source_costs_;
    });
}

void GaussianLayer::updateBounds(
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

void GaussianLayer::rebuildKernel()
{
  offsets_.clear();
  broad_costs_.clear();
  if (broad_enabled_) {
    // Squared distances between cell centers are integers. Cache the entire
    // nonzero Gaussian tail once, so rebuilding a field needs no exponentials.
    const double scale = broad_sigma_ / resolution_;
    const double farthest = std::pow(width_ - 1.0, 2) + std::pow(height_ - 1.0, 2);
    const auto limit = static_cast<std::size_t>(std::ceil(std::min(
          farthest, 2.0 * std::log(2.0 * broad_peak_cost_) * scale * scale)));
    broad_costs_.resize(limit + 1);
    for (std::size_t squared = 0; squared <= limit; ++squared) {
      broad_costs_[squared] = static_cast<std::uint8_t>(std::lround(
          broad_peak_cost_ * std::exp(-0.5 * squared / scale / scale)));
    }
  }
  for (auto & values : kernels_) {
    values.clear();
  }
  // Beyond four standard deviations every 8-bit contribution rounds to zero.
  // Projected covariance bounds remain valid for correlated Gaussians too.
  const int rx = static_cast<int>(std::min(
      static_cast<double>(width_ - 1), std::ceil(4.0 * sigma_x_ / resolution_)));
  const int ry = static_cast<int>(std::min(
      static_cast<double>(height_ - 1), std::ceil(4.0 * sigma_y_ / resolution_)));
  const double denominator = (1.0 - correlation_) * (1.0 + correlation_);
  for (int y = -ry; y <= ry; ++y) {
    for (int x = -rx; x <= rx; ++x) {
      const double nx = x * resolution_ / sigma_x_;
      const double ny = y * resolution_ / sigma_y_;
      const double residual = nx - correlation_ * ny;
      const double q = residual * residual / denominator + ny * ny;
      const double weight = std::exp(-0.5 * q);
      if (254.0 * weight >= 0.5) {
        offsets_.push_back({x, y, weight});
      }
    }
  }
}

const std::vector<GaussianLayer::CostOffset> & GaussianLayer::kernel(const std::uint8_t peak)
{
  auto & values = kernels_[peak];
  if (values.empty()) {
    for (const auto & offset : offsets_) {
      const auto cost = std::min(252L, std::lround(peak * offset.weight));
      if (cost > 0) {
        values.push_back({offset.x, offset.y, static_cast<std::uint8_t>(cost)});
      }
    }
  }
  return values;
}

void GaussianLayer::rebuildField(const nav2_costmap_2d::Costmap2D & source)
{
  const bool geometry_changed = width_ != source.getSizeInCellsX() ||
    height_ != source.getSizeInCellsY() || resolution_ != source.getResolution();
  width_ = source.getSizeInCellsX();
  height_ = source.getSizeInCellsY();
  resolution_ = source.getResolution();
  const auto size = static_cast<std::size_t>(width_) * height_;
  source_costs_.clear();
  inflated_costs_.clear();
  if (!size) {
    return;
  }
  source_costs_.assign(source.getCharMap(), source.getCharMap() + size);
  inflated_costs_ = source_costs_;
  if (geometry_changed || offsets_.empty()) {
    rebuildKernel();
  }
  for (unsigned int y = 0; y < height_; ++y) {
    for (unsigned int x = 0; x < width_; ++x) {
      const auto peak = source_costs_[y * width_ + x];
      if (peak == nav2_costmap_2d::NO_INFORMATION || peak < obstacle_cost_threshold_) {
        continue;
      }
      for (const auto & offset : kernel(peak)) {
        const int tx = static_cast<int>(x) + offset.x;
        const int ty = static_cast<int>(y) + offset.y;
        if (tx < 0 || ty < 0 || tx >= static_cast<int>(width_) || ty >= static_cast<int>(height_)) {
          continue;
        }
        const auto index = static_cast<std::size_t>(ty) * width_ + tx;
        if (source_costs_[index] != nav2_costmap_2d::NO_INFORMATION) {
          inflated_costs_[index] = std::max(inflated_costs_[index], offset.cost);
        }
      }
    }
  }
  if (broad_enabled_) {
    applyBroadField();
  }
}

void GaussianLayer::applyBroadField()
{
  cv::Mat mask(height_, width_, CV_8UC1, cv::Scalar(255));
  bool has_sources = false;
  for (unsigned int y = 0; y < height_; ++y) {
    auto * row = mask.ptr<unsigned char>(y);
    for (unsigned int x = 0; x < width_; ++x) {
      const auto cost = source_costs_[y * width_ + x];
      if (cost != nav2_costmap_2d::NO_INFORMATION && cost >= obstacle_cost_threshold_) {
        row[x] = 0;
        has_sources = true;
      }
    }
  }
  if (!has_sources) {
    return;
  }
  // All broad sources share one peak and isotropic sigma. Their MAX is exactly
  // the Gaussian of distance to the nearest ORIGINAL source, in O(width*height).
  // Never use inflated_costs_ as seeds: that would inflate the narrow halo again.
  cv::Mat distances;
  cv::distanceTransform(mask, distances, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  for (unsigned int y = 0; y < height_; ++y) {
    const auto * row = distances.ptr<float>(y);
    for (unsigned int x = 0; x < width_; ++x) {
      const auto index = static_cast<std::size_t>(y) * width_ + x;
      if (source_costs_[index] == nav2_costmap_2d::NO_INFORMATION) {
        continue;
      }
      const auto squared = static_cast<std::size_t>(std::lround(row[x] * row[x]));
      if (squared < broad_costs_.size()) {
        inflated_costs_[index] = std::max(inflated_costs_[index], broad_costs_[squared]);
      }
    }
  }
}

void GaussianLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  source_stamp_ = node_->now();
  source_origin_x_ = master_grid.getOriginX();
  source_origin_y_ = master_grid.getOriginY();
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
      master_grid.setCost(x, y, inflated_costs_[y * width_ + x]);
    }
  }
  current_ = true;
}

void GaussianLayer::reset()
{
  source_costs_.clear();
  inflated_costs_.clear();
  offsets_.clear();
  broad_costs_.clear();
  for (auto & values : kernels_) {
    values.clear();
  }
  current_ = false;
}

}  // namespace limo_inflation

PLUGINLIB_EXPORT_CLASS(limo_inflation::GaussianLayer, nav2_costmap_2d::Layer)
