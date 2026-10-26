#include "limo_inflation/path_distance_layer.hpp"
#include "limo_inflation/footprint_cells.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace limo_inflation
{
namespace
{
// Clip in floating point before converting to pixel integers. Sparse path
// segments crossing the window are included even if both endpoints are outside.
bool drawSegment(cv::Mat & mask, cv::Point2d start, cv::Point2d end)
{
  const double dx = end.x - start.x, dy = end.y - start.y;
  if (!std::isfinite(dx) || !std::isfinite(dy)) {return false;}
  const double p[] = {-dx, dx, -dy, dy};
  const double q[] = {start.x, mask.cols - 1.0 - start.x,
    start.y, mask.rows - 1.0 - start.y};
  double lo = 0.0, hi = 1.0;
  for (int i = 0; i < 4; ++i) {
    if (p[i] == 0.0) {
      if (q[i] < 0.0) {return false;}
    } else if (p[i] < 0.0) {
      lo = std::max(lo, q[i] / p[i]);
    } else {
      hi = std::min(hi, q[i] / p[i]);
    }
    if (lo > hi) {return false;}
  }
  const auto pixel = [&](double t) {
      return cv::Point(
        static_cast<int>(std::clamp(start.x + t * dx, 0.0, mask.cols - 1.0)),
        static_cast<int>(std::clamp(start.y + t * dy, 0.0, mask.rows - 1.0)));
    };
  cv::line(mask, pixel(lo), pixel(hi), cv::Scalar(0), 1, cv::LINE_8);
  return true;
}
}  // namespace

void PathDistanceLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("path_topic", rclcpp::ParameterValue(std::string("/limo/planning/path")));
  declareParameter("max_cost", rclcpp::ParameterValue(50));
  declareParameter("distance_for_max_cost", rclcpp::ParameterValue(0.50));
  declareParameter("tube_radius", rclcpp::ParameterValue(0.60));
  std::string topic;
  node_->get_parameter(name_ + ".enabled", enabled_);
  node_->get_parameter(name_ + ".path_topic", topic);
  node_->get_parameter(name_ + ".max_cost", max_cost_);
  node_->get_parameter(name_ + ".distance_for_max_cost", distance_for_max_cost_);
  node_->get_parameter(name_ + ".tube_radius", tube_radius_);
  if (topic.empty() || max_cost_ < 0 || max_cost_ > 252 ||
    !std::isfinite(distance_for_max_cost_) || distance_for_max_cost_ <= 0.0 ||
    !std::isfinite(tube_radius_) || tube_radius_ <= 0.0)
  {
    throw std::invalid_argument("Invalid path distance topic, maximum cost or distance");
  }
  subscription_ = node_->create_subscription<nav_msgs::msg::Path>(
    topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](nav_msgs::msg::Path::ConstSharedPtr path) {
      setPath(path);
    });
  current_ = true;
  start_service_ = node_->create_service<std_srvs::srv::Trigger>(
    "~/" + name_ + "/check_start",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
        *layered_costmap_->getCostmap()->getMutex());
      response->success = false;
      response->message = "START blocked: path tube is not ready";
      try {
        std::string base = "base_link";
        node_->get_parameter("robot_base_frame", base);
        if (!tf_) {return;}
        const auto tf = tf_->lookupTransform(layered_costmap_->getGlobalFrameID(),
          base, tf2::TimePointZero);
        geometry_msgs::msg::Pose2D pose;
        pose.x = tf.transform.translation.x; pose.y = tf.transform.translation.y;
        pose.theta = tf2::getYaw(tf.transform.rotation);
        if (!containsFootprint(pose)) {
          response->message = "START blocked: entire footprint must be inside the path tube";
          return;
        }
        std::vector<nav2_costmap_2d::MapLocation> cells;
        auto & map = *layered_costmap_->getCostmap();
        if (!footprintCells(map, pose, getFootprint(), cells)) {return;}
        for (const auto & cell : cells) {
          if (map.getCost(cell.x, cell.y) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
            response->message = "START blocked: footprint intersects lethal or unknown cells";
            return;
          }
        }
        response->success = true;
        response->message = "Entire footprint is inside the tube and collision-free";
      } catch (const tf2::TransformException &) {
        response->message = "START blocked: robot transform unavailable";
      }
    });
  RCLCPP_INFO(node_->get_logger(), "Path distance cost: 0..%d, linear up to %.3f m, topic=%s",
    max_cost_, distance_for_max_cost_, topic.c_str());
}

void PathDistanceLayer::setPath(nav_msgs::msg::Path::ConstSharedPtr path)
{
  bool valid = path && !path->header.frame_id.empty() && !path->poses.empty();
  if (path) {
    for (const auto & pose : path->poses) {
      valid = valid && std::isfinite(pose.pose.position.x) &&
        std::isfinite(pose.pose.position.y) &&
        (pose.header.frame_id.empty() || pose.header.frame_id == path->header.frame_id);
    }
  }
  std::lock_guard<std::mutex> lock(path_mutex_);
  path_ = valid ? path : nullptr;
}

void PathDistanceLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {return;}
  const auto * map = layered_costmap_->getCostmap();
  *min_x = std::min(*min_x, map->getOriginX());
  *min_y = std::min(*min_y, map->getOriginY());
  *max_x = std::max(*max_x, map->getOriginX() + map->getSizeInMetersX());
  *max_y = std::max(*max_y, map->getOriginY() + map->getSizeInMetersY());
}

void PathDistanceLayer::rebuildField(
  const nav2_costmap_2d::Costmap2D & map, const nav_msgs::msg::Path & path,
  const std::array<double, 3> & transform)
{
  const auto width = map.getSizeInCellsX(), height = map.getSizeInCellsY();
  // Padding retains nearby path segments outside the local window. Otherwise
  // a path just outside its edge would incorrectly cost the maximum everywhere.
  const double requested_margin = std::ceil(
    std::max(distance_for_max_cost_, tube_radius_) / map.getResolution()) + 1;
  if (requested_margin > 4096 ||
    (width + 2.0 * requested_margin) * (height + 2.0 * requested_margin) > 4194304)
  {
    throw std::invalid_argument("Path distance raster exceeds 4M cells; reduce distance_for_max_cost");
  }
  const int margin = static_cast<int>(requested_margin);
  cv::Mat mask(height + 2 * margin, width + 2 * margin, CV_8UC1, cv::Scalar(255));
  const double cosine = std::cos(transform[2]), sine = std::sin(transform[2]);
  const auto pixel = [&](const geometry_msgs::msg::PoseStamped & pose) {
      const auto & p = pose.pose.position;
      return cv::Point2d(
        (transform[0] + cosine * p.x - sine * p.y - map.getOriginX()) /
        map.getResolution() + margin,
        (transform[1] + sine * p.x + cosine * p.y - map.getOriginY()) /
        map.getResolution() + margin);
    };
  bool intersects = false;
  path_points_.clear();
  double arc = 0.0;
  for (const auto & pose : path.poses) {
    const auto & p = pose.pose.position;
    const double x = transform[0] + cosine * p.x - sine * p.y;
    const double y = transform[1] + sine * p.x + cosine * p.y;
    if (!path_points_.empty()) {
      arc += std::hypot(x - path_points_.back()[0], y - path_points_.back()[1]);
    }
    path_points_.push_back({x, y, arc});
  }
  auto previous = pixel(path.poses.front());
  if (path.poses.size() == 1) {intersects = drawSegment(mask, previous, previous);}
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto next = pixel(path.poses[i]);
    intersects = drawSegment(mask, previous, next) || intersects;
    previous = next;
  }
  const bool same_mask = width_ == width && height_ == height &&
    resolution_ == map.getResolution() && cached_mask_.size() == mask.total() &&
    std::equal(cached_mask_.begin(), cached_mask_.end(), mask.data);
  width_ = width;
  height_ = height;
  resolution_ = map.getResolution();
  origin_x_ = map.getOriginX();
  origin_y_ = map.getOriginY();
  cached_transform_ = transform;
  if (same_mask && !path_costs_.empty()) {return;}
  cached_mask_.assign(mask.data, mask.data + mask.total());
  path_costs_.assign(static_cast<std::size_t>(width) * height, max_cost_);
  path_distances_.assign(path_costs_.size(), std::numeric_limits<float>::infinity());
  if (!intersects) {return;}
  cv::Mat distances;
  cv::distanceTransform(mask, distances, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  const double scale = max_cost_ * resolution_ / distance_for_max_cost_;
  for (unsigned int y = 0; y < height; ++y) {
    const auto * row = distances.ptr<float>(y + margin) + margin;
    for (unsigned int x = 0; x < width; ++x) {
      path_distances_[y * width + x] = row[x] * resolution_;
      path_costs_[y * width + x] = static_cast<std::uint8_t>(
        std::min(static_cast<long>(max_cost_), std::lround(row[x] * scale)));
    }
  }
}

void PathDistanceLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & map, int min_i, int min_j, int max_i, int max_j)
{
  tube_field_valid_ = false;
  if (!enabled_) {return;}
  nav_msgs::msg::Path::ConstSharedPtr path;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    path = path_;
  }
  if (!path || !map.getSizeInCellsX() || !map.getSizeInCellsY()) {return;}
  std::array<double, 3> transform{};
  if (path->header.frame_id != layered_costmap_->getGlobalFrameID()) {
    try {
      if (!tf_) {return;}
      const auto tf = tf_->lookupTransform(
        layered_costmap_->getGlobalFrameID(), path->header.frame_id, tf2::TimePointZero);
      transform = {tf.transform.translation.x, tf.transform.translation.y,
        tf2::getYaw(tf.transform.rotation)};
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "Path distance waiting for TF: %s", error.what());
      return;  // Keep obstacle costs, without applying an incorrectly aligned field.
    }
  }
  if (cached_path_ != path || width_ != map.getSizeInCellsX() ||
    height_ != map.getSizeInCellsY() || resolution_ != map.getResolution() ||
    origin_x_ != map.getOriginX() || origin_y_ != map.getOriginY() ||
    cached_transform_ != transform || path_costs_.empty())
  {
    rebuildField(map, *path, transform);
    cached_path_ = path;
  }
  tube_field_valid_ = true;
  for (int y = std::max(0, min_j); y < std::min(static_cast<int>(height_), max_j); ++y) {
    for (int x = std::max(0, min_i); x < std::min(static_cast<int>(width_), max_i); ++x) {
      const auto cost = map.getCost(x, y);
      if (cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        map.setCost(x, y, static_cast<unsigned char>(std::min(252,
            cost + static_cast<int>(path_costs_[y * width_ + x]))));
      }
    }
  }
  current_ = true;
}

void PathDistanceLayer::reset()
{
  tube_field_valid_ = false;
  cached_path_.reset();
  cached_mask_.clear();
  path_costs_.clear();
  path_distances_.clear();
  path_points_.clear();
  current_ = true;
}

bool PathDistanceLayer::matchesPath(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  if (!tube_field_valid_ || !path_ || path_ != cached_path_ || path_->header.frame_id != path.header.frame_id ||
    path_->poses.size() != path.poses.size()) {return false;}
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    if (path_->poses[i].pose.position.x != path.poses[i].pose.position.x ||
      path_->poses[i].pose.position.y != path.poses[i].pose.position.y) {return false;}
  }
  return true;
}

bool PathDistanceLayer::containsFootprint(const geometry_msgs::msg::Pose2D & pose)
{
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!tube_field_valid_ || !path_ || path_ != cached_path_) {return false;}
  }
  auto & map = *layered_costmap_->getCostmap();
  if (path_distances_.size() != static_cast<std::size_t>(map.getSizeInCellsX()) *
    map.getSizeInCellsY() || width_ != map.getSizeInCellsX() || height_ != map.getSizeInCellsY() ||
    resolution_ != map.getResolution() || origin_x_ != map.getOriginX() ||
    origin_y_ != map.getOriginY()) {return false;}
  std::vector<nav2_costmap_2d::MapLocation> cells;
  if (!footprintCells(map, pose, getFootprint(), cells)) {return false;}
  // Reserve half the cell diagonal so every occupied cell fits in the tube,
  // rather than testing only the robot center or the polygon's four corners.
  const double allowed = tube_radius_ - resolution_ * std::sqrt(0.5);
  for (const auto & cell : cells) {
    if (path_distances_[cell.y * width_ + cell.x] > allowed) {return false;}
  }
  return true;
}

bool PathDistanceLayer::projectProgress(const geometry_msgs::msg::Pose2D & pose,
  double & progress, double & heading, double minimum, double maximum) const
{
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < path_points_.size(); ++i) {
    const auto & a = path_points_[i - 1]; const auto & b = path_points_[i];
    const double length = b[2] - a[2];
    if (length <= 1e-9 || b[2] < minimum || a[2] > maximum) {continue;}
    const double dx = b[0] - a[0], dy = b[1] - a[1];
    const double low = std::clamp((minimum - a[2]) / length, 0.0, 1.0);
    const double high = std::clamp((maximum - a[2]) / length, 0.0, 1.0);
    const double f = std::clamp(((pose.x - a[0]) * dx + (pose.y - a[1]) * dy) /
        (length * length), low, high);
    const double ex = pose.x - a[0] - f * dx, ey = pose.y - a[1] - f * dy;
    if (ex * ex + ey * ey < best) {
      best = ex * ex + ey * ey; progress = a[2] + f * length;
      heading = std::atan2(dy, dx);
    }
  }
  return std::isfinite(best);
}

}  // namespace limo_inflation

PLUGINLIB_EXPORT_CLASS(limo_inflation::PathDistanceLayer, nav2_costmap_2d::Layer)
