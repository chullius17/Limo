#include "limo_dwb_critics/obstacle_guidance.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

namespace limo_dwb_critics
{
namespace
{
constexpr double infinity = std::numeric_limits<double>::infinity();
}

void ObstacleGuidance::update(
  unsigned int width, unsigned int height, double resolution,
  const unsigned char * costs, double clearance, double cost_weight)
{
  if (!width || !height || !costs || !std::isfinite(resolution) || resolution <= 0.0 ||
    !std::isfinite(clearance) || clearance < 0.0 ||
    !std::isfinite(cost_weight) || cost_weight < 0.0)
  {
    throw std::invalid_argument("Invalid obstacle guidance grid");
  }
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  const std::size_t count = static_cast<std::size_t>(width) * height;
  blocked_.assign(count, false);
  penalties_.resize(count);
  distances_.assign(count, infinity);
  // Include a half-cell diagonal so clearance applies to obstacle cell area.
  const double radius = clearance / resolution + std::sqrt(0.5);
  if (clearance_radius_ != radius) {
    clearance_offsets_.clear();
    const int cells = static_cast<int>(std::ceil(radius));
    for (int dy = -cells; dy <= cells; ++dy) {
      for (int dx = -cells; dx <= cells; ++dx) {
        if (std::hypot(dx, dy) <= radius) {clearance_offsets_.push_back({dx, dy});}
      }
    }
    clearance_radius_ = radius;
  }
  if (penalty_weight_ != cost_weight) {
    for (std::size_t cost = 0; cost < penalty_lookup_.size(); ++cost) {
      penalty_lookup_[cost] = 1.0 + cost_weight * std::min<int>(cost, 252) / 252.0;
    }
    penalty_weight_ = cost_weight;
  }
  std::size_t neighbor = 0;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (!dx && !dy) {continue;}
      const double length = resolution * std::hypot(dx, dy);
      neighbors_[neighbor++] = {dx, dy, length, length * 0.5, std::atan2(dy, dx)};
    }
  }
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      const auto index = static_cast<std::size_t>(y) * width + x;
      blocked_[index] = blocked_[index] || costs[index] >= 253 ||
        (x + 0.5) * resolution < clearance || (y + 0.5) * resolution < clearance ||
        (width - x - 0.5) * resolution < clearance ||
        (height - y - 0.5) * resolution < clearance;
      penalties_[index] = penalty_lookup_[costs[index]];
      if (costs[index] < 254) {
        continue;
      }
      for (const auto & offset : clearance_offsets_) {
        const int nx = static_cast<int>(x) + offset.x;
        const int ny = static_cast<int>(y) + offset.y;
        if (nx >= 0 && ny >= 0 && nx < static_cast<int>(width) &&
          ny < static_cast<int>(height))
        {
          blocked_[static_cast<std::size_t>(ny) * width + nx] = true;
        }
      }
    }
  }
}

bool ObstacleGuidance::traversable(int x, int y) const
{
  return x >= 0 && y >= 0 && x < static_cast<int>(width_) &&
         y < static_cast<int>(height_) &&
         !blocked_[static_cast<std::size_t>(y) * width_ + x];
}

bool ObstacleGuidance::setTarget(int x, int y)
{
  std::fill(distances_.begin(), distances_.end(), infinity);
  if (!traversable(x, y)) {
    return false;
  }
  queue_.clear();
  const auto order = std::greater<QueueEntry>();
  const auto target = static_cast<std::size_t>(y) * width_ + x;
  distances_[target] = 0.0;
  queue_.emplace_back(0.0, target);
  while (!queue_.empty()) {
    std::pop_heap(queue_.begin(), queue_.end(), order);
    const auto entry = queue_.back();
    queue_.pop_back();
    if (entry.first > distances_[entry.second]) {
      continue;
    }
    const int cx = entry.second % width_;
    const int cy = entry.second / width_;
    for (const auto & step : neighbors_) {
      const int nx = cx + step.x;
      const int ny = cy + step.y;
      if (!traversable(nx, ny) ||
        (step.x && step.y &&
        (!traversable(cx + step.x, cy) || !traversable(cx, cy + step.y))))
      {
        continue;
      }
      const auto next = static_cast<std::size_t>(ny) * width_ + nx;
      const double candidate = entry.first + step.half_length *
        (penalties_[entry.second] + penalties_[next]);
      if (candidate < distances_[next]) {
        distances_[next] = candidate;
        queue_.emplace_back(candidate, next);
        std::push_heap(queue_.begin(), queue_.end(), order);
      }
    }
  }
  return true;
}

double ObstacleGuidance::distance(double x, double y) const
{
  if (!std::isfinite(x) || !std::isfinite(y) || x < -0.5 || y < -0.5 ||
    x >= width_ - 0.5 || y >= height_ - 0.5)
  {
    return infinity;
  }
  const int nearest_x = static_cast<int>(std::floor(x + 0.5));
  const int nearest_y = static_cast<int>(std::floor(y + 0.5));
  if (!traversable(nearest_x, nearest_y) ||
    !std::isfinite(distances_[static_cast<std::size_t>(nearest_y) * width_ + nearest_x]))
  {
    return infinity;
  }
  // Smooth the finite grid values so sub-cell MPC progress receives a reward.
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  double sum = 0.0;
  double weights = 0.0;
  for (int dy = 0; dy <= 1; ++dy) {
    for (int dx = 0; dx <= 1; ++dx) {
      const int nx = x0 + dx;
      const int ny = y0 + dy;
      if (!traversable(nx, ny)) {
        continue;
      }
      const double value = distances_[static_cast<std::size_t>(ny) * width_ + nx];
      const double weight = (dx ? x - x0 : 1.0 - (x - x0)) *
        (dy ? y - y0 : 1.0 - (y - y0));
      if (std::isfinite(value) && weight > 0.0) {
        sum += weight * value;
        weights += weight;
      }
    }
  }
  return weights > 0.0 ? sum / weights : infinity;
}

double ObstacleGuidance::heading(double x, double y, double current_heading) const
{
  if (!std::isfinite(x) || !std::isfinite(y) || x < -0.5 || y < -0.5 ||
    x >= width_ - 0.5 || y >= height_ - 0.5)
  {
    return current_heading;
  }
  int cx = static_cast<int>(std::floor(x + 0.5));
  int cy = static_cast<int>(std::floor(y + 0.5));
  if (!traversable(cx, cy)) {
    return current_heading;
  }
  const int start_x = cx;
  const int start_y = cy;
  double length = 0.0;
  double direction = current_heading;
  // Follow a short section of the optimal grid route rather than using one
  // neighbor's 45-degree direction. This suppresses cell-to-cell heading jumps.
  while (length < 0.35) {
    const auto index = static_cast<std::size_t>(cy) * width_ + cx;
    const double current = distances_[index];
    double best_cost = infinity;
    double best_angle = infinity;
    int step_x = 0;
    int step_y = 0;
    for (const auto & step : neighbors_) {
      if (!traversable(cx + step.x, cy + step.y) ||
        (step.x && step.y &&
        (!traversable(cx + step.x, cy) || !traversable(cx, cy + step.y))))
      {
        continue;
      }
      const auto next = static_cast<std::size_t>(cy + step.y) * width_ + cx + step.x;
      if (distances_[next] >= current) {
        continue;
      }
      const double cost = distances_[next] + step.half_length *
        (penalties_[index] + penalties_[next]);
      const double angle = std::abs(std::atan2(
          std::sin(step.heading - direction), std::cos(step.heading - direction)));
      if (cost < best_cost - 1e-9 ||
        (std::abs(cost - best_cost) <= 1e-9 && angle < best_angle))
      {
        best_cost = cost;
        best_angle = angle;
        step_x = step.x;
        step_y = step.y;
      }
    }
    if (!step_x && !step_y) {
      break;
    }
    cx += step_x;
    cy += step_y;
    length += resolution_ * std::hypot(step_x, step_y);
    direction = std::atan2(step_y, step_x);
  }
  return length > 0.0 ? std::atan2(cy - start_y, cx - start_x) : current_heading;
}

}  // namespace limo_dwb_critics
