#include "limo_dwb_critics/obstacle_guidance.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
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
  const int cells = static_cast<int>(std::ceil(radius));
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      const auto index = static_cast<std::size_t>(y) * width + x;
      blocked_[index] = blocked_[index] || costs[index] >= 253 ||
        (x + 0.5) * resolution < clearance || (y + 0.5) * resolution < clearance ||
        (width - x - 0.5) * resolution < clearance ||
        (height - y - 0.5) * resolution < clearance;
      penalties_[index] = 1.0 + cost_weight * std::min<int>(costs[index], 252) / 252.0;
      if (costs[index] < 254) {
        continue;
      }
      for (int dy = -cells; dy <= cells; ++dy) {
        for (int dx = -cells; dx <= cells; ++dx) {
          const int nx = static_cast<int>(x) + dx;
          const int ny = static_cast<int>(y) + dy;
          if (nx >= 0 && ny >= 0 && nx < static_cast<int>(width) &&
            ny < static_cast<int>(height) && std::hypot(dx, dy) <= radius)
          {
            blocked_[static_cast<std::size_t>(ny) * width + nx] = true;
          }
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
  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  const auto target = static_cast<std::size_t>(y) * width_ + x;
  distances_[target] = 0.0;
  queue.emplace(0.0, target);
  while (!queue.empty()) {
    const auto entry = queue.top();
    queue.pop();
    if (entry.first > distances_[entry.second]) {
      continue;
    }
    const int cx = entry.second % width_;
    const int cy = entry.second / width_;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const int nx = cx + dx;
        const int ny = cy + dy;
        if ((!dx && !dy) || !traversable(nx, ny) ||
          (dx && dy && (!traversable(cx + dx, cy) || !traversable(cx, cy + dy))))
        {
          continue;
        }
        const auto next = static_cast<std::size_t>(ny) * width_ + nx;
        const double candidate = entry.first + resolution_ * std::hypot(dx, dy) *
          0.5 * (penalties_[entry.second] + penalties_[next]);
        if (candidate < distances_[next]) {
          distances_[next] = candidate;
          queue.emplace(candidate, next);
        }
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
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if ((!dx && !dy) || !traversable(cx + dx, cy + dy) ||
          (dx && dy && (!traversable(cx + dx, cy) || !traversable(cx, cy + dy))))
        {
          continue;
        }
        const auto next = static_cast<std::size_t>(cy + dy) * width_ + cx + dx;
        if (distances_[next] >= current) {
          continue;
        }
        const double cost = distances_[next] + resolution_ * std::hypot(dx, dy) *
          0.5 * (penalties_[index] + penalties_[next]);
        const double heading = std::atan2(dy, dx);
        const double angle = std::abs(std::atan2(
            std::sin(heading - direction), std::cos(heading - direction)));
        if (cost < best_cost - 1e-9 ||
          (std::abs(cost - best_cost) <= 1e-9 && angle < best_angle))
        {
          best_cost = cost;
          best_angle = angle;
          step_x = dx;
          step_y = dy;
        }
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
