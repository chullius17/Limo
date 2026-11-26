#include "limo_map/grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace limo_map
{
namespace
{
bool finite(Pose p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
}
int64_t cell(double coordinate, double resolution)
{
  const double value = std::floor(coordinate / resolution);
  // Keep headroom for tile arithmetic and centre coordinates.
  constexpr double bound = static_cast<double>(int64_t{1} << 50);
  if (!std::isfinite(value) || value <= -bound || value >= bound) {
    throw std::invalid_argument("Point coordinate exceeds grid range");
  }
  return static_cast<int64_t>(value);
}
int64_t tile_coord(int64_t value)
{
  return value >= 0 ? value / 32 : (value + 1) / 32 - 1;
}
size_t winner(const float * scores, size_t count, bool & unique)
{
  size_t best = 0;
  unique = true;
  for (size_t i = 1; i < count; ++i) {
    if (scores[i] > scores[best]) {best = i; unique = true;}
    else if (scores[i] == scores[best]) {unique = false;}
  }
  return best;
}
}  // namespace

size_t Geometry::size() const {return static_cast<size_t>(width) * height;}
void Geometry::validate(size_t max_cells) const
{
  if (!std::isfinite(resolution) || resolution <= 0 || !finite(origin) ||
    !width || !height || size() > max_cells)
  {
    throw std::invalid_argument("Invalid or oversized reference map geometry");
  }
}

Point transform(Point point, Pose pose, bool inverse)
{
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  if (inverse) {
    const double x = point.x - pose.x, y = point.y - pose.y;
    return {c * x + s * y, -s * x + c * y, point.class_id};
  }
  return {c * point.x - s * point.y + pose.x,
    s * point.x + c * point.y + pose.y, point.class_id};
}

Grid::Grid(Options options) : options_(std::move(options))
{
  const auto & o = options_;
  for (double value : {o.resolution, o.hit, o.miss, o.limit, o.threshold}) {
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument("Resolution and evidence parameters must be finite and positive");
    }
  }
  if (o.threshold > o.limit || o.max_cells < tile_area || !o.max_output_cells ||
    !o.free_confirmations || o.free_confirmations > 255)
  {
    throw std::invalid_argument("Invalid evidence threshold, memory limits or free_confirmations");
  }
  if (o.class_ids.empty() || o.class_ids.size() != o.class_costs.size() ||
    (!o.evidence_groups.empty() && o.evidence_groups.size() != o.class_ids.size()))
  {
    throw std::invalid_argument("class_ids, class_costs and optional evidence_groups must match");
  }
  lookup_.fill(-1);
  std::map<int64_t, size_t> groups;
  for (size_t i = 0; i < o.class_ids.size(); ++i) {
    const auto id = o.class_ids[i], cost = o.class_costs[i];
    if (id < 0 || id > 255 || lookup_[id] >= 0 || cost < 0 || cost > 100) {
      throw std::invalid_argument("Class IDs must be unique uint8 values; costs must be in [0, 100]");
    }
    const auto group_id = o.evidence_groups.empty() ? id : o.evidence_groups[i];
    const auto inserted = groups.emplace(group_id, costs_.size());
    const auto index = inserted.first->second;
    if (inserted.second) {costs_.push_back(static_cast<int8_t>(cost));}
    else if (costs_[index] != cost) {
      throw std::invalid_argument("Classes sharing an evidence group must have the same cost");
    }
    lookup_[id] = static_cast<int16_t>(index);
  }
}

bool Grid::set_pose(Submap key, Pose pose)
{
  if (!finite(pose)) {throw std::invalid_argument("Nonfinite submap pose");}
  const auto old = poses_.find(key);
  const bool changed = old == poses_.end() || old->second.x != pose.x ||
    old->second.y != pose.y || old->second.yaw != pose.yaw;
  poses_[key] = pose;
  return changed;
}
Pose Grid::pose(Submap key) const {return poses_.at(key);}
void Grid::set_visible(const std::set<Submap> & keys)
{
  filter_visible_ = true;
  visible_ = keys;
}

bool Grid::update(const std::vector<Point> & points, Submap key)
{
  if (!poses_.count(key)) {throw std::invalid_argument("Submap pose unavailable");}
  std::map<std::pair<int64_t, int64_t>, std::vector<size_t>> counts;
  std::set<TileKey> additions;
  for (const auto & p : points) {
    const int group_id = lookup_[p.class_id];
    if (group_id < 0 || !std::isfinite(p.x) || !std::isfinite(p.y)) {continue;}
    const auto x = cell(p.x, options_.resolution), y = cell(p.y, options_.resolution);
    auto & count = counts[{x, y}];
    if (count.empty()) {count.resize(costs_.size());}
    ++count[group_id];
    const TileKey tile{key, tile_coord(x), tile_coord(y)};
    if (!tiles_.count(tile)) {additions.insert(tile);}
  }
  if (counts.empty()) {return false;}
  if (additions.size() > options_.max_cells / tile_area - tiles_.size()) {
    throw std::length_error("Semantic tile limit reached; save/reset or increase max_cells");
  }
  if (sequence_ == std::numeric_limits<uint64_t>::max()) {
    throw std::overflow_error("Observation sequence exhausted; save and reset");
  }
  // Allocate every new tile before changing existing evidence.
  std::map<TileKey, Tile> new_tiles;
  for (const auto & key_to_add : additions) {
    Tile tile;
    tile.scores.resize(tile_area * costs_.size(), 0);
    new_tiles.emplace(key_to_add, std::move(tile));
  }
  tiles_.merge(new_tiles);
  ++sequence_;
  for (const auto & entry : counts) {
    const auto x = entry.first.first, y = entry.first.second;
    const auto tx = tile_coord(x), ty = tile_coord(y);
    auto & tile = tiles_.at({key, tx, ty});
    const size_t flat = (y - ty * tile_size) * tile_size + (x - tx * tile_size);
    float * scores = tile.scores.data() + flat * costs_.size();
    double total = 0, free_count = 0;
    for (size_t i = 0; i < costs_.size(); ++i) {
      total += entry.second[i];
      if (!costs_[i]) {free_count += entry.second[i];}
    }
    bool unique;
    const size_t best = winner(scores, costs_.size(), unique);
    const bool clearing = unique && scores[best] >= options_.threshold &&
      costs_[best] > 0 && free_count / total > 0.5;
    tile.free_streak[flat] = clearing ? static_cast<uint8_t>(
      std::min<unsigned>(tile.free_streak[flat] + 1, options_.free_confirmations)) : 0;
    if (!clearing || tile.free_streak[flat] >= options_.free_confirmations) {
      for (size_t i = 0; i < costs_.size(); ++i) {
        const double fraction = entry.second[i] / total;
        const double next = scores[i] + options_.hit * fraction - options_.miss * (1 - fraction);
        scores[i] = static_cast<float>(std::clamp(next, -options_.limit, options_.limit));
      }
    }
    tile.seen[flat] = sequence_;
  }
  return true;
}

Raster Grid::render(const Geometry & geometry) const
{
  geometry.validate(options_.max_output_cells);
  Raster output;
  output.semantic.assign(geometry.size(), -1);
  output.groups.assign(geometry.size(), -1);
  std::vector<uint64_t> seen(geometry.size(), 0);
  std::vector<float> confidence(geometry.size(), -std::numeric_limits<float>::infinity());
  for (const auto & entry : tiles_) {
    const auto & key = std::get<0>(entry.first);
    if (filter_visible_ && !visible_.count(key)) {continue;}
    const auto tx = std::get<1>(entry.first), ty = std::get<2>(entry.first);
    const auto & tile = entry.second;
    const auto pose = poses_.at(key);
    const auto translation = transform({pose.x, pose.y, 0}, geometry.origin, true);
    const double c = std::cos(pose.yaw - geometry.origin.yaw);
    const double s = std::sin(pose.yaw - geometry.origin.yaw);
    for (size_t i = 0; i < tile_area; ++i) {
      if (!tile.seen[i]) {continue;}
      Point p{(tx * tile_size + static_cast<int64_t>(i % tile_size) + 0.5) * options_.resolution,
        (ty * tile_size + static_cast<int64_t>(i / tile_size) + 0.5) * options_.resolution, 0};
      const double x = std::floor((c * p.x - s * p.y + translation.x) / geometry.resolution);
      const double y = std::floor((s * p.x + c * p.y + translation.y) / geometry.resolution);
      if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || y < 0 ||
        x >= geometry.width || y >= geometry.height) {continue;}
      const size_t target = static_cast<size_t>(y) * geometry.width + static_cast<size_t>(x);
      bool unique;
      const auto * scores = tile.scores.data() + i * costs_.size();
      const auto best = winner(scores, costs_.size(), unique);
      if (tile.seen[i] < seen[target] ||
        (tile.seen[i] == seen[target] && scores[best] < confidence[target])) {continue;}
      seen[target] = tile.seen[i];
      confidence[target] = scores[best];
      const bool certain = unique && scores[best] >= options_.threshold;
      output.semantic[target] = certain ? costs_[best] : -1;
      output.groups[target] = certain ? static_cast<int16_t>(best) : -1;
    }
  }
  return output;
}

std::vector<int8_t> combine(
  const std::vector<int8_t> & semantic, const std::vector<int8_t> & laser)
{
  if (semantic.size() != laser.size()) {throw std::invalid_argument("Map size mismatch");}
  auto result = laser;
  for (size_t i = 0; i < result.size(); ++i) {
    if (semantic[i] >= 0 && laser[i] != 100) {result[i] = semantic[i];}
  }
  return result;
}

std::vector<int8_t> median_filter(
  const std::vector<int8_t> & values, const Geometry & geometry, unsigned kernel)
{
  if (!kernel || kernel % 2 == 0 || values.size() != geometry.size()) {
    throw std::invalid_argument("Median kernel must be positive and odd; geometry must match");
  }
  if (kernel == 1) {return values;}
  auto output = values;
  const int64_t radius = kernel / 2;
  std::vector<int8_t> neighbors(static_cast<size_t>(kernel) * kernel);
  for (int64_t y = 0; y < geometry.height; ++y) {
    for (int64_t x = 0; x < geometry.width; ++x) {
      const size_t target = y * geometry.width + x;
      if (values[target] < 0) {continue;}
      size_t n = 0;
      for (int64_t dy = -radius; dy <= radius; ++dy) {
        for (int64_t dx = -radius; dx <= radius; ++dx) {
          const auto sy = std::clamp<int64_t>(y + dy, 0, geometry.height - 1);
          const auto sx = std::clamp<int64_t>(x + dx, 0, geometry.width - 1);
          neighbors[n++] = values[sy * geometry.width + sx];
        }
      }
      auto middle = neighbors.begin() + neighbors.size() / 2;
      std::nth_element(neighbors.begin(), middle, neighbors.end());
      output[target] = *middle;
    }
  }
  return output;
}

std::vector<int8_t> obstacle_map(
  const std::vector<int8_t> & semantic, const std::vector<int8_t> & laser,
  int free_max, int occupied_max)
{
  if (semantic.size() != laser.size() || free_max < 0 || occupied_max <= free_max ||
    occupied_max > 100) {throw std::invalid_argument("Invalid obstacle thresholds or map sizes");}
  std::vector<int8_t> result(semantic.size(), -1);
  for (size_t i = 0; i < result.size(); ++i) {
    if ((semantic[i] >= 0 && semantic[i] <= free_max) ||
      (semantic[i] == -1 && laser[i] == 0)) {result[i] = 0;}
    else if (semantic[i] > free_max && semantic[i] <= occupied_max) {result[i] = 100;}
  }
  return result;
}
}  // namespace limo_map
