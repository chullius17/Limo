#ifndef LIMO_MAP__GRID_HPP_
#define LIMO_MAP__GRID_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <tuple>
#include <vector>

namespace limo_map
{
struct Pose {double x = 0, y = 0, yaw = 0;};
struct Point {double x, y; uint8_t class_id;};
using Submap = std::pair<int, int>;

struct Geometry
{
  double resolution = 0.05;
  uint32_t width = 0, height = 0;
  Pose origin;
  size_t size() const;
  void validate(size_t max_cells) const;
};

struct Options
{
  double resolution = 0.05;
  std::vector<int64_t> class_ids{1, 2, 3, 4, 5, 6};
  std::vector<int64_t> class_costs{0, 60, 30, 90, 0, 90};
  // Empty: each input ID has independent evidence, even at the same cost.
  // Equal group IDs explicitly share evidence and must have equal costs.
  std::vector<int64_t> evidence_groups{1, 2, 3, 4, 1, 4};
  double hit = 0.85, miss = 0.4, limit = 3.0, threshold = 0.5;
  unsigned free_confirmations = 1;
  size_t max_cells = 2000000, max_output_cells = 4000000;
};

struct Raster
{
  std::vector<int8_t> semantic;
  // Winning evidence-group index per output cell, -1 for unknown.
  std::vector<int16_t> groups;
};

// ROS-independent sparse evidence. One normalized update per cell and cloud;
// only observed endpoints supply hits/misses. Costs never scale with confidence.
class Grid
{
public:
  explicit Grid(Options options = Options());
  bool set_pose(Submap key, Pose pose);
  void set_visible(const std::set<Submap> & keys);
  bool update(const std::vector<Point> & local_points, Submap key = {0, 0});
  Raster render(const Geometry & geometry) const;
  Pose pose(Submap key) const;
  int group(uint8_t id) const {return lookup_[id];}
  size_t allocated_cells() const {return tiles_.size() * tile_area;}
  const Options & options() const {return options_;}

private:
  static constexpr int64_t tile_size = 32;
  static constexpr size_t tile_area = 1024;
  struct Tile
  {
    std::vector<float> scores;
    std::array<uint64_t, tile_area> seen{};
    std::array<uint8_t, tile_area> free_streak{};
  };
  using TileKey = std::tuple<Submap, int64_t, int64_t>;
  Options options_;
  std::array<int16_t, 256> lookup_;
  std::vector<int8_t> costs_;
  std::map<Submap, Pose> poses_;
  std::map<TileKey, Tile> tiles_;
  std::set<Submap> visible_;
  bool filter_visible_ = false;
  uint64_t sequence_ = 0;
};

Point transform(Point point, Pose pose, bool inverse = false);
std::vector<int8_t> combine(
  const std::vector<int8_t> & semantic, const std::vector<int8_t> & laser);
std::vector<int8_t> median_filter(
  const std::vector<int8_t> & values, const Geometry & geometry, unsigned kernel);
std::vector<int8_t> obstacle_map(
  const std::vector<int8_t> & semantic, const std::vector<int8_t> & laser,
  int free_max = 39, int occupied_max = 95);
}  // namespace limo_map
#endif  // LIMO_MAP__GRID_HPP_
