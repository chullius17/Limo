#include "limo_map/grid.hpp"
#include "limo_map/map_io.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <unistd.h>

using namespace limo_map;  // NOLINT
namespace
{
void check(bool condition, const char * text)
{
  if (!condition) {throw std::runtime_error(text);}
}
template<typename Callable> void rejects(Callable callable)
{
  bool failed = false;
  try {callable();} catch (const std::exception &) {failed = true;}
  check(failed, "Expected invalid input to be rejected");
}
Grid make_grid(Options options = Options())
{
  options.resolution = 1;
  Grid grid(options);
  grid.set_pose({0, 0}, {});
  return grid;
}
std::vector<int8_t> view(const Grid & grid, uint32_t width = 1, uint32_t height = 1)
{
  return grid.render({1, width, height, {}}).semantic;
}
void same(const std::vector<int8_t> & actual, std::initializer_list<int> expected)
{
  check(actual.size() == expected.size(), "Unexpected map size");
  size_t i = 0;
  for (int cost : expected) {
    if (actual[i++] != cost) {
      std::cerr << "Expected " << cost << ", got " << static_cast<int>(actual[i - 1]) << '\n';
      throw std::runtime_error("Unexpected map cost");
    }
  }
}
void class_costs_and_generic_ids()
{
  auto grid = make_grid();
  grid.update({{0.1, 0.1, 1}, {1.1, 0.1, 2}, {2.1, 0.1, 3}, {3.1, 0.1, 4},
      {4.1, 0.1, 5}, {5.1, 0.1, 6}, {6.1, 0.1, 255}});
  same(view(grid, 7), {0, 60, 30, 90, 0, 90, -1});
  Options o;
  o.class_ids = {0, 42, 255}; o.class_costs = {12, 0, 100}; o.evidence_groups.clear();
  grid = make_grid(o);
  grid.update({{0.1, 0.1, 0}, {1.1, 0.1, 42}, {2.1, 0.1, 255}});
  same(view(grid, 3), {12, 0, 100});
}
void evidence_and_aliases()
{
  auto one = make_grid(), dense = make_grid();
  one.update({{0.1, 0.1, 4}});
  dense.update(std::vector<Point>(10000, {0.1, 0.1, 4}));
  one.update({{0.1, 0.1, 1}});
  dense.update({{0.1, 0.1, 5}});
  check(view(one) == view(dense), "Point density must not multiply evidence");
  auto grid = make_grid();
  grid.update({{0.1, 0.1, 1}, {0.1, 0.1, 5}});
  same(view(grid), {0});
  auto tie = make_grid();
  for (int i = 0; i < 20; ++i) {tie.update({{0.1, 0.1, 2}, {0.1, 0.1, 4}});}
  same(view(tie), {-1});
  Options o;
  o.class_ids = {7, 8}; o.class_costs = {60, 60}; o.evidence_groups.clear();
  auto separate = make_grid(o);
  separate.update({{0.1, 0.1, 7}, {0.1, 0.1, 8}});
  same(view(separate), {-1});
}
void corrections_and_confirmation()
{
  Options o;
  o.limit = 1; o.free_confirmations = 4;
  auto grid = make_grid(o);
  for (int i = 0; i < 30; ++i) {grid.update({{0.1, 0.1, 4}, {1.1, 0.1, 4}});}
  for (int i = 0; i < 4; ++i) {
    grid.update({{0.1, 0.1, 1}});
    same(view(grid, 2), {90, 90});
  }
  grid.update({{0.1, 0.1, 5}});
  same(view(grid, 2), {0, 90});
  for (int i = 0; i < 30; ++i) {grid.update({{0.1, 0.1, 4}});}
  grid.update({{0.1, 0.1, 3}});
  same(view(grid, 2), {90, 90});
  grid.update({{0.1, 0.1, 3}});
  same(view(grid, 2), {30, 90});
}
void geometry_submaps_and_limits()
{
  auto grid = make_grid();
  grid.update({{-0.1, -0.1, 2}, {0.1, 0.1, 3}});
  same(grid.render({1, 2, 2, {-1, -1, 0}}).semantic, {60, -1, -1, 30});
  grid = make_grid();
  grid.set_pose({0, 1}, {2, 0, 0});
  grid.update({{0.1, 0.1, 2}});
  grid.update({{0.1, 0.1, 4}}, {0, 1});
  same(view(grid, 5), {60, -1, 90, -1, -1});
  grid.set_pose({0, 0}, {2, 0, std::acos(-1) / 2});
  grid.set_pose({0, 1}, {4, 1, 0});
  same(view(grid, 5, 2), {-1, 60, -1, -1, -1, -1, -1, -1, -1, 90});
  grid.set_pose({0, 1}, {2, 0, std::acos(-1) / 2});
  same(view(grid, 3), {-1, 90, -1});  // Newer observation wins the overlap.
  grid.set_visible({{0, 0}});
  same(view(grid, 3), {-1, 60, -1});
  Options o; o.max_cells = 1024;
  auto bounded = make_grid(o);
  bounded.update({{0.1, 0.1, 2}});
  rejects([&] {bounded.update({{0.1, 0.1, 4}, {40.1, 0.1, 4}});});
  same(view(bounded), {60});
  check(bounded.allocated_cells() == 1024, "Memory rejection must be atomic");
  rejects([&] {bounded.render({1, 0, 1, {}});});
  rejects([&] {bounded.update({{1e300, 0, 2}});});
}
void fusion_filter_and_save()
{
  same(combine({60, 90, -1, 0}, {100, 0, -1, -1}), {100, 90, -1, 0});
  same(combine({60}, {0}), {60});  // New laser message clears old laser obstacle.
  same(obstacle_map({0, 39, 40, 95, 100, -1, -1}, {-1, -1, -1, -1, -1, 0, 100}),
    {0, 0, 100, 100, -1, 0, -1});
  same(obstacle_map({100}, {0}, 39, 100), {100});
  Geometry geometry{1, 3, 3, {}};
  const std::vector<int8_t> speckle{-1, -1, -1, -1, 90, -1, -1, -1, -1};
  same(median_filter(speckle, geometry, 3), {-1, -1, -1, -1, -1, -1, -1, -1, -1});
  check(speckle[4] == 90, "Save filter must not modify evidence");
  geometry = {0.5, 2, 2, {1, 2, 0.3}};
  const auto directory = std::filesystem::temp_directory_path() /
    ("limo_map_core_" + std::to_string(getpid()));
  std::filesystem::remove_all(directory);
  save_maps(directory, "test", geometry, {0, 60, 30, -1}, {100, 0, -1, -1});
  std::ifstream pgm(directory / "test_complete.pgm", std::ios::binary);
  const std::string bytes((std::istreambuf_iterator<char>(pgm)), {});
  check(bytes.size() > 4, "Missing saved image");
  const auto offset = bytes.size() - 4;
  check(static_cast<uint8_t>(bytes[offset]) == 30 &&
    static_cast<uint8_t>(bytes[offset + 1]) == 255 &&
    static_cast<uint8_t>(bytes[offset + 2]) == 100 &&
    static_cast<uint8_t>(bytes[offset + 3]) == 60, "Raw image must preserve costs, unknown and row flip");
  check(std::distance(std::filesystem::directory_iterator(directory), {}) == 6, "Expected six saved files");
  rejects([&] {save_maps(directory, "../escape", geometry, {0}, {0});});
  std::filesystem::remove_all(directory);
}
void invalid_configuration()
{
  Options o;
  o.class_costs.pop_back(); rejects([&] {Grid grid(o);});
  o = Options(); o.class_ids[0] = 256; rejects([&] {Grid grid(o);});
  o = Options(); o.class_ids[0] = 2; rejects([&] {Grid grid(o);});
  o = Options(); o.class_costs[0] = -1; rejects([&] {Grid grid(o);});
  o = Options(); o.class_costs[0] = 1; rejects([&] {Grid grid(o);});
  o = Options(); o.free_confirmations = 256; rejects([&] {Grid grid(o);});
}
}  // namespace
int main()
{
  try {
    class_costs_and_generic_ids(); evidence_and_aliases(); corrections_and_confirmation();
    geometry_submaps_and_limits(); fusion_filter_and_save(); invalid_configuration();
    std::cout << "Generic costs, evidence, correction, geometry, fusion and serialization passed\n";
  } catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
}
