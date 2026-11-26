#ifndef LIMO_MAP__MAP_IO_HPP_
#define LIMO_MAP__MAP_IO_HPP_
#include <filesystem>
#include <string>
#include "limo_map/grid.hpp"

namespace limo_map
{
// Stage all six files before replacing destinations. Each individual rename is atomic.
void save_maps(
  const std::filesystem::path & directory, const std::string & name,
  const Geometry & geometry, const std::vector<int8_t> & semantic,
  const std::vector<int8_t> & laser, unsigned median_kernel = 1,
  int free_max = 39, int occupied_max = 95);
}  // namespace limo_map
#endif  // LIMO_MAP__MAP_IO_HPP_
