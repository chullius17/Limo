#include "limo_map/map_io.hpp"

#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <unistd.h>

namespace limo_map
{
namespace
{
void pgm(
  const std::filesystem::path & path, const Geometry & geometry,
  const std::vector<int8_t> & data, bool trinary)
{
  std::ofstream stream;
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream.open(path, std::ios::binary);
  stream << "P5\n# CREATOR: limo_map\n" << geometry.width << ' ' << geometry.height << "\n255\n";
  for (uint32_t row = geometry.height; row > 0; --row) {
    for (uint32_t x = 0; x < geometry.width; ++x) {
      const int value = data[(row - 1) * static_cast<size_t>(geometry.width) + x];
      const unsigned char pixel = trinary ? (value < 0 ? 205 : (value == 0 ? 254 : 0)) :
        (value < 0 ? 255 : value);
      stream.put(static_cast<char>(pixel));
    }
  }
  stream.close();
}
void yaml(
  const std::filesystem::path & path, const std::string & image,
  const Geometry & geometry, bool trinary)
{
  std::ofstream stream;
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream.open(path);
  stream << std::setprecision(17) << "image: '" << image << "'\nmode: " <<
    (trinary ? "trinary" : "raw") << "\nresolution: " << geometry.resolution <<
    "\norigin: [" << geometry.origin.x << ", " << geometry.origin.y << ", " <<
    geometry.origin.yaw << "]\nnegate: 0\noccupied_thresh: " <<
    (trinary ? "0.65" : "1.0") << "\nfree_thresh: " << (trinary ? "0.196" : "0.0") << '\n';
  stream.close();
}
std::filesystem::path temporary(const std::filesystem::path & directory)
{
  auto pattern = (directory / ".limo_map_XXXXXX").string();
  const int fd = mkstemp(&pattern[0]);
  if (fd == -1) {throw std::system_error(errno, std::generic_category(), "mkstemp");}
  close(fd);
  return pattern;
}
}  // namespace

void save_maps(
  const std::filesystem::path & directory, const std::string & name,
  const Geometry & geometry, const std::vector<int8_t> & semantic,
  const std::vector<int8_t> & laser, unsigned median_kernel,
  int free_max, int occupied_max)
{
  // Restricted stems keep both file paths and quoted YAML image names unambiguous.
  if (name.empty() || name == "." || name == ".." ||
    name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-.") !=
    std::string::npos || std::filesystem::path(name).extension() == ".pgm" ||
    std::filesystem::path(name).extension() == ".yaml")
  {
    throw std::invalid_argument("save_map_name must be a filename stem (letters, numbers, '.', '_' or '-')");
  }
  geometry.validate(std::numeric_limits<size_t>::max());
  if (semantic.size() != geometry.size() || laser.size() != geometry.size()) {
    throw std::invalid_argument("Saved map size differs from geometry");
  }
  for (const auto * values : {&semantic, &laser}) {
    for (auto value : *values) {
      if (value < -1 || value > 100) {throw std::invalid_argument("Invalid map cost");}
    }
  }
  const auto filtered = median_filter(semantic, geometry, median_kernel);
  const auto complete = combine(filtered, laser);
  const auto obstacles = obstacle_map(filtered, laser, free_max, occupied_max);
  const std::array<std::string, 3> stems{{name + "_complete", name + "_laser", name + "_cv_obstacle"}};
  const std::array<const std::vector<int8_t> *, 3> maps{{&complete, &laser, &obstacles}};
  std::filesystem::create_directories(directory);
  std::vector<std::pair<std::filesystem::path, std::filesystem::path>> files;
  try {
    for (size_t i = 0; i < maps.size(); ++i) {
      for (bool metadata : {false, true}) {
        const auto destination = directory / (stems[i] + (metadata ? ".yaml" : ".pgm"));
        files.emplace_back(temporary(directory), destination);
        if (metadata) {
          yaml(files.back().first, stems[i] + ".pgm", geometry, i == 2);
        } else {pgm(files.back().first, geometry, *maps[i], i == 2);}
      }
    }
    for (const auto & file : files) {std::filesystem::rename(file.first, file.second);}
  } catch (...) {
    for (const auto & file : files) {
      std::error_code ignored;
      std::filesystem::remove(file.first, ignored);
    }
    throw;
  }
}
}  // namespace limo_map
