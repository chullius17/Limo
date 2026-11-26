#include "limo_map/cloud.hpp"
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace limo_map;  // NOLINT
int main()
{
  try {
    for (bool big : {false, true}) {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.width = 2; cloud.height = 2; cloud.point_step = 20; cloud.row_step = 48;
      cloud.is_bigendian = big;
      const std::array<std::string, 4> names{{"x", "y", "z", "class_id"}};
      const std::array<uint32_t, 4> offsets{{4, 8, 12, 0}};
      for (size_t i = 0; i < 4; ++i) {
        sensor_msgs::msg::PointField field;
        field.name = names[i]; field.offset = offsets[i];
        field.datatype = i == 3 ? 2 : 7; field.count = 1;
        cloud.fields.push_back(field);
      }
      cloud.data.resize(96);
      for (size_t row = 0; row < 2; ++row) {
        for (size_t col = 0; col < 2; ++col) {
          const auto base = row * 48 + col * 20;
          cloud.data[base] = row == 1 && col == 1 ? 250 : 3;
          for (size_t i = 0; i < 3; ++i) {
            float value = i == 0 ? 1 : (i == 1 ? 2 : 0);
            if (row == 0 && col == 0 && i == 2) {value = std::nanf("");}
            uint32_t bits;
            std::memcpy(&bits, &value, 4);
            for (size_t b = 0; b < 4; ++b) {
              cloud.data[base + offsets[i] + b] = bits >> (8 * (big ? 3 - b : b));
            }
          }
        }
      }
      Grid grid;
      const auto points = read_cloud(cloud, grid, {1, 0, 0}, {});
      if (points.size() != 2 || points[0].x != 2 || points[0].y != 2 || points[0].class_id != 3) {
        throw std::runtime_error("Incorrect endian/padding/NaN/unknown-class handling");
      }
      cloud.data.pop_back();
      bool failed = false;
      try {read_cloud(cloud, grid, {}, {});} catch (const std::invalid_argument &) {failed = true;}
      if (!failed) {throw std::runtime_error("Truncated cloud accepted");}
      cloud.data.push_back(0);
      cloud.fields[3].datatype = 7;
      failed = false;
      try {read_cloud(cloud, grid, {}, {});} catch (const std::invalid_argument &) {failed = true;}
      if (!failed) {throw std::runtime_error("Invalid class field accepted");}
    }
    std::cout << "Cloud endian, padding, invalid samples and schema checks passed\n";
  } catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
}
