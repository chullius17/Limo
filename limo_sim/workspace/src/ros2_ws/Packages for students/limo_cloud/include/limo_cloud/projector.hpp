#ifndef LIMO_CLOUD__PROJECTOR_HPP_
#define LIMO_CLOUD__PROJECTOR_HPP_

#include <array>
#include <cstdint>
#include <vector>

#include "geometry_msgs/msg/transform.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace limo_cloud
{

struct ProjectionOptions
{
  double input_crop_y_min{0.0};
  double roi_y_min{0.0};
  double roi_y_max{1.0};
  float min_depth{0.1F};
  float max_depth{2.5F};
  // -1 preserves every label, including zero. Zero is invalid in the CV pipeline.
  int invalid_class_id{0};
  bool flatten_to_bev{true};
};

struct Point
{
  float x;
  float y;
  float z;
  uint8_t class_id;
  uint8_t padding[3];
};
static_assert(sizeof(Point) == 16, "Semantic points must occupy 16 bytes");

// Optional ordered projection requests. Repeated pixels and explicit class IDs
// are intentional: image processors may sample several masks at the same pixel.
struct PixelSample
{
  uint32_t row;
  uint32_t col;
  uint8_t class_id;
  uint8_t padding[3];
};
static_assert(sizeof(PixelSample) == 12, "Pixel samples must occupy 12 bytes");

void validate_image(const sensor_msgs::msg::Image & image, uint32_t bytes_per_pixel);
void validate_cloud(const sensor_msgs::msg::PointCloud2 & cloud);

// Stateful per-worker cache. No intermediate per-pixel coordinate/mask arrays.
class Projector
{
public:
  void project(
    const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::Image & labels,
    const sensor_msgs::msg::CameraInfo & info,
    const geometry_msgs::msg::Transform & transform, const ProjectionOptions & options,
    sensor_msgs::msg::PointCloud2 & output,
    const std::vector<PixelSample> * samples = nullptr);

private:
  void update_rays(
    const sensor_msgs::msg::CameraInfo & info, uint32_t width, uint32_t height,
    double crop);
  std::array<double, 9> calibration_{};
  uint32_t info_width_{0}, info_height_{0}, width_{0}, height_{0};
  double crop_{-1.0};
  std::vector<float> ray_x_, ray_y_;
  std::vector<std::array<float, 3>> columns_, rows_;
};

}  // namespace limo_cloud

#endif  // LIMO_CLOUD__PROJECTOR_HPP_
