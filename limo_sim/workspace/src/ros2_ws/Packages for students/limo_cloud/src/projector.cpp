#include "limo_cloud/projector.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "sensor_msgs/msg/point_field.hpp"

namespace limo_cloud
{
namespace
{
bool host_bigendian()
{
  const uint16_t value = 1;
  return reinterpret_cast<const uint8_t *>(&value)[0] == 0;
}

sensor_msgs::msg::PointField field(const char * name, uint32_t offset, uint8_t type)
{
  sensor_msgs::msg::PointField result;
  result.name = name;
  result.offset = offset;
  result.datatype = type;
  result.count = 1;
  return result;
}

template<typename T, bool Swap>
float read_depth(const uint8_t * source)
{
  uint8_t bytes[sizeof(T)];
  if (Swap) {
    for (size_t i = 0; i < sizeof(T); ++i) {
      bytes[i] = source[sizeof(T) - 1 - i];
    }
  } else {
    std::memcpy(bytes, source, sizeof(T));
  }
  T value;
  std::memcpy(&value, bytes, sizeof(T));
  return sizeof(T) == 2 ? static_cast<float>(value) * 0.001F : static_cast<float>(value);
}

template<typename T, bool Swap, bool Flat>
size_t project_pixels(
  const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::Image & labels,
  const std::vector<std::array<float, 3>> & columns,
  const std::vector<std::array<float, 3>> & rows,
  const geometry_msgs::msg::Vector3 & translation, const ProjectionOptions & options,
  uint8_t * destination)
{
  const uint32_t first = static_cast<uint32_t>(labels.height * options.roi_y_min);
  const uint32_t last = static_cast<uint32_t>(labels.height * options.roi_y_max);
  const float tx = translation.x, ty = translation.y, tz = translation.z;
  size_t count = 0;
  for (uint32_t row = first; row < last; ++row) {
    const auto * classes = labels.data.data() + static_cast<size_t>(row) * labels.step;
    const auto * depths = depth.data.data() + static_cast<size_t>(row) * depth.step;
    const auto & r = rows[row];
    for (uint32_t col = 0; col < labels.width; ++col) {
      const uint8_t label = classes[col];
      if (static_cast<int>(label) == options.invalid_class_id) {
        continue;
      }
      const float d = read_depth<T, Swap>(depths + col * sizeof(T));
      // This ordered comparison rejects NaN, infinities, zero and outliers.
      if (!(d >= options.min_depth && d <= options.max_depth)) {
        continue;
      }
      const auto & c = columns[col];
      const Point point{
        d * (c[0] + r[0]) + tx,
        d * (c[1] + r[1]) + ty,
        Flat ? 0.0F : d * (c[2] + r[2]) + tz,
        label, {0, 0, 0}};
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      std::memcpy(destination + count * sizeof(Point), &point, sizeof(Point));
      ++count;
    }
  }
  return count;
}

template<typename T, bool Swap>
size_t project_dispatch(
  const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::Image & labels,
  const std::vector<std::array<float, 3>> & columns,
  const std::vector<std::array<float, 3>> & rows,
  const geometry_msgs::msg::Vector3 & translation, const ProjectionOptions & options,
  uint8_t * destination)
{
  if (options.flatten_to_bev) {
    return project_pixels<T, Swap, true>(
      depth, labels, columns, rows, translation, options, destination);
  }
  return project_pixels<T, Swap, false>(
    depth, labels, columns, rows, translation, options, destination);
}

template<typename T, bool Swap>
size_t project_samples(
  const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::Image & labels,
  const std::vector<PixelSample> & samples, const std::vector<float> & ray_x,
  const std::vector<float> & ray_y, const float rotation[3][3],
  const geometry_msgs::msg::Vector3 & translation, const ProjectionOptions & options,
  uint8_t * destination)
{
  const uint32_t first = static_cast<uint32_t>(labels.height * options.roi_y_min);
  const uint32_t last = static_cast<uint32_t>(labels.height * options.roi_y_max);
  const float t[3] = {static_cast<float>(translation.x),
    static_cast<float>(translation.y), static_cast<float>(translation.z)};
  size_t count = 0;
  for (const auto & sample : samples) {
    if (sample.row >= labels.height || sample.col >= labels.width) {
      throw std::invalid_argument("Projection sample is outside the input image");
    }
    if (sample.row < first || sample.row >= last ||
      static_cast<int>(sample.class_id) == options.invalid_class_id)
    {
      continue;
    }
    const float d = read_depth<T, Swap>(depth.data.data() +
      static_cast<size_t>(sample.row) * depth.step + sample.col * sizeof(T));
    if (!(d >= options.min_depth && d <= options.max_depth)) {continue;}
    const float x = ray_x[sample.col] * d;
    const float y = ray_y[sample.row] * d;
    // Retain float32 operation order used by the NumPy sampled projection.
    const auto axis = [&](size_t i) {
        return ((x * rotation[i][0] + y * rotation[i][1]) + d * rotation[i][2]) + t[i];
      };
    const Point point{axis(0), axis(1), options.flatten_to_bev ? 0.0F : axis(2),
      sample.class_id, {0, 0, 0}};
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {continue;}
    std::memcpy(destination + count * sizeof(Point), &point, sizeof(Point));
    ++count;
  }
  return count;
}
}  // namespace

void validate_image(const sensor_msgs::msg::Image & image, uint32_t bytes_per_pixel)
{
  if (!image.width || !image.height ||
    static_cast<uint64_t>(image.step) < static_cast<uint64_t>(image.width) * bytes_per_pixel ||
    static_cast<uint64_t>(image.step) * image.height > image.data.size())
  {
    throw std::invalid_argument("Invalid image dimensions, stride or data length");
  }
}

void validate_cloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  using sensor_msgs::msg::PointField;
  if (cloud.height != 1 || cloud.point_step != sizeof(Point) ||
    static_cast<uint64_t>(cloud.width) * cloud.point_step != cloud.row_step ||
    cloud.data.size() != cloud.row_step || cloud.fields.size() != 4 ||
    cloud.is_bigendian != host_bigendian())
  {
    throw std::invalid_argument("Plugin returned an invalid compact semantic cloud");
  }
  const auto expected = std::array<PointField, 4>{
    field("x", 0, PointField::FLOAT32), field("y", 4, PointField::FLOAT32),
    field("z", 8, PointField::FLOAT32), field("class_id", 12, PointField::UINT8)};
  for (size_t i = 0; i < expected.size(); ++i) {
    if (cloud.fields[i] != expected[i]) {
      throw std::invalid_argument("Plugin changed the semantic cloud fields");
    }
  }
}

void Projector::update_rays(
  const sensor_msgs::msg::CameraInfo & info, uint32_t width, uint32_t height, double crop)
{
  if (info.k == calibration_ && info.width == info_width_ && info.height == info_height_ &&
    width == width_ && height == height_ && crop == crop_)
  {
    return;
  }
  if (!info.width || !info.height || !(info.k[0] > 0) || !(info.k[4] > 0) ||
    !std::all_of(info.k.begin(), info.k.end(), [](double v) {return std::isfinite(v);}))
  {
    throw std::invalid_argument("Invalid camera calibration");
  }
  calibration_ = info.k;
  info_width_ = info.width;
  info_height_ = info.height;
  width_ = width;
  height_ = height;
  crop_ = crop;
  ray_x_.resize(width);
  ray_y_.resize(height);
  columns_.resize(width);
  rows_.resize(height);
  const uint32_t start = static_cast<uint32_t>(info.height * crop);
  for (uint32_t col = 0; col < width; ++col) {
    const float u = ((static_cast<float>(col) + 0.5F) * info.width) / width - 0.5F;
    ray_x_[col] = (u - static_cast<float>(info.k[2])) / static_cast<float>(info.k[0]);
  }
  for (uint32_t row = 0; row < height; ++row) {
    const float v = start + ((static_cast<float>(row) + 0.5F) * (info.height - start)) / height - 0.5F;
    ray_y_[row] = (v - static_cast<float>(info.k[5])) / static_cast<float>(info.k[4]);
  }
}

void Projector::project(
  const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::Image & labels,
  const sensor_msgs::msg::CameraInfo & info, const geometry_msgs::msg::Transform & transform,
  const ProjectionOptions & options, sensor_msgs::msg::PointCloud2 & output,
  const std::vector<PixelSample> * samples)
{
  using sensor_msgs::msg::PointField;
  if (!(options.input_crop_y_min >= 0 && options.input_crop_y_min < 1) ||
    !(options.roi_y_min >= 0 && options.roi_y_min < options.roi_y_max && options.roi_y_max <= 1) ||
    !(options.min_depth > 0 && options.min_depth < options.max_depth) ||
    !std::isfinite(options.max_depth) || options.invalid_class_id < -1 ||
    options.invalid_class_id > 255)
  {
    throw std::invalid_argument("Invalid projection options");
  }
  if (labels.encoding != "mono8" && labels.encoding != "8UC1") {
    throw std::invalid_argument("Classified image must use mono8 or 8UC1 class IDs");
  }
  const bool millimetres = depth.encoding == "16UC1" || depth.encoding == "mono16";
  if (!millimetres && depth.encoding != "32FC1") {
    throw std::invalid_argument("Depth must use 32FC1 metres or 16UC1/mono16 millimetres");
  }
  validate_image(labels, 1);
  validate_image(depth, millimetres ? 2 : 4);
  if (depth.width != labels.width || depth.height != labels.height) {
    throw std::invalid_argument("Depth and classified image dimensions differ");
  }
  update_rays(info, labels.width, labels.height, options.input_crop_y_min);
  const auto & q = transform.rotation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!(norm > 1e-12) || !std::isfinite(norm) ||
    !std::isfinite(transform.translation.x) || !std::isfinite(transform.translation.y) ||
    !std::isfinite(transform.translation.z))
  {
    throw std::invalid_argument("Invalid TF rotation or translation");
  }
  const double s = 2.0 / norm;
  const float rotation[3][3] = {
    {static_cast<float>(1 - s * (q.y * q.y + q.z * q.z)),
      static_cast<float>(s * (q.x * q.y - q.z * q.w)),
      static_cast<float>(s * (q.x * q.z + q.y * q.w))},
    {static_cast<float>(s * (q.x * q.y + q.z * q.w)),
      static_cast<float>(1 - s * (q.x * q.x + q.z * q.z)),
      static_cast<float>(s * (q.y * q.z - q.x * q.w))},
    {static_cast<float>(s * (q.x * q.z - q.y * q.w)),
      static_cast<float>(s * (q.y * q.z + q.x * q.w)),
      static_cast<float>(1 - s * (q.x * q.x + q.y * q.y))}};
  // Fold rotation into O(width + height) terms, leaving just add/multiply per pixel.
  for (uint32_t col = 0; col < labels.width; ++col) {
    for (size_t axis = 0; axis < 3; ++axis) {
      columns_[col][axis] = rotation[axis][0] * ray_x_[col];
    }
  }
  for (uint32_t row = 0; row < labels.height; ++row) {
    for (size_t axis = 0; axis < 3; ++axis) {
      rows_[row][axis] = rotation[axis][1] * ray_y_[row] + rotation[axis][2];
    }
  }
  const uint64_t max_points = samples ? samples->size() :
    static_cast<uint64_t>(labels.width) * labels.height;
  if (max_points > std::numeric_limits<uint32_t>::max() / sizeof(Point)) {
    throw std::invalid_argument("Image exceeds PointCloud2 row size limit");
  }
  output.height = 1;
  output.is_bigendian = host_bigendian();
  output.is_dense = true;
  output.point_step = sizeof(Point);
  if (output.fields.empty()) {
    output.fields = {
      field("x", 0, PointField::FLOAT32), field("y", 4, PointField::FLOAT32),
      field("z", 8, PointField::FLOAT32), field("class_id", 12, PointField::UINT8)};
  }
  output.data.resize(static_cast<size_t>(max_points) * sizeof(Point));
  const bool swap = static_cast<bool>(depth.is_bigendian) != host_bigendian();
  size_t count;
  if (samples) {
    if (millimetres) {
      count = swap ? project_samples<uint16_t, true>(depth, labels, *samples, ray_x_, ray_y_,
        rotation, transform.translation, options, output.data.data()) :
        project_samples<uint16_t, false>(depth, labels, *samples, ray_x_, ray_y_,
        rotation, transform.translation, options, output.data.data());
    } else {
      count = swap ? project_samples<float, true>(depth, labels, *samples, ray_x_, ray_y_,
        rotation, transform.translation, options, output.data.data()) :
        project_samples<float, false>(depth, labels, *samples, ray_x_, ray_y_,
        rotation, transform.translation, options, output.data.data());
    }
  } else if (millimetres) {
    count = swap ? project_dispatch<uint16_t, true>(
      depth, labels, columns_, rows_, transform.translation, options, output.data.data()) :
      project_dispatch<uint16_t, false>(
      depth, labels, columns_, rows_, transform.translation, options, output.data.data());
  } else {
    count = swap ? project_dispatch<float, true>(
      depth, labels, columns_, rows_, transform.translation, options, output.data.data()) :
      project_dispatch<float, false>(
      depth, labels, columns_, rows_, transform.translation, options, output.data.data());
  }
  output.width = static_cast<uint32_t>(count);
  output.row_step = output.width * output.point_step;
  output.data.resize(output.row_step);
}

}  // namespace limo_cloud
