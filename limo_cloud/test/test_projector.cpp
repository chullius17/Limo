#include "limo_cloud/projector.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

namespace
{
sensor_msgs::msg::CameraInfo calibration()
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 640;
  info.height = 480;
  info.k = {513.3, 0, 319.4, 0, 514.2, 241.2, 0, 0, 1};
  return info;
}

sensor_msgs::msg::Image labels(uint32_t width, uint32_t height)
{
  sensor_msgs::msg::Image result;
  result.width = width;
  result.height = height;
  result.encoding = "mono8";
  result.step = width + 3;  // Exercise non-contiguous rows.
  result.data.resize(result.step * height, 255);
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      result.data[y * result.step + x] = static_cast<uint8_t>((y * width + x) % 256);
    }
  }
  return result;
}

template<typename T>
sensor_msgs::msg::Image depths(uint32_t width, uint32_t height, T value, bool bigendian = false)
{
  sensor_msgs::msg::Image result;
  result.width = width;
  result.height = height;
  result.encoding = sizeof(T) == 2 ? "16UC1" : "32FC1";
  result.is_bigendian = bigendian;
  result.step = width * sizeof(T) + 4;
  result.data.resize(result.step * height, 99);
  uint8_t bytes[sizeof(T)];
  std::memcpy(bytes, &value, sizeof(T));
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      for (size_t i = 0; i < sizeof(T); ++i) {
        result.data[y * result.step + x * sizeof(T) + i] = bytes[bigendian ? sizeof(T) - i - 1 : i];
      }
    }
  }
  return result;
}

std::vector<limo_cloud::Point> points(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<limo_cloud::Point> result(cloud.width);
  if (!result.empty()) {std::memcpy(result.data(), cloud.data.data(), cloud.data.size());}
  return result;
}

TEST(Projector, PreservesAllClassesAndMatchesCropAwareGeometry)
{
  limo_cloud::Projector projector;
  auto classes = labels(32, 8);
  auto depth = depths<float>(32, 8, 1.2F);
  auto info = calibration();
  geometry_msgs::msg::Transform transform;
  transform.rotation.z = std::sin(0.3);
  transform.rotation.w = std::cos(0.3);
  transform.translation.x = 0.2;
  transform.translation.y = -0.7;
  transform.translation.z = 0.4;
  for (double crop : {0.0, 0.1, 0.5, 0.73}) {
    limo_cloud::ProjectionOptions options;
    options.input_crop_y_min = crop;
    options.invalid_class_id = -1;
    options.flatten_to_bev = false;
    sensor_msgs::msg::PointCloud2 output;
    projector.project(depth, classes, info, transform, options, output);
    EXPECT_NO_THROW(limo_cloud::validate_cloud(output));
    auto result = points(output);
    ASSERT_EQ(result.size(), 256U);
    for (size_t i = 0; i < result.size(); ++i) {
      const double u = (i % 32 + 0.5) * 640 / 32 - 0.5;
      const int start = 480 * crop;
      const double v = start + (i / 32 + 0.5) * (480 - start) / 8 - 0.5;
      const double x = (u - info.k[2]) / info.k[0] * 1.2;
      const double y = (v - info.k[5]) / info.k[4] * 1.2;
      EXPECT_NEAR(result[i].x, std::cos(0.6) * x - std::sin(0.6) * y + 0.2, 2e-6);
      EXPECT_NEAR(result[i].y, std::sin(0.6) * x + std::cos(0.6) * y - 0.7, 2e-6);
      EXPECT_NEAR(result[i].z, 1.6, 2e-6);
      EXPECT_EQ(result[i].class_id, i);
    }
  }
}

TEST(Projector, SameGeometryForFloatAndMillimetresBothByteOrders)
{
  limo_cloud::Projector projector;
  auto classes = labels(5, 3);
  auto info = calibration();
  geometry_msgs::msg::Transform tf;
  tf.rotation.w = 1;
  limo_cloud::ProjectionOptions options;
  sensor_msgs::msg::PointCloud2 expected;
  projector.project(depths<float>(5, 3, 1.25F), classes, info, tf, options, expected);
  for (bool big : {false, true}) {
    for (const auto & depth : {depths<uint16_t>(5, 3, 1250, big), depths<float>(5, 3, 1.25F, big)}) {
      sensor_msgs::msg::PointCloud2 actual;
      projector.project(depth, classes, info, tf, options, actual);
      const auto a = points(actual), e = points(expected);
      ASSERT_EQ(a.size(), e.size());
      for (size_t i = 0; i < a.size(); ++i) {
        EXPECT_FLOAT_EQ(a[i].x, e[i].x);
        EXPECT_FLOAT_EQ(a[i].y, e[i].y);
        EXPECT_EQ(a[i].class_id, e[i].class_id);
        EXPECT_EQ(a[i].z, 0);
      }
    }
  }
}

TEST(Projector, InvalidDepthAndRoiAreUniformAcrossLabels)
{
  limo_cloud::Projector projector;
  auto classes = labels(6, 2);
  auto depth = depths<float>(6, 2, 1.0F);
  const float values[] = {0, -1, NAN, INFINITY, 0.1F, 2.5F};
  std::memcpy(depth.data.data() + depth.step, values, sizeof(values));
  geometry_msgs::msg::Transform tf;
  tf.rotation.w = 1;
  limo_cloud::ProjectionOptions options;
  options.roi_y_min = 0.5;
  sensor_msgs::msg::PointCloud2 cloud;
  projector.project(depth, classes, calibration(), tf, options, cloud);
  auto result = points(cloud);
  ASSERT_EQ(result.size(), 2U);
  EXPECT_EQ(result[0].class_id, 10);
  EXPECT_EQ(result[1].class_id, 11);
  std::fill(classes.data.begin(), classes.data.end(), 0);
  projector.project(depth, classes, calibration(), tf, options, cloud);
  EXPECT_EQ(cloud.width, 0U);
  EXPECT_TRUE(cloud.data.empty());
  EXPECT_NO_THROW(limo_cloud::validate_cloud(cloud));
}

TEST(Projector, RejectsMalformedImagesCalibrationAndTransforms)
{
  limo_cloud::Projector projector;
  auto classes = labels(4, 2);
  auto depth = depths<float>(4, 2, 1.0F);
  auto info = calibration();
  geometry_msgs::msg::Transform tf;
  tf.rotation.w = 1;
  limo_cloud::ProjectionOptions options;
  sensor_msgs::msg::PointCloud2 cloud;
  depth.data.pop_back();
  EXPECT_THROW(projector.project(depth, classes, info, tf, options, cloud), std::invalid_argument);
  depth = depths<float>(4, 2, 1.0F);
  tf.rotation.w = 0;
  EXPECT_THROW(projector.project(depth, classes, info, tf, options, cloud), std::invalid_argument);
  tf.rotation.w = 1;
  info.k[0] = NAN;
  EXPECT_THROW(projector.project(depth, classes, info, tf, options, cloud), std::invalid_argument);
}

TEST(Projector, SparseSamplesPreserveOrderDuplicatesAndExplicitClasses)
{
  limo_cloud::Projector projector;
  auto classes = labels(4, 2);
  auto depth = depths<float>(4, 2, 1.0F);
  geometry_msgs::msg::Transform tf;
  tf.rotation.w = 1;
  limo_cloud::ProjectionOptions options;
  std::vector<limo_cloud::PixelSample> samples{
    {1, 3, 5, {0, 0, 0}}, {0, 0, 1, {0, 0, 0}}, {1, 3, 3, {0, 0, 0}}};
  sensor_msgs::msg::PointCloud2 cloud;
  projector.project(depth, classes, calibration(), tf, options, cloud, &samples);
  const auto output = points(cloud);
  ASSERT_EQ(output.size(), 3U);
  EXPECT_EQ(output[0].class_id, 5);
  EXPECT_EQ(output[1].class_id, 1);
  EXPECT_EQ(output[2].class_id, 3);
  EXPECT_EQ(output[0].x, output[2].x);
  EXPECT_EQ(output[0].y, output[2].y);
  samples.clear();
  projector.project(depth, classes, calibration(), tf, options, cloud, &samples);
  EXPECT_EQ(cloud.width, 0U);
  samples.push_back({2, 0, 1, {0, 0, 0}});
  EXPECT_THROW(projector.project(depth, classes, calibration(), tf, options, cloud, &samples),
    std::invalid_argument);
}
}  // namespace
