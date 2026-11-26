#include "limo_depth/geometry.hpp"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <random>
#include <stdexcept>

using namespace limo_depth;  // NOLINT

TEST(DepthBuffer, ReadsPaddedUnalignedBothEndianAndUnits)
{
  std::vector<uint8_t> bytes{0xff, 0x03, 0xe8, 0x07, 0xd0, 0xff, 0xff, 0, 0, 0, 1, 0xff, 0xff};
  DepthView view{bytes.data() + 1, 12, 6, 2, 2, true, true};
  ASSERT_NO_THROW(view.validate());
  EXPECT_FLOAT_EQ(view.at(0, 0), 1.0f);
  EXPECT_FLOAT_EQ(view.at(0, 1), 2.0f);
  EXPECT_FLOAT_EQ(view.at(1, 0), 0.0f);
  EXPECT_FLOAT_EQ(view.at(1, 1), 0.001f);
  std::vector<uint8_t> floating{0xff, 0x3f, 0xa0, 0, 0, 0, 0, 0, 0};
  DepthView f32{floating.data() + 1, 8, 8, 2, 1, false, true};
  EXPECT_FLOAT_EQ(f32.at(0, 0), 1.25f);
  EXPECT_FLOAT_EQ(f32.at(0, 1), 0.0f);
  floating = {0xff, 0, 0, 0xa0, 0x3f, 0, 0, 0, 0};
  f32.data = floating.data() + 1; f32.big_endian = false;
  EXPECT_FLOAT_EQ(f32.at(0, 0), 1.25f);
  view.step = 3;
  EXPECT_THROW(view.validate(), std::invalid_argument);
  view.step = 6; view.size = 11;
  EXPECT_THROW(view.validate(), std::invalid_argument);
}

TEST(Geometry, CropUsesTargetPixelCentresAndOddHeights)
{
  Camera camera{4, 5, 2, 2, 1.5, 2};
  ImageGeometry output{2, 2, 0.4, 0.1, 5};
  const auto rays = pixel_rays(camera, output);
  ASSERT_EQ(rays.size(), 4u);
  EXPECT_DOUBLE_EQ(rays[0].x(), -0.5);
  EXPECT_DOUBLE_EQ(rays[0].y(), 0.125);
  EXPECT_DOUBLE_EQ(rays[3].x(), 0.5);
  EXPECT_DOUBLE_EQ(rays[3].y(), 0.875);
}

TEST(Registration, IdentityPreservesMeasurementsAndRejectsInvalidDepth)
{
  Camera camera{4, 2, 2, 2, 1.5, 0.5};
  ImageGeometry output{4, 2, 0, 0.1, 5};
  std::vector<float> values{1, 2, 3, 4, 0, missing_depth, INFINITY, 6};
  DepthView input{reinterpret_cast<const uint8_t *>(values.data()), values.size() * 4,
    16, 4, 2, false, false};
  Registration registration(camera, camera, {}, output, false);
  std::vector<float> projected;
  registration.project(input, projected);
  for (int i = 0; i < 4; ++i) {EXPECT_FLOAT_EQ(projected[i], values[i]);}
  for (int i = 4; i < 8; ++i) {EXPECT_TRUE(std::isnan(projected[i]));}
}

TEST(Registration, CollisionsKeepNearestTransformedZ)
{
  Camera source{2, 1, 2, 2, 0.5, 0};
  Camera target{1, 1, 0.1, 0.1, 0, 0};
  ImageGeometry output{1, 1, 0, 0.1, 5};
  std::vector<float> values{2, 1};
  DepthView input{reinterpret_cast<const uint8_t *>(values.data()), 8, 8, 2, 1, false, false};
  Transform transform;
  transform.translation.z() = 0.25;
  Registration registration(source, target, transform, output, false);
  std::vector<float> projected;
  registration.project(input, projected);
  EXPECT_FLOAT_EQ(projected[0], 1.25f);
}

TEST(Registration, ReductionUsesExactNativePixelRays)
{
  Camera camera{8, 6, 5, 5, 3.5, 2.5};
  ImageGeometry output{4, 2, 0.5, 0.1, 5};
  std::vector<float> values(48, 1.5f);
  DepthView input{reinterpret_cast<const uint8_t *>(values.data()), 192, 32, 8, 6, false, false};
  Registration registration(camera, camera, {}, output, true);
  EXPECT_EQ(registration.sample_count(), 8u);
  std::vector<float> projected;
  registration.project(input, projected);
  for (auto value : projected) {EXPECT_FLOAT_EQ(value, 1.5f);}
}

TEST(Registration, DisjointFrustumLeavesHoles)
{
  Camera camera{8, 6, 5, 5, 3.5, 2.5};
  Transform transform;
  transform.translation.x() = 100;
  Registration registration(camera, camera, transform, {4, 2, 0.5, 0.1, 5}, true);
  EXPECT_EQ(registration.sample_count(), 0u);
}

TEST(Geometry, NativeResizeCropsBeforeSampling)
{
  std::vector<uint8_t> bytes(4 * 4 * 2);
  for (int i = 0; i < 16; ++i) {bytes[2 * i] = 0xe8; bytes[2 * i + 1] = 0x03;}
  bytes[16] = 0xd0; bytes[17] = 0x07;
  DepthView input{bytes.data(), bytes.size(), 8, 4, 4, true, false};
  std::vector<float> output;
  resize_depth(input, {2, 2, 0.5, 0.1, 5}, output);
  EXPECT_EQ(output, (std::vector<float>{2, 1, 1, 1}));
}

Points plane_points(const Plane & plane)
{
  Points points;
  for (int row = 0; row < 24; ++row) {
    for (int col = 0; col < 24; ++col) {
      const double x = row * 0.025, y = col * 0.025 - 0.3;
      points.emplace_back(x, y, -(plane.w() + plane.x() * x + plane.y() * y) / plane.z());
    }
  }
  return points;
}

TEST(GroundFit, RecoversTiltWithOutliersAndMissingPoints)
{
  Plane plane(0.034, 0.011, 1, 0.18);
  plane /= plane.head<3>().norm();
  auto points = plane_points(plane);
  for (int i = 0; i < 100; ++i) {points.emplace_back(i * 0.01, 0.3, 0.2);}
  points.emplace_back(NAN, 0, 0);
  const auto fitted = fit_ground_plane(points);
  EXPECT_LT((fitted - plane).norm(), 1e-10);
}

TEST(GroundFit, RejectsWallsNarrowSupportAndWrongHeight)
{
  auto wall = plane_points(Plane(0, 0, 1, 0.18));
  for (auto & p : wall) {std::swap(p.x(), p.z());}
  EXPECT_THROW(fit_ground_plane(wall), std::invalid_argument);
  auto narrow = plane_points(Plane(0, 0, 1, 0.18));
  for (auto & p : narrow) {p.y() *= 0.001;}
  EXPECT_THROW(fit_ground_plane(narrow), std::invalid_argument);
  EXPECT_THROW(fit_ground_plane(plane_points(Plane(0, 0, 1, 0.9))), std::invalid_argument);
  EXPECT_THROW(fit_ground_plane(Points(10, Eigen::Vector3d::Zero())), std::invalid_argument);
}

TEST(GroundFit, BoundsFittingWorkWithoutChangingAPlanarSolution)
{
  const Plane plane(0, 0, 1, 0.18);
  auto points = plane_points(plane);
  const auto original = points;
  for (int i = 0; i < 10; ++i) {points.insert(points.end(), original.begin(), original.end());}
  EXPECT_LT((fit_ground_plane(points) - plane).norm(), 1e-10);
}

TEST(GroundLut, IntersectsTiltedPlaneAndHonorsHorizonAndRange)
{
  Camera camera{3, 3, 1, 1, 1, 1};
  Transform transform;
  transform.rotation << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  transform.translation.z() = 0.02;
  const auto rays = pixel_rays(camera, {3, 3, 0, 0.1, 5});
  const auto lut = ground_depth(rays, transform, Plane(0, 0, 1, 0.18), 0.1, 5);
  for (int i = 0; i < 6; ++i) {EXPECT_TRUE(std::isnan(lut[i]));}
  for (int i = 6; i < 9; ++i) {EXPECT_FLOAT_EQ(lut[i], 0.2f);}
}

TEST(Completion, KeepsForegroundMeasurementsAndUnfillableHoles)
{
  std::vector<float> depth{0.4f, missing_depth, 0, INFINITY, -1, missing_depth};
  complete_depth(depth, {1, 2, 3, 4, missing_depth, missing_depth});
  EXPECT_FLOAT_EQ(depth[0], 0.4f);
  EXPECT_FLOAT_EQ(depth[1], 2);
  EXPECT_FLOAT_EQ(depth[2], 3);
  EXPECT_FLOAT_EQ(depth[3], 4);
  EXPECT_FLOAT_EQ(depth[4], -1);
  EXPECT_TRUE(std::isnan(depth[5]));
  EXPECT_THROW(complete_depth(depth, {}), std::invalid_argument);
}

TEST(Validation, RejectsInvalidGeometryAndFittingOptions)
{
  EXPECT_THROW((ImageGeometry{0, 2, 0, 0.1, 5}.validate()), std::invalid_argument);
  EXPECT_THROW((ImageGeometry{2, 2, NAN, 0.1, 5}.validate()), std::invalid_argument);
  EXPECT_THROW((ImageGeometry{2, 2, 0, 0, INFINITY}.validate()), std::invalid_argument);
  EXPECT_THROW((Camera{2, 2, -1, 1, 0, 0}.validate()), std::invalid_argument);
  FitOptions options;
  options.min_points = 2;
  EXPECT_THROW(options.validate(), std::invalid_argument);
}
