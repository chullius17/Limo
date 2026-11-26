#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace limo_depth
{
constexpr float missing_depth = std::numeric_limits<float>::quiet_NaN();

struct Camera
{
  int width = 0, height = 0;
  double fx = 0, fy = 0, cx = 0, cy = 0;
  void validate() const;
  bool operator==(const Camera & other) const;
  Eigen::Vector3d ray(double u, double v) const;
};

struct Transform
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  bool near(const Transform & other, double tolerance = 1e-8) const;
};

// Borrowed sensor buffer. Samples may be padded, unaligned and either endian.
struct DepthView
{
  const uint8_t * data = nullptr;
  size_t size = 0, step = 0;
  int width = 0, height = 0;
  bool millimetres = false, big_endian = false;
  void validate() const;
  float at(int row, int col) const;
};

struct ImageGeometry
{
  int width = 320, height = 120;
  double crop = 0.5, min_depth = 0.1, max_depth = 5.0;
  void validate() const;
};

struct FitOptions
{
  double threshold = 0.008, min_inlier_ratio = 0.6, max_tilt_deg = 20.0;
  double min_height = 0.05, max_height = 0.6, min_spread = 0.025;
  int min_points = 150, max_points = 2500, iterations = 96;
  void validate() const;
};

// Plane n.p + d = 0 in a Z-up frame; n is unit length and points upward.
using Plane = Eigen::Vector4d;
using Points = std::vector<Eigen::Vector3d>;
Plane fit_ground_plane(Points points, const FitOptions & options = FitOptions());
std::vector<Eigen::Vector3d> pixel_rays(const Camera & camera, const ImageGeometry & output);
std::vector<float> ground_depth(
  const std::vector<Eigen::Vector3d> & rays, const Transform & optical_to_plane,
  const Plane & plane, double min_depth, double max_depth);
void complete_depth(std::vector<float> & measured, const std::vector<float> & ground);
void resize_depth(const DepthView & source, const ImageGeometry & output, std::vector<float> & target);

// Cache source selection and rotated rays; only Z projection is evaluated per frame.
class Registration
{
public:
  Registration(
    const Camera & source, const Camera & target, const Transform & source_to_target,
    const ImageGeometry & output, bool reduce_input);
  void project(const DepthView & source, std::vector<float> & output) const;
  size_t sample_count() const {return samples_.size();}

private:
  struct Sample {int row, col; Eigen::Vector3d direction;};
  Camera source_, target_;
  Transform transform_;
  ImageGeometry output_;
  std::vector<Sample> samples_;
};
}  // namespace limo_depth
