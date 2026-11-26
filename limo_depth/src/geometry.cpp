#include "limo_depth/geometry.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>
#include <random>
#include <stdexcept>

namespace limo_depth
{
namespace
{
bool accepted(double value, double low, double high)
{
  return std::isfinite(value) && value >= low && value <= high;
}

// Clip one source ray's permitted depth interval against the RGB crop frustum.
bool reaches_crop(
  const Eigen::Vector3d & direction, const Transform & transform,
  const Camera & camera, const ImageGeometry & output)
{
  double lower = output.min_depth, upper = output.max_depth;
  const auto clip = [&](double coefficient, double offset) {
      if (coefficient > 0) {lower = std::max(lower, -offset / coefficient);}
      else if (coefficient < 0) {upper = std::min(upper, -offset / coefficient);}
      else if (offset < 0) {upper = -std::numeric_limits<double>::infinity();}
    };
  const auto & t = transform.translation;
  clip(direction.z(), t.z() - output.min_depth);
  clip(-direction.z(), output.max_depth - t.z());
  const double low_x = (-0.5 - camera.cx) / camera.fx;
  const double high_x = (camera.width - 0.5 - camera.cx) / camera.fx;
  const double low_y = (int(camera.height * output.crop) - 0.5 - camera.cy) / camera.fy;
  const double high_y = (camera.height - 0.5 - camera.cy) / camera.fy;
  clip(direction.x() - low_x * direction.z(), t.x() - low_x * t.z());
  clip(high_x * direction.z() - direction.x(), high_x * t.z() - t.x());
  clip(direction.y() - low_y * direction.z(), t.y() - low_y * t.z());
  clip(high_y * direction.z() - direction.y(), high_y * t.z() - t.y());
  return lower <= upper;
}
}  // namespace

void Camera::validate() const
{
  if (width <= 0 || height <= 0 || !std::isfinite(fx) || !std::isfinite(fy) ||
    !std::isfinite(cx) || !std::isfinite(cy) || fx <= 0 || fy <= 0)
  {throw std::invalid_argument("Invalid camera calibration/dimensions");}
}

bool Camera::operator==(const Camera & other) const
{
  return width == other.width && height == other.height && fx == other.fx &&
         fy == other.fy && cx == other.cx && cy == other.cy;
}

Eigen::Vector3d Camera::ray(double u, double v) const
{
  return {(u - cx) / fx, (v - cy) / fy, 1.0};
}

bool Transform::near(const Transform & other, double tolerance) const
{
  return (rotation - other.rotation).cwiseAbs().maxCoeff() <= tolerance &&
         (translation - other.translation).cwiseAbs().maxCoeff() <= tolerance;
}

void DepthView::validate() const
{
  const size_t bytes = millimetres ? 2 : 4;
  if (!data || width <= 0 || height <= 0 || step < size_t(width) * bytes ||
    step > size / size_t(height))
  {throw std::invalid_argument("Malformed depth buffer: dimensions, step or data size");}
}

float DepthView::at(int row, int col) const
{
  const auto * p = data + size_t(row) * step + size_t(col) * (millimetres ? 2 : 4);
  if (millimetres) {
    const uint16_t raw = big_endian ? (uint16_t(p[0]) << 8) | p[1] :
      (uint16_t(p[1]) << 8) | p[0];
    return raw * 0.001f;
  }
  const uint32_t bits = big_endian ?
    (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3] :
    (uint32_t(p[3]) << 24) | (uint32_t(p[2]) << 16) | (uint32_t(p[1]) << 8) | p[0];
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void ImageGeometry::validate() const
{
  if (width <= 0 || height <= 0 || !(crop >= 0 && crop < 1) ||
    !std::isfinite(max_depth) || !(min_depth > 0 && min_depth < max_depth))
  {throw std::invalid_argument("Invalid output dimensions, crop or depth range");}
}

void FitOptions::validate() const
{
  if (!accepted(threshold, 1e-9, 1.0) || !(min_inlier_ratio > 0 && min_inlier_ratio <= 1) ||
    !accepted(max_tilt_deg, 0, 89) || !std::isfinite(max_height) ||
    !(min_height > 0 && min_height < max_height) || !std::isfinite(min_spread) ||
    min_spread <= 0 || min_points < 3 || max_points < min_points || iterations <= 0)
  {throw std::invalid_argument("Invalid ground plane fitting parameters");}
}

std::vector<Eigen::Vector3d> pixel_rays(const Camera & camera, const ImageGeometry & output)
{
  camera.validate();
  output.validate();
  std::vector<Eigen::Vector3d> rays;
  rays.reserve(size_t(output.width) * output.height);
  const int start = int(camera.height * output.crop);
  for (int row = 0; row < output.height; ++row) {
    const double v = start + (row + 0.5) * (camera.height - start) / output.height - 0.5;
    for (int col = 0; col < output.width; ++col) {
      rays.push_back(camera.ray((col + 0.5) * camera.width / output.width - 0.5, v));
    }
  }
  return rays;
}

Registration::Registration(
  const Camera & source, const Camera & target, const Transform & transform,
  const ImageGeometry & output, bool reduce_input)
: source_(source), target_(target), transform_(transform), output_(output)
{
  source.validate();
  target.validate();
  output.validate();
  if (!transform.rotation.allFinite() || !transform.translation.allFinite()) {
    throw std::invalid_argument("Invalid registration transform");
  }
  int top = 0, left = 0, bottom = source.height, right = source.width;
  if (reduce_input) {
    top = source.height; left = source.width; bottom = right = 0;
    for (int row = 0; row < source.height; ++row) {
      for (int col = 0; col < source.width; ++col) {
        if (reaches_crop(transform.rotation * source.ray(col, row), transform, target, output)) {
          top = std::min(top, row); left = std::min(left, col);
          bottom = std::max(bottom, row + 1); right = std::max(right, col + 1);
        }
      }
    }
    if (top == source.height) {return;}
  }
  const int width = reduce_input ? std::min(output.width, right - left) : source.width;
  const int height = reduce_input ? std::min(output.height, bottom - top) : source.height;
  samples_.reserve(size_t(width) * height);
  for (int row = 0; row < height; ++row) {
    const int y = top + int(int64_t(row) * (bottom - top) / height);
    for (int col = 0; col < width; ++col) {
      const int x = left + int(int64_t(col) * (right - left) / width);
      samples_.push_back({y, x, transform.rotation * source.ray(x, y)});
    }
  }
}

void Registration::project(const DepthView & source, std::vector<float> & output) const
{
  source.validate();
  if (source.width != source_.width || source.height != source_.height) {
    throw std::invalid_argument("Depth size changed without rebuilding registration");
  }
  output.assign(size_t(output_.width) * output_.height, missing_depth);
  const int start = int(target_.height * output_.crop);
  for (const auto & sample : samples_) {
    const double depth = source.at(sample.row, sample.col);
    if (!accepted(depth, output_.min_depth, output_.max_depth)) {continue;}
    const Eigen::Vector3d point = sample.direction * depth + transform_.translation;
    if (!point.allFinite() || !accepted(point.z(), output_.min_depth, output_.max_depth)) {continue;}
    const double u = ((point.x() / point.z() * target_.fx + target_.cx) + 0.5) *
      output_.width / target_.width;
    const double v = ((point.y() / point.z() * target_.fy + target_.cy) - start + 0.5) *
      output_.height / (target_.height - start);
    if (!(u >= 0 && u < output_.width && v >= 0 && v < output_.height)) {continue;}
    auto & pixel = output[size_t(int(v)) * output_.width + int(u)];
    if (!std::isfinite(pixel) || point.z() < pixel) {pixel = float(point.z());}
  }
}

void resize_depth(const DepthView & source, const ImageGeometry & output, std::vector<float> & target)
{
  source.validate();
  output.validate();
  target.resize(size_t(output.width) * output.height);
  const int start = int(source.height * output.crop);
  for (int row = 0; row < output.height; ++row) {
    const int y = start + int(int64_t(row) * (source.height - start) / output.height);
    for (int col = 0; col < output.width; ++col) {
      const float z = source.at(y, int(int64_t(col) * source.width / output.width));
      target[size_t(row) * output.width + col] = accepted(z, output.min_depth, output.max_depth) ?
        z : missing_depth;
    }
  }
}

Plane fit_ground_plane(Points points, const FitOptions & options)
{
  options.validate();
  points.erase(std::remove_if(points.begin(), points.end(),
      [](const Eigen::Vector3d & p) {return !p.allFinite();}), points.end());
  if (points.size() < size_t(options.min_points)) {
    throw std::invalid_argument("Not enough measured ground candidates");
  }
  std::mt19937 rng(42);
  if (points.size() > size_t(options.max_points)) {
    std::shuffle(points.begin(), points.end(), rng);
    points.resize(options.max_points);
  }
  const double min_z = std::cos(options.max_tilt_deg * std::acos(-1.0) / 180.0);
  const auto plausible = [&](const Plane & plane) {
      return plane.z() >= min_z && accepted(plane.w(), options.min_height, options.max_height);
    };
  const auto inliers = [&](const Plane & plane) {
      std::vector<size_t> result;
      for (size_t i = 0; i < points.size(); ++i) {
        if (std::abs(plane.head<3>().dot(points[i]) + plane.w()) <= options.threshold) {
          result.push_back(i);
        }
      }
      return result;
    };
  std::uniform_int_distribution<size_t> index(0, points.size() - 1);
  std::vector<size_t> best;
  for (int trial = 0; trial < options.iterations; ++trial) {
    const size_t a = index(rng), b = index(rng), c = index(rng);
    if (a == b || a == c || b == c) {continue;}
    Eigen::Vector3d normal = (points[b] - points[a]).cross(points[c] - points[a]);
    if (normal.norm() < 1e-9) {continue;}
    normal.normalize();
    if (normal.z() < 0) {normal = -normal;}
    Plane plane;
    plane << normal, -normal.dot(points[a]);
    if (!plausible(plane)) {continue;}
    auto selected = inliers(plane);
    if (selected.size() > best.size()) {best = std::move(selected);}
  }
  const auto supported = [&]() {
      return best.size() >= size_t(options.min_points) &&
             best.size() >= options.min_inlier_ratio * points.size();
    };
  if (!supported()) {throw std::invalid_argument("No sufficiently supported ground plane");}
  Plane plane;
  double spread = 0;
  for (int refinement = 0; refinement < 3; ++refinement) {
    if (best.size() < size_t(options.min_points)) {
      throw std::invalid_argument("Ground support lost during refinement");
    }
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const auto i : best) {center += points[i];}
    center /= double(best.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto i : best) {
      const Eigen::Vector3d delta = points[i] - center;
      covariance.noalias() += delta * delta.transpose();
    }
    covariance /= double(best.size());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {throw std::invalid_argument("Ground eigensolver failed");}
    Eigen::Vector3d normal = solver.eigenvectors().col(0);
    if (normal.z() < 0) {normal = -normal;}
    plane << normal, -normal.dot(center);
    spread = std::sqrt(std::max(0.0, solver.eigenvalues()[1]));
    best = inliers(plane);
  }
  if (!plausible(plane) || !supported() || spread < options.min_spread) {
    throw std::invalid_argument("Ground plane is degenerate or has insufficient coverage");
  }
  return plane;
}

std::vector<float> ground_depth(
  const std::vector<Eigen::Vector3d> & rays, const Transform & transform,
  const Plane & plane, double min_depth, double max_depth)
{
  std::vector<float> output(rays.size(), missing_depth);
  const Eigen::Vector3d normal = transform.rotation.transpose() * plane.head<3>();
  const double numerator = -(plane.w() + plane.head<3>().dot(transform.translation));
  for (size_t i = 0; i < rays.size(); ++i) {
    const double denominator = normal.dot(rays[i]);
    if (denominator >= -1e-6) {continue;}
    const double depth = numerator / denominator;
    if (accepted(depth, min_depth, max_depth)) {output[i] = float(depth);}
  }
  return output;
}

void complete_depth(std::vector<float> & measured, const std::vector<float> & ground)
{
  if (measured.size() != ground.size()) {throw std::invalid_argument("Ground LUT size mismatch");}
  for (size_t i = 0; i < measured.size(); ++i) {
    if ((!std::isfinite(measured[i]) || measured[i] <= 0) &&
      std::isfinite(ground[i]) && ground[i] > 0)
    {measured[i] = ground[i];}
  }
}
}  // namespace limo_depth
