#include "limo_depth/geometry.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace limo_depth
{
namespace
{
using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;
using Jpeg = sensor_msgs::msg::CompressedImage;
using Steady = std::chrono::steady_clock;

double median(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  const size_t middle = values.size() / 2;
  return values.size() % 2 ? values[middle] : (values[middle - 1] + values[middle]) * 0.5;
}

Camera camera(const Info & info)
{
  if (info.width > uint32_t(INT32_MAX) || info.height > uint32_t(INT32_MAX)) {
    throw std::invalid_argument("Camera dimensions exceed supported integer range");
  }
  Camera result{int(info.width), int(info.height), info.k[0], info.k[4], info.k[2], info.k[5]};
  result.validate();
  if (info.header.frame_id.empty() ||
    !std::all_of(info.k.begin(), info.k.end(), [](double v) {return std::isfinite(v);}) ||
    !std::all_of(info.d.begin(), info.d.end(), [](double v) {
        return std::isfinite(v) && std::abs(v) <= 1e-8;
      }) ||
    std::abs(info.k[1]) > 1e-8 || std::abs(info.k[3]) > 1e-8 ||
    std::abs(info.k[6]) > 1e-8 || std::abs(info.k[7]) > 1e-8 ||
    std::abs(info.k[8] - 1) > 1e-8 || info.binning_x > 1 || info.binning_y > 1 ||
    info.roi.x_offset || info.roi.y_offset || info.roi.width || info.roi.height)
  {throw std::invalid_argument("CameraInfo requires a frame and rectified, unbinned pinhole geometry");}
  return result;
}

DepthView view(const Image & image)
{
  if (image.encoding != "32FC1" && image.encoding != "16UC1" && image.encoding != "mono16") {
    throw std::invalid_argument("Depth encoding must be 32FC1 (m), 16UC1 or mono16 (mm)");
  }
  if (image.width > uint32_t(INT32_MAX) || image.height > uint32_t(INT32_MAX)) {
    throw std::invalid_argument("Depth dimensions exceed supported integer range");
  }
  DepthView result{image.data.data(), image.data.size(), image.step,
    int(image.width), int(image.height), image.encoding != "32FC1", image.is_bigendian != 0};
  result.validate();
  return result;
}
}  // namespace

class DepthNode : public rclcpp::Node
{
public:
  DepthNode() : Node("limo_depth")
  {
    geometry_.width = integer("output_width", 320);
    geometry_.height = integer("output_height", 120);
    geometry_.crop = parameter("input_crop_y_min", 0.5);
    geometry_.min_depth = parameter("min_depth_m", 0.1);
    geometry_.max_depth = parameter("max_depth_m", 5.0);
    geometry_.validate();
    registered_ = parameter("register_to_rgb", true);
    reduce_ = parameter("registration_reduce_input", false);
    plane_frame_ = parameter("plane_frame", std::string("camera_link"));
    plane_height_ = parameter("plane_height_m", std::numeric_limits<double>::quiet_NaN());
    calibration_frames_ = integer("calibration_frames", 30);
    fit_.threshold = parameter("ground_inlier_threshold_m", 0.008);
    fit_.min_inlier_ratio = parameter("ground_min_inlier_ratio", 0.6);
    fit_.max_tilt_deg = parameter("ground_max_tilt_deg", 20.0);
    fit_.min_height = parameter("ground_min_height_m", 0.05);
    fit_.max_height = parameter("ground_max_height_m", 0.6);
    fit_.min_spread = parameter("ground_min_spread_m", 0.025);
    fit_.min_points = integer("ground_min_points", 150);
    fit_.max_points = integer("ground_max_points", 2500);
    fit_.iterations = integer("ground_ransac_iterations", 96);
    fit_.validate();
    sample_y_ = parameter("calibration_y_min", 0.6);
    sample_left_ = parameter("calibration_x_min", 0.08);
    sample_right_ = parameter("calibration_x_max", 0.92);
    sample_stride_ = integer("calibration_stride", 3);
    sample_min_ = parameter("calibration_min_depth_m", 0.25);
    sample_max_ = parameter("calibration_max_depth_m", 2.5);
    sample_z_ = parameter("calibration_max_z_m", -0.04);
    stable_angle_ = parameter("calibration_max_angle_deg", 1.0);
    stable_offset_ = parameter("calibration_max_offset_m", 0.015);
    jpeg_quality_ = integer("debug_jpeg_quality", 80);
    debug_ = parameter("enable_debug_publications", true);
    window_ = integer("telemetry_window_size", 30);
    log_interval_ = integer("telemetry_log_interval_frames", 30);
    telemetry_ = parameter("enable_telemetry", false, false);
    rate_ = parameter("max_processing_rate_hz", 0.0, false);
    if (plane_frame_.empty() || std::isinf(plane_height_) ||
      !(sample_y_ >= 0 && sample_y_ < 1) ||
      !(sample_left_ >= 0 && sample_left_ < sample_right_ && sample_right_ <= 1) ||
      !(sample_min_ > 0 && sample_min_ < sample_max_) || !std::isfinite(sample_max_) ||
      !std::isfinite(sample_z_) || !(stable_angle_ > 0 && stable_angle_ <= 90) ||
      !(stable_offset_ > 0) || !std::isfinite(stable_offset_) || jpeg_quality_ > 100)
    {throw std::invalid_argument("Invalid calibration, plane frame/height or JPEG parameters");}
    validate_rate(rate_);

    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    depth_pub_ = create_publisher<Image>(parameter("depth_output_topic", std::string(
          "limo/cv_package/depth_correction/depth_corrected/raw")), qos);
    ground_pub_ = create_publisher<Image>(parameter("ground_depth_topic", std::string(
          "limo/cv_package/depth_correction/ground_depth/raw")), qos);
    raw_debug_ = create_publisher<Jpeg>(parameter("output_topic", std::string(
          "limo/cv_package/depth_correction/image_jet/compressed")), rclcpp::QoS(1).reliable());
    corrected_debug_ = create_publisher<Jpeg>(parameter("corrected_output_topic", std::string(
          "limo/cv_package/depth_correction/image_jet_corrected/compressed")),
      rclcpp::QoS(1).reliable());
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this, false);
    info_sub_ = create_subscription<Info>(parameter("camera_info_topic", std::string(
          "/depth_camera/depth/camera_info")), qos,
      [this](Info::ConstSharedPtr info) {info_ = std::move(info);});
    const auto rgb_topic = parameter("rgb_camera_info_topic", std::string("/rgb/camera_info"));
    if (registered_) {
      rgb_sub_ = create_subscription<Info>(rgb_topic, qos,
        [this](Info::ConstSharedPtr info) {rgb_info_ = std::move(info);});
    }
    input_sub_ = create_subscription<Image>(parameter("input_topic", std::string(
          "/depth_camera/depth/image_raw")), qos, [this](Image::ConstSharedPtr image) {
        ++received_;
        if (rate_ > 0) {
          if (pending_) {++overwritten_;}
          pending_ = std::move(image);
        } else {process(*image);}
      });
    reset_service_ = create_service<std_srvs::srv::Trigger>("~/reset_calibration",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        reset_plane();
        pending_.reset();
        response->success = true;
        response->message = "Ground calibration cleared; subsequent measured frames recalibrate";
      });
    set_rate(rate_);
    parameters_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        try {
          for (const auto & p : parameters) {
            if (p.get_name() == "max_processing_rate_hz") {validate_rate(p.as_double());}
            if (p.get_name() == "enable_telemetry") {p.as_bool();}
          }
          for (const auto & p : parameters) {
            if (p.get_name() == "max_processing_rate_hz") {set_rate(p.as_double());}
            if (p.get_name() == "enable_telemetry") {telemetry_ = p.as_bool();}
          }
          result.successful = true;
        } catch (const std::exception & error) {result.reason = error.what();}
        return result;
      });
    RCLCPP_INFO(get_logger(), "LIMO depth ready: %dx%d, RGB registration=%s, plane frame=%s",
      geometry_.width, geometry_.height, registered_ ? "true" : "false", plane_frame_.c_str());
  }

private:
  template<typename T>
  T parameter(const std::string & name, const T & fallback, bool read_only = true)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = read_only;
    return declare_parameter<T>(name, fallback, descriptor);
  }

  int integer(const std::string & name, int fallback)
  {
    const auto value = parameter<int64_t>(name, fallback);
    if (value <= 0 || value > INT32_MAX) {
      throw std::invalid_argument(name + " must be a positive 32-bit integer");
    }
    return int(value);
  }

  static void validate_rate(double rate)
  {
    // Nanosecond resolution, with a strictly positive representable timer period.
    if (!std::isfinite(rate) || rate < 0 || rate > 1e9 ||
      (rate > 0 && 1e9 / rate > double(INT64_MAX)))
    {throw std::invalid_argument("max_processing_rate_hz must be 0 or a representable positive rate");}
  }

  void set_rate(double rate)
  {
    if (timer_) {timer_->cancel(); timer_.reset();}
    pending_.reset();
    rate_ = rate;
    if (rate > 0) {
      timer_ = create_wall_timer(std::chrono::nanoseconds(int64_t(1e9 / rate)), [this]() {
          auto image = std::move(pending_);
          if (image) {process(*image);}
        });
    }
  }

  Transform transform(const std::string & target, const std::string & source,
    const builtin_interfaces::msg::Time & stamp)
  {
    if (target == source) {return {};}
    const auto tf = buffer_->lookupTransform(target, source, rclcpp::Time(stamp)).transform;
    const auto & q = tf.rotation;
    Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    Transform result;
    result.translation = {tf.translation.x, tf.translation.y, tf.translation.z};
    if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-6 ||
      !result.translation.allFinite())
    {throw std::invalid_argument("Invalid TF quaternion or translation");}
    result.rotation = rotation.normalized().toRotationMatrix();
    return result;
  }

  void reset_plane()
  {
    calibrated_ = false;
    planes_.clear();
    ground_.assign(size_t(geometry_.width) * geometry_.height, missing_depth);
    if (have_geometry_ && std::isfinite(plane_height_)) {
      plane_ = Plane(0, 0, 1, -plane_height_);
      ground_ = ground_depth(rays_, target_to_plane_, plane_, geometry_.min_depth, geometry_.max_depth);
      calibrated_ = true;
    }
  }

  void prepare(const Image & image)
  {
    if (!info_ || (registered_ && !rgb_info_)) {
      throw std::invalid_argument("Waiting for depth/RGB CameraInfo");
    }
    const auto & target_info = registered_ ? *rgb_info_ : *info_;
    const auto source = camera(*info_), target = camera(target_info);
    if (image.header.frame_id != info_->header.frame_id ||
      image.width != info_->width || image.height != info_->height)
    {throw std::invalid_argument("Depth image frame/dimensions differ from CameraInfo");}
    const auto to_target = transform(target_info.header.frame_id, image.header.frame_id, image.header.stamp);
    const auto to_plane = transform(plane_frame_, image.header.frame_id, image.header.stamp);
    const auto target_to_plane = transform(plane_frame_, target_info.header.frame_id, image.header.stamp);
    if (have_geometry_ && source == source_ && target == target_ &&
      image.header.frame_id == source_frame_ && target_info.header.frame_id == target_frame_ &&
      to_target.near(to_target_) && to_plane.near(source_to_plane_) &&
      target_to_plane.near(target_to_plane_))
    {return;}
    source_ = source; target_ = target;
    source_frame_ = image.header.frame_id; target_frame_ = target_info.header.frame_id;
    to_target_ = to_target; source_to_plane_ = to_plane; target_to_plane_ = target_to_plane;
    if (registered_) {
      registration_ = std::make_unique<Registration>(source, target, to_target, geometry_, reduce_);
    }
    rays_ = pixel_rays(target, geometry_);
    have_geometry_ = true;
    reset_plane();
    RCLCPP_INFO(get_logger(), "Geometry updated; ground calibration %s (%zu registration samples)",
      calibrated_ ? "configured" : "restarted", registration_ ? registration_->sample_count() : 0);
  }

  void calibrate(const DepthView & depth)
  {
    if (calibrated_) {return;}
    Points points;
    const int top = int(source_.height * sample_y_);
    const int left = int(source_.width * sample_left_), right = int(source_.width * sample_right_);
    const double near = std::max(sample_min_, geometry_.min_depth);
    const double far = std::min(sample_max_, geometry_.max_depth);
    for (int row = top; row < source_.height; row += sample_stride_) {
      for (int col = left; col < right; col += sample_stride_) {
        const float z = depth.at(row, col);
        if (!std::isfinite(z) || z < near || z > far) {continue;}
        const Eigen::Vector3d point = source_to_plane_.rotation * (source_.ray(col, row) * z) +
          source_to_plane_.translation;
        if (point.z() < sample_z_) {points.push_back(point);}
      }
    }
    try {planes_.push_back(fit_ground_plane(std::move(points), fit_));}
    catch (const std::invalid_argument & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", error.what());
      return;
    }
    if (planes_.size() < size_t(calibration_frames_)) {return;}
    for (int component = 0; component < 4; ++component) {
      std::vector<double> values;
      for (const auto & plane : planes_) {values.push_back(plane[component]);}
      plane_[component] = median(std::move(values));
    }
    plane_ /= plane_.head<3>().norm();
    double low = planes_.front().w(), high = low, angle = 0;
    for (const auto & plane : planes_) {
      const double dot = std::max(-1.0, std::min(1.0, plane.head<3>().dot(plane_.head<3>())));
      angle = std::max(angle, std::acos(dot) * 180.0 / std::acos(-1.0));
      low = std::min(low, plane.w()); high = std::max(high, plane.w());
    }
    if (angle > stable_angle_ || high - low > stable_offset_) {
      planes_.erase(planes_.begin());
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Ground fit is not stable yet");
      return;
    }
    ground_ = ground_depth(rays_, target_to_plane_, plane_, geometry_.min_depth, geometry_.max_depth);
    calibrated_ = true;
    planes_.clear();
    RCLCPP_INFO(get_logger(), "Ground LUT ready: n=[%.6f, %.6f, %.6f], d=%.6f m",
      plane_.x(), plane_.y(), plane_.z(), plane_.w());
  }

  void publish_depth(const std::vector<float> & depth, const std_msgs::msg::Header & header,
    const rclcpp::Publisher<Image>::SharedPtr & publisher)
  {
    Image message;
    message.header = header;
    message.width = geometry_.width; message.height = geometry_.height;
    message.encoding = "32FC1";
    const uint16_t endian_test = 1;
    message.is_bigendian = *reinterpret_cast<const uint8_t *>(&endian_test) == 0;
    message.step = uint32_t(geometry_.width) * sizeof(float);
    message.data.resize(depth.size() * sizeof(float));
    std::memcpy(message.data.data(), depth.data(), message.data.size());
    publisher->publish(message);
  }

  void publish_debug(const std::vector<float> & depth, const std_msgs::msg::Header & header,
    const rclcpp::Publisher<Jpeg>::SharedPtr & publisher)
  {
    if (!debug_ || !publisher->get_subscription_count()) {return;}
    gray_.create(geometry_.height, geometry_.width, CV_8UC1);
    for (size_t i = 0; i < depth.size(); ++i) {
      const double fraction = (depth[i] - geometry_.min_depth) / (geometry_.max_depth - geometry_.min_depth);
      gray_.data[i] = std::isfinite(fraction) && depth[i] > 0 ?
        uint8_t(std::max(0.0, std::min(255.0, fraction * 255))) : 0;
    }
    cv::applyColorMap(gray_, color_, cv::COLORMAP_JET);
    for (size_t i = 0; i < depth.size(); ++i) {
      if (!std::isfinite(depth[i]) || depth[i] <= 0) {color_.ptr<cv::Vec3b>()[i] = {0, 0, 0};}
    }
    Jpeg message;
    message.header = header;
    message.format = "bgr8; jpeg compressed bgr8";
    if (cv::imencode(".jpg", color_, message.data, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_})) {
      publisher->publish(message);
    }
  }

  void process(const Image & image)
  {
    const auto started = Steady::now();
    try {
      const auto depth = view(image);
      prepare(image);
      calibrate(depth);  // Always use original measured pixels, before reduction or filling.
      if (registered_) {registration_->project(depth, measured_);}
      else {resize_depth(depth, geometry_, measured_);}
      auto header = image.header;
      header.frame_id = target_frame_;
      publish_debug(measured_, header, raw_debug_);
      complete_depth(measured_, ground_);
      publish_depth(measured_, header, depth_pub_);
      if (ground_pub_->get_subscription_count()) {publish_depth(ground_, header, ground_pub_);}
      publish_debug(measured_, header, corrected_debug_);
      ++published_;
      if (telemetry_) {
        const double ms = std::chrono::duration<double, std::milli>(Steady::now() - started).count();
        times_.push_back(ms); total_ms_ += ms;
        if (times_.size() > size_t(window_)) {total_ms_ -= times_.front(); times_.pop_front();}
        if (published_ % log_interval_ == 0) {
          RCLCPP_INFO(get_logger(), "Depth: %.2f ms mean, received=%zu published=%zu overwritten=%zu skipped=%zu calibrated=%s",
            total_ms_ / times_.size(), received_, published_, overwritten_, skipped_, calibrated_ ? "true" : "false");
        }
      }
    } catch (const std::exception & error) {
      ++skipped_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Depth frame skipped: %s", error.what());
    }
  }

  ImageGeometry geometry_;
  FitOptions fit_;
  Camera source_, target_;
  Transform to_target_, source_to_plane_, target_to_plane_;
  std::string plane_frame_, source_frame_, target_frame_;
  bool registered_, reduce_, telemetry_, debug_, have_geometry_ = false, calibrated_ = false;
  double plane_height_, sample_y_, sample_left_, sample_right_, sample_min_, sample_max_, sample_z_;
  double stable_angle_, stable_offset_, rate_, total_ms_ = 0;
  int calibration_frames_, sample_stride_, jpeg_quality_, window_, log_interval_;
  size_t received_ = 0, published_ = 0, overwritten_ = 0, skipped_ = 0;
  Plane plane_;
  std::vector<Plane> planes_;
  std::vector<Eigen::Vector3d> rays_;
  std::vector<float> ground_, measured_;
  std::deque<double> times_;
  cv::Mat gray_, color_;
  std::unique_ptr<Registration> registration_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  Info::ConstSharedPtr info_, rgb_info_;
  Image::ConstSharedPtr pending_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Info>::SharedPtr info_sub_, rgb_sub_;
  rclcpp::Subscription<Image>::SharedPtr input_sub_;
  rclcpp::Publisher<Image>::SharedPtr depth_pub_, ground_pub_;
  rclcpp::Publisher<Jpeg>::SharedPtr raw_debug_, corrected_debug_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_;
};
}  // namespace limo_depth

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {rclcpp::spin(std::make_shared<limo_depth::DepthNode>());}
  catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("limo_depth"), "%s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
