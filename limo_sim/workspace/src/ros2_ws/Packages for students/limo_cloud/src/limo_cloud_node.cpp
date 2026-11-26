#include "limo_cloud/processor.hpp"
#include "limo_cloud/node.hpp"

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace limo_cloud
{
namespace
{
using Steady = std::chrono::steady_clock;

template<typename T>
T parameter(rclcpp::Node & node, const std::string & name, const T & fallback)
{
  if (!node.has_parameter(name)) {node.declare_parameter<T>(name, fallback);}
  return node.get_parameter(name).get_value<T>();
}

double elapsed_ms(Steady::time_point start)
{
  return std::chrono::duration<double, std::milli>(Steady::now() - start).count();
}

uint32_t depth_bytes(const sensor_msgs::msg::Image & image)
{
  if (image.encoding == "32FC1") {return 4;}
  if (image.encoding == "16UC1" || image.encoding == "mono16") {return 2;}
  throw std::invalid_argument("Depth requires 32FC1, 16UC1 or mono16");
}
}  // namespace

class LimoCloudNode : public rclcpp::Node
{
public:
  LimoCloudNode(
    const std::string & name, rclcpp::NodeOptions node_options,
    const std::vector<rclcpp::Parameter> & defaults)
  : Node(name, node_options.automatically_declare_parameters_from_overrides(true)),
    loader_("limo_cloud", "limo_cloud::Processor")
  {
    for (const auto & fallback : defaults) {
      if (!has_parameter(fallback.get_name())) {
        declare_parameter(fallback.get_name(), fallback.get_parameter_value());
      }
    }
    options_.input_crop_y_min = parameter(*this, "input_crop_y_min", 0.0);
    options_.roi_y_min = parameter(*this, "roi_y_min", 0.0);
    options_.roi_y_max = parameter(*this, "roi_y_max", 1.0);
    options_.min_depth = parameter(*this, "pointcloud_min_depth_m", 0.1);
    options_.max_depth = parameter(*this, "pointcloud_max_depth_m", 2.5);
    options_.invalid_class_id = parameter(*this, "invalid_class_id", 0);
    options_.flatten_to_bev = parameter(*this, "flatten_to_bev", true);
    require_rgb_depth_ = parameter(*this, "require_rgb_depth", true);
    corrected_timeout_ = parameter(*this, "corrected_depth_timeout_sec", 1.0);
    fallback_width_ = parameter(*this, "fallback_depth_width", 320);
    fallback_height_ = parameter(*this, "fallback_depth_height", 120);
    max_stamp_delta_ = parameter(*this, "max_rgb_depth_delta_sec", 0.0);
    telemetry_ = parameter(*this, "enable_telemetry", true);
    window_size_ = parameter(*this, "telemetry_window_size", 60);
    log_interval_ = parameter(*this, "telemetry_log_interval_frames", 30);
    bev_frame_ = parameter(*this, "bev_frame", std::string("base_link"));
    if (!(options_.input_crop_y_min >= 0 && options_.input_crop_y_min < 1) ||
      !(options_.roi_y_min >= 0 && options_.roi_y_min < options_.roi_y_max && options_.roi_y_max <= 1) ||
      !(options_.min_depth > 0 && options_.min_depth < options_.max_depth) ||
      !std::isfinite(options_.max_depth) || options_.invalid_class_id < -1 ||
      options_.invalid_class_id > 255 || !(corrected_timeout_ > 0) ||
      !std::isfinite(corrected_timeout_) || fallback_width_ <= 0 || fallback_height_ <= 0 ||
      !std::isfinite(max_stamp_delta_) || max_stamp_delta_ < 0 ||
      window_size_ <= 0 || log_interval_ <= 0 || bev_frame_.empty())
    {
      throw std::invalid_argument("Invalid limo_cloud parameters");
    }
    const auto names = parameter(*this, "plugins", std::vector<std::string>{});
    for (const auto & name : names) {
      if (name.empty()) {throw std::invalid_argument("Plugin instance names cannot be empty");}
      const auto type = parameter(*this, name + ".type", std::string{});
      auto processor = type.compare(0, 7, "python:") == 0 ?
        make_python_processor(type.substr(7)) : loader_.createSharedInstance(type);
      processor->configure(*this, name);
      processors_.push_back(std::move(processor));
      RCLCPP_INFO(get_logger(), "Plugin %s: %s", name.c_str(), type.c_str());
    }
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    const auto pipeline_qos = rclcpp::QoS(1).reliable();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      parameter(*this, "pointcloud_topic", std::string("limo/cv_package/visual_ptcld/points")),
      pipeline_qos);
    labels_sub_ = create_subscription<sensor_msgs::msg::Image>(
      parameter(*this, "classified_image_topic", std::string("limo/cv_package/detection/lane_labels/raw")),
      pipeline_qos, [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        {
          std::lock_guard<std::mutex> guard(mutex_);
          if (pending_) {++overwritten_;}
          pending_ = std::move(message);
        }
        ready_.notify_one();
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      parameter(*this, "depth_topic", std::string("limo/cv_package/depth_correction/depth_corrected/raw")),
      sensor_qos, [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        try {
          validate_image(*message, depth_bytes(*message));
          std::lock_guard<std::mutex> guard(mutex_);
          depth_ = std::move(message);
          corrected_at_ = Steady::now();
          have_corrected_ = true;
        } catch (const std::exception & error) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
        }
      });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      parameter(*this, "camera_info_topic", std::string("/rgb/camera_info")), sensor_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        std::lock_guard<std::mutex> guard(mutex_);
        info_ = std::move(message);
      });
    const auto fallback_topic = parameter(*this, "fallback_depth_topic", std::string("/depth_camera/depth/image_raw"));
    if (!require_rgb_depth_) {
      fallback_sub_ = create_subscription<sensor_msgs::msg::Image>(fallback_topic, sensor_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {fallback(std::move(message));});
    }
    worker_ = std::thread([this]() {work();});
    RCLCPP_INFO(get_logger(), "LIMO cloud ready: %zu plugins, %s in %s",
      processors_.size(), options_.flatten_to_bev ? "BEV" : "XYZ", bev_frame_.c_str());
  }

  ~LimoCloudNode() override
  {
    {
      std::lock_guard<std::mutex> guard(mutex_);
      stop_ = true;
    }
    ready_.notify_one();
    if (worker_.joinable()) {worker_.join();}
  }

private:
  bool corrected_fresh() const
  {
    return have_corrected_ &&
           std::chrono::duration<double>(Steady::now() - corrected_at_).count() <= corrected_timeout_;
  }

  void fallback(sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (corrected_fresh()) {return;}
    }
    try {
      const auto bytes = depth_bytes(*message);
      validate_image(*message, bytes);
      auto resized = std::make_shared<sensor_msgs::msg::Image>();
      resized->header = message->header;
      resized->encoding = message->encoding;
      resized->is_bigendian = message->is_bigendian;
      resized->width = fallback_width_;
      resized->height = fallback_height_;
      resized->step = resized->width * bytes;
      resized->data.resize(static_cast<size_t>(resized->step) * resized->height);
      const uint32_t crop = message->height * options_.input_crop_y_min;
      for (uint32_t y = 0; y < resized->height; ++y) {
        const uint32_t sy = crop + static_cast<uint64_t>(y) * (message->height - crop) / resized->height;
        for (uint32_t x = 0; x < resized->width; ++x) {
          const uint32_t sx = static_cast<uint64_t>(x) * message->width / resized->width;
          std::memcpy(resized->data.data() + static_cast<size_t>(y) * resized->step + x * bytes,
            message->data.data() + static_cast<size_t>(sy) * message->step + sx * bytes, bytes);
        }
      }
      std::lock_guard<std::mutex> guard(mutex_);
      if (!corrected_fresh()) {depth_ = std::move(resized);}
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
    }
  }

  void work()
  {
    Projector projector;
    auto reusable = std::make_shared<sensor_msgs::msg::PointCloud2>();
    while (true) {
      Frame frame;
      size_t overwritten;
      {
        std::unique_lock<std::mutex> guard(mutex_);
        ready_.wait(guard, [this]() {return stop_ || static_cast<bool>(pending_);});
        if (stop_) {return;}
        frame.labels = std::move(pending_);
        frame.depth = depth_;
        frame.camera_info = info_;
        overwritten = overwritten_;
      }
      const auto started = Steady::now();
      try {
        if (!frame.depth || !frame.camera_info) {
          throw std::runtime_error("Waiting for depth and RGB CameraInfo");
        }
        const auto & source = frame.labels->header.frame_id;
        if (source.empty() || (require_rgb_depth_ &&
          (source != frame.depth->header.frame_id || source != frame.camera_info->header.frame_id)))
        {
          throw std::runtime_error("Depth, classified image and CameraInfo must share the RGB frame");
        }
        if (max_stamp_delta_ > 0 && std::abs(
            (rclcpp::Time(frame.labels->header.stamp) - rclcpp::Time(frame.depth->header.stamp)).seconds()) >
          max_stamp_delta_)
        {
          throw std::runtime_error("Depth/classified image timestamps exceed max_rgb_depth_delta_sec");
        }
        frame.transform = tf_buffer_->lookupTransform(
          bev_frame_, source, rclcpp::Time(frame.labels->header.stamp));
        frame.options = options_;
        const auto pre_started = Steady::now();
        for (const auto & processor : processors_) {processor->process_images(frame);}
        const double pre_ms = elapsed_ms(pre_started);
        // A retained Python view owns its old buffer; never overwrite/reallocate it.
        if (reusable.use_count() != 1) {
          reusable = std::make_shared<sensor_msgs::msg::PointCloud2>();
        }
        frame.cloud = reusable;
        frame.cloud->fields.clear();
        const auto projection_started = Steady::now();
        projector.project(*frame.depth, *frame.labels, *frame.camera_info,
          frame.transform.transform, frame.options, *frame.cloud, frame.samples.get());
        const double projection_ms = elapsed_ms(projection_started);
        frame.cloud->header = frame.labels->header;
        frame.cloud->header.frame_id = bev_frame_;
        const auto post_started = Steady::now();
        for (const auto & processor : processors_) {
          processor->process_cloud(frame);
          if (!frame.cloud) {throw std::invalid_argument("Plugin removed the output cloud");}
          validate_cloud(*frame.cloud);
        }
        const double post_ms = elapsed_ms(post_started);
        frame.cloud->header = frame.labels->header;
        frame.cloud->header.frame_id = bev_frame_;
        const auto publish_started = Steady::now();
        publisher_->publish(*frame.cloud);
        const double publish_ms = elapsed_ms(publish_started);
        if (telemetry_) {
          durations_.push_back(elapsed_ms(started));
          if (durations_.size() > static_cast<size_t>(window_size_)) {durations_.pop_front();}
          if (++frames_ % log_interval_ == 0) {
            double sum = 0;
            for (double duration : durations_) {sum += duration;}
            RCLCPP_INFO(get_logger(),
              "cloud=%u points; projection=%.3f ms; plugins=%.3f/%.3f ms; publish=%.3f ms; "
              "worker mean=%.3f ms (%zu frames); overwritten=%zu",
              frame.cloud->width, projection_ms, pre_ms, post_ms, publish_ms,
              sum / durations_.size(), durations_.size(), overwritten);
          }
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Frame skipped: %s", error.what());
      }
    }
  }

  pluginlib::ClassLoader<Processor> loader_;
  std::vector<std::shared_ptr<Processor>> processors_;
  ProjectionOptions options_;
  bool require_rgb_depth_, telemetry_;
  double corrected_timeout_, max_stamp_delta_;
  int fallback_width_, fallback_height_, window_size_, log_interval_;
  std::string bev_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr labels_sub_, depth_sub_, fallback_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  std::mutex mutex_;
  std::condition_variable ready_;
  bool stop_{false}, have_corrected_{false};
  size_t overwritten_{0}, frames_{0};
  Steady::time_point corrected_at_{};
  sensor_msgs::msg::Image::ConstSharedPtr pending_, depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  std::deque<double> durations_;
  std::thread worker_;
};

rclcpp::Node::SharedPtr make_node(
  const rclcpp::NodeOptions & options, const std::string & name,
  const std::vector<rclcpp::Parameter> & defaults)
{
  return std::make_shared<LimoCloudNode>(name, options, defaults);
}

}  // namespace limo_cloud
