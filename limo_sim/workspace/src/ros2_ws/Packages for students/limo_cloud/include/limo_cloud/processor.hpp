#ifndef LIMO_CLOUD__PROCESSOR_HPP_
#define LIMO_CLOUD__PROCESSOR_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "limo_cloud/projector.hpp"

namespace limo_cloud
{

// One frame travels through every plugin in order. Inputs are borrowed until a
// plugin explicitly requests a writable copy. Named intermediates cross stages
// and languages without baking any semantic operation into the projector.
// Plugins may edit existing samples in place. To change a buffer's allocation,
// dimensions or point count, REPLACE its shared message pointer: earlier Python
// hooks may retain views of the old allocation. Never resize an exposed vector.
// Keep the source camera geometry and input headers unchanged. Processing is
// serial; a throwing hook drops the entire frame, including previous edits.
struct Frame
{
  sensor_msgs::msg::Image::ConstSharedPtr depth;
  sensor_msgs::msg::Image::ConstSharedPtr labels;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info;
  geometry_msgs::msg::TransformStamped transform;
  ProjectionOptions options;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  // nullptr projects the dense input grid; an empty vector projects zero points.
  std::shared_ptr<std::vector<PixelSample>> samples;
  std::unordered_map<std::string, sensor_msgs::msg::Image::SharedPtr> images;
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> clouds;
  // Adapter-owned state, e.g. a Python scratch dictionary shared across hooks.
  std::unordered_map<std::string, std::shared_ptr<void>> state;

  sensor_msgs::msg::Image & edit_depth()
  {
    if (!writable_depth_ || writable_depth_.get() != depth.get()) {
      writable_depth_ = std::make_shared<sensor_msgs::msg::Image>(*depth);
      depth = writable_depth_;
    }
    return *writable_depth_;
  }

  sensor_msgs::msg::Image & edit_labels()
  {
    if (!writable_labels_ || writable_labels_.get() != labels.get()) {
      writable_labels_ = std::make_shared<sensor_msgs::msg::Image>(*labels);
      labels = writable_labels_;
    }
    return *writable_labels_;
  }

private:
  sensor_msgs::msg::Image::SharedPtr writable_depth_, writable_labels_;
};

class Processor
{
public:
  virtual ~Processor() = default;
  virtual void configure(rclcpp::Node &, const std::string &) {}
  virtual void process_images(Frame &) {}
  virtual void process_cloud(Frame &) {}
};

// Factory for the optional embedded-Python adapter. No interpreter is started
// unless a python:module:Class instance is actually configured.
std::shared_ptr<Processor> make_python_processor(const std::string & specification);

}  // namespace limo_cloud

#endif  // LIMO_CLOUD__PROCESSOR_HPP_
