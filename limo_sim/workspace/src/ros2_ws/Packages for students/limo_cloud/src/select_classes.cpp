#include "limo_cloud/processor.hpp"

#include <array>
#include <cstring>
#include <stdexcept>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace limo_cloud
{
class SelectClasses : public Processor
{
public:
  void configure(rclcpp::Node & node, const std::string & name) override
  {
    const auto key = name + ".keep_classes";
    if (!node.has_parameter(key)) {node.declare_parameter(key, std::vector<int64_t>{});}
    const auto classes = node.get_parameter(key).as_integer_array();
    enabled_ = !classes.empty();
    keep_.fill(false);
    for (auto id : classes) {
      if (id < 0 || id > 255) {throw std::invalid_argument("keep_classes must be in [0, 255]");}
      keep_[id] = true;
    }
  }

  void process_cloud(Frame & frame) override
  {
    if (!enabled_) {return;}
    validate_cloud(*frame.cloud);
    // Replacement preserves buffers previously exported to NumPy.
    auto selected = std::make_shared<sensor_msgs::msg::PointCloud2>(*frame.cloud);
    size_t count = 0;
    for (size_t i = 0; i < selected->width; ++i) {
      const auto * point = selected->data.data() + i * sizeof(Point);
      if (keep_[point[12]]) {
        if (i != count) {std::memcpy(selected->data.data() + count * sizeof(Point), point, sizeof(Point));}
        ++count;
      }
    }
    selected->width = count;
    selected->row_step = count * sizeof(Point);
    selected->data.resize(selected->row_step);
    frame.cloud = std::move(selected);
  }

private:
  bool enabled_{false};
  std::array<bool, 256> keep_{};
};
}  // namespace limo_cloud

PLUGINLIB_EXPORT_CLASS(limo_cloud::SelectClasses, limo_cloud::Processor)
