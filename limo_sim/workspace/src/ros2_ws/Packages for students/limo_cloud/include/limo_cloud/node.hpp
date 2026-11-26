#ifndef LIMO_CLOUD__NODE_HPP_
#define LIMO_CLOUD__NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace limo_cloud
{
// Share the optimized RGBD projection host with dedicated processing executables.
// CLI/YAML overrides win; defaults fill only parameters that are not already set.
// An empty defaults list leaves the generic node's plugin chain empty.
rclcpp::Node::SharedPtr make_node(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
  const std::string & name = "limo_cloud",
  const std::vector<rclcpp::Parameter> & defaults = {});
}  // namespace limo_cloud

#endif  // LIMO_CLOUD__NODE_HPP_
