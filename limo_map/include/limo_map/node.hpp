#ifndef LIMO_MAP__NODE_HPP_
#define LIMO_MAP__NODE_HPP_
#include "rclcpp/rclcpp.hpp"

namespace limo_map
{
rclcpp::Node::SharedPtr make_node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
}  // namespace limo_map
#endif  // LIMO_MAP__NODE_HPP_
