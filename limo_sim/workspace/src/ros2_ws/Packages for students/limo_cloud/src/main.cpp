#include "limo_cloud/node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(limo_cloud::make_node());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("limo_cloud"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
