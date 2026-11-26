#include "limo_map/node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(limo_map::make_node());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("limo_map"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
