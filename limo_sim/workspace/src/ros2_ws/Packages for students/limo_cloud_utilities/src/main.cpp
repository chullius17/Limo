#include "limo_cloud/node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    // Only the example chooses semantic processors. The shared C++ host handles
    // subscribed classified/depth images, calibration, TF and projection.
    const std::vector<rclcpp::Parameter> defaults{
      {"plugins", std::vector<std::string>{
          "masks", "sampling", "debug", "boardwalk", "class_filter", "voxel"}},
      {"masks.type", "python:limo_cloud_utilities.image_plugins:BoundaryMasks"},
      {"sampling.type", "python:limo_cloud_utilities.image_plugins:PixelSampling"},
      {"debug.type", "python:limo_cloud_utilities.debug:DebugOutputs"},
      {"boardwalk.type", "python:limo_cloud_utilities.cloud_plugins:Boardwalk"},
      {"class_filter.type", "python:limo_cloud_utilities.cloud_plugins:RoadBoardwalkOnly"},
      {"voxel.type", "python:limo_cloud_utilities.cloud_plugins:MetricVoxel"},
    };
    rclcpp::spin(limo_cloud::make_node(rclcpp::NodeOptions(), "limo_cloud_utilities", defaults));
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("limo_cloud_utilities"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
