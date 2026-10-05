#include <gtest/gtest.h>
#include <chrono>
#include "limo_interfaces/srv/compute_path_with_start.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_loader.hpp"

TEST(ExplicitStartPlanner, RecoversAnOccupiedStartWithAnUncheckedDubinsPrefix)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("explicit_start_test");
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("explicit_start_test_costmap");
  map->set_parameters({rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("global_frame", "map"),
    rclcpp::Parameter("footprint", "[[-0.161,-0.110],[-0.161,0.110],[0.161,0.110],[0.161,-0.110]]"),
    rclcpp::Parameter("footprint_padding", 0.0)});
  ASSERT_EQ(map->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
  auto * grid = map->getCostmap();
  grid->resizeMap(100, 100, 0.05, -2.5, -2.5);
  std::fill(grid->getCharMap(), grid->getCharMap() + 10000, nav2_costmap_2d::FREE_SPACE);
  grid->setCost(50, 50, nav2_costmap_2d::LETHAL_OBSTACLE);
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core", "nav2_core::GlobalPlanner");
  auto planner = loader.createSharedInstance("limo_smac_planner/ExplicitStartPlanner");
  planner->configure(node, "GridBased", tf, map);
  planner->activate();
  auto client_node = std::make_shared<rclcpp::Node>("explicit_start_client_test");
  auto client = client_node->create_client<limo_interfaces::srv::ComputePathWithStart>(
    "/compute_path_with_start");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto request = std::make_shared<limo_interfaces::srv::ComputePathWithStart::Request>();
  request->real_start.header.frame_id = "map";
  request->real_start.pose.position.x = request->real_start.pose.position.y = 0.025;
  request->real_start.pose.orientation.w = 1.0;
  request->start = request->real_start;
  request->start.pose.position.x = 0.525;
  request->goal = request->real_start;
  request->goal.pose.position.x = 1.525;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(client_node);
  auto future = client->async_send_request(request);
  ASSERT_EQ(executor.spin_until_future_complete(future, std::chrono::seconds(3)),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto result = future.get();
  ASSERT_TRUE(result->success) << result->error;
  ASSERT_GT(result->connector_end_index, 0U);
  ASSERT_LT(result->connector_end_index, result->path.poses.size() - 1);
  EXPECT_DOUBLE_EQ(result->path.poses.front().pose.position.x, 0.025);
  EXPECT_NEAR(result->path.poses[result->connector_end_index].pose.position.x, 0.525, 0.026);
  EXPECT_NEAR(result->path.poses.back().pose.position.x, 1.525, 0.026);
  request->start = request->real_start;
  future = client->async_send_request(request);
  ASSERT_EQ(executor.spin_until_future_complete(future, std::chrono::seconds(3)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(future.get()->success);  // No exception on an ordinary SMAC path.
  planner->deactivate();
  planner->cleanup();
  planner.reset();
  map->on_cleanup(rclcpp_lifecycle::State());
  rclcpp::shutdown();
}
