#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include "limo_interfaces/srv/compute_path_with_start.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_loader.hpp"

TEST(ExplicitStartPlanner, RecoversAnOccupiedStartWithAnUncheckedDubinsPrefix)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("explicit_start_test");
  node->declare_parameter("GridBased.connect_adjusted_start", true);
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("explicit_start_test_costmap");
  map->set_parameters(
    {rclcpp::Parameter("plugins", std::vector<std::string>{}),
      rclcpp::Parameter("global_frame", "map"),
      rclcpp::Parameter(
        "footprint",
        "[[-0.161,-0.110],[-0.161,0.110],[0.161,0.110],[0.161,-0.110]]"),
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
  request->start.pose.position.x = 0.519;
  request->start.pose.position.y = 0.019;
  request->start.pose.orientation.z = std::sin(0.005);
  request->start.pose.orientation.w = std::cos(0.005);
  request->goal = request->real_start;
  request->goal.pose.position.x = 1.525;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(client_node);
  auto future = client->async_send_request(request);
  ASSERT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds(3)),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto result = future.get();
  ASSERT_TRUE(result->success) << result->error;
  EXPECT_TRUE(result->connector_checked);
  EXPECT_TRUE(result->connector_required);
  EXPECT_FALSE(result->start_rejected);
  EXPECT_NEAR(result->planning_start.pose.position.x, 0.525, 1e-9);
  EXPECT_NEAR(result->planning_start.pose.position.y, 0.025, 1e-9);
  EXPECT_NEAR(result->planning_start.pose.orientation.z, 0.0, 1e-9);
  EXPECT_NEAR(result->connector_length, 0.5, 1e-9);
  EXPECT_DOUBLE_EQ(result->connector_max_length, 2.0);
  ASSERT_GT(result->connector_end_index, 0U);
  ASSERT_LT(result->connector_end_index, result->path.poses.size() - 1);
  EXPECT_DOUBLE_EQ(result->path.poses.front().pose.position.x, 0.025);
  EXPECT_NEAR(result->path.poses[result->connector_end_index].pose.position.x, 0.525, 0.026);
  EXPECT_NEAR(result->path.poses.back().pose.position.x, 1.525, 0.026);
  request->start = request->real_start;
  future = client->async_send_request(request);
  ASSERT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds(3)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(future.get()->success);  // No exception on an ordinary SMAC path.

  // An overlong connector is rejected before SMAC or even goal validation.
  // The old ordering would report the out-of-map goal and hide this failure.
  request->start = request->real_start;
  request->start.pose.position.x = -0.475;
  request->goal.pose.position.x = 10.0;
  future = client->async_send_request(request);
  ASSERT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds(3)),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto rejected = future.get();
  EXPECT_FALSE(rejected->success);
  EXPECT_TRUE(rejected->start_rejected);
  EXPECT_TRUE(rejected->connector_checked);
  EXPECT_TRUE(rejected->connector_required);
  EXPECT_GT(rejected->connector_length, rejected->connector_max_length);
  EXPECT_NEAR(rejected->planning_start.pose.position.x, -0.475, 1e-9);
  EXPECT_EQ(rejected->error, "Initial Dubins connector exceeds start_connector_max_length");
  EXPECT_TRUE(rejected->path.poses.empty());

  // Exact starts retain ordinary obstacle checks and need no connector.
  grid->setCost(50, 50, nav2_costmap_2d::FREE_SPACE);
  request->start = request->real_start;
  request->goal.pose.position.x = 1.525;
  future = client->async_send_request(request);
  ASSERT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds(3)),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto exact = future.get();
  ASSERT_TRUE(exact->success) << exact->error;
  EXPECT_FALSE(exact->start_rejected);
  EXPECT_TRUE(exact->connector_checked);
  EXPECT_FALSE(exact->connector_required);
  EXPECT_DOUBLE_EQ(exact->connector_length, 0.0);
  EXPECT_EQ(exact->connector_end_index, 0U);

  // A failed goal must not poison the start cache. Diagnostics survive it.
  request->start.pose.position.x = 0.525;
  grid->setCost(80, 50, nav2_costmap_2d::LETHAL_OBSTACLE);
  future = client->async_send_request(request);
  ASSERT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds(3)),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto bad_goal = future.get();
  EXPECT_FALSE(bad_goal->success);
  EXPECT_FALSE(bad_goal->start_rejected);
  EXPECT_TRUE(bad_goal->connector_checked);
  EXPECT_NEAR(bad_goal->connector_length, 0.5, 1e-9);
  planner->deactivate();
  planner->cleanup();
  planner.reset();
  map->on_cleanup(rclcpp_lifecycle::State());
  rclcpp::shutdown();
}

TEST(ExplicitStartPlanner, PlansFromALateralVirtualStartWithoutConnectingRobot)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("virtual_start_test");
  // Even a tiny connector budget must not constrain a free virtual start.
  node->declare_parameter("GridBased.start_connector_max_length", 0.01);
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("virtual_start_test_costmap");
  map->set_parameters(
  {
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("global_frame", "map"),
    rclcpp::Parameter("footprint", "[[-0.161,-0.110],[-0.161,0.110],[0.161,0.110],[0.161,-0.110]]"),
    rclcpp::Parameter("footprint_padding", 0.0)});
  ASSERT_EQ(map->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
  auto * grid = map->getCostmap();
  grid->resizeMap(100, 100, 0.05, -2.5, -2.5);
  std::fill(grid->getCharMap(), grid->getCharMap() + 10000, nav2_costmap_2d::FREE_SPACE);
  grid->setCost(50, 50, nav2_costmap_2d::LETHAL_OBSTACLE);
  // An unbroken barrier separates the measured robot from the virtual start.
  for (unsigned int x = 0; x < 100; ++x) {
    grid->setCost(x, 55, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core", "nav2_core::GlobalPlanner");
  auto planner = loader.createSharedInstance("limo_smac_planner/ExplicitStartPlanner");
  planner->configure(node, "GridBased", tf, map);
  EXPECT_FALSE(node->get_parameter("GridBased.connect_adjusted_start").as_bool());
  planner->activate();
  auto client_node = std::make_shared<rclcpp::Node>("virtual_start_client_test");
  auto client = client_node->create_client<limo_interfaces::srv::ComputePathWithStart>(
    "/compute_path_with_start");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto request = std::make_shared<limo_interfaces::srv::ComputePathWithStart::Request>();
  request->real_start.header.frame_id = "map";
  request->real_start.pose.position.x = request->real_start.pose.position.y = 0.025;
  request->real_start.pose.orientation.w = 1.0;
  request->start = request->real_start;
  request->start.pose.position.y = 0.525;
  request->goal = request->start;
  request->goal.pose.position.x = 1.525;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(client_node);
  auto future = client->async_send_request(request);
  ASSERT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds(3)),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto result = future.get();
  ASSERT_TRUE(result->success) << result->error;
  EXPECT_FALSE(result->start_rejected);
  EXPECT_FALSE(result->connector_required);
  EXPECT_FALSE(result->connector_checked);
  EXPECT_EQ(result->connector_end_index, 0U);
  ASSERT_FALSE(result->path.poses.empty());
  EXPECT_NEAR(result->path.poses.front().pose.position.x, 0.025, 1e-9);
  EXPECT_NEAR(result->path.poses.front().pose.position.y, 0.525, 1e-9);
  EXPECT_NEAR(result->planning_start.pose.position.y, 0.525, 1e-9);
  EXPECT_NEAR(result->path.poses.back().pose.position.x, 1.525, 0.026);
  for (const auto & pose : result->path.poses) {
    EXPECT_GT(pose.pose.position.y, 0.3);  // No fabricated robot-to-start prefix.
  }

  // The measured pose is only context; it may even lie outside the map.
  request->real_start.pose.position.x = 10.0;
  future = client->async_send_request(request);
  ASSERT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds(3)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(future.get()->success);

  // The virtual start itself must still have a collision-free full footprint.
  grid->setCost(53, 62, nav2_costmap_2d::LETHAL_OBSTACLE);
  future = client->async_send_request(request);
  ASSERT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds(3)),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto occupied = future.get();
  EXPECT_FALSE(occupied->success);
  EXPECT_FALSE(occupied->start_rejected);
  EXPECT_EQ(occupied->error, "Quantized virtual start footprint is occupied");
  planner->deactivate();
  planner->cleanup();
  planner.reset();
  map->on_cleanup(rclcpp_lifecycle::State());
  rclcpp::shutdown();
}
