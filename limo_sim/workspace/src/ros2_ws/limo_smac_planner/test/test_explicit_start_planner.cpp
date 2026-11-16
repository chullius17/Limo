#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include "limo_interfaces/srv/compute_path_with_start.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_loader.hpp"

class ExplicitStartPlannerTest : public ::testing::Test
{
protected:
  using Service = limo_interfaces::srv::ComputePathWithStart;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("explicit_start_test");
    map_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("explicit_start_test_costmap");
    map_->set_parameters(
      {rclcpp::Parameter("plugins", std::vector<std::string>{}),
        rclcpp::Parameter("global_frame", "map"),
        rclcpp::Parameter("footprint",
        "[[-0.161,-0.110],[-0.161,0.110],[0.161,0.110],[0.161,-0.110]]"),
        rclcpp::Parameter("footprint_padding", 0.0)});
    ASSERT_EQ(map_->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
    auto * grid = map_->getCostmap();
    grid->resizeMap(100, 100, 0.05, -2.5, -2.5);
    std::fill(grid->getCharMap(), grid->getCharMap() + 10000, nav2_costmap_2d::FREE_SPACE);
    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::GlobalPlanner>>(
      "nav2_core", "nav2_core::GlobalPlanner");
    planner_ = loader_->createSharedInstance("limo_smac_planner/ExplicitStartPlanner");
    planner_->configure(node_, "GridBased", std::make_shared<tf2_ros::Buffer>(node_->get_clock()), map_);
    planner_->activate();
    client_node_ = std::make_shared<rclcpp::Node>("explicit_start_client_test");
    client_ = client_node_->create_client<Service>("/compute_path_with_start");
    ASSERT_TRUE(client_->wait_for_service(std::chrono::seconds(2)));
    executor_->add_node(node_->get_node_base_interface());
    executor_->add_node(client_node_);
    request_ = std::make_shared<Service::Request>();
    request_->real_start.header.frame_id = "map";
    request_->real_start.pose.position.x = request_->real_start.pose.position.y = 0.025;
    request_->real_start.pose.orientation.w = 1.0;
    request_->start = request_->real_start;
    request_->goal = request_->real_start;
    request_->goal.pose.position.x = 1.525;
  }

  void TearDown() override
  {
    if (planner_) {
      planner_->deactivate();
      planner_->cleanup();
      planner_.reset();
    }
    loader_.reset();
    if (map_) {map_->on_cleanup(rclcpp_lifecycle::State());}
    executor_.reset();
    rclcpp::shutdown();
  }

  Service::Response::SharedPtr plan()
  {
    auto future = client_->async_send_request(request_);
    if (executor_->spin_until_future_complete(future, std::chrono::seconds(10)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("Explicit-start service timed out");
    }
    return future.get();
  }

  static void expectNoConnector(const Service::Response & result)
  {
    EXPECT_FALSE(result.start_rejected);
    EXPECT_FALSE(result.connector_required);
    EXPECT_FALSE(result.connector_checked);
    EXPECT_EQ(result.connector_end_index, 0U);
    EXPECT_DOUBLE_EQ(result.connector_length, 0.0);
    EXPECT_DOUBLE_EQ(result.connector_max_length, 0.0);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GlobalPlanner>> loader_;
  nav2_core::GlobalPlanner::Ptr planner_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<Service>::SharedPtr client_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  Service::Request::SharedPtr request_;
};

TEST_F(ExplicitStartPlannerTest, StartsAtQuantizedVirtualPoseAcrossAnUnbrokenBarrier)
{
  auto * grid = map_->getCostmap();
  grid->setCost(50, 50, nav2_costmap_2d::LETHAL_OBSTACLE);
  for (unsigned int x = 0; x < 100; ++x) {
    grid->setCost(x, 55, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  request_->start.pose.position.x = 0.019;
  request_->start.pose.position.y = 0.519;
  request_->start.pose.orientation.z = std::sin(0.005);
  request_->start.pose.orientation.w = std::cos(0.005);
  request_->goal.pose.position.y = 0.525;
  auto result = plan();
  ASSERT_TRUE(result->success) << result->error;
  expectNoConnector(*result);
  ASSERT_GT(result->path.poses.size(), 1U);
  EXPECT_NEAR(result->path.poses.front().pose.position.x, 0.025, 1e-9);
  EXPECT_NEAR(result->path.poses.front().pose.position.y, 0.525, 1e-9);
  EXPECT_NEAR(result->planning_start.pose.position.y, 0.525, 1e-9);
  EXPECT_NEAR(result->planning_start.pose.orientation.z, 0.0, 1e-9);
  for (const auto & pose : result->path.poses) {
    EXPECT_EQ(pose.header.stamp, result->path.header.stamp);
    EXPECT_EQ(pose.header.frame_id, "map");
  }
  // The measured pose need not lie in the map when a different start is selected.
  request_->real_start.pose.position.x = 10.0;
  result = plan();
  ASSERT_TRUE(result->success) << result->error;
  expectNoConnector(*result);
}

TEST_F(ExplicitStartPlannerTest, RejectsOccupiedStartFootprintAndGoal)
{
  auto * grid = map_->getCostmap();
  grid->setCost(50, 50, nav2_costmap_2d::LETHAL_OBSTACLE);
  auto result = plan();
  EXPECT_FALSE(result->success);
  expectNoConnector(*result);
  request_->start.pose.position.x = 0.525;
  // A free center is insufficient if the virtual start's footprint hits a wall.
  for (unsigned int x = 57; x <= 63; ++x) {
    grid->setCost(x, 52, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  result = plan();
  EXPECT_FALSE(result->success);
  EXPECT_EQ(result->error, "Quantized virtual start footprint is occupied");
  for (unsigned int x = 57; x <= 63; ++x) {grid->setCost(x, 52, nav2_costmap_2d::FREE_SPACE);}
  grid->setCost(80, 50, nav2_costmap_2d::LETHAL_OBSTACLE);
  result = plan();
  EXPECT_FALSE(result->success);
  expectNoConnector(*result);
  EXPECT_EQ(result->error, "Virtual start or goal is occupied");
}

TEST_F(ExplicitStartPlannerTest, ExactStartsUseNormalSmacAndUniquePlanStamps)
{
  const auto first = plan();
  ASSERT_TRUE(first->success) << first->error;
  expectNoConnector(*first);
  const auto second = plan();
  ASSERT_TRUE(second->success) << second->error;
  expectNoConnector(*second);
  EXPECT_GT(rclcpp::Time(second->path.header.stamp).nanoseconds(),
    rclcpp::Time(first->path.header.stamp).nanoseconds());
  EXPECT_NEAR(second->path.poses.back().pose.position.x, 1.525, 0.026);
}
