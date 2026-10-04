#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "dwb_core/exceptions.hpp"
#include "limo_dwb_critics/ackermann_mpc_controller.hpp"
#include "limo_dwb_critics/mppi_cost_critics.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace
{
geometry_msgs::msg::Pose2D pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::Pose2D result;
  result.x = x;
  result.y = y;
  result.theta = yaw;
  return result;
}

nav_2d_msgs::msg::Path2D straightPath()
{
  nav_2d_msgs::msg::Path2D result;
  result.header.frame_id = "odom";
  for (int i = 0; i <= 30; ++i) {
    result.poses.push_back(pose(0.05 * i, 0.0));
  }
  return result;
}
}  // namespace

class MppiCostsTest : public ::testing::Test
{
protected:
  virtual const char * parameterFile() const {return LIMO_MPC_PARAMS_PATH;}

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    auto options = rclcpp::NodeOptions().arguments(
      {"--ros-args", "--params-file", parameterFile()});
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", options);
    // Nav2's controller_server declares this before configuring its plugin;
    // mirror that lifecycle here so the YAML's 10 Hz override is visible.
    node_->declare_parameter("controller_frequency", rclcpp::ParameterValue(20.0));
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("mpc_test_costmap");
    costmap_->set_parameters({
      rclcpp::Parameter("plugins", std::vector<std::string>{}),
      rclcpp::Parameter("global_frame", "odom"),
      rclcpp::Parameter("footprint", "[[-0.161,-0.110],[-0.161,0.110],[0.161,0.110],[0.161,-0.110]]"),
      rclcpp::Parameter("footprint_padding", 0.0)});
    ASSERT_EQ(costmap_->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
    costmap_->getCostmap()->resizeMap(100, 100, 0.05, -2.5, -2.5);
    clearMap();
  }

  void TearDown() override
  {
    costmap_->on_cleanup(rclcpp_lifecycle::State());
    costmap_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void clearMap()
  {
    auto * map = costmap_->getCostmap();
    std::fill(map->getCharMap(), map->getCharMap() + map->getSizeInCellsX() *
      map->getSizeInCellsY(), nav2_costmap_2d::FREE_SPACE);
  }

  void setCost(double x, double y, unsigned char cost)
  {
    unsigned int mx, my;
    ASSERT_TRUE(costmap_->getCostmap()->worldToMap(x, y, mx, my));
    costmap_->getCostmap()->setCost(mx, my, cost);
  }

  template<typename Critic>
  void initialize(Critic & critic, std::string name)
  {
    std::string parent = "FollowPath";
    critic.initialize(node_, name, parent, costmap_);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
};

class MppiOnlineCostsTest : public MppiCostsTest
{
protected:
  const char * parameterFile() const override {return LIMO_MPC_SIM_PARAMS_PATH;}

  nav_2d_msgs::msg::Path2D pathBehindCube(double length = 2.4)
  {
    auto path = straightPath();
    for (int i = 31; i <= static_cast<int>(std::lround(length / 0.05)); ++i) {
      path.poses.push_back(pose(i * 0.05, 0.0));
    }
    return path;
  }

  void addCube()
  {
    auto * map = costmap_->getCostmap();
    for (unsigned int y = 0; y < map->getSizeInCellsY(); ++y) {
      for (unsigned int x = 0; x < map->getSizeInCellsX(); ++x) {
        double wx, wy;
        map->mapToWorld(x, y, wx, wy);
        if (wx >= 0.8 && wx < 1.8 && wy >= -0.5 && wy < 0.5) {
          map->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }
};

class InspectableMppiPathCritic : public limo_dwb_critics::MppiPathCritic
{
public:
  bool detouring() const {return guidance_active_;}
};

TEST_F(MppiOnlineCostsTest, PreservesCollisionFreePlannedPathBesideCube)
{
  addCube();
  auto path = pathBehindCube();
  for (auto & point : path.poses) {
    point.y = 0.66;
  }
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  ASSERT_TRUE(critic.prepare(
    pose(0.0, 0.66), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0.66), path));
  // Rectangle half-width is 0.11 m: this reference leaves 0.05 m of space.
  // The old circumscribed-circle margin falsely replaced this valid plan.
  EXPECT_FALSE(critic.detouring());
  dwb_msgs::msg::Trajectory2D stopped;
  stopped.poses = {pose(0, 0.66), pose(0, 0.66)};
  dwb_msgs::msg::Trajectory2D forward;
  forward.poses = {pose(0, 0.66), pose(0.5, 0.66)};
  EXPECT_LT(critic.scoreTrajectory(forward), critic.scoreTrajectory(stopped));
}

TEST_F(MppiOnlineCostsTest, FinishesDetourBeforeRestoringReferenceAlignment)
{
  addCube();
  auto path = pathBehindCube();
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  ASSERT_TRUE(critic.prepare(
    pose(0.1, 0), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), path));
  ASSERT_TRUE(critic.detouring());
  clearMap();  // One observation update temporarily removes the obstruction.
  ASSERT_TRUE(critic.prepare(
    pose(0.4, 0.8), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), path));
  EXPECT_TRUE(critic.detouring());
  ASSERT_TRUE(critic.prepare(
    pose(0.6, 0.05), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), path));
  EXPECT_FALSE(critic.detouring());
  addCube();
  ASSERT_TRUE(critic.prepare(
    pose(0.1, 0), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), path));
  ASSERT_TRUE(critic.detouring());
  clearMap();
  critic.reset();  // A new FollowPath action must not inherit the old detour.
  ASSERT_TRUE(critic.prepare(
    pose(0.4, 0.8), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), path));
  EXPECT_FALSE(critic.detouring());
}

TEST_F(MppiOnlineCostsTest, DetourHeadingFollowsMotionInForwardAndReverse)
{
  addCube();
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  ASSERT_TRUE(critic.prepare(
    pose(0.1, 0), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), pathBehindCube()));
  ASSERT_TRUE(critic.detouring());
  const double angle = std::acos(-1.0) / 4.0;
  dwb_msgs::msg::Trajectory2D forward;
  forward.poses = {pose(0.1, 0.1, angle), pose(0.2, 0.2, angle), pose(0.3, 0.3, angle)};
  auto reverse = forward;
  for (auto & point : reverse.poses) {
    point.theta -= std::acos(-1.0);
  }
  // Same geometric motion and footprint, opposite chassis orientations.
  // Reverse distance/direction penalties belong to MPC effort, not this
  // geometric route-direction cost.
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(forward), critic.scoreTrajectory(reverse));
}

TEST_F(MppiOnlineCostsTest, OnlineCubeRewardsDetourAndThenRestoresOriginalPathTracking)
{
  limo_dwb_critics::MppiPathCritic critic;
  initialize(critic, "MppiPath");
  const auto path = pathBehindCube();
  dwb_msgs::msg::Trajectory2D stopped;
  stopped.poses = {pose(0.35, 0), pose(0.35, 0)};
  dwb_msgs::msg::Trajectory2D detour;
  detour.poses = {pose(0.35, 0), pose(0.4, 0.6, 0.4), pose(0.6, 0.75, 0.3)};
  ASSERT_TRUE(critic.prepare(
    pose(0.35, 0), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), path));
  EXPECT_GT(critic.scoreTrajectory(detour), critic.scoreTrajectory(stopped));
  addCube();  // The stored/global path has not changed.
  ASSERT_TRUE(critic.prepare(
    pose(0.35, 0), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), path));
  EXPECT_LT(critic.scoreTrajectory(detour), critic.scoreTrajectory(stopped));
  clearMap();
  ASSERT_TRUE(critic.prepare(
    pose(0.35, 0), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), path));
  EXPECT_GT(critic.scoreTrajectory(detour), critic.scoreTrajectory(stopped));
}

TEST_F(MppiOnlineCostsTest, MpcPassesCubeDiscoveredAfterStartingWithoutReplacingGlobalPlan)
{
  limo_dwb_critics::AckermannMPCController controller;
  auto tf = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  controller.configure(node_, "FollowPath", tf, costmap_);
  controller.activate();
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  for (const auto & point : pathBehindCube(3.5).poses) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = path.header;
    stamped.pose.position.x = point.x;
    stamped.pose.orientation.w = 1.0;
    path.poses.push_back(stamped);
  }
  controller.setPlan(path);
  limo_dwb_critics::MpcConfig config;
  config.wheelbase = 0.24;
  config.rear_axle_to_base = 0.12;
  config.min_turning_radius = 0.55;
  config.steering_rate = 0.5;
  limo_dwb_critics::SamplingMpc plant(config);
  limo_dwb_critics::MpcState state;
  double lateral_excursion = 0.0;
  double max_cycle_ms = 0.0;
  int direction_changes = 0;
  int previous_direction = 0;
  double reverse_distance = 0.0;
  for (int cycle = 0; cycle < 480 && std::hypot(state.x - 3.5, state.y) > 0.15; ++cycle) {
    costmap_->getCostmap()->updateOrigin(state.x - 2.5, state.y - 2.5);
    clearMap();
    if (cycle >= 10) {
      addCube();  // Path execution continues; there is no setPlan/reset here.
    }
    geometry_msgs::msg::PoseStamped current;
    current.header.frame_id = "odom";
    current.pose.position.x = state.x;
    current.pose.position.y = state.y;
    current.pose.orientation.z = std::sin(state.yaw / 2.0);
    current.pose.orientation.w = std::cos(state.yaw / 2.0);
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = state.control.velocity;
    velocity.angular.z = plant.yawRate(state.control);
    const auto begin = std::chrono::steady_clock::now();
    geometry_msgs::msg::TwistStamped command;
    ASSERT_NO_THROW(command = controller.computeVelocityCommands(current, velocity));
    max_cycle_ms = std::max(max_cycle_ms, std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count());
    limo_dwb_critics::MpcControl target;
    target.velocity = command.twist.linear.x;
    const int direction = target.velocity > 0.01 ? 1 : target.velocity < -0.01 ? -1 : 0;
    if (direction) {
      direction_changes += previous_direction && direction != previous_direction;
      previous_direction = direction;
    }
    reverse_distance += std::max(0.0, -target.velocity) * config.dt;
    if (std::abs(target.velocity) > 1e-6) {
      target.steering = std::atan(config.wheelbase * command.twist.angular.z / target.velocity);
    }
    state = plant.rollout(state, std::vector<limo_dwb_critics::MpcControl>(
        config.time_steps, target)).states[1];
    lateral_excursion = std::max(lateral_excursion, std::abs(state.y));
    if (cycle >= 10) {
      // Independent separating-axis check for the oriented rectangular robot
      // footprint and the physical cube, not the critic's cell contour test.
      const double dx = state.x - 1.3;
      const double dy = state.y;
      const double c = std::cos(state.yaw);
      const double s = std::sin(state.yaw);
      const double sum = std::abs(c) + std::abs(s);
      EXPECT_TRUE(
        std::abs(dx) > 0.5 + 0.161 * std::abs(c) + 0.110 * std::abs(s) ||
        std::abs(dy) > 0.5 + 0.161 * std::abs(s) + 0.110 * std::abs(c) ||
        std::abs(dx * c + dy * s) > 0.161 + 0.5 * sum ||
        std::abs(-dx * s + dy * c) > 0.110 + 0.5 * sum);
    }
  }
  RecordProperty("max_cycle_ms", std::to_string(max_cycle_ms));
  RecordProperty("direction_changes", direction_changes);
  RecordProperty("reverse_distance", std::to_string(reverse_distance));
  RecordProperty("final_x", std::to_string(state.x));
  RecordProperty("final_y", std::to_string(state.y));
  RecordProperty("final_yaw", std::to_string(state.yaw));
  RecordProperty("final_speed", std::to_string(state.control.velocity));
  RecordProperty("lateral_excursion", std::to_string(lateral_excursion));
  EXPECT_GT(state.x, 3.25);
  EXPECT_LT(std::abs(state.y), 0.20);
  EXPECT_GT(lateral_excursion, 0.70);
  // At most one repositioning maneuver; repeated forward/reverse jerks fail.
  EXPECT_LE(direction_changes, 2);
  EXPECT_LT(reverse_distance, 0.30);
  controller.deactivate();
  controller.cleanup();
}

TEST_F(MppiOnlineCostsTest, CapturedGazeboCostmapMakesProgressWithoutRepeatedReversals)
{
  std::ifstream input(LIMO_MPC_SNAPSHOT_PATH);
  ASSERT_TRUE(input.good());
  std::string comment;
  std::getline(input, comment);
  unsigned int width, height;
  double resolution, ox, oy, measured_yaw_rate;
  limo_dwb_critics::MpcState state;
  input >> width >> height >> resolution >> ox >> oy >> state.x >> state.y >>
    state.yaw >> state.control.velocity >> measured_yaw_rate;
  costmap_->getCostmap()->resizeMap(width, height, resolution, ox, oy);
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  std::size_t count;
  input >> count;
  for (std::size_t i = 0; i < count; ++i) {
    geometry_msgs::msg::PoseStamped point;
    point.header = path.header;
    double angle;
    input >> point.pose.position.x >> point.pose.position.y >> angle;
    point.pose.orientation.z = std::sin(angle / 2.0);
    point.pose.orientation.w = std::cos(angle / 2.0);
    path.poses.push_back(point);
  }
  input >> count;
  std::size_t cell = 0;
  for (std::size_t i = 0; i < count; ++i) {
    std::size_t length;
    int cost;
    input >> length >> cost;
    ASSERT_LE(cell + length, static_cast<std::size_t>(width) * height);
    std::fill_n(costmap_->getCostmap()->getCharMap() + cell, length, cost);
    cell += length;
  }
  ASSERT_EQ(cell, static_cast<std::size_t>(width) * height);
  ASSERT_TRUE(input.good());
  const auto nearest = [&](const limo_dwb_critics::MpcState & current) {
      std::size_t result = 0;
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < path.poses.size(); ++i) {
        const auto & point = path.poses[i].pose.position;
        const double d = std::hypot(point.x - current.x, point.y - current.y);
        if (d < best) {
          best = d;
          result = i;
        }
      }
      return result;
    };
  const std::size_t start = nearest(state);
  limo_dwb_critics::AckermannMPCController controller;
  auto tf = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  controller.configure(node_, "FollowPath", tf, costmap_);
  controller.activate();
  controller.setPlan(path);
  limo_dwb_critics::MpcConfig config;
  config.wheelbase = 0.24;
  config.rear_axle_to_base = 0.12;
  config.min_turning_radius = 0.55;
  config.steering_rate = 0.5;
  limo_dwb_critics::SamplingMpc plant(config);
  int changes = 0;
  int previous_direction = 0;
  double reverse_distance = 0.0;
  for (int cycle = 0; cycle < 300 && nearest(state) < start + 16; ++cycle) {
    geometry_msgs::msg::PoseStamped current;
    current.header = path.header;
    current.pose.position.x = state.x;
    current.pose.position.y = state.y;
    current.pose.orientation.z = std::sin(state.yaw / 2.0);
    current.pose.orientation.w = std::cos(state.yaw / 2.0);
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = state.control.velocity;
    velocity.angular.z = plant.yawRate(state.control);
    geometry_msgs::msg::TwistStamped command;
    ASSERT_NO_THROW(command = controller.computeVelocityCommands(current, velocity));
    limo_dwb_critics::MpcControl target;
    target.velocity = command.twist.linear.x;
    if (std::abs(target.velocity) > 1e-6) {
      target.steering = std::atan(config.wheelbase * command.twist.angular.z / target.velocity);
    }
    const int direction = target.velocity > 0.01 ? 1 : target.velocity < -0.01 ? -1 : 0;
    if (direction) {
      changes += previous_direction && direction != previous_direction;
      previous_direction = direction;
    }
    reverse_distance += std::max(0.0, -target.velocity) * config.dt;
    state = plant.rollout(state, std::vector<limo_dwb_critics::MpcControl>(
        config.time_steps, target)).states[1];
  }
  RecordProperty("path_progress", std::to_string(
      static_cast<int>(nearest(state)) - static_cast<int>(start)));
  RecordProperty("direction_changes", changes);
  RecordProperty("reverse_distance", std::to_string(reverse_distance));
  EXPECT_GE(nearest(state), start + 16);
  EXPECT_LE(changes, 1);
  EXPECT_LT(reverse_distance, 0.05);
  controller.deactivate();
  controller.cleanup();
}

TEST_F(MppiCostsTest, NormalizesObstacleCostIndependentlyOfHorizonAndResolution)
{
  limo_dwb_critics::MppiObstacleCritic critic;
  initialize(critic, "MppiObstacle");
  ASSERT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(2, 0), straightPath()));
  setCost(0.0, 0.0, 127);
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.assign(11, pose(0, 0));
  EXPECT_DOUBLE_EQ(critic.getScale(), 6.0);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(trajectory) * critic.getScale(), 3.0);
  trajectory.poses.assign(51, pose(0, 0));
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(trajectory) * critic.getScale(), 3.0);
  costmap_->getCostmap()->resizeMap(50, 50, 0.1, -2.5, -2.5);
  clearMap();
  setCost(0.0, 0.0, 127);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(trajectory) * critic.getScale(), 3.0);
}

TEST_F(MppiCostsTest, NearGoalDisablesRepulsionButNeverCollisionRejection)
{
  limo_dwb_critics::MppiObstacleCritic critic;
  initialize(critic, "MppiObstacle");
  ASSERT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(0.3, 0), straightPath()));
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.assign(5, pose(0, 0));
  setCost(0.0, 0.0, 200);
  EXPECT_NEAR(critic.scoreTrajectory(trajectory), 200.0 / 254.0, 1e-12);
  ASSERT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(0.05, 0), straightPath()));
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(trajectory), 0.0);
  setCost(0.0, 0.0, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_NEAR(critic.scoreTrajectory(trajectory), 300.0 / 254.0, 1e-12);
  setCost(0.0, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(critic.scoreTrajectory(trajectory), dwb_core::IllegalTrajectoryException);
  clearMap();
  setCost(0.15, 0.1, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(critic.scoreTrajectory(trajectory), dwb_core::IllegalTrajectoryException);
  clearMap();
  trajectory.poses.back().x = 3.0;
  EXPECT_THROW(critic.scoreTrajectory(trajectory), dwb_core::IllegalTrajectoryException);
}

TEST_F(MppiCostsTest, UsesHumbleGoalWeightsAndWrappedYawError)
{
  limo_dwb_critics::MppiPathCritic critic;
  initialize(critic, "MppiPath");
  ASSERT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(0.3, 0, -3.1), straightPath()));
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses = {pose(0, 0), pose(0.1, 0, 3.1), pose(0.2, 0, 3.1)};
  const double wrapped_angle = 2.0 * std::acos(-1.0) - 6.2;
  EXPECT_NEAR(critic.scoreTrajectory(trajectory), 5.0 * 0.15 + 3.0 * wrapped_angle, 1e-12);
}

TEST_F(MppiCostsTest, RewardsProgressAndRelaxesAlignmentWhenPathBlocked)
{
  limo_dwb_critics::MppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto path = straightPath();
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(2, 0), path);
    };
  ASSERT_TRUE(prepare());
  dwb_msgs::msg::Trajectory2D stopped;
  stopped.poses = {pose(0, 0), pose(0, 0), pose(0, 0)};
  dwb_msgs::msg::Trajectory2D progress;
  progress.poses = {pose(0, 0), pose(0.2, 0), pose(0.4, 0)};
  EXPECT_LT(critic.scoreTrajectory(progress), critic.scoreTrajectory(stopped));
  dwb_msgs::msg::Trajectory2D detour;
  detour.poses = {pose(0, 0), pose(0.2, 0.2), pose(0.4, 0.2)};
  const double clear_path_cost = critic.scoreTrajectory(detour);
  for (int i = 2; i < 20; ++i) {
    setCost(i * 0.05, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  ASSERT_TRUE(prepare());
  EXPECT_LT(critic.scoreTrajectory(detour), clear_path_cost);
}

TEST_F(MppiCostsTest, RealYamlConfiguresAndRunsTheFullDwbControllerPipeline)
{
  limo_dwb_critics::AckermannMPCController controller;
  auto tf = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  ASSERT_NO_THROW(controller.configure(node_, "FollowPath", tf, costmap_));
  controller.activate();
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  for (int i = 0; i <= 40; ++i) {
    geometry_msgs::msg::PoseStamped point;
    point.header = path.header;
    point.pose.position.x = i * 0.05;
    point.pose.orientation.w = 1.0;
    path.poses.push_back(point);
  }
  controller.setPlan(path);
  geometry_msgs::msg::PoseStamped current;
  current.header.frame_id = "odom";
  current.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist velocity;
  const auto begin = std::chrono::steady_clock::now();
  const auto command = controller.computeVelocityCommands(current, velocity);
  const auto duration = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - begin).count();
  RecordProperty("full_pipeline_ms", duration);
  EXPECT_GT(command.twist.linear.x, 0.0);
  EXPECT_LE(command.twist.linear.x, 1.3 * 0.10 + 1e-12);
  EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
  EXPECT_LE(std::abs(command.twist.angular.z), command.twist.linear.x / 0.462 + 1e-12);
  controller.deactivate();
  controller.cleanup();
}
