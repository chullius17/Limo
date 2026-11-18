#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <fstream>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "limo_dwb_critics/ackermann_mpc_controller.hpp"
#include "limo_dwb_critics/guidance_unavailable.hpp"
#include "limo_dwb_critics/mppi_cost_critics.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav_2d_utils/conversions.hpp"

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

void expectLookahead(const geometry_msgs::msg::Pose2D * point,
  double x, double y = 0.0, double yaw = 0.0, double x_tolerance = 1e-9)
{
  ASSERT_NE(point, nullptr);
  EXPECT_NEAR(point->x, x, x_tolerance);
  EXPECT_NEAR(point->y, y, 1e-9);
  EXPECT_NEAR(point->theta, yaw, 1e-9);
}
}  // namespace

class MppiCostsTest : public ::testing::Test
{
protected:
  virtual const char * parameterFile() const {return LIMO_MPC_PARAMS_PATH;}
  virtual bool structuredPipeline() const {return false;}

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    auto options = rclcpp::NodeOptions().arguments(
      {"--ros-args", "--params-file", parameterFile()});
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", options);
    // Nav2's controller_server declares this before configuring its plugin;
    // mirror that lifecycle here so the YAML's frequency override is visible.
    node_->declare_parameter("controller_frequency", rclcpp::ParameterValue(20.0));
    // Synthetic regression cycles must not mix with runtime timing captures.
    node_->declare_parameter("FollowPath.MPC.Timing.csv_directory", std::string());
    node_->set_parameter(rclcpp::Parameter("FollowPath.MPC.Timing.csv_directory", std::string()));
    // Exercise each backend explicitly below, independently of runtime tuning.
    node_->declare_parameter("FollowPath.MppiPath.ObstacleGuidance.use_smac", false);
    node_->set_parameter(rclcpp::Parameter("FollowPath.MppiPath.ObstacleGuidance.use_smac", false));
    // Preserve deterministic synchronous regressions while runtime profiles
    // use the worker. The dedicated async fixture opts in explicitly below.
    node_->declare_parameter("FollowPath.MppiPath.ObstacleGuidance.Smac.async_planning", false);
    node_->set_parameter(rclcpp::Parameter(
      "FollowPath.MppiPath.ObstacleGuidance.Smac.async_planning", false));
    // Existing scoring regressions and backend benchmarks deliberately plan
    // every cycle. Dedicated cadence tests below exercise runtime thresholds.
    node_->declare_parameter("FollowPath.MppiPath.ObstacleGuidance.Smac.replan_frequency", 0.0);
    node_->set_parameter(rclcpp::Parameter(
      "FollowPath.MppiPath.ObstacleGuidance.Smac.replan_frequency", 0.0));
    node_->declare_parameter("FollowPath.MppiPath.ObstacleGuidance.Smac.replan_goal_distance", 0.0);
    node_->set_parameter(rclcpp::Parameter(
      "FollowPath.MppiPath.ObstacleGuidance.Smac.replan_goal_distance", 0.0));
    node_->declare_parameter("FollowPath.MppiPath.ObstacleGuidance.Smac.replan_goal_angle", 0.0);
    node_->set_parameter(rclcpp::Parameter(
      "FollowPath.MppiPath.ObstacleGuidance.Smac.replan_goal_angle", 0.0));
    // Legacy route and event regressions retry the same target. Exercise
    // forward movement after a failed search in its dedicated fixture below.
    node_->declare_parameter("FollowPath.MppiPath.ObstacleGuidance.Smac.failure_advance_distance", 0.0);
    node_->set_parameter(rclcpp::Parameter(
      "FollowPath.MppiPath.ObstacleGuidance.Smac.failure_advance_distance", 0.0));
    // Most fixtures model a single fused grid. Sensor provenance is exercised
    // explicitly with a separate ObstacleLayer in the laser-trigger tests.
    node_->declare_parameter("FollowPath.MppiPath.ObstacleGuidance.trigger_layer", std::string());
    node_->set_parameter(rclcpp::Parameter(
      "FollowPath.MppiPath.ObstacleGuidance.trigger_layer", std::string()));
    if (!structuredPipeline()) {
      // Keep the legacy sampler/critic regression suite independent of the
      // newly selected runtime strategy. Structured integration is tested below.
      const bool simulation = std::string(parameterFile()) == LIMO_MPC_SIM_PARAMS_PATH;
      const auto override = [&](const std::string & name, const rclcpp::ParameterValue & value) {
          if (!node_->has_parameter(name)) {node_->declare_parameter(name, value);}
          node_->set_parameter(rclcpp::Parameter(name, value));
        };
      override("FollowPath.MPC.structured_actions", rclcpp::ParameterValue(false));
      override("FollowPath.MPC.Startup.enabled", rclcpp::ParameterValue(false));
      override("FollowPath.MPC.deduplicate", rclcpp::ParameterValue(false));
      override("FollowPath.MPC.Tube.enabled", rclcpp::ParameterValue(false));
      override("FollowPath.MPC.batch_size", rclcpp::ParameterValue(simulation ? 768 : 128));
      override("FollowPath.MPC.control_segments", rclcpp::ParameterValue(5));
      override("FollowPath.MPC.Debug.samples_per_family", rclcpp::ParameterValue(6));
      override("FollowPath.MppiPath.PathFollowCritic.lookahead_distance", rclcpp::ParameterValue(1.25));
      override("FollowPath.MppiPath.ObstacleGuidance.lookahead_distance", rclcpp::ParameterValue(1.25));
      override("FollowPath.MppiObstacle.full_footprint_check", rclcpp::ParameterValue(false));
      override("FollowPath.min_vel_x", rclcpp::ParameterValue(simulation ? -0.1 : 0.0));
    }
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

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> addLaserTriggerLayer()
  {
    laser_tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    auto laser = std::make_shared<nav2_costmap_2d::ObstacleLayer>();
    laser->initialize(costmap_->getLayeredCostmap(), "obstacle_layer", laser_tf_.get(),
      costmap_, nullptr, nullptr);
    costmap_->getLayeredCostmap()->addPlugin(laser);
    const auto * map = costmap_->getCostmap();
    // A rolling sensor layer can have a different origin from the fused map.
    laser->resizeMap(map->getSizeInCellsX(), map->getSizeInCellsY(), map->getResolution(),
      map->getOriginX() + .25, map->getOriginY() + .15);
    clearLaser(laser);
    node_->set_parameter(rclcpp::Parameter(
      "FollowPath.MppiPath.ObstacleGuidance.trigger_layer", std::string("obstacle_layer")));
    return laser;
  }

  void clearLaser(const std::shared_ptr<nav2_costmap_2d::ObstacleLayer> & laser,
    unsigned char cost = nav2_costmap_2d::FREE_SPACE)
  {
    std::fill(laser->getCharMap(), laser->getCharMap() +
      laser->getSizeInCellsX() * laser->getSizeInCellsY(), cost);
  }

  void setLaserCost(const std::shared_ptr<nav2_costmap_2d::ObstacleLayer> & laser,
    double x, double y, unsigned char cost = nav2_costmap_2d::LETHAL_OBSTACLE)
  {
    unsigned int mx, my;
    ASSERT_TRUE(laser->worldToMap(x, y, mx, my));
    laser->setCost(mx, my, cost);
  }

  template<typename Critic>
  void initialize(Critic & critic, std::string name)
  {
    std::string parent = "FollowPath";
    critic.initialize(node_, name, parent, costmap_);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> laser_tf_;
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
  bool guidanceTraversable(unsigned int x, unsigned int y) const
  {return guidance_.traversable(x, y);}
  std::size_t startIndex() const {return start_index_;}
  const auto & smacPath() const {return smac_path_;}
  const auto & smacGoal() const {return smac_goal_;}
  double failedSmacGoalProgress() const {return smac_failed_goal_progress_;}
  const auto & targetPose() const {return path_[target_index_];}
  bool hasDijkstraField() const
  {
    auto * map = costmap_ros_->getCostmap();
    return std::isfinite(guidance_.distance(
      -map->getOriginX() / map->getResolution() - 0.5,
      -map->getOriginY() / map->getResolution() - 0.5));
  }
  void replacePlanner(nav2_core::GlobalPlanner::Ptr planner)
  {
    smac_planner_->deactivate();
    smac_planner_ = std::move(planner);
  }
  void elapseSmacPlanPeriod()
  {
    ASSERT_TRUE(smac_last_search_start_.has_value());
    if (smac_last_plan_start_) {*smac_last_plan_start_ -= std::chrono::milliseconds(251);}
    *smac_last_search_start_ -= std::chrono::milliseconds(251);
  }
  std::int64_t limiterStartSteadyNs() const
  {
    return smac_last_search_start_ ? std::chrono::duration_cast<std::chrono::nanoseconds>(
      smac_last_search_start_->time_since_epoch()).count() : 0;
  }
  bool waitForSmacResult() const
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (smac_worker_ && smac_worker_->busy()) {
      if (std::chrono::steady_clock::now() >= deadline) {return false;}
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return smac_worker_ != nullptr;
  }
};

class LocalGuidanceBackendsTest : public MppiOnlineCostsTest,
  public ::testing::WithParamInterface<bool> {};

INSTANTIATE_TEST_CASE_P(DijkstraAndSmac, LocalGuidanceBackendsTest, ::testing::Bool());

TEST_P(LocalGuidanceBackendsTest, PlansAroundObstacleWithoutRunningTheOtherBackend)
{
  costmap_->getCostmap()->resizeMap(250, 250, 0.02, -2.5, -2.5);
  clearMap();
  node_->set_parameter(rclcpp::Parameter("FollowPath.MppiPath.ObstacleGuidance.use_smac", GetParam()));
  for (double x = 0.8; x < 1.0; x += 0.025) {
    for (double y = -0.08; y < 0.08; y += 0.025) {setCost(x, y, 254);}
  }
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  critic.setTimingEnabled(true);
  const auto path = pathBehindCube();
  const auto * map = costmap_->getCostmap();
  const std::vector<unsigned char> original_costs(map->getCharMap(),
    map->getCharMap() + map->getSizeInCellsX() * map->getSizeInCellsY());
  ASSERT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), path));
  ASSERT_TRUE(critic.detouring());
  EXPECT_EQ(critic.usesSmac(), GetParam());
  EXPECT_EQ(critic.hasDijkstraField(), !GetParam());
  EXPECT_EQ(critic.smacPath().empty(), !GetParam());
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanFailed());
  EXPECT_GT(critic.guidancePlanMs(), 0.0);
  EXPECT_GT(critic.guidancePlanStartSteadyNs(), 0);
  EXPECT_EQ(critic.guidancePlanEpoch(), GetParam() ? 1U : 0U);
  EXPECT_DOUBLE_EQ(critic.guidanceReplanHz(), 0.0);
  EXPECT_TRUE(std::equal(original_costs.begin(), original_costs.end(), map->getCharMap()));
  const auto reference = critic.guidanceReference(pose(0, 0));
  ASSERT_GE(reference.size(), 2U);
  EXPECT_NEAR(reference.back().x, GetParam() ? critic.smacGoal().x : critic.targetPose().x, 0.10);
  if (GetParam()) {
    double lateral = 0.0;
    for (const auto & point : critic.smacPath()) {lateral = std::max(lateral, std::abs(point.y));}
    EXPECT_GT(lateral, 0.15);
    EXPECT_NEAR(critic.smacPath().back().x, critic.smacGoal().x, 0.10);
  }
  // A fixed live grid isolates backend preparation; initialization is excluded.
  double plan_sum = 0.0, plan_max = 0.0, prepare_sum = 0.0;
  for (int i = 0; i < 23; ++i) {
    const auto begin = std::chrono::steady_clock::now();
    ASSERT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), path));
    const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - begin).count();
    if (i < 3) {continue;}
    plan_sum += critic.guidancePlanMs();
    plan_max = std::max(plan_max, critic.guidancePlanMs());
    prepare_sum += elapsed;
  }
  std::cout << "GUIDANCE_BENCH backend=" << (GetParam() ? "SMAC" : "DIJKSTRA")
            << " calls=20 search_mean_ms=" << plan_sum / 20.0
            << " search_max_ms=" << plan_max
            << " prepare_mean_ms=" << prepare_sum / 20.0 << std::endl;
  clearMap();
  // Both backends exit once the detection clears and the robot is back on
  // the original reference; reaching the temporary SMAC goal is unnecessary.
  ASSERT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), path));
  EXPECT_FALSE(critic.detouring());
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_DOUBLE_EQ(critic.guidancePlanMs(), 0.0);
  EXPECT_EQ(critic.guidancePlanStartSteadyNs(), 0);
  EXPECT_TRUE(critic.guidanceReference(pose(0, 0)).empty());
}

class RecordingLocalPlanner : public nav2_core::GlobalPlanner
{
public:
  geometry_msgs::msg::PoseStamped arrival;
  std::vector<geometry_msgs::msg::Pose2D> via;
  bool empty{false};
  int calls{0};
  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void activate() override {}
  void deactivate() override {}
  void cleanup() override {}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    ++calls;
    arrival = goal;
    nav_msgs::msg::Path result;
    result.header = start.header;
    if (empty) {return result;}
    result.poses.push_back(start);
    for (const auto & point : via) {
      auto stamped = start;
      stamped.pose.position.x = point.x;
      stamped.pose.position.y = point.y;
      stamped.pose.orientation.z = std::sin(point.theta * 0.5);
      stamped.pose.orientation.w = std::cos(point.theta * 0.5);
      result.poses.push_back(stamped);
    }
    result.poses.push_back(goal);
    return result;
  }
};

// Each call keeps its own response and remains blocked until explicitly
// released. All observations are synchronized with the planning thread.
class BlockingLocalPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void activate() override {}
  void deactivate() override {}
  void cleanup() override {}

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    goals_.push_back(goal);
    starts_.push_back(std::chrono::steady_clock::now());
    const auto call = goals_.size();
    const bool empty = empty_;
    ++active_;
    max_active_ = std::max(max_active_, active_);
    changed_.notify_all();
    changed_.wait(lock, [&]() {return release_all_ || released_ >= call;});
    --active_;
    lock.unlock();
    nav_msgs::msg::Path result;
    result.header = start.header;
    if (!empty) {result.poses = {start, goal};}
    return result;
  }

  bool waitForCalls(std::size_t count,
    std::chrono::milliseconds timeout = std::chrono::seconds(2))
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return changed_.wait_for(lock, timeout, [&]() {return goals_.size() >= count;});
  }
  void release(std::size_t count)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = std::max(released_, count);
    changed_.notify_all();
  }
  void releaseAll()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    release_all_ = true;
    changed_.notify_all();
  }
  void setEmpty(bool empty)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    empty_ = empty;
  }
  std::size_t calls() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return goals_.size();
  }
  std::size_t active() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_;
  }
  std::size_t maxActive() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return max_active_;
  }
  geometry_msgs::msg::PoseStamped goal(std::size_t call) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return goals_.at(call - 1);
  }
  std::chrono::steady_clock::time_point started(std::size_t call) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return starts_.at(call - 1);
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  std::vector<std::chrono::steady_clock::time_point> starts_;
  std::size_t released_{0};
  std::size_t active_{0};
  std::size_t max_active_{0};
  bool empty_{false};
  bool release_all_{false};
};

class ReleaseBlockedPlanner
{
public:
  explicit ReleaseBlockedPlanner(std::shared_ptr<BlockingLocalPlanner> planner)
  : planner_(std::move(planner)) {}
  ~ReleaseBlockedPlanner() {planner_->releaseAll();}

private:
  std::shared_ptr<BlockingLocalPlanner> planner_;
};

class SmacCadenceTest : public MppiOnlineCostsTest
{
protected:
  void SetUp() override
  {
    MppiOnlineCostsTest::SetUp();
    node_->set_parameter(rclcpp::Parameter(
      "FollowPath.MppiPath.ObstacleGuidance.Smac.replan_frequency", 4.0));
    node_->set_parameter(rclcpp::Parameter(
      "FollowPath.MppiPath.ObstacleGuidance.Smac.replan_goal_distance", .25));
    node_->set_parameter(rclcpp::Parameter(
      "FollowPath.MppiPath.ObstacleGuidance.Smac.replan_goal_angle", .35));
    node_->set_parameter(rclcpp::Parameter("FollowPath.MppiPath.ObstacleGuidance.use_smac", true));
    node_->set_parameter(rclcpp::Parameter(
      "FollowPath.MppiPath.ObstacleGuidance.lookahead_distance", .5));
    laser_ = addLaserTriggerLayer();
    // Isolate sensor triggering from route validity, which uses the master.
    setLaserCost(laser_, .725, .025);
  }

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> laser_;
};

class SmacAsyncTest : public SmacCadenceTest
{
protected:
  void SetUp() override
  {
    SmacCadenceTest::SetUp();
    node_->set_parameter(rclcpp::Parameter(
      "FollowPath.MppiPath.ObstacleGuidance.Smac.async_planning", true));
  }

  template<typename Prepare>
  bool prepareWithoutWaiting(Prepare prepare, const std::shared_ptr<BlockingLocalPlanner> & planner)
  {
    auto result = std::async(std::launch::async, [&]() {
        try {return prepare();} catch (const limo_dwb_critics::GuidanceUnavailable &) {
          return false;
        }
      });
    const auto status = result.wait_for(std::chrono::seconds(2));
    // A synchronous regression must fail without leaving either the future's
    // destructor or the critic's worker join waiting on the test planner.
    if (status != std::future_status::ready) {planner->releaseAll();}
    EXPECT_EQ(status, std::future_status::ready);
    return result.get();
  }
};

TEST_F(SmacAsyncTest, FirstPrepareReturnsWhilePlanningAndPublishesTimingOnlyOnCompletion)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<BlockingLocalPlanner>();
  ReleaseBlockedPlanner release(planner);
  critic.replacePlanner(planner);
  critic.setTimingEnabled(true);
  const auto path = pathBehindCube();
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };
  const auto submitted = std::chrono::steady_clock::now();
  EXPECT_FALSE(prepareWithoutWaiting(prepare, planner));
  ASSERT_TRUE(planner->waitForCalls(1));
  EXPECT_TRUE(critic.guidanceAsync());
  EXPECT_TRUE(critic.guidancePlanRunning());
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanFailed());
  EXPECT_DOUBLE_EQ(critic.guidancePlanMs(), 0.0);
  EXPECT_EQ(critic.guidancePlanStartSteadyNs(), 0);
  EXPECT_FALSE(critic.detouring());
  EXPECT_TRUE(critic.smacPath().empty());

  EXPECT_FALSE(prepareWithoutWaiting(prepare, planner));
  EXPECT_EQ(planner->calls(), 1U);
  EXPECT_FALSE(critic.guidancePlanCalled());
  const auto released = std::chrono::steady_clock::now();
  planner->release(1);
  ASSERT_TRUE(critic.waitForSmacResult());
  ASSERT_TRUE(prepare());
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanRunning());
  EXPECT_FALSE(critic.guidancePlanFailed());
  EXPECT_FALSE(critic.guidancePlanDiscarded());
  EXPECT_GT(critic.guidancePlanMs(), 0.0);
  const double held_worker_ms = std::chrono::duration<double, std::milli>(
    released - planner->started(1)).count();
  EXPECT_GE(critic.guidancePlanMs(), held_worker_ms);
  EXPECT_EQ(critic.guidancePlanStartSteadyNs(), critic.limiterStartSteadyNs());
  const auto submitted_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    submitted.time_since_epoch()).count();
  const auto planner_entered_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    planner->started(1).time_since_epoch()).count();
  EXPECT_GE(critic.guidancePlanStartSteadyNs(), submitted_ns);
  EXPECT_LE(critic.guidancePlanStartSteadyNs(), planner_entered_ns);
  EXPECT_TRUE(critic.detouring());
  EXPECT_FALSE(critic.smacPath().empty());

  ASSERT_TRUE(prepare());
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_DOUBLE_EQ(critic.guidancePlanMs(), 0.0);
  EXPECT_EQ(critic.guidancePlanStartSteadyNs(), 0);
}

TEST_F(SmacAsyncTest, KeepsSafeCacheDuringRetargetWithOneWorkerAndNoQueuedRequests)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<BlockingLocalPlanner>();
  ReleaseBlockedPlanner release(planner);
  critic.replacePlanner(planner);
  const auto path = pathBehindCube();
  const auto prepare = [&](double x) {
      return critic.prepare(pose(x, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };
  EXPECT_FALSE(prepareWithoutWaiting([&]() {return prepare(0);}, planner));
  ASSERT_TRUE(planner->waitForCalls(1));
  planner->release(1);
  ASSERT_TRUE(critic.waitForSmacResult());
  ASSERT_TRUE(prepare(0));
  const auto accepted_path = critic.smacPath();
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  const auto accepted_goal = *critic.guidanceTarget();

  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepareWithoutWaiting([&]() {return prepare(.3);}, planner));
  ASSERT_TRUE(planner->waitForCalls(2));
  for (double x : {.6, .9, .3}) {
    ASSERT_TRUE(prepareWithoutWaiting([&]() {return prepare(x);}, planner));
    EXPECT_TRUE(critic.guidancePlanRunning());
    EXPECT_FALSE(critic.guidancePlanCalled());
    EXPECT_FALSE(critic.guidancePlanFailed());
    EXPECT_EQ(critic.smacPath(), accepted_path);
    ASSERT_NE(critic.guidanceTarget(), nullptr);
    EXPECT_DOUBLE_EQ(critic.guidanceTarget()->x, accepted_goal.x);
    EXPECT_FALSE(critic.guidanceReference(pose(x, 0)).empty());
    EXPECT_EQ(planner->calls(), 2U);
    EXPECT_EQ(planner->active(), 1U);
  }
  EXPECT_EQ(planner->maxActive(), 1U);
  planner->release(2);
  ASSERT_TRUE(critic.waitForSmacResult());
  ASSERT_TRUE(prepare(.3));
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  EXPECT_DOUBLE_EQ(critic.guidanceTarget()->x, planner->goal(2).pose.position.x);
  ASSERT_TRUE(prepare(.3));
  EXPECT_FALSE(planner->waitForCalls(3, std::chrono::milliseconds(30)));
  EXPECT_FALSE(critic.guidancePlanRunning());
}

TEST_F(SmacAsyncTest, RejectsCompletedRouteAgainstCurrentFullFootprint)
{
  const auto margin = "FollowPath.MppiPath.ObstacleGuidance.clearance_margin";
  if (!node_->has_parameter(margin)) {node_->declare_parameter(margin, 0.0);}
  node_->set_parameter(rclcpp::Parameter(margin, 0.0));
  std::vector<geometry_msgs::msg::Point> footprint(4);
  footprint[0].x = -.45; footprint[0].y = -.04;
  footprint[1].x = -.45; footprint[1].y = .04;
  footprint[2].x = .45; footprint[2].y = .04;
  footprint[3].x = .45; footprint[3].y = -.04;
  costmap_->setRobotFootprint(footprint);
  clearLaser(laser_);
  setLaserCost(laser_, .525, .325);
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<BlockingLocalPlanner>();
  ReleaseBlockedPlanner release(planner);
  critic.replacePlanner(planner);
  auto path = pathBehindCube();
  const double yaw = std::acos(-1.0) / 2.0;
  for (auto & point : path.poses) {point.theta = yaw;}
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0, yaw), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };
  EXPECT_FALSE(prepareWithoutWaiting(prepare, planner));
  ASSERT_TRUE(planner->waitForCalls(1));
  // The worker's captured map is free. This later obstacle intersects the
  // long oriented footprint while every path center remains traversable.
  setCost(.525, .325, nav2_costmap_2d::LETHAL_OBSTACLE);
  planner->release(1);
  ASSERT_TRUE(critic.waitForSmacResult());
  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  unsigned int x, y;
  ASSERT_TRUE(costmap_->getCostmap()->worldToMap(.525, 0, x, y));
  EXPECT_TRUE(critic.guidanceTraversable(x, y));
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_TRUE(critic.guidancePlanFailed());
  EXPECT_FALSE(critic.detouring());
  EXPECT_TRUE(critic.smacPath().empty());
  EXPECT_EQ(critic.guidanceTarget(), nullptr);
  EXPECT_EQ(planner->calls(), 1U);
}

TEST_F(SmacAsyncTest, ResetAndNewReferenceDiscardInFlightCompletion)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<BlockingLocalPlanner>();
  ReleaseBlockedPlanner release(planner);
  critic.replacePlanner(planner);
  critic.setTimingEnabled(true);
  auto path = pathBehindCube();
  const auto prepare = [&](double y) {
      return critic.prepare(pose(0, y), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };
  EXPECT_FALSE(prepareWithoutWaiting([&]() {return prepare(0);}, planner));
  ASSERT_TRUE(planner->waitForCalls(1));
  // Model a submission stamp older than the worker's actual start. A stale
  // completion must restore the actual clock before considering a retry.
  critic.elapseSmacPlanPeriod();
  const auto provisional_start = critic.limiterStartSteadyNs();
  critic.reset();
  EXPECT_EQ(critic.limiterStartSteadyNs(), provisional_start);
  for (auto & point : path.poses) {point.y = .5;}
  clearLaser(laser_);
  setLaserCost(laser_, .725, .525);
  EXPECT_FALSE(prepareWithoutWaiting([&]() {return prepare(.5);}, planner));
  EXPECT_EQ(planner->calls(), 1U);
  EXPECT_TRUE(critic.smacPath().empty());
  planner->release(1);
  ASSERT_TRUE(critic.waitForSmacResult());
  EXPECT_FALSE(prepareWithoutWaiting([&]() {return prepare(.5);}, planner));
  EXPECT_TRUE(critic.guidancePlanDiscarded());
  EXPECT_FALSE(critic.detouring());
  EXPECT_TRUE(critic.smacPath().empty());
  EXPECT_EQ(critic.guidanceTarget(), nullptr);
  EXPECT_EQ(critic.limiterStartSteadyNs(), critic.guidancePlanStartSteadyNs());
  EXPECT_GT(critic.limiterStartSteadyNs(), provisional_start);
  EXPECT_EQ(planner->calls(), 1U);
  EXPECT_FALSE(critic.guidancePlanRunning());
  critic.elapseSmacPlanPeriod();
  EXPECT_FALSE(prepareWithoutWaiting([&]() {return prepare(.5);}, planner));
  ASSERT_TRUE(planner->waitForCalls(2));
  EXPECT_DOUBLE_EQ(planner->goal(1).pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(planner->goal(2).pose.position.y, .5);
  planner->release(2);
  ASSERT_TRUE(critic.waitForSmacResult());
  ASSERT_TRUE(prepare(.5));
  EXPECT_FALSE(critic.guidancePlanDiscarded());
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  EXPECT_DOUBLE_EQ(critic.guidanceTarget()->y, .5);
  ASSERT_FALSE(critic.smacPath().empty());
  for (const auto & point : critic.smacPath()) {EXPECT_DOUBLE_EQ(point.y, .5);}
}

TEST_F(SmacAsyncTest, DrainsDiscardedCompletionAfterReturningToNormalReference)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<BlockingLocalPlanner>();
  ReleaseBlockedPlanner release(planner);
  critic.replacePlanner(planner);
  const auto path = pathBehindCube();
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };
  EXPECT_FALSE(prepareWithoutWaiting(prepare, planner));
  ASSERT_TRUE(planner->waitForCalls(1));
  clearLaser(laser_);
  ASSERT_TRUE(prepareWithoutWaiting(prepare, planner));
  EXPECT_FALSE(critic.detouring());
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_TRUE(critic.smacPath().empty());
  planner->release(1);
  ASSERT_TRUE(critic.waitForSmacResult());

  ASSERT_TRUE(prepare());
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_TRUE(critic.guidancePlanDiscarded());
  EXPECT_FALSE(critic.guidancePlanRunning());
  EXPECT_FALSE(critic.detouring());
  EXPECT_TRUE(critic.smacPath().empty());
  EXPECT_EQ(critic.guidanceTarget(), nullptr);
  EXPECT_EQ(planner->calls(), 1U);

  // A normal cycle consumes the stale result while preserving its actual
  // search start, so a fresh laser trigger still observes the cooldown.
  setLaserCost(laser_, .725, .025);
  EXPECT_FALSE(prepareWithoutWaiting(prepare, planner));
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanDiscarded());
  EXPECT_EQ(planner->calls(), 1U);
  EXPECT_FALSE(critic.guidancePlanRunning());
  critic.elapseSmacPlanPeriod();
  EXPECT_FALSE(prepareWithoutWaiting(prepare, planner));
  ASSERT_TRUE(planner->waitForCalls(2));
  planner->release(2);
  ASSERT_TRUE(critic.waitForSmacResult());
  ASSERT_TRUE(prepare());
  EXPECT_TRUE(critic.detouring());
  EXPECT_FALSE(critic.guidancePlanDiscarded());
  EXPECT_FALSE(critic.smacPath().empty());
}

TEST_F(SmacAsyncTest, CadenceUsesActualWorkerStartsAndNeverStartsDuringAnActiveSearch)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<BlockingLocalPlanner>();
  planner->setEmpty(true);
  ReleaseBlockedPlanner release(planner);
  critic.replacePlanner(planner);
  critic.setTimingEnabled(true);
  const auto path = pathBehindCube();
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };
  EXPECT_FALSE(prepareWithoutWaiting(prepare, planner));
  ASSERT_TRUE(planner->waitForCalls(1));
  EXPECT_DOUBLE_EQ(critic.guidanceReplanHz(), 4.0);
  const auto earliest_retry = planner->started(1) + std::chrono::milliseconds(250);
  // Let the real cooldown expire while the first native call is held, so a
  // retry may start immediately after consuming its completed failure.
  std::this_thread::sleep_until(earliest_retry);
  EXPECT_FALSE(prepareWithoutWaiting(prepare, planner));
  EXPECT_EQ(planner->calls(), 1U);
  EXPECT_FALSE(critic.guidancePlanCalled());
  planner->release(1);
  ASSERT_TRUE(critic.waitForSmacResult());
  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_TRUE(critic.guidancePlanFailed());
  EXPECT_EQ(planner->calls(), 1U);
  EXPECT_FALSE(prepareWithoutWaiting(prepare, planner));
  ASSERT_TRUE(planner->waitForCalls(2));
  EXPECT_GE(planner->started(2) - planner->started(1), std::chrono::milliseconds(250));
  EXPECT_EQ(planner->maxActive(), 1U);
  planner->release(2);
  ASSERT_TRUE(critic.waitForSmacResult());
  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  EXPECT_FALSE(prepareWithoutWaiting(prepare, planner));
  EXPECT_EQ(planner->calls(), 2U);
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanRunning());
}

TEST_F(SmacAsyncTest, AdvancesCandidateOnlyAfterConsumedWorkerFailure)
{
  node_->set_parameter(rclcpp::Parameter(
    "FollowPath.MppiPath.ObstacleGuidance.Smac.failure_advance_distance", .25));
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<BlockingLocalPlanner>();
  planner->setEmpty(true);
  ReleaseBlockedPlanner release(planner);
  critic.replacePlanner(planner);
  const auto path = pathBehindCube();
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };
  EXPECT_FALSE(prepareWithoutWaiting(prepare, planner));
  ASSERT_TRUE(planner->waitForCalls(1));
  const double first_goal = planner->goal(1).pose.position.x;
  for (int cycle = 0; cycle < 3; ++cycle) {
    EXPECT_FALSE(prepareWithoutWaiting(prepare, planner));
    EXPECT_FALSE(critic.guidancePlanCalled());
    EXPECT_FALSE(critic.guidancePlanFailed());
    ASSERT_NE(critic.guidanceCandidateTarget(), nullptr);
    EXPECT_DOUBLE_EQ(critic.guidanceCandidateTarget()->x, first_goal);
    EXPECT_DOUBLE_EQ(critic.failedSmacGoalProgress(), 0.0);
  }
  planner->release(1);
  ASSERT_TRUE(critic.waitForSmacResult());
  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_TRUE(critic.guidancePlanFailed());
  EXPECT_GT(critic.failedSmacGoalProgress(), 0.0);
  critic.elapseSmacPlanPeriod();
  EXPECT_FALSE(prepareWithoutWaiting(prepare, planner));
  ASSERT_TRUE(planner->waitForCalls(2));
  EXPECT_NEAR(planner->goal(2).pose.position.x - first_goal, .25, .051);
  const double second_goal = planner->goal(2).pose.position.x;
  for (int cycle = 0; cycle < 3; ++cycle) {
    EXPECT_FALSE(prepareWithoutWaiting(prepare, planner));
    ASSERT_NE(critic.guidanceCandidateTarget(), nullptr);
    EXPECT_DOUBLE_EQ(critic.guidanceCandidateTarget()->x, second_goal);
    EXPECT_EQ(planner->calls(), 2U);
    EXPECT_FALSE(critic.guidancePlanCalled());
  }
}

TEST_F(SmacCadenceTest, ChoosesTargetsUsingTheFilledFootprintAndReferenceOrientation)
{
  std::vector<geometry_msgs::msg::Point> footprint(4);
  footprint[0].x = -.45; footprint[0].y = -.10;
  footprint[1].x = -.45; footprint[1].y = .10;
  footprint[2].x = .45; footprint[2].y = .10;
  footprint[3].x = .45; footprint[3].y = -.10;
  costmap_->setRobotFootprint(footprint);
  setCost(1.325, .025, nav2_costmap_2d::LETHAL_OBSTACLE);
  setLaserCost(laser_, .525, .325);  // Also trigger the rotated footprint.
  auto * map = costmap_->getCostmap();
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map);
  // The obstacle lies inside the nominal footprint, away from its center and
  // all polygon edges. An edge-only collision checker therefore sees free space.
  EXPECT_DOUBLE_EQ(checker.footprintCostAtPose(1.0, 0.0, 0.0, footprint), 0.0);
  unsigned int mx, my;
  ASSERT_TRUE(map->worldToMap(1.0, 0, mx, my));
  EXPECT_EQ(map->getCost(mx, my), nav2_costmap_2d::FREE_SPACE);

  for (double yaw : {0.0, std::acos(-1.0) / 2.0}) {
    SCOPED_TRACE(yaw);
    InspectableMppiPathCritic critic;
    initialize(critic, "MppiPath");
    auto planner = std::make_shared<RecordingLocalPlanner>();
    planner->empty = true;
    critic.replacePlanner(planner);
    auto path = pathBehindCube();
    for (auto & point : path.poses) {point.theta = yaw;}
    EXPECT_THROW(critic.prepare(pose(0, 0, yaw), nav_2d_msgs::msg::Twist2D(),
        path.poses.back(), path), limo_dwb_critics::GuidanceUnavailable);
    EXPECT_TRUE(critic.guidanceTraversable(mx, my));
    ASSERT_EQ(planner->calls, 1);
    ASSERT_NE(critic.guidanceCandidateTarget(), nullptr);
    const auto candidate = *critic.guidanceCandidateTarget();
    EXPECT_DOUBLE_EQ(planner->arrival.pose.position.x, candidate.x);
    EXPECT_DOUBLE_EQ(candidate.theta, yaw);
    EXPECT_LE(candidate.x, path.poses.back().x);
    if (yaw == 0.0) {
      EXPECT_NEAR(candidate.x, 1.8, .051);
      EXPECT_GT(candidate.x, 1.325 + .4);
    } else {
      // Rotating the same long body removes the nominal target collision.
      EXPECT_NEAR(candidate.x, 1.0, .051);
    }
  }
}

TEST_F(SmacCadenceTest, MissingFootprintSafeTargetsDoNotDispatchOrAdvancePastTheReference)
{
  std::vector<geometry_msgs::msg::Point> footprint(4);
  footprint[0].x = -.45; footprint[0].y = -.10;
  footprint[1].x = -.45; footprint[1].y = .10;
  footprint[2].x = .45; footprint[2].y = .10;
  footprint[3].x = .45; footprint[3].y = -.10;
  costmap_->setRobotFootprint(footprint);
  enum class Blocker {Lethal, Unknown, MapEdge};
  for (const auto blocker : {Blocker::Lethal, Blocker::Unknown, Blocker::MapEdge}) {
    SCOPED_TRACE(static_cast<int>(blocker));
    costmap_->getCostmap()->resizeMap(
      blocker == Blocker::MapEdge ? 76 : 100, 100, .05, -2.5, -2.5);
    clearMap();
    if (blocker != Blocker::MapEdge) {
      setCost(1.325, .025, blocker == Blocker::Lethal ?
        nav2_costmap_2d::LETHAL_OBSTACLE : nav2_costmap_2d::NO_INFORMATION);
    }
    InspectableMppiPathCritic critic;
    initialize(critic, "MppiPath");
    auto planner = std::make_shared<RecordingLocalPlanner>();
    planner->empty = true;
    critic.replacePlanner(planner);
    auto path = straightPath();  // Clipped at x=1.5, before any footprint fits.
    const auto prepare = [&]() {
        return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), path);
      };
    for (int cycle = 0; cycle < 3; ++cycle) {
      EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
      EXPECT_EQ(planner->calls, 0);
      EXPECT_FALSE(critic.guidancePlanCalled());
      EXPECT_FALSE(critic.guidancePlanFailed());
      EXPECT_DOUBLE_EQ(critic.failedSmacGoalProgress(), 0.0);
      EXPECT_EQ(critic.guidanceCandidateTarget(), nullptr);
      EXPECT_EQ(critic.guidanceTarget(), nullptr);
      EXPECT_TRUE(critic.smacPath().empty());
      EXPECT_NE(critic.guidanceLookahead(), nullptr);
    }
    // Extending the reference permits a forward endpoint beyond the interior
    // obstacle. For a map-edge rejection, restore room for the whole body.
    if (blocker == Blocker::MapEdge) {
      costmap_->getCostmap()->resizeMap(100, 100, .05, -2.5, -2.5);
      clearMap();
    }
    path = pathBehindCube();
    EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
    EXPECT_EQ(planner->calls, 1);
    EXPECT_TRUE(critic.guidancePlanCalled());
    ASSERT_NE(critic.guidanceCandidateTarget(), nullptr);
    EXPECT_LE(critic.guidanceCandidateTarget()->x, path.poses.back().x);
  }
}

TEST_F(SmacCadenceTest, ReusesAcceptedRouteWhileUpdatingProspectiveLookaheads)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  critic.replacePlanner(planner);
  critic.setTimingEnabled(true);
  const auto path = pathBehindCube();
  const auto prepare = [&](double x) {
      return critic.prepare(pose(x, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };

  ASSERT_TRUE(prepare(0));
  ASSERT_TRUE(critic.detouring());
  EXPECT_EQ(planner->calls, 1);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanFailed());
  const auto epoch = critic.guidancePlanEpoch();
  EXPECT_GT(epoch, 0U);
  EXPECT_GT(critic.guidancePlanStartSteadyNs(), 0);
  EXPECT_EQ(critic.guidancePlanStartSteadyNs(), critic.limiterStartSteadyNs());
  EXPECT_DOUBLE_EQ(critic.guidanceReplanHz(), 4.0);
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  const auto accepted = *critic.guidanceTarget();
  EXPECT_NEAR(accepted.x, 1.0, .051);

  for (int cycle = 0; cycle < 3; ++cycle) {
    ASSERT_TRUE(prepare(.25));
    ASSERT_TRUE(critic.detouring());
    EXPECT_EQ(planner->calls, 1);
    EXPECT_FALSE(critic.guidancePlanCalled());
    EXPECT_FALSE(critic.guidancePlanFailed());
    EXPECT_DOUBLE_EQ(critic.guidancePlanMs(), 0.0);
    EXPECT_EQ(critic.guidancePlanStartSteadyNs(), 0);
    EXPECT_EQ(critic.guidancePlanEpoch(), epoch);
    ASSERT_NE(critic.guidanceTarget(), nullptr);
    EXPECT_DOUBLE_EQ(critic.guidanceTarget()->x, accepted.x);
    expectLookahead(critic.pathLookahead(), 1.50);
    expectLookahead(critic.guidanceLookahead(), .75);
    expectLookahead(critic.guidanceCandidateTarget(), 1.25, 0, 0, .051);
    EXPECT_FALSE(critic.guidanceReference(pose(.25, 0)).empty());
  }

  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepare(.25));
  EXPECT_EQ(planner->calls, 2);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanFailed());
  EXPECT_EQ(critic.guidancePlanStartSteadyNs(), critic.limiterStartSteadyNs());
  EXPECT_EQ(critic.guidancePlanEpoch(), epoch);
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  ASSERT_NE(critic.guidanceCandidateTarget(), nullptr);
  EXPECT_DOUBLE_EQ(critic.guidanceTarget()->x, critic.guidanceCandidateTarget()->x);
  EXPECT_GT(critic.guidanceTarget()->x, accepted.x + .15);
}

TEST_F(SmacCadenceTest, ElapsedCooldownDoesNotReplanAnUnchangedSafeRoute)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  critic.replacePlanner(planner);
  critic.setTimingEnabled(true);
  const auto path = pathBehindCube();
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };

  ASSERT_TRUE(prepare());
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  const auto accepted = *critic.guidanceTarget();
  const auto accepted_path = critic.smacPath();
  const auto epoch = critic.guidancePlanEpoch();
  for (int cycle = 0; cycle < 3; ++cycle) {
    critic.elapseSmacPlanPeriod();
    ASSERT_TRUE(prepare());
    EXPECT_EQ(planner->calls, 1);
    EXPECT_FALSE(critic.guidancePlanCalled());
    EXPECT_FALSE(critic.guidancePlanFailed());
    EXPECT_DOUBLE_EQ(critic.guidancePlanMs(), 0.0);
    EXPECT_EQ(critic.guidancePlanStartSteadyNs(), 0);
    EXPECT_EQ(critic.guidancePlanEpoch(), epoch);
    EXPECT_TRUE(critic.detouring());
    ASSERT_NE(critic.guidanceTarget(), nullptr);
    EXPECT_DOUBLE_EQ(critic.guidanceTarget()->x, accepted.x);
    EXPECT_EQ(critic.smacPath(), accepted_path);
  }
}

TEST_F(SmacCadenceTest, AccumulatesGoalMovementFromTheLastAcceptedTarget)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  critic.replacePlanner(planner);
  const auto path = pathBehindCube();
  const auto prepare = [&](double x) {
      return critic.prepare(pose(x, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };

  ASSERT_TRUE(prepare(0));
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  const auto accepted = *critic.guidanceTarget();
  for (double x : {.1, .2}) {
    critic.elapseSmacPlanPeriod();
    ASSERT_TRUE(prepare(x));
    EXPECT_EQ(planner->calls, 1);
    EXPECT_FALSE(critic.guidancePlanCalled());
    ASSERT_NE(critic.guidanceCandidateTarget(), nullptr);
    EXPECT_GT(critic.guidanceCandidateTarget()->x, accepted.x);
    EXPECT_LT(critic.guidanceCandidateTarget()->x - accepted.x, .25);
    ASSERT_NE(critic.guidanceTarget(), nullptr);
    EXPECT_DOUBLE_EQ(critic.guidanceTarget()->x, accepted.x);
  }

  // Each step is small, but the accepted target has now moved over 25 cm.
  ASSERT_TRUE(prepare(.3));
  EXPECT_EQ(planner->calls, 2);
  EXPECT_TRUE(critic.guidancePlanCalled());
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  ASSERT_NE(critic.guidanceCandidateTarget(), nullptr);
  EXPECT_GE(critic.guidanceTarget()->x - accepted.x, .25);
  EXPECT_DOUBLE_EQ(critic.guidanceTarget()->x, critic.guidanceCandidateTarget()->x);
}

TEST_F(SmacCadenceTest, ReplansForWrappedArrivalYawChanges)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  critic.replacePlanner(planner);
  auto path = pathBehindCube();
  const double pi = std::acos(-1.0);
  // Keep the trigger inside the swept footprint when the reference faces back.
  setLaserCost(laser_, .325, .025);
  const auto prepare = [&](double yaw) {
      for (auto & point : path.poses) {point.theta = yaw;}
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };

  ASSERT_TRUE(prepare(-pi + .1));
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  const auto accepted = *critic.guidanceTarget();
  critic.elapseSmacPlanPeriod();
  // Crossing the wrap changes yaw by only -0.2 rad, below the 0.35 rad threshold.
  ASSERT_TRUE(prepare(pi - .1));
  EXPECT_EQ(planner->calls, 1);
  EXPECT_FALSE(critic.guidancePlanCalled());
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  EXPECT_DOUBLE_EQ(critic.guidanceTarget()->theta, accepted.theta);

  // The wrapped change is now -0.5 rad, with no positional movement.
  ASSERT_TRUE(prepare(pi - .4));
  EXPECT_EQ(planner->calls, 2);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanFailed());
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  EXPECT_DOUBLE_EQ(critic.guidanceTarget()->x, accepted.x);
  EXPECT_DOUBLE_EQ(critic.guidanceTarget()->y, accepted.y);
  EXPECT_DOUBLE_EQ(critic.guidanceTarget()->theta, pi - .4);
}

TEST_F(SmacCadenceTest, FailedMovedGoalKeepsTheAcceptedRouteAndRetriesAfterCooldown)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  critic.replacePlanner(planner);
  const auto path = pathBehindCube();
  const auto prepare = [&](double x) {
      return critic.prepare(pose(x, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };

  ASSERT_TRUE(prepare(0));
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  const auto accepted = *critic.guidanceTarget();
  const auto accepted_path = critic.smacPath();
  planner->empty = true;
  for (int attempt = 2; attempt <= 3; ++attempt) {
    critic.elapseSmacPlanPeriod();
    ASSERT_TRUE(prepare(.3));
    EXPECT_EQ(planner->calls, attempt);
    EXPECT_TRUE(critic.guidancePlanCalled());
    EXPECT_TRUE(critic.guidancePlanFailed());
    EXPECT_TRUE(critic.detouring());
    ASSERT_NE(critic.guidanceTarget(), nullptr);
    EXPECT_DOUBLE_EQ(critic.guidanceTarget()->x, accepted.x);
    EXPECT_EQ(critic.smacPath(), accepted_path);
    ASSERT_NE(critic.guidanceCandidateTarget(), nullptr);
    EXPECT_GE(critic.guidanceCandidateTarget()->x - accepted.x, .25);

    ASSERT_TRUE(prepare(.3));
    EXPECT_EQ(planner->calls, attempt);
    EXPECT_FALSE(critic.guidancePlanCalled());
    EXPECT_FALSE(critic.guidancePlanFailed());
  }

  planner->empty = false;
  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepare(.3));
  EXPECT_EQ(planner->calls, 4);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanFailed());
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  EXPECT_DOUBLE_EQ(critic.guidanceTarget()->x, critic.guidanceCandidateTarget()->x);
  EXPECT_GE(critic.guidanceTarget()->x - accepted.x, .25);
}

TEST_F(SmacCadenceTest, ThrottlesFailedSearchesWithoutAcceptingAMissingRoute)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  planner->empty = true;
  critic.replacePlanner(planner);
  critic.setTimingEnabled(true);
  const auto path = pathBehindCube();
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };

  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  EXPECT_EQ(planner->calls, 1);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_TRUE(critic.guidancePlanFailed());
  EXPECT_FALSE(critic.detouring());
  const auto first_start = critic.guidancePlanStartSteadyNs();
  const auto epoch = critic.guidancePlanEpoch();
  EXPECT_GT(first_start, 0);
  EXPECT_EQ(first_start, critic.limiterStartSteadyNs());
  EXPECT_GT(epoch, 0U);

  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  EXPECT_EQ(planner->calls, 1);
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanFailed());
  EXPECT_DOUBLE_EQ(critic.guidancePlanMs(), 0.0);
  EXPECT_EQ(critic.guidancePlanStartSteadyNs(), 0);
  EXPECT_EQ(critic.guidancePlanEpoch(), epoch);
  EXPECT_EQ(critic.guidanceTarget(), nullptr);

  critic.elapseSmacPlanPeriod();
  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  EXPECT_EQ(planner->calls, 2);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_TRUE(critic.guidancePlanFailed());
  EXPECT_GT(critic.guidancePlanStartSteadyNs(), first_start);
  EXPECT_EQ(critic.guidancePlanStartSteadyNs(), critic.limiterStartSteadyNs());
  EXPECT_EQ(critic.guidancePlanEpoch(), epoch);
  // Recovery also waits for the next scheduled attempt; controller cycles
  // between searches must not command motion without an accepted reference.
  planner->empty = false;
  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  EXPECT_EQ(planner->calls, 2);
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_EQ(critic.guidancePlanStartSteadyNs(), 0);
  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepare());
  EXPECT_EQ(planner->calls, 3);
  EXPECT_TRUE(critic.detouring());
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanFailed());
  EXPECT_EQ(critic.guidancePlanStartSteadyNs(), critic.limiterStartSteadyNs());
  EXPECT_EQ(critic.guidancePlanEpoch(), epoch);
}

TEST_F(SmacCadenceTest, MeasuresActualAttemptStartsAndDelimitsIndependentDetours)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  critic.replacePlanner(planner);
  critic.setTimingEnabled(true);
  const auto path = pathBehindCube();
  const auto prepare = [&](double x = 0.0) {
      return critic.prepare(pose(x, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };
  EXPECT_EQ(critic.guidancePlanEpoch(), 0U);
  EXPECT_EQ(critic.guidancePlanStartSteadyNs(), 0);
  ASSERT_TRUE(prepare());
  const auto first_start = critic.guidancePlanStartSteadyNs();
  const auto epoch = critic.guidancePlanEpoch();
  ASSERT_GT(first_start, 0);
  EXPECT_EQ(first_start, critic.limiterStartSteadyNs());
  EXPECT_EQ(epoch, 1U);

  ASSERT_TRUE(prepare(.3));
  EXPECT_EQ(planner->calls, 1);
  EXPECT_EQ(critic.guidancePlanStartSteadyNs(), 0);
  EXPECT_EQ(critic.guidancePlanEpoch(), epoch);
  ASSERT_NE(critic.guidanceCandidateTarget(), nullptr);
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  EXPECT_GE(critic.guidanceCandidateTarget()->x - critic.guidanceTarget()->x, .25);
  // Use the real limiter clock here: telemetry must measure start-to-start
  // spacing, rather than controller completion or the duration of createPlan.
  const auto first_time = std::chrono::steady_clock::time_point(
    std::chrono::nanoseconds(first_start));
  std::this_thread::sleep_until(first_time + std::chrono::milliseconds(250));
  ASSERT_TRUE(prepare(.3));
  EXPECT_EQ(planner->calls, 2);
  EXPECT_EQ(critic.guidancePlanStartSteadyNs(), critic.limiterStartSteadyNs());
  EXPECT_GE(critic.guidancePlanStartSteadyNs() - first_start, 250000000LL);
  EXPECT_EQ(critic.guidancePlanEpoch(), epoch);

  clearLaser(laser_);
  for (int cycle = 0; cycle < 3; ++cycle) {
    ASSERT_TRUE(prepare());
    EXPECT_FALSE(critic.detouring());
    EXPECT_EQ(critic.guidancePlanStartSteadyNs(), 0);
    EXPECT_EQ(critic.guidancePlanEpoch(), epoch);
  }
  setLaserCost(laser_, .725, .025);
  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepare());
  EXPECT_EQ(planner->calls, 3);
  EXPECT_EQ(critic.guidancePlanEpoch(), epoch + 1);
  EXPECT_GT(critic.guidancePlanStartSteadyNs(), 0);
  critic.reset();
  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepare());
  EXPECT_EQ(planner->calls, 4);
  EXPECT_EQ(critic.guidancePlanEpoch(), epoch + 2);
  EXPECT_EQ(critic.guidancePlanStartSteadyNs(), critic.limiterStartSteadyNs());
}

TEST_F(SmacCadenceTest, RejectsNewCollisionsBetweenScheduledSearches)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  planner->via = {pose(0, .4), pose(.7, .4)};
  critic.replacePlanner(planner);
  const auto path = pathBehindCube();
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };

  ASSERT_TRUE(prepare());
  EXPECT_EQ(planner->calls, 1);
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  const auto accepted = *critic.guidanceTarget();
  // Block the cached detour beyond its 50 cm lookahead without changing the
  // prospective target on the original reference.
  setCost(.325, .425, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  EXPECT_EQ(planner->calls, 1);
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanFailed());
  EXPECT_FALSE(critic.detouring());
  EXPECT_EQ(critic.guidanceTarget(), nullptr);
  EXPECT_TRUE(critic.guidanceReference(pose(0, 0)).empty());
  ASSERT_NE(critic.guidanceCandidateTarget(), nullptr);
  EXPECT_DOUBLE_EQ(critic.guidanceCandidateTarget()->x, accepted.x);
  EXPECT_DOUBLE_EQ(critic.guidanceCandidateTarget()->y, accepted.y);

  planner->via = {pose(0, -.4), pose(.7, -.4)};
  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepare());
  EXPECT_EQ(planner->calls, 2);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanFailed());
  EXPECT_TRUE(critic.detouring());
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  EXPECT_DOUBLE_EQ(critic.guidanceTarget()->x, accepted.x);
  EXPECT_DOUBLE_EQ(critic.guidanceTarget()->y, accepted.y);
}

TEST_F(SmacCadenceTest, RetainsSafeRouteWhenNoNewCandidateIsAvailable)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  critic.replacePlanner(planner);
  const auto path = pathBehindCube();
  ASSERT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path));
  EXPECT_EQ(planner->calls, 1);
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  const auto accepted = *critic.guidanceTarget();
  // The accepted route ends at x=1.0. Block only the original reference
  // farther ahead, leaving that route and its full footprint collision free.
  auto * map = costmap_->getCostmap();
  for (unsigned int y = 0; y < map->getSizeInCellsY(); ++y) {
    for (unsigned int x = 0; x < map->getSizeInCellsX(); ++x) {
      double wx, wy;
      map->mapToWorld(x, y, wx, wy);
      if (wx >= 1.2 && std::abs(wy) < .05) {
        map->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(critic.prepare(pose(.75, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path));
  EXPECT_TRUE(critic.detouring());
  EXPECT_EQ(planner->calls, 1);
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanFailed());
  EXPECT_EQ(critic.guidanceCandidateTarget(), nullptr);
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  EXPECT_DOUBLE_EQ(critic.guidanceTarget()->x, accepted.x);
  EXPECT_FALSE(critic.guidanceReference(pose(.75, 0)).empty());
}

TEST_F(SmacCadenceTest, KeepsWaitingForASafeDetourOffTheReferenceAfterLaserClears)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  critic.replacePlanner(planner);
  const auto path = pathBehindCube();
  ASSERT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path));
  EXPECT_EQ(planner->calls, 1);

  clearLaser(laser_);
  setCost(.325, .025, nav2_costmap_2d::LETHAL_OBSTACLE);
  const auto prepare = [&]() {
      return critic.prepare(pose(0, .2), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };
  // A stopped detour must remain pending while the robot is off the original
  // path, even after the sensor trigger clears and guidance_active_ is false.
  for (int cycle = 0; cycle < 2; ++cycle) {
    EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
    EXPECT_EQ(planner->calls, 1);
    EXPECT_FALSE(critic.guidancePlanCalled());
    EXPECT_FALSE(critic.detouring());
    EXPECT_EQ(critic.guidanceTarget(), nullptr);
  }

  planner->via = {pose(0, .4), pose(.7, .4)};
  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepare());
  EXPECT_EQ(planner->calls, 2);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanFailed());
  EXPECT_TRUE(critic.detouring());
}

TEST_F(SmacCadenceTest, NewDetourAndReferenceResetRespectSearchCooldown)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  critic.replacePlanner(planner);
  const auto path = pathBehindCube();
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };

  ASSERT_TRUE(prepare());
  EXPECT_EQ(planner->calls, 1);
  const auto first_epoch = critic.guidancePlanEpoch();
  const auto first_start = critic.limiterStartSteadyNs();
  clearLaser(laser_);
  ASSERT_TRUE(prepare());
  EXPECT_FALSE(critic.detouring());
  EXPECT_FALSE(critic.guidancePlanCalled());
  setLaserCost(laser_, .725, .025);
  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  EXPECT_EQ(planner->calls, 1);
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_EQ(critic.limiterStartSteadyNs(), first_start);
  EXPECT_EQ(critic.guidancePlanEpoch(), first_epoch);
  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepare());
  EXPECT_EQ(planner->calls, 2);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_TRUE(critic.detouring());
  EXPECT_EQ(critic.guidancePlanEpoch(), first_epoch + 1);

  critic.reset();
  EXPECT_FALSE(critic.detouring());
  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  EXPECT_EQ(planner->calls, 2);
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_EQ(critic.guidancePlanEpoch(), first_epoch + 1);
  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepare());
  EXPECT_EQ(planner->calls, 3);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_TRUE(critic.detouring());
  EXPECT_EQ(critic.guidancePlanEpoch(), first_epoch + 2);
}

class SmacFailureAdvanceTest : public SmacCadenceTest
{
protected:
  void SetUp() override
  {
    SmacCadenceTest::SetUp();
    node_->set_parameter(rclcpp::Parameter(
      "FollowPath.MppiPath.ObstacleGuidance.Smac.failure_advance_distance", .25));
  }
};

TEST_F(SmacFailureAdvanceTest, AdvancesOnlyAfterActualFailuresWhileTheRobotIsStationary)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  planner->empty = true;
  critic.replacePlanner(planner);
  const auto path = pathBehindCube();
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };

  double previous_goal = 0.0;
  for (int attempt = 1; attempt <= 4; ++attempt) {
    if (attempt > 1) {critic.elapseSmacPlanPeriod();}
    EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
    EXPECT_EQ(planner->calls, attempt);
    EXPECT_TRUE(critic.guidancePlanCalled());
    EXPECT_TRUE(critic.guidancePlanFailed());
    EXPECT_EQ(critic.guidanceTarget(), nullptr);
    const double failed_goal = planner->arrival.pose.position.x;
    if (attempt == 1) {
      EXPECT_NEAR(failed_goal, 1.0, .051);
    } else {
      EXPECT_NEAR(failed_goal - previous_goal, .25, .051);
    }
    previous_goal = failed_goal;

    double next_goal = 0.0;
    for (int cycle = 0; cycle < 3; ++cycle) {
      EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
      EXPECT_EQ(planner->calls, attempt);
      EXPECT_FALSE(critic.guidancePlanCalled());
      EXPECT_FALSE(critic.guidancePlanFailed());
      ASSERT_NE(critic.guidanceCandidateTarget(), nullptr);
      if (cycle == 0) {
        next_goal = critic.guidanceCandidateTarget()->x;
        EXPECT_NEAR(next_goal - failed_goal, .25, .051);
      }
      EXPECT_DOUBLE_EQ(critic.guidanceCandidateTarget()->x, next_goal);
      EXPECT_DOUBLE_EQ(planner->arrival.pose.position.x, failed_goal);
    }
  }
}

TEST_F(SmacFailureAdvanceTest, AdvancesAlongReferenceArcThroughACorner)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  planner->empty = true;
  critic.replacePlanner(planner);
  auto path = straightPath();
  path.poses.resize(23);  // Turn after x=1.1, within the next 25 cm advance.
  for (int i = 1; i <= 20; ++i) {
    path.poses.push_back(pose(1.1, .05 * i, std::acos(-1.0) / 2.0));
  }
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };

  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  const double failed_x = planner->arrival.pose.position.x;
  EXPECT_NEAR(failed_x, 1.0, .051);
  critic.elapseSmacPlanPeriod();
  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  EXPECT_EQ(planner->calls, 2);
  const auto & advanced = planner->arrival.pose.position;
  EXPECT_NEAR(advanced.x, 1.1, 1e-9);
  EXPECT_NEAR((advanced.x - failed_x) + advanced.y, .25, .051);
  EXPECT_GT(advanced.y, .05);
}

TEST_F(SmacFailureAdvanceTest, PreservesForwardProgressAcrossIdenticalReferenceLaps)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  planner->empty = true;
  critic.replacePlanner(planner);
  auto path = straightPath();
  path.poses = {
    pose(0, 0), pose(1, 0), pose(1, 1), pose(0, 1), pose(0, 0),
    pose(1, 0), pose(1, 1), pose(0, 1), pose(0, 0)};
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };

  for (int attempt = 1; attempt <= 9; ++attempt) {
    if (attempt > 1) {critic.elapseSmacPlanPeriod();}
    EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
    EXPECT_EQ(planner->calls, attempt);
    EXPECT_TRUE(critic.guidancePlanFailed());
    const auto index = static_cast<std::size_t>(std::min(attempt, 8));
    EXPECT_DOUBLE_EQ(planner->arrival.pose.position.x, path.poses[index].x);
    EXPECT_DOUBLE_EQ(planner->arrival.pose.position.y, path.poses[index].y);
    // Equal pose and yaw on both laps cannot distinguish a rewind spatially.
    EXPECT_DOUBLE_EQ(critic.failedSmacGoalProgress(), static_cast<double>(index));

    EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
    EXPECT_EQ(planner->calls, attempt);
    EXPECT_FALSE(critic.guidancePlanCalled());
    const auto next = std::min(index + 1, path.poses.size() - 1);
    expectLookahead(critic.guidanceCandidateTarget(), path.poses[next].x, path.poses[next].y);
    EXPECT_DOUBLE_EQ(critic.failedSmacGoalProgress(), static_cast<double>(index));
  }
}

TEST_F(SmacFailureAdvanceTest, ClampsAdvancesAndContinuesRetryingAtTheFinalGoal)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  planner->empty = true;
  critic.replacePlanner(planner);
  auto path = straightPath();
  path.poses.resize(23);  // The final goal at 1.1 is less than one advance away.
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };

  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  EXPECT_LT(planner->arrival.pose.position.x, path.poses.back().x);
  for (int attempt = 2; attempt <= 4; ++attempt) {
    critic.elapseSmacPlanPeriod();
    EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
    EXPECT_EQ(planner->calls, attempt);
    EXPECT_TRUE(critic.guidancePlanCalled());
    EXPECT_TRUE(critic.guidancePlanFailed());
    EXPECT_DOUBLE_EQ(planner->arrival.pose.position.x, path.poses.back().x);
    expectLookahead(critic.guidanceCandidateTarget(), path.poses.back().x);

    EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
    EXPECT_EQ(planner->calls, attempt);
    EXPECT_FALSE(critic.guidancePlanCalled());
    expectLookahead(critic.guidanceCandidateTarget(), path.poses.back().x);
  }
}

TEST_F(SmacFailureAdvanceTest, KeepsASafeCacheAndRetriesAnAdvancedGoalBelowTheMovementThreshold)
{
  node_->set_parameter(rclcpp::Parameter(
    "FollowPath.MppiPath.ObstacleGuidance.Smac.replan_goal_distance", .5));
  setLaserCost(laser_, .325, .025);
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  critic.replacePlanner(planner);
  auto path = pathBehindCube();
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };

  ASSERT_TRUE(prepare());
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  const auto accepted = *critic.guidanceTarget();
  const auto accepted_path = critic.smacPath();
  // An arrival-angle change requests a search without moving its position.
  for (auto & point : path.poses) {point.theta = .5;}
  planner->empty = true;
  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepare());
  EXPECT_EQ(planner->calls, 2);
  EXPECT_TRUE(critic.guidancePlanFailed());
  EXPECT_EQ(critic.smacPath(), accepted_path);
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  EXPECT_DOUBLE_EQ(critic.guidanceTarget()->x, accepted.x);

  // Restore yaw so neither accepted-goal threshold can request the retry.
  for (auto & point : path.poses) {point.theta = 0.0;}
  ASSERT_TRUE(prepare());
  EXPECT_EQ(planner->calls, 2);
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_EQ(critic.smacPath(), accepted_path);
  ASSERT_NE(critic.guidanceCandidateTarget(), nullptr);
  const auto advanced = *critic.guidanceCandidateTarget();
  EXPECT_NEAR(advanced.x - accepted.x, .25, .051);
  EXPECT_LT(advanced.x - accepted.x, .5);
  EXPECT_DOUBLE_EQ(advanced.theta, accepted.theta);

  planner->empty = false;
  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepare());
  EXPECT_EQ(planner->calls, 3);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanFailed());
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  EXPECT_DOUBLE_EQ(critic.guidanceTarget()->x, advanced.x);

  // Clearing the failure after success must retain the forward target floor.
  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepare());
  EXPECT_EQ(planner->calls, 3);
  EXPECT_FALSE(critic.guidancePlanCalled());
  expectLookahead(critic.guidanceCandidateTarget(), advanced.x);
  expectLookahead(critic.guidanceTarget(), advanced.x);
}

TEST_F(SmacFailureAdvanceTest, ClearsFailureAndAdvanceStateForNewDetoursAndReferenceResets)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  planner->empty = true;
  critic.replacePlanner(planner);
  const auto path = pathBehindCube();
  const auto prepare = [&](double x = 0.0) {
      return critic.prepare(pose(x, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };

  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  const double nominal = planner->arrival.pose.position.x;
  planner->empty = false;
  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepare());
  EXPECT_GT(planner->arrival.pose.position.x, nominal + .15);
  clearLaser(laser_);
  ASSERT_TRUE(prepare());
  EXPECT_FALSE(critic.detouring());
  setLaserCost(laser_, .725, .025);
  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepare());
  EXPECT_EQ(planner->calls, 3);
  EXPECT_DOUBLE_EQ(planner->arrival.pose.position.x, nominal);

  planner->empty = true;
  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepare(.3));
  EXPECT_TRUE(critic.guidancePlanFailed());
  planner->empty = false;
  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepare(.3));
  EXPECT_GT(planner->arrival.pose.position.x, nominal + .4);
  critic.reset();
  critic.elapseSmacPlanPeriod();
  ASSERT_TRUE(prepare());
  EXPECT_EQ(planner->calls, 6);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_DOUBLE_EQ(planner->arrival.pose.position.x, nominal);
}

TEST_F(SmacFailureAdvanceTest, SkipsBlockedAdvancePointsAndStopsAtAClippedReferenceEndpoint)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  critic.setTimingEnabled(true);
  auto planner = std::make_shared<RecordingLocalPlanner>();
  planner->empty = true;
  critic.replacePlanner(planner);
  auto path = straightPath();
  const auto final_goal = pose(2.4, 0);  // The local reference is clipped before this goal.
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), final_goal, path);
    };

  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  const double nominal_goal = planner->arrival.pose.position.x;
  const double blocked_x = nominal_goal + .25;
  setCost(blocked_x, 0, nav2_costmap_2d::LETHAL_OBSTACLE);
  critic.elapseSmacPlanPeriod();
  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  // Skip center-free candidates whose footprints still touch the blocked
  // advance point, then dispatch only the fitting clipped endpoint.
  EXPECT_EQ(planner->calls, 2);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_TRUE(critic.guidancePlanFailed());
  EXPECT_GT(critic.guidancePlanMs(), 0.0);
  EXPECT_GT(critic.guidancePlanStartSteadyNs(), 0);
  ASSERT_NE(critic.guidanceCandidateTarget(), nullptr);
  const auto advanced_goal = *critic.guidanceCandidateTarget();
  EXPECT_GT(advanced_goal.x, blocked_x);
  unsigned int mx, my;
  ASSERT_TRUE(costmap_->getCostmap()->worldToMap(
    advanced_goal.x, advanced_goal.y, mx, my));
  EXPECT_TRUE(critic.guidanceTraversable(mx, my));
  EXPECT_DOUBLE_EQ(advanced_goal.x, path.poses.back().x);
  EXPECT_DOUBLE_EQ(planner->arrival.pose.position.x, advanced_goal.x);
  EXPECT_DOUBLE_EQ(critic.failedSmacGoalProgress(), advanced_goal.x);
  EXPECT_FALSE(critic.detouring());

  clearMap();
  critic.elapseSmacPlanPeriod();
  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  EXPECT_EQ(planner->calls, 3);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_TRUE(critic.guidancePlanFailed());
  EXPECT_DOUBLE_EQ(planner->arrival.pose.position.x, path.poses.back().x);
  EXPECT_LT(planner->arrival.pose.position.x, final_goal.x);
  critic.elapseSmacPlanPeriod();
  EXPECT_THROW(prepare(), limo_dwb_critics::GuidanceUnavailable);
  EXPECT_EQ(planner->calls, 4);
  EXPECT_DOUBLE_EQ(planner->arrival.pose.position.x, path.poses.back().x);
}

TEST_P(LocalGuidanceBackendsTest, ReportsCurrentLookaheadsWithoutCalculatingADetour)
{
  node_->set_parameter(rclcpp::Parameter("FollowPath.MppiPath.ObstacleGuidance.use_smac", GetParam()));
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  if (GetParam()) {critic.replacePlanner(planner);}
  const auto path = pathBehindCube();
  EXPECT_EQ(critic.pathLookahead(), nullptr);
  EXPECT_EQ(critic.guidanceLookahead(), nullptr);
  EXPECT_EQ(critic.guidanceCandidateTarget(), nullptr);

  ASSERT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path));
  expectLookahead(critic.pathLookahead(), 1.25);
  expectLookahead(critic.guidanceLookahead(), 1.25);
  expectLookahead(critic.guidanceCandidateTarget(), 1.75);
  EXPECT_FALSE(critic.detouring());
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_EQ(critic.guidanceTarget(), nullptr);
  EXPECT_TRUE(critic.guidanceReference(pose(0, 0)).empty());
  EXPECT_EQ(planner->calls, 0);

  // These are poses considered in this cycle, not a retained detour target.
  ASSERT_TRUE(critic.prepare(pose(.25, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path));
  expectLookahead(critic.pathLookahead(), 1.50);
  expectLookahead(critic.guidanceLookahead(), 1.50);
  expectLookahead(critic.guidanceCandidateTarget(), 2.0);
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_EQ(planner->calls, 0);

  ASSERT_FALSE(critic.prepare(pose(.25, 0), nav_2d_msgs::msg::Twist2D(),
    path.poses.back(), nav_2d_msgs::msg::Path2D()));
  EXPECT_EQ(critic.pathLookahead(), nullptr);
  EXPECT_EQ(critic.guidanceLookahead(), nullptr);
  EXPECT_EQ(critic.guidanceCandidateTarget(), nullptr);
}

TEST_P(LocalGuidanceBackendsTest, HalfMeterLookaheadUsesTheSameTriggerAndTargetForBothBackends)
{
  node_->set_parameter(rclcpp::Parameter("FollowPath.MppiPath.ObstacleGuidance.lookahead_distance", .5));
  node_->set_parameter(rclcpp::Parameter("FollowPath.MppiPath.ObstacleGuidance.use_smac", GetParam()));
  const auto laser = addLaserTriggerLayer();
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  if (GetParam()) {critic.replacePlanner(planner);}
  const auto path = pathBehindCube();
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };
  ASSERT_TRUE(prepare());
  EXPECT_FALSE(critic.detouring());
  EXPECT_NEAR(critic.targetPose().x, 1.25, 1e-9);
  expectLookahead(critic.pathLookahead(), 1.25);
  expectLookahead(critic.guidanceLookahead(), .5);
  expectLookahead(critic.guidanceCandidateTarget(), 1.0);
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_EQ(planner->calls, 0);
  // Shortening laser anticipation must not pull normal four-second
  // predictions toward a 50 cm target and favor the short startup action.
  dwb_msgs::msg::Trajectory2D normal_forward, startup;
  normal_forward.poses = {pose(0, 0), pose(1.0, 0)};
  startup.poses = {pose(0, 0), pose(.3, 0)};
  EXPECT_LT(critic.scoreTrajectory(normal_forward), critic.scoreTrajectory(startup));
  // This cell is beyond the half-meter reference plus the extended front.
  setLaserCost(laser, .825, .025);
  ASSERT_TRUE(prepare());
  EXPECT_FALSE(critic.detouring());
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_NEAR(critic.targetPose().x, 1.25, 1e-9);
  clearLaser(laser);
  // Isolate the laser trigger from fused-grid planning. This mark lies in the
  // 5 cm margin plus conservative cell coverage. Rejoin adds another 50 cm.
  setLaserCost(laser, .725, .025);
  ASSERT_TRUE(prepare());
  ASSERT_TRUE(critic.detouring());
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  EXPECT_NEAR(critic.guidanceTarget()->x, 1.0, .051);
  EXPECT_NEAR(critic.guidanceTarget()->y, 0.0, 1e-9);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanFailed());
  EXPECT_EQ(planner->calls, GetParam() ? 1 : 0);
  expectLookahead(critic.pathLookahead(), 1.25);
  expectLookahead(critic.guidanceLookahead(), .5);
  expectLookahead(critic.guidanceCandidateTarget(), 1.0);
}

TEST_F(MppiOnlineCostsTest, SmacUsesReferenceArrivalYawAndKeepsOnlyACollisionFreeCachedRoute)
{
  node_->declare_parameter("FollowPath.MppiPath.ObstacleGuidance.clearance_margin", 0.0);
  node_->set_parameter(rclcpp::Parameter("FollowPath.MppiPath.ObstacleGuidance.clearance_margin", 0.0));
  node_->set_parameter(rclcpp::Parameter("FollowPath.MppiPath.ObstacleGuidance.use_smac", true));
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  critic.replacePlanner(planner);
  auto path = pathBehindCube();
  for (auto & point : path.poses) {point.theta = 0.4;}
  // A valid nearby obstacle triggers guidance without obstructing the straight route.
  setCost(0.525, 0.175, 254);
  ASSERT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), path));
  ASSERT_TRUE(critic.detouring());
  EXPECT_EQ(planner->arrival.header.frame_id, "odom");
  EXPECT_NEAR(planner->arrival.pose.orientation.z, std::sin(0.2), 1e-12);
  EXPECT_NEAR(planner->arrival.pose.orientation.w, std::cos(0.2), 1e-12);
  planner->empty = true;
  EXPECT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), path));
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_TRUE(critic.guidancePlanFailed());
  EXPECT_TRUE(critic.detouring());
  EXPECT_FALSE(critic.hasDijkstraField());
  EXPECT_FALSE(critic.smacPath().empty());
  const auto calls = planner->calls;
  // Every active cycle retries the planner, and an unsafe cached route is rejected.
  setCost(0.325, 0.025, 254);
  EXPECT_THROW(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), path),
    limo_dwb_critics::GuidanceUnavailable);
  EXPECT_EQ(planner->calls, calls + 1);
  EXPECT_FALSE(critic.detouring());
  // A failed search still exports the original points considered this cycle.
  expectLookahead(critic.pathLookahead(), 1.25, 0.0, .4);
  expectLookahead(critic.guidanceLookahead(), 1.25, 0.0, .4);
  expectLookahead(critic.guidanceCandidateTarget(), 1.75, 0.0, .4);
  // The next cycle can recover without a new navigation goal.
  setCost(0.325, 0.025, nav2_costmap_2d::FREE_SPACE);
  planner->empty = false;
  EXPECT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), path));
  EXPECT_EQ(planner->calls, calls + 2);
  EXPECT_TRUE(critic.detouring());
}

TEST_P(LocalGuidanceBackendsTest, UsesTheLocalLookaheadAndRobotConnectorOnEveryActiveCycle)
{
  node_->set_parameter(rclcpp::Parameter("FollowPath.MppiPath.ObstacleGuidance.use_smac", GetParam()));
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  if (GetParam()) {critic.replacePlanner(planner);}
  const auto path = pathBehindCube();
  const auto prepare = [&](const geometry_msgs::msg::Pose2D & robot) {
      return critic.prepare(robot, nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };
  // This laser-sized lethal cell is beyond the footprint band of the 1.25 m
  // local lookahead. A longer remaining reference must not trigger avoidance.
  setCost(1.625, .175, 254);
  ASSERT_TRUE(prepare(pose(0, 0)));
  EXPECT_FALSE(critic.detouring());
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_EQ(planner->calls, 0);
  clearMap();
  // The robot-to-reference connector is part of the trigger, even when the
  // footprint swept along the reference itself never reaches this cell.
  setCost(-.125, .325, 254);
  for (int cycle = 0; cycle < 3; ++cycle) {
    ASSERT_TRUE(prepare(pose(0, .5)));
    EXPECT_TRUE(critic.detouring());
    EXPECT_TRUE(critic.guidancePlanCalled());
    EXPECT_FALSE(critic.guidancePlanFailed());
    ASSERT_NE(critic.guidanceTarget(), nullptr);
    EXPECT_NEAR(critic.guidanceTarget()->x, 1.75, 1e-9);
    EXPECT_NEAR(critic.guidanceTarget()->y, 0.0, 1e-9);
  }
  EXPECT_EQ(planner->calls, GetParam() ? 3 : 0);
  clearMap();
  // Removing the detection does not end an ongoing detour while the robot
  // remains farther than 15 cm from the original local reference.
  ASSERT_TRUE(prepare(pose(0, .2)));
  EXPECT_TRUE(critic.detouring());
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_EQ(planner->calls, GetParam() ? 4 : 0);
  ASSERT_TRUE(prepare(pose(0, .1)));
  EXPECT_FALSE(critic.detouring());
  EXPECT_FALSE(critic.guidancePlanCalled());
  EXPECT_EQ(critic.guidanceTarget(), nullptr);
  EXPECT_TRUE(critic.guidanceReference(pose(0, .1)).empty());
}

TEST_P(LocalGuidanceBackendsTest, RejoinTargetAdvancesFromLookaheadAlongTheReference)
{
  costmap_->getCostmap()->resizeMap(250, 250, .02, -2.5, -2.5);
  clearMap();
  node_->set_parameter(rclcpp::Parameter("FollowPath.MppiPath.ObstacleGuidance.use_smac", GetParam()));
  const auto laser = addLaserTriggerLayer();
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  if (GetParam()) {critic.replacePlanner(planner);}
  nav_2d_msgs::msg::Path2D reference;
  reference.header.frame_id = "odom";
  for (int i = 0; i <= 20; ++i) {reference.poses.push_back(pose(i * .05, 0));}
  for (int i = 1; i <= 20; ++i) {reference.poses.push_back(pose(1, i * .05));}
  for (int i = 1; i <= 20; ++i) {reference.poses.push_back(pose(1 + i * .05, 1));}
  // The target/scoring comparison needs both candidate center paths free.
  setLaserCost(laser, .51, -.09);
  ASSERT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), reference.poses.back(), reference));
  ASSERT_TRUE(critic.detouring());
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  // The 1.25 m lookahead is (1,.25); the next free 50 cm follows the bend.
  // The target stays on a reference sample, so allow one 5 cm interval.
  EXPECT_NEAR(critic.guidanceTarget()->x, 1.0, 1e-9);
  EXPECT_NEAR(critic.guidanceTarget()->y, .75, .051);
  EXPECT_NEAR(critic.guidanceTarget()->theta, 0.0, 1e-9);
  // SMAC replaces path_ with its diagonal reference. Diagnostic points must
  // retain the original bent reference used to decide where to rejoin.
  expectLookahead(critic.pathLookahead(), 1.0, .25);
  expectLookahead(critic.guidanceLookahead(), 1.0, .25);
  expectLookahead(critic.guidanceCandidateTarget(), 1.0, critic.guidanceTarget()->y);
  if (GetParam()) {
    // The MPC attraction and terminal tube still follow the selected SMAC
    // reference, and its suffix continues beyond the temporary target.
    EXPECT_NEAR(critic.activeGuidancePoints().back()[0], 2.0, 1e-9);
    EXPECT_NEAR(critic.activeGuidancePoints().back()[1], 1.0, 1e-9);
    const auto middle = pose(.5, .5 * critic.guidanceTarget()->y);
    EXPECT_TRUE(critic.guidanceTubeContains(middle, .3));
    dwb_msgs::msg::Trajectory2D on_smac, on_global;
    on_smac.poses = {pose(0, 0), middle};
    on_global.poses = {pose(0, 0), pose(.5, 0)};
    EXPECT_LT(critic.scoreTrajectory(on_smac), critic.scoreTrajectory(on_global));
  }
}

TEST_P(LocalGuidanceBackendsTest, RejoinSkipsBlockedLookaheadAndStopsAtTheNextBlockedRun)
{
  costmap_->getCostmap()->resizeMap(250, 250, .02, -2.5, -2.5);
  clearMap();
  node_->set_parameter(rclcpp::Parameter("FollowPath.MppiPath.ObstacleGuidance.use_smac", GetParam()));
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  auto planner = std::make_shared<RecordingLocalPlanner>();
  planner->via = {pose(.5, -.4), pose(1.45, -.4)};
  if (GetParam()) {critic.replacePlanner(planner);}
  const auto path = pathBehindCube();
  for (double x = .95; x < 1.35; x += .02) {
    for (double y = -.05; y < .05; y += .02) {setCost(x, y, 254);}
  }
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), path.poses.back(), path);
    };
  ASSERT_TRUE(prepare());
  ASSERT_TRUE(critic.detouring());
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  // The target starts at the first traversable point past the occupied
  // lookahead and leaves about 50 cm of free reference for the rejoin.
  const double unobstructed_target = critic.guidanceTarget()->x;
  EXPECT_GE(unobstructed_target, 1.9);
  EXPECT_LE(unobstructed_target, 2.05);
  // A subsequent inscribed-cost run is blocked in the shared guidance grid,
  // without placing a new lethal cell inside the chosen arrival footprint.
  for (double x = 1.7; x < 1.9; x += .02) {
    for (double y = -.05; y < .05; y += .02) {setCost(x, y, 253);}
  }
  ASSERT_TRUE(prepare());
  ASSERT_TRUE(critic.detouring());
  ASSERT_NE(critic.guidanceTarget(), nullptr);
  EXPECT_GT(critic.guidanceTarget()->x, 1.4);
  EXPECT_LT(critic.guidanceTarget()->x, 1.7);
  EXPECT_LT(critic.guidanceTarget()->x, unobstructed_target);
  EXPECT_TRUE(critic.guidancePlanCalled());
  EXPECT_FALSE(critic.guidancePlanFailed());
  EXPECT_EQ(planner->calls, GetParam() ? 2 : 0);
}

TEST_F(MppiCostsTest, GuidanceGridTracksOffPathChangesEvenWhileReferenceIsFree)
{
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  critic.setTimingEnabled(true);
  const auto path = straightPath();
  unsigned int x, y;
  ASSERT_TRUE(costmap_->getCostmap()->worldToMap(0.0, 1.0, x, y));
  const auto prepare = [&]() {
      ASSERT_TRUE(critic.prepare(
          pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(2, 0), path));
      EXPECT_FALSE(critic.detouring());
      EXPECT_TRUE(critic.gridUpdated());
      EXPECT_GE(critic.guidanceGridMs(), 0.0);
    };
  prepare();
  EXPECT_TRUE(critic.guidanceTraversable(x, y));
  setCost(0.0, 1.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  prepare();
  EXPECT_FALSE(critic.guidanceTraversable(x, y));
  clearMap();
  prepare();
  EXPECT_TRUE(critic.guidanceTraversable(x, y));
}

TEST_F(MppiCostsTest, PathCriticDisambiguatesNearbyPosesUsingChassisHeading)
{
  const double pi = std::acos(-1.0);
  nav_2d_msgs::msg::Path2D reference;
  reference.poses = {pose(-1, 0), pose(0, 0), pose(1, 0),
    pose(1, 0.05, pi), pose(0, 0.05, pi), pose(-1, 0.05, pi)};
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  ASSERT_TRUE(
    critic.prepare(
      pose(0, 0.01, pi), nav_2d_msgs::msg::Twist2D(),
      reference.poses.back(), reference));
  EXPECT_EQ(critic.startIndex(), 4U);
  ASSERT_TRUE(
    critic.prepare(
      pose(0, 0.04, 0), nav_2d_msgs::msg::Twist2D(),
      reference.poses.back(), reference));
  EXPECT_EQ(critic.startIndex(), 1U);
}

class InspectablePlanController : public limo_dwb_critics::AckermannMPCController
{
public:
  using AckermannMPCController::transformGlobalPlan;
};

TEST_F(MppiCostsTest, TrackedCriticDoesNotSkipAheadToAFutureLapWithIdenticalYaw)
{
  nav_2d_msgs::msg::Path2D reference;
  reference.poses = {pose(0, 0), pose(1, 0), pose(1, 1), pose(0, 1),
    pose(0, 0.05), pose(1, 0.05)};
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  critic.setTrackedReference(true);
  ASSERT_TRUE(
    critic.prepare(
      pose(0, 0.04), nav_2d_msgs::msg::Twist2D(),
      reference.poses.back(), reference));
  EXPECT_EQ(critic.startIndex(), 0U);
}

TEST_F(MppiCostsTest, DwbLocalReferenceKeepsTheCompletedLapEvenAfterRepublishing)
{
  InspectablePlanController controller;
  auto tf = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  controller.configure(node_, "FollowPath", tf, costmap_);
  nav_2d_msgs::msg::Path2D reference;
  reference.header.frame_id = "odom";
  const double pi = std::acos(-1.0);
  for (int i = 0; i <= 200; ++i) {
    const double angle = 2 * pi * i / 200;
    reference.poses.push_back(pose(0.5 * std::sin(angle), 0.5 * (1 - std::cos(angle)), angle));
  }
  for (int i = 1; i <= 30; ++i) {
    reference.poses.push_back(pose(i * 0.05, 0, 0));
  }
  auto path = nav_2d_utils::pathToPath(reference);
  controller.setPlan(path);
  nav_2d_msgs::msg::Pose2DStamped robot;
  robot.header.frame_id = "odom";
  for (int i = 0; i <= 200; ++i) {
    robot.pose = reference.poses[i];
    ASSERT_FALSE(controller.transformGlobalPlan(robot).poses.empty());
  }
  ++path.header.stamp.sec;
  controller.setPlan(path);
  const auto local = controller.transformGlobalPlan(robot);
  ASSERT_FALSE(local.poses.empty());
  EXPECT_GT(local.poses.back().x, 1.0);
  for (const auto & point : local.poses) {
    EXPECT_NEAR(point.y, 0.0, 0.01);
    EXPECT_GT(point.x, -0.03);
  }
  reference.poses.back().x += 0.1;
  controller.setPlan(nav_2d_utils::pathToPath(reference));
  const auto restarted = controller.transformGlobalPlan(robot);
  ASSERT_FALSE(restarted.poses.empty());
  bool contains_loop = false;
  for (const auto & point : restarted.poses) {
    contains_loop |= point.y > 0.5;
  }
  EXPECT_TRUE(contains_loop);
  controller.cleanup();
}

TEST_F(MppiOnlineCostsTest, ObstaclesRejectCollidingPredictionsWhileDetourGuidanceRemainsEnabled)
{
  addCube();
  limo_dwb_critics::MppiObstacleCritic obstacle;
  InspectableMppiPathCritic path;
  initialize(obstacle, "MppiObstacle");
  initialize(path, "MppiPath");
  const auto reference = pathBehindCube();
  ASSERT_TRUE(obstacle.prepare(pose(0.0, 0.0), nav_2d_msgs::msg::Twist2D(),
    reference.poses.back(), reference));
  ASSERT_TRUE(path.prepare(pose(0.0, 0.0), nav_2d_msgs::msg::Twist2D(),
    reference.poses.back(), reference));
  EXPECT_TRUE(path.detouring());
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.velocity.x = 0.1;
  trajectory.poses = {pose(1.0, 0.0), pose(1.1, 0.0)};
  EXPECT_THROW(obstacle.scoreTrajectory(trajectory), dwb_core::IllegalTrajectoryException);
  EXPECT_THROW(path.scoreTrajectory(trajectory), dwb_core::IllegalTrajectoryException);
}

TEST_F(MppiOnlineCostsTest, ControllerKeepsCollisionChecksThroughoutThePlan)
{
  limo_dwb_critics::AckermannMPCController controller;
  auto tf = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  controller.configure(node_, "FollowPath", tf, costmap_);
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "odom";
  plan.header.stamp.sec = 123;
  for (int i = 0; i <= 75; ++i) {
    geometry_msgs::msg::PoseStamped point;
    point.header = plan.header;
    point.pose.position.x = i * 0.02;
    point.pose.orientation.w = 1.0;
    plan.poses.push_back(point);
  }
  controller.setPlan(plan);
  controller.activate();
  setCost(0.0, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(controller.computeVelocityCommands(
    plan.poses.front(), geometry_msgs::msg::Twist()), dwb_core::NoLegalTrajectoriesException);
  clearMap();
  for (std::size_t i = 1; i < 20; ++i) {
    EXPECT_NO_THROW(controller.computeVelocityCommands(plan.poses[i], geometry_msgs::msg::Twist()));
  }
  setCost(0.4, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(controller.computeVelocityCommands(plan.poses[20], geometry_msgs::msg::Twist()),
    dwb_core::NoLegalTrajectoriesException);
  controller.deactivate();
  controller.cleanup();
}

TEST_F(MppiOnlineCostsTest, PublishesBoundedCoherentSnapshotWithTheSelectedTrajectory)
{
  setCost(1.5, 1.5, 200);
  auto receiver = std::make_shared<rclcpp::Node>("mpc_preview_receiver");
  limo_interfaces::msg::MpcDebug::SharedPtr snapshot;
  auto subscription = receiver->create_subscription<limo_interfaces::msg::MpcDebug>(
    "/limo/control/mpc_debug", rclcpp::QoS(1).best_effort(),
    [&](limo_interfaces::msg::MpcDebug::SharedPtr msg) {snapshot = msg;});
  limo_dwb_critics::AckermannMPCController controller;
  auto tf = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  controller.configure(node_, "FollowPath", tf, costmap_);
  controller.activate();
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";
  for (const auto & point : pathBehindCube().poses) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = path.header;
    stamped.pose.position.x = point.x;
    stamped.pose.orientation.w = 1.0;
    path.poses.push_back(stamped);
  }
  controller.setPlan(path);
  geometry_msgs::msg::PoseStamped current;
  current.header.frame_id = "odom";
  current.pose.orientation.w = 1.0;
  for (int attempt = 0; attempt < 100 && !snapshot; ++attempt) {
    controller.computeVelocityCommands(current, geometry_msgs::msg::Twist());
    rclcpp::spin_some(receiver);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_NE(snapshot, nullptr);
  EXPECT_EQ(snapshot->header.frame_id, "odom");
  EXPECT_EQ(snapshot->base_frame, "base_link");
  EXPECT_DOUBLE_EQ(snapshot->wheelbase, 0.24);
  EXPECT_DOUBLE_EQ(snapshot->rear_axle_to_base, 0.12);
  EXPECT_EQ(snapshot->costmap.data.size(), 10000U);
  const auto * map = costmap_->getCostmap();
  EXPECT_TRUE(std::equal(snapshot->costmap.data.begin(), snapshot->costmap.data.end(),
    map->getCharMap()));
  EXPECT_EQ(snapshot->generated_count, 768U);
  EXPECT_EQ(snapshot->time_steps, 80U);
  EXPECT_LE(snapshot->candidates.size(), 21U);
  ASSERT_GE(snapshot->selected_id, 0);
  bool found_winner = false;
  std::vector<int> families(5);
  for (const auto & candidate : snapshot->candidates) {
    ASSERT_LT(candidate.family, families.size());
    ++families[candidate.family];
    ASSERT_EQ(candidate.poses.size(), 41U);
    EXPECT_DOUBLE_EQ(candidate.poses.front().x, snapshot->robot_pose.x);
    EXPECT_DOUBLE_EQ(candidate.poses.front().y, snapshot->robot_pose.y);
    if (candidate.candidate_id == static_cast<std::uint32_t>(snapshot->selected_id)) {
      found_winner = true;
      EXPECT_EQ(candidate.score_status, candidate.SCORED);
      EXPECT_TRUE(std::isfinite(candidate.total_cost));
    }
  }
  EXPECT_TRUE(found_winner);
  for (int count : families) {EXPECT_LE(count, 7);}
  controller.deactivate();
  controller.cleanup();
}

class MppiGuidanceProfilesTest : public MppiOnlineCostsTest,
  public ::testing::WithParamInterface<const char *>
{
protected:
  const char * parameterFile() const override {return GetParam();}
};

INSTANTIATE_TEST_CASE_P(
  RealAndSimulation, MppiGuidanceProfilesTest,
  ::testing::Values(LIMO_MPC_PARAMS_PATH, LIMO_MPC_SIM_PARAMS_PATH));

TEST_P(MppiGuidanceProfilesTest, UniformMarginTriggersBeforeTheFootprintReachesTheCube)
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
  // The 5 cm side margin and cell area trigger before the body hits the cube.
  EXPECT_TRUE(critic.detouring());
  dwb_msgs::msg::Trajectory2D stopped;
  stopped.poses = {pose(0, 0.66), pose(0, 0.66)};
  dwb_msgs::msg::Trajectory2D forward;
  forward.poses = {pose(0, 0.66), pose(0.5, 0.66)};
  EXPECT_LT(critic.scoreTrajectory(forward), critic.scoreTrajectory(stopped));
}

TEST_P(MppiGuidanceProfilesTest, OnlyLaserMarksWithinFiveCentimetersOfTheFootprintStartDetours)
{
  node_->set_parameter(rclcpp::Parameter("FollowPath.MppiPath.ObstacleGuidance.lookahead_distance", .5));
  const auto laser = addLaserTriggerLayer();
  for (const bool use_smac : {false, true}) {
    clearMap();
    clearLaser(laser, nav2_costmap_2d::NO_INFORMATION);
    node_->set_parameter(rclcpp::Parameter("FollowPath.MppiPath.ObstacleGuidance.use_smac", use_smac));
    InspectableMppiPathCritic critic;
    initialize(critic, "MppiPath");
    const auto prepare = [&]() {
        return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), pathBehindCube());
      };
    // A semantic/static lethal cell in the master is not a laser detection;
    // unobserved sensor cells (255) must not initiate a detour either.
    setCost(0.525, 0.025, nav2_costmap_2d::LETHAL_OBSTACLE);
    ASSERT_TRUE(prepare());
    EXPECT_FALSE(critic.detouring());
    EXPECT_FALSE(critic.guidancePlanCalled());
    EXPECT_NEAR(critic.targetPose().x, 1.25, 1e-9);
    limo_dwb_critics::MppiObstacleCritic obstacle;
    initialize(obstacle, "MppiObstacle");
    ASSERT_TRUE(obstacle.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(2.4, 0), pathBehindCube()));
    dwb_msgs::msg::Trajectory2D collision;
    collision.poses = {pose(0.525, 0.025), pose(0.525, 0.025)};
    EXPECT_THROW(obstacle.scoreTrajectory(collision), dwb_core::IllegalTrajectoryException);
    clearMap();
    // Cells beyond the uniform margin must not trigger, including at corners.
    for (const auto & mark : {pose(.525, .225), pose(-.275, .025), pose(.775, .025),
      pose(-.225, .175)})
    {
      clearLaser(laser);
      setLaserCost(laser, mark.x, mark.y);
      ASSERT_TRUE(prepare());
      EXPECT_FALSE(critic.detouring());
      EXPECT_FALSE(critic.guidancePlanCalled());
    }
    // Sensor-only marks isolate triggering from fused-grid planning. The
    // conservative cell envelope applies to the front, both sides and rear.
    for (const auto & mark : {pose(.725, .025), pose(.525, .175),
      pose(.525, -.175), pose(-.225, .025)})
    {
      clearLaser(laser);
      setLaserCost(laser, mark.x, mark.y);
      ASSERT_TRUE(prepare());
      EXPECT_TRUE(critic.detouring());
      EXPECT_TRUE(critic.guidancePlanCalled());
      EXPECT_FALSE(critic.guidancePlanFailed());
      critic.reset();
      clearLaser(laser);
      ASSERT_TRUE(prepare());
      EXPECT_FALSE(critic.detouring());
      EXPECT_FALSE(critic.guidancePlanCalled());
    }
  }
}

TEST_F(MppiCostsTest, MissingLaserTriggerLayerCannotFallBackToTheMaster)
{
  node_->set_parameter(rclcpp::Parameter(
    "FollowPath.MppiPath.ObstacleGuidance.trigger_layer", std::string("obstacle_layer")));
  InspectableMppiPathCritic critic;
  EXPECT_THROW(initialize(critic, "MppiPath"), std::invalid_argument);
}

TEST_F(MppiCostsTest, ZeroTriggerMarginStillChecksTheOriginalFootprint)
{
  node_->set_parameter(rclcpp::Parameter("FollowPath.MppiPath.ObstacleGuidance.lookahead_distance", .5));
  node_->declare_parameter("FollowPath.MppiPath.ObstacleGuidance.trigger_front_extension", 0.0);
  node_->set_parameter(rclcpp::Parameter(
    "FollowPath.MppiPath.ObstacleGuidance.trigger_front_extension", 0.0));
  node_->declare_parameter("FollowPath.MppiPath.ObstacleGuidance.trigger_margin", 0.0);
  node_->set_parameter(rclcpp::Parameter(
    "FollowPath.MppiPath.ObstacleGuidance.trigger_margin", 0.0));
  const auto laser = addLaserTriggerLayer();
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  const auto reference = straightPath();
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(),
        reference.poses.back(), reference);
    };
  setLaserCost(laser, .725, .025);
  ASSERT_TRUE(prepare());
  EXPECT_FALSE(critic.detouring());
  clearLaser(laser);
  setLaserCost(laser, .625, .025);
  ASSERT_TRUE(prepare());
  EXPECT_TRUE(critic.detouring());
}

TEST_F(MppiCostsTest, EarlyDetourDoesNotRejectEscapeFromProximityBand)
{
  // Use cell centers: a boundary coordinate can round into the adjacent cell.
  setCost(0.275, 0.075, nav2_costmap_2d::LETHAL_OBSTACLE);
  InspectableMppiPathCritic critic;
  limo_dwb_critics::MppiObstacleCritic obstacle;
  initialize(critic, "MppiPath");
  initialize(obstacle, "MppiObstacle");
  const auto reference = straightPath();
  ASSERT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(),
    reference.poses.back(), reference));
  ASSERT_TRUE(critic.detouring());
  ASSERT_TRUE(obstacle.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(),
    reference.poses.back(), reference));
  dwb_msgs::msg::Trajectory2D escape;
  escape.poses = {pose(0, 0), pose(0.05, -0.03), pose(0.10, -0.10)};
  EXPECT_TRUE(std::isfinite(critic.scoreTrajectory(escape)));
  EXPECT_TRUE(std::isfinite(obstacle.scoreTrajectory(escape)));
  dwb_msgs::msg::Trajectory2D collision;
  collision.poses = {pose(0, 0), pose(0.275, 0.075)};
  EXPECT_THROW(obstacle.scoreTrajectory(collision), dwb_core::IllegalTrajectoryException);
}

TEST_F(MppiCostsTest, EarlyDetourUsesOrientedFootprintAndIgnoresSoftCosts)
{
  auto reference = straightPath();
  for (auto & point : reference.poses) {
    point.y = point.x;
    point.x = 0.0;
    point.theta = std::acos(-1.0) / 2.0;
  }
  InspectableMppiPathCritic critic;
  initialize(critic, "MppiPath");
  const auto prepare = [&]() {
      return critic.prepare(reference.poses.front(), nav_2d_msgs::msg::Twist2D(),
        reference.poses.back(), reference);
    };
  // After rotation the lateral half-width is 0.11 m, not 0.161 m.
  setCost(0.20, 0.55, nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(prepare());
  EXPECT_FALSE(critic.detouring());
  clearMap();
  // The margin also covers the footprint's front after rotation into world +y.
  setCost(0.025, 1.475, 120);  // Inflation alone does not trigger the proximity test.
  ASSERT_TRUE(prepare());
  EXPECT_FALSE(critic.detouring());
  setCost(0.025, 1.475, nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(prepare());
  EXPECT_TRUE(critic.detouring());
}

TEST_P(MppiGuidanceProfilesTest, FinishesDetourBeforeRestoringReferenceAlignment)
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

TEST_P(MppiGuidanceProfilesTest, DetourHeadingFollowsMotionInForwardAndReverse)
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

TEST_P(MppiGuidanceProfilesTest, OnlineCubeRewardsDetourAndThenRestoresOriginalPathTracking)
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
  EXPECT_DOUBLE_EQ(critic.getScale(), 12.0);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(trajectory) * critic.getScale(), 6.0);
  trajectory.poses.assign(51, pose(0, 0));
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(trajectory) * critic.getScale(), 6.0);
  costmap_->getCostmap()->resizeMap(50, 50, 0.1, -2.5, -2.5);
  clearMap();
  setCost(0.0, 0.0, 127);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(trajectory) * critic.getScale(), 6.0);
}

TEST_P(MppiGuidanceProfilesTest, NearGoalRepulsionMatchesProfileWithoutDisablingCollisionRejection)
{
  limo_dwb_critics::MppiObstacleCritic critic;
  initialize(critic, "MppiObstacle");
  // The strict distance gate keeps repulsion active at exactly 15 cm.
  ASSERT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(0.15, 0), straightPath()));
  dwb_msgs::msg::Trajectory2D trajectory;
  trajectory.poses.assign(5, pose(0, 0));
  setCost(0.0, 0.0, 200);
  EXPECT_NEAR(critic.scoreTrajectory(trajectory), 200.0 / 254.0, 1e-12);
  const bool real_profile = std::string(GetParam()) == LIMO_MPC_PARAMS_PATH;
  // Real retains finite repulsion up to the goal; simulation keeps its 15 cm gate.
  for (const double goal_distance : {0.14, 0.0}) {
    ASSERT_TRUE(critic.prepare(
        pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(goal_distance, 0), straightPath()));
    EXPECT_NEAR(critic.scoreTrajectory(trajectory), real_profile ? 200.0 / 254.0 : 0.0, 1e-12);
  }
  setCost(0.0, 0.0, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_NEAR(critic.scoreTrajectory(trajectory), 300.0 / 254.0, 1e-12);
  setCost(0.0, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(critic.scoreTrajectory(trajectory), dwb_core::IllegalTrajectoryException);
  setCost(0.0, 0.0, nav2_costmap_2d::NO_INFORMATION);
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

TEST_F(MppiCostsTest, RewardsProgressAndRelaxesAlignmentWithoutObstacleGuidance)
{
  // Custom profiles may disable guidance and retain occupancy-based scoring.
  node_->declare_parameter("FollowPath.MppiPath.ObstacleGuidance.enabled", true);
  node_->set_parameter(rclcpp::Parameter("FollowPath.MppiPath.ObstacleGuidance.enabled", false));
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
  EXPECT_LE(command.twist.linear.x,
    1.3 / node_->get_parameter("controller_frequency").as_double() + 1e-12);
  EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
  EXPECT_LE(std::abs(command.twist.angular.z), command.twist.linear.x / 0.462 + 1e-12);
  controller.deactivate();
  controller.cleanup();
}

class StructuredPipelineTest : public MppiCostsTest
{
protected:
  bool structuredPipeline() const override {return true;}
  void SetUp() override
  {
    MppiCostsTest::SetUp();
    costmap_->getCostmap()->resizeMap(250, 250, 0.02, -2.5, -2.5);
    clearMap();
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    costmap_->declare_parameter("path_distance_layer.max_cost", 70);
    tube_ = std::make_shared<limo_inflation::PathDistanceLayer>();
    tube_->initialize(costmap_->getLayeredCostmap(), "path_distance_layer", tf_.get(),
      costmap_, nullptr, nullptr);
    costmap_->getLayeredCostmap()->addPlugin(tube_);
    reference_.header.frame_id = "odom";
    for (int i = -20; i <= 40; ++i) {
      geometry_msgs::msg::PoseStamped point;
      point.header = reference_.header;
      point.pose.position.x = i * 0.05;
      point.pose.orientation.w = 1.0;
      reference_.poses.push_back(point);
    }
    tube_->setPath(std::make_shared<nav_msgs::msg::Path>(reference_));
    costmap_->getLayeredCostmap()->updateMap(0.0, 0.0, 0.0);
  }
  geometry_msgs::msg::PoseStamped current(double y = 0.0)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    return pose;
  }
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<limo_inflation::PathDistanceLayer> tube_;
  nav_msgs::msg::Path reference_;
};

TEST_F(StructuredPipelineTest, RuntimeProfileRunsStructuredSearchAndRejectsOutsideStart)
{
  limo_dwb_critics::AckermannMPCController controller;
  ASSERT_NO_THROW(controller.configure(node_, "FollowPath", tf_, costmap_));
  controller.activate();
  controller.setPlan(reference_);
  EXPECT_THROW(controller.computeVelocityCommands(current(0.8), geometry_msgs::msg::Twist()),
    nav2_core::PlannerException);
  controller.setPlan(reference_);
  const auto begin = std::chrono::steady_clock::now();
  const auto command = controller.computeVelocityCommands(current(), geometry_msgs::msg::Twist());
  const auto ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - begin).count();
  std::cout << "Structured full pipeline, up to 59 candidates, 40 steps: " << ms << " ms\n";
  EXPECT_GT(command.twist.linear.x, 0.0);
  EXPECT_LE(command.twist.linear.x, 0.13 + 1e-9);
  EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
  EXPECT_EQ(node_->get_parameter("FollowPath.MPC.batch_size").as_int(), 59);
  EXPECT_TRUE(node_->get_parameter("FollowPath.MPC.Tube.enabled").as_bool());
  controller.deactivate();
  controller.cleanup();
}

TEST_F(StructuredPipelineTest, FootprintInteriorAndMotionBetweenStatesAreChecked)
{
  limo_dwb_critics::MppiObstacleCritic critic;
  initialize(critic, "MppiObstacle");
  const auto path = straightPath();
  setCost(0.075, 0.025, nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(2, 0), path));
  dwb_msgs::msg::Trajectory2D stopped;
  stopped.poses = {pose(0, 0), pose(0, 0)};
  EXPECT_THROW(critic.scoreTrajectory(stopped), dwb_core::IllegalTrajectoryException);
  clearMap();
  setCost(0.525, 0.025, nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(2, 0), path));
  dwb_msgs::msg::Trajectory2D sparse;
  sparse.poses = {pose(0, 0), pose(1.0, 0)};
  EXPECT_THROW(critic.scoreTrajectory(sparse), dwb_core::IllegalTrajectoryException);
  clearMap();
  setCost(0.075, 0.025, nav2_costmap_2d::NO_INFORMATION);
  ASSERT_TRUE(critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(2, 0), path));
  EXPECT_THROW(critic.scoreTrajectory(stopped), dwb_core::IllegalTrajectoryException);
}

TEST_F(StructuredPipelineTest, ReusedCollisionStorageCannotKeepRejectedOrOldCells)
{
  limo_dwb_critics::MppiObstacleCritic critic;
  initialize(critic, "MppiObstacle");
  const auto path = straightPath();
  const auto prepare = [&]() {
      return critic.prepare(pose(0, 0), nav_2d_msgs::msg::Twist2D(), pose(2, 0), path);
    };
  dwb_msgs::msg::Trajectory2D safe;
  safe.poses = {pose(0, 0), pose(0.1, 0)};
  setCost(0.075, 0.025, nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(prepare());
  EXPECT_THROW(critic.scoreTrajectory(safe), dwb_core::IllegalTrajectoryException);
  clearMap();
  ASSERT_TRUE(prepare());
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(safe), 0.0);
  dwb_msgs::msg::Trajectory2D outside;
  outside.poses = {pose(0, 0), pose(3.0, 0)};
  EXPECT_THROW(critic.scoreTrajectory(outside), dwb_core::IllegalTrajectoryException);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(safe), 0.0);
  setCost(0.075, 0.025, nav2_costmap_2d::NO_INFORMATION);
  ASSERT_TRUE(prepare());
  EXPECT_THROW(critic.scoreTrajectory(safe), dwb_core::IllegalTrajectoryException);
  clearMap();
  ASSERT_TRUE(prepare());
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(safe), 0.0);
}

TEST_F(StructuredPipelineTest, NoCollisionFreeCandidateRaisesRatherThanReusingACommand)
{
  limo_dwb_critics::AckermannMPCController controller;
  controller.configure(node_, "FollowPath", tf_, costmap_);
  controller.activate();
  controller.setPlan(reference_);
  ASSERT_NO_THROW(controller.computeVelocityCommands(current(), geometry_msgs::msg::Twist()));
  setCost(0.075, 0.025, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(controller.computeVelocityCommands(current(), geometry_msgs::msg::Twist()),
    dwb_core::NoLegalTrajectoriesException);
  controller.deactivate();
  controller.cleanup();
}

TEST_F(StructuredPipelineTest, ForwardBlockActionsCanGoAroundAnObstacleAndReturnToTube)
{
  limo_dwb_critics::AckermannMPCController controller;
  controller.configure(node_, "FollowPath", tf_, costmap_);
  controller.activate();
  controller.setPlan(reference_);
  limo_dwb_critics::MpcConfig plant_config;
  plant_config.dt = 0.1;
  plant_config.time_steps = 40;
  plant_config.max_velocity = 0.4;
  plant_config.min_velocity = 0.0;
  plant_config.steering_rate = 0.5;
  limo_dwb_critics::SamplingMpc plant(plant_config);
  limo_dwb_critics::MpcState state;
  double lateral = 0.0;
  for (int cycle = 0; cycle < 100 && state.x < 1.4; ++cycle) {
    costmap_->getLayeredCostmap()->updateMap(0.0, 0.0, 0.0);
    for (double x = 0.80; x <= 1.00; x += 0.02) {
      for (double y = -0.08; y <= 0.08; y += 0.02) {setCost(x, y, 254);}
    }
    auto observed = current(state.y);
    observed.pose.position.x = state.x;
    observed.pose.orientation.z = std::sin(state.yaw * 0.5);
    observed.pose.orientation.w = std::cos(state.yaw * 0.5);
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = state.control.velocity;
    velocity.angular.z = plant.yawRate(state.control);
    geometry_msgs::msg::TwistStamped command;
    ASSERT_NO_THROW(command = controller.computeVelocityCommands(observed, velocity));
    EXPECT_GE(command.twist.linear.x, 0.0);
    limo_dwb_critics::MpcControl target;
    target.velocity = command.twist.linear.x;
    target.steering = target.velocity > 1e-6 ? std::atan(plant_config.wheelbase *
      command.twist.angular.z / target.velocity) : 0.0;
    state = plant.rollout(state, std::vector<limo_dwb_critics::MpcControl>(40, target)).states[1];
    lateral = std::max(lateral, std::abs(state.y));
  }
  EXPECT_GT(state.x, 1.3);
  EXPECT_GT(lateral, 0.15);
  geometry_msgs::msg::Pose2D terminal;
  terminal.x = state.x; terminal.y = state.y; terminal.theta = state.yaw;
  EXPECT_TRUE(tube_->containsFootprint(terminal));
  controller.deactivate();
  controller.cleanup();
}

TEST_F(StructuredPipelineTest, DebugExportsAllStructuredCandidatesAndTerminalTubeRejections)
{
  auto receiver = std::make_shared<rclcpp::Node>("structured_preview_receiver");
  limo_interfaces::msg::MpcDebug::SharedPtr snapshot;
  auto subscription = receiver->create_subscription<limo_interfaces::msg::MpcDebug>(
    "/limo/control/mpc_debug", rclcpp::QoS(1).best_effort(),
    [&](limo_interfaces::msg::MpcDebug::SharedPtr msg) {snapshot = msg;});
  limo_dwb_critics::AckermannMPCController controller;
  controller.configure(node_, "FollowPath", tf_, costmap_);
  controller.activate();
  controller.setPlan(reference_);
  for (int attempt = 0; attempt < 100 && !snapshot; ++attempt) {
    controller.computeVelocityCommands(current(), geometry_msgs::msg::Twist());
    rclcpp::spin_some(receiver);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_NE(snapshot, nullptr);
  EXPECT_GT(snapshot->generated_count, 0u);
  EXPECT_LT(snapshot->generated_count, 59u);
  EXPECT_EQ(snapshot->candidates.size(), snapshot->generated_count);
  std::vector<int> families(8);
  for (const auto & candidate : snapshot->candidates) {
    ASSERT_LT(candidate.family, families.size());
    ++families[candidate.family];
    if (candidate.score_status == candidate.SCORED) {
      EXPECT_TRUE(tube_->containsFootprint(candidate.poses.back()));
    }
  }
  EXPECT_EQ(families[0], 1);
  EXPECT_EQ(families[2] + families[3] + families[4], 0);
  EXPECT_GT(families[5], 0);
  EXPECT_GT(families[6], 0);
  EXPECT_EQ(families[7], 3);
  controller.deactivate();
  controller.cleanup();
}

class SmacControllerProfilesTest : public StructuredPipelineTest,
  public ::testing::WithParamInterface<const char *>
{
protected:
  const char * parameterFile() const override {return GetParam();}
  void SetUp() override
  {
    StructuredPipelineTest::SetUp();
    // These export regressions retain their original reference geometry as
    // runtime obstacle detection is tuned independently.
    const auto parameter = "FollowPath.MppiPath.ObstacleGuidance.lookahead_distance";
    if (!node_->has_parameter(parameter)) {node_->declare_parameter(parameter, 1.25);}
    node_->set_parameter(rclcpp::Parameter(parameter, 1.25));
  }
};

INSTANTIATE_TEST_CASE_P(RealAndSimulation, SmacControllerProfilesTest,
  ::testing::Values(LIMO_MPC_PARAMS_PATH, LIMO_MPC_SIM_PARAMS_PATH));

TEST_P(SmacControllerProfilesTest, ReferenceIsExportedWithTheControllerCycleAndClearsOnNewPlan)
{
  node_->set_parameter(rclcpp::Parameter("FollowPath.MppiPath.ObstacleGuidance.use_smac", true));
  // Isolate native-reference snapshot export from fused-grid obstacle search.
  const auto laser = addLaserTriggerLayer();
  setLaserCost(laser, .725, .025);
  auto receiver = std::make_shared<rclcpp::Node>("guidance_preview_receiver");
  limo_interfaces::msg::MpcDebug::SharedPtr snapshot;
  auto subscription = receiver->create_subscription<limo_interfaces::msg::MpcDebug>(
    "/limo/control/mpc_debug", rclcpp::QoS(1).best_effort(),
    [&](limo_interfaces::msg::MpcDebug::SharedPtr msg) {snapshot = msg;});
  limo_dwb_critics::AckermannMPCController controller;
  controller.configure(node_, "FollowPath", tf_, costmap_);
  controller.activate();
  controller.setPlan(reference_);
  const auto capture = [&]() {
      for (int attempt = 0; attempt < 100 && !snapshot; ++attempt) {
        controller.computeVelocityCommands(current(), geometry_msgs::msg::Twist());
        rclcpp::spin_some(receiver);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    };
  capture();
  ASSERT_NE(snapshot, nullptr);
  EXPECT_EQ(snapshot->guidance_backend, "SMAC");
  ASSERT_GE(snapshot->guidance_path.size(), 2U);
  EXPECT_TRUE(snapshot->guidance_target_valid);
  EXPECT_NEAR(snapshot->guidance_target.x, 1.75, .051);
  EXPECT_NEAR(snapshot->guidance_target.y, 0.0, 1e-9);
  ASSERT_TRUE(snapshot->path_lookahead_valid);
  ASSERT_TRUE(snapshot->guidance_lookahead_valid);
  ASSERT_TRUE(snapshot->guidance_candidate_target_valid);
  // DWB may prune the pose at zero; allow one reference sample in x.
  expectLookahead(&snapshot->path_lookahead, 1.25, 0.0, 0.0, .051);
  expectLookahead(&snapshot->guidance_lookahead, snapshot->path_lookahead.x);
  expectLookahead(&snapshot->guidance_candidate_target, 1.75, 0.0, 0.0, .051);
  expectLookahead(&snapshot->guidance_candidate_target, snapshot->guidance_target.x,
    snapshot->guidance_target.y, snapshot->guidance_target.theta);
  EXPECT_LE(snapshot->guidance_path.size(), 512U);
  EXPECT_EQ(snapshot->header.frame_id, "odom");
  EXPECT_EQ(snapshot->costmap.header.stamp, snapshot->header.stamp);
  EXPECT_GT(snapshot->candidates.size(), 0U);
  clearMap();
  clearLaser(laser);
  controller.setPlan(reference_);
  snapshot.reset();
  capture();
  ASSERT_NE(snapshot, nullptr);
  EXPECT_EQ(snapshot->guidance_backend, "SMAC");
  EXPECT_TRUE(snapshot->guidance_path.empty());
  EXPECT_FALSE(snapshot->guidance_target_valid);
  // A fresh plan with no laser intersection still publishes all considered
  // lookaheads, while keeping the active avoidance target unset.
  ASSERT_TRUE(snapshot->path_lookahead_valid);
  ASSERT_TRUE(snapshot->guidance_lookahead_valid);
  ASSERT_TRUE(snapshot->guidance_candidate_target_valid);
  expectLookahead(&snapshot->path_lookahead, 1.25, 0.0, 0.0, .051);
  expectLookahead(&snapshot->guidance_lookahead, snapshot->path_lookahead.x);
  expectLookahead(&snapshot->guidance_candidate_target, 1.75, 0.0, 0.0, .051);
  controller.deactivate();
  controller.cleanup();
}

TEST_P(SmacControllerProfilesTest, UnavailableGuidanceStillExportsConsideredLookaheads)
{
  node_->set_parameter(rclcpp::Parameter("FollowPath.MppiPath.ObstacleGuidance.use_smac", true));
  const auto laser = addLaserTriggerLayer();
  setLaserCost(laser, .725, .025);
  auto * map = costmap_->getCostmap();
  std::fill(map->getCharMap(), map->getCharMap() +
    map->getSizeInCellsX() * map->getSizeInCellsY(), nav2_costmap_2d::LETHAL_OBSTACLE);
  auto receiver = std::make_shared<rclcpp::Node>("unavailable_guidance_preview_receiver");
  limo_interfaces::msg::MpcDebug::SharedPtr snapshot;
  auto subscription = receiver->create_subscription<limo_interfaces::msg::MpcDebug>(
    "/limo/control/mpc_debug", rclcpp::QoS(1).best_effort(),
    [&](limo_interfaces::msg::MpcDebug::SharedPtr msg) {snapshot = msg;});
  limo_dwb_critics::AckermannMPCController controller;
  controller.configure(node_, "FollowPath", tf_, costmap_);
  controller.activate();
  controller.setPlan(reference_);
  // No free rejoin exists, so this reaches the waiting branch without an
  // expensive native SMAC search. Its empty frame must retain useful points.
  for (int attempt = 0; attempt < 100 && !snapshot; ++attempt) {
    geometry_msgs::msg::TwistStamped command;
    ASSERT_NO_THROW(command = controller.computeVelocityCommands(current(), geometry_msgs::msg::Twist()));
    EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
    rclcpp::spin_some(receiver);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_NE(snapshot, nullptr);
  EXPECT_EQ(snapshot->guidance_backend, "SMAC");
  EXPECT_TRUE(snapshot->guidance_path.empty());
  EXPECT_FALSE(snapshot->guidance_target_valid);
  EXPECT_TRUE(snapshot->candidates.empty());
  EXPECT_EQ(snapshot->selected_id, -1);
  EXPECT_DOUBLE_EQ(snapshot->command_velocity, 0.0);
  ASSERT_TRUE(snapshot->path_lookahead_valid);
  ASSERT_TRUE(snapshot->guidance_lookahead_valid);
  EXPECT_FALSE(snapshot->guidance_candidate_target_valid);
  expectLookahead(&snapshot->path_lookahead, 1.25, 0.0, 0.0, .051);
  expectLookahead(&snapshot->guidance_lookahead, snapshot->path_lookahead.x);
  controller.deactivate();
  controller.cleanup();
}

TEST_F(StructuredPipelineTest, SlowStartMovesBeforeAWallWithoutRelaxingTubeOrCollisionChecks)
{
  // A wall rules out full-speed four-second rollouts but leaves room to creep.
  for (unsigned int y = 0; y < costmap_->getCostmap()->getSizeInCellsY(); ++y) {
    for (double x = 0.35; x <= 0.40; x += 0.02) {
      double wx, wy;
      costmap_->getCostmap()->mapToWorld(0, y, wx, wy);
      setCost(x, wy, 254);
    }
  }
  limo_dwb_critics::AckermannMPCController controller;
  controller.configure(node_, "FollowPath", tf_, costmap_);
  controller.activate();
  controller.setPlan(reference_);
  const auto command = controller.computeVelocityCommands(current(), geometry_msgs::msg::Twist());
  EXPECT_GT(command.twist.linear.x, 0.0);
  EXPECT_LE(command.twist.linear.x, 0.005 + 1e-12);
  EXPECT_NEAR(command.twist.angular.z, 0.0, 1e-12);
  controller.deactivate();
  controller.cleanup();
}
