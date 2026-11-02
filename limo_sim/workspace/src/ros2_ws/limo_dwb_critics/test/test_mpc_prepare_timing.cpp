#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "limo_dwb_critics/ackermann_mpc_controller.hpp"

namespace limo_dwb_critics
{
class AckermannMPCControllerTestPeer
{
public:
  static void begin2D(AckermannMPCController & controller, bool enabled)
  {
    controller.timing_sample_.critic_prepare.resize(controller.critics_.size());
    controller.timing_sample_.reset();
    controller.timing_sample_active_ = enabled;
  }

  static void enableOuterTiming(
    AckermannMPCController & controller, const nav2_util::LifecycleNode::SharedPtr & node)
  {
    node->declare_parameter("FollowPath.MPC.Timing.enabled", true);
    controller.timing_publisher_ = std::make_unique<MpcTimingPublisher>(node, "FollowPath", 100.0);
    std::vector<std::string> names;
    for (const auto & critic : controller.critics_) {names.push_back(critic->getName());}
    controller.timing_publisher_->setCriticNames(names);
    controller.timing_publisher_->activate();
    controller.timing_sample_.critic_prepare.resize(controller.critics_.size());
  }

  static const MpcTimingSample & sample(const AckermannMPCController & controller)
  {return controller.timing_sample_;}

  static bool active(const AckermannMPCController & controller)
  {return controller.timing_sample_active_;}
};
}  // namespace limo_dwb_critics

namespace
{
using limo_dwb_critics::AckermannMPCControllerTestPeer;

// Costmap2D uses a recursive mutex: checking from the calling thread would
// succeed even while the controller still owned it. No sleeps or DDS needed.
bool lockAvailableFromAnotherThread(nav2_costmap_2d::Costmap2D::mutex_t * mutex)
{
  bool available = false;
  std::thread probe([&]() {
      available = mutex->try_lock();
      if (available) {mutex->unlock();}
    });
  probe.join();
  return available;
}

void finiteWork()
{
  volatile double result = 0.0;
  for (int i = 0; i < 1024; ++i) {result += std::sin(static_cast<double>(i));}
}

class RecordingCritic : public dwb_core::TrajectoryCritic
{
public:
  RecordingCritic(std::string label, std::vector<std::string> & events)
  : label_(std::move(label)), events_(events) {}

  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & plan) override
  {
    events_.push_back(label_ + ".prepare");
    prepare_locked.push_back(!lockAvailableFromAnotherThread(costmap_ros_->getCostmap()->getMutex()));
    last_pose = pose;
    last_velocity = velocity;
    last_goal = goal;
    last_plan = plan;
    finiteWork();
    if (throw_prepare) {throw std::runtime_error(label_ + " prepare failure");}
    return prepare_result;
  }

  void debrief(const nav_2d_msgs::msg::Twist2D & command) override
  {
    events_.push_back(label_ + ".debrief");
    debrief_locked.push_back(!lockAvailableFromAnotherThread(costmap_ros_->getCostmap()->getMutex()));
    commands.push_back(command);
    if (throw_debrief) {throw std::runtime_error(label_ + " debrief failure");}
  }

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &) override {return 0.0;}

  bool prepare_result{true};
  bool throw_prepare{false};
  bool throw_debrief{false};
  geometry_msgs::msg::Pose2D last_pose;
  geometry_msgs::msg::Pose2D last_goal;
  nav_2d_msgs::msg::Twist2D last_velocity;
  nav_2d_msgs::msg::Path2D last_plan;
  std::vector<bool> prepare_locked;
  std::vector<bool> debrief_locked;
  std::vector<nav_2d_msgs::msg::Twist2D> commands;

private:
  std::string label_;
  std::vector<std::string> & events_;
};

class RecordingController : public limo_dwb_critics::AckermannMPCController
{
public:
  enum class CoreResult {success, no_legal, unexpected_error};

  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & node,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & map,
    const std::vector<dwb_core::TrajectoryCritic::Ptr> & critics,
    std::vector<std::string> & events)
  {
    node_ = node;
    costmap_ros_ = map;
    tf_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    critics_ = critics;
    events_ = &events;
    dwb_plugin_name_ = "FollowPath";
    prune_plan_ = false;
    prune_distance_ = 1.0;
    debug_trajectory_details_ = false;
    shorten_transformed_plan_ = false;
    short_circuit_trajectory_evaluation_ = false;
    global_plan_.header.frame_id = "odom";
    global_plan_.poses.resize(2);
    global_plan_.poses.back().x = 1.0;
    pub_ = std::make_unique<dwb_core::DWBPublisher>(node_, "FollowPath");
    ASSERT_EQ(pub_->on_configure(), nav2_util::CallbackReturn::SUCCESS);
    ASSERT_EQ(pub_->on_activate(), nav2_util::CallbackReturn::SUCCESS);
  }

  void stopPublisher()
  {
    pub_->on_deactivate();
    pub_->on_cleanup();
  }

  CoreResult core_result{CoreResult::success};
  bool throw_plan{false};
  std::vector<bool> core_locked;

protected:
  nav_2d_msgs::msg::Path2D transformGlobalPlan(const nav_2d_msgs::msg::Pose2DStamped &) override
  {
    events_->push_back("plan");
    finiteWork();
    if (throw_plan) {throw std::runtime_error("plan failure");}
    return global_plan_;
  }

  dwb_msgs::msg::TrajectoryScore coreScoringAlgorithm(
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D,
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> &) override
  {
    events_->push_back("core");
    core_locked.push_back(!lockAvailableFromAnotherThread(costmap_ros_->getCostmap()->getMutex()));
    if (core_result == CoreResult::no_legal) {
      throw dwb_core::NoLegalTrajectoriesException(dwb_core::IllegalTrajectoryTracker());
    }
    if (core_result == CoreResult::unexpected_error) {throw std::runtime_error("core failure");}
    dwb_msgs::msg::TrajectoryScore best;
    best.total = 7.0;
    best.traj.velocity.x = 0.23;
    best.traj.velocity.y = 0.0;
    best.traj.velocity.theta = -0.17;
    best.traj.poses = global_plan_.poses;
    return best;
  }

private:
  std::vector<std::string> * events_{nullptr};
};

class MpcPrepareTimingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    auto options = rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("FollowPath.publish_evaluation", false),
        rclcpp::Parameter("FollowPath.publish_trajectories", false),
        rclcpp::Parameter("FollowPath.publish_cost_grid_pc", false),
        rclcpp::Parameter("FollowPath.publish_local_plan", false),
        rclcpp::Parameter("FollowPath.publish_transformed_plan", false)});
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("prepare_timing_controller", options);
    map_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("prepare_timing_costmap");
    map_->set_parameters({
      rclcpp::Parameter("plugins", std::vector<std::string>{}),
      rclcpp::Parameter("global_frame", "odom")});
    ASSERT_EQ(map_->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
    map_->getCostmap()->resizeMap(20, 20, .1, -1., -1.);
    std::vector<dwb_core::TrajectoryCritic::Ptr> critics;
    for (std::string name : {"A", "B", "C"}) {
      auto critic = std::make_shared<RecordingCritic>(name, events_);
      std::string parent = "FollowPath";
      critic->initialize(node_, name, parent, map_);
      critics_.push_back(critic);
      critics.push_back(critic);
    }
    controller_ = std::make_unique<RecordingController>();
    controller_->initialize(node_, map_, critics, events_);
    robot_.header.frame_id = "odom";
    robot_.pose.x = .12;
    velocity_.x = .07;
  }

  void TearDown() override
  {
    controller_->stopPublisher();
    controller_.reset();
    critics_.clear();
    map_->on_cleanup(rclcpp_lifecycle::State());
    map_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  nav_2d_msgs::msg::Twist2DStamped run2D(bool enabled)
  {
    AckermannMPCControllerTestPeer::begin2D(*controller_, enabled);
    return controller_->computeVelocityCommands(robot_, velocity_, results_);
  }

  geometry_msgs::msg::TwistStamped run3D()
  {
    geometry_msgs::msg::PoseStamped robot;
    robot.header = robot_.header;
    robot.pose.position.x = robot_.pose.x;
    robot.pose.orientation.w = 1.0;
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = velocity_.x;
    return controller_->computeVelocityCommands(robot, velocity);
  }

  void expectUnlocked()
  {EXPECT_TRUE(lockAvailableFromAnotherThread(map_->getCostmap()->getMutex()));}

  nav2_util::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  std::unique_ptr<RecordingController> controller_;
  std::vector<std::shared_ptr<RecordingCritic>> critics_;
  std::vector<std::string> events_;
  nav_2d_msgs::msg::Pose2DStamped robot_;
  nav_2d_msgs::msg::Twist2D velocity_;
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results_;
};

TEST_F(MpcPrepareTimingTest, EnabledAndDisabledPreserveCommandsOrderArgumentsAndLocking)
{
  const std::vector<std::string> expected{
    "plan", "A.prepare", "B.prepare", "C.prepare", "core", "A.debrief", "B.debrief", "C.debrief"};
  for (bool enabled : {false, true}) {
    events_.clear();
    results_ = std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
    const auto command = run2D(enabled);
    EXPECT_DOUBLE_EQ(command.velocity.x, .23);
    EXPECT_DOUBLE_EQ(command.velocity.theta, -.17);
    EXPECT_EQ(events_, expected);
    EXPECT_EQ(results_->header.frame_id, "odom");
    EXPECT_TRUE(controller_->core_locked.back());
    for (const auto & critic : critics_) {
      EXPECT_DOUBLE_EQ(critic->last_pose.x, .12);
      EXPECT_DOUBLE_EQ(critic->last_velocity.x, .07);
      EXPECT_DOUBLE_EQ(critic->last_goal.x, 1.0);
      ASSERT_EQ(critic->last_plan.poses.size(), 2U);
      EXPECT_TRUE(critic->prepare_locked.back());
      EXPECT_TRUE(critic->debrief_locked.back());
      EXPECT_DOUBLE_EQ(critic->commands.back().x, .23);
      EXPECT_DOUBLE_EQ(critic->commands.back().theta, -.17);
    }
    expectUnlocked();
  }
  const auto & sample = AckermannMPCControllerTestPeer::sample(*controller_);
  ASSERT_EQ(sample.critic_prepare.size(), 3U);
  double critic_sum = 0.0;
  for (const auto & critic : sample.critic_prepare) {
    EXPECT_TRUE(critic.called);
    EXPECT_GT(critic.prepare_ms, 0.0);
    EXPECT_GE(critic.prepare_cpu_ms, 0.0);
    critic_sum += critic.prepare_ms;
  }
  EXPECT_GE(sample.critics_prepare_ms, critic_sum);
  EXPECT_GE(sample.critics_prepare_cpu_ms, 0.0);
  EXPECT_GT(sample.plan_prepare_ms, 0.0);
  EXPECT_GE(sample.plan_prepare_cpu_ms, 0.0);
  EXPECT_GE(sample.costmap_lock_wait_ms, 0.0);
  EXPECT_GT(sample.debrief_ms, 0.0);
  EXPECT_GT(sample.dwb_publish_ms, 0.0);
}

TEST_F(MpcPrepareTimingTest, AFalsePrepareResultStillEvaluatesAndDebriefsEveryCritic)
{
  critics_[1]->prepare_result = false;
  for (bool enabled : {false, true}) {
    events_.clear();
    EXPECT_NO_THROW(run2D(enabled));
    EXPECT_EQ(events_, (std::vector<std::string>{
      "plan", "A.prepare", "B.prepare", "C.prepare", "core", "A.debrief", "B.debrief", "C.debrief"}));
    expectUnlocked();
  }
}

TEST_F(MpcPrepareTimingTest, ThrowingPrepareIsTimedAndDoesNotRunLaterCriticsCoreOrDebrief)
{
  critics_[1]->throw_prepare = true;
  for (bool enabled : {false, true}) {
    events_.clear();
    EXPECT_THROW(run2D(enabled), std::runtime_error);
    EXPECT_EQ(events_, (std::vector<std::string>{"plan", "A.prepare", "B.prepare"}));
    expectUnlocked();
  }
  const auto & sample = AckermannMPCControllerTestPeer::sample(*controller_);
  ASSERT_EQ(sample.critic_prepare.size(), 3U);
  EXPECT_TRUE(sample.critic_prepare[0].called);
  EXPECT_TRUE(sample.critic_prepare[1].called);
  EXPECT_GT(sample.critic_prepare[1].prepare_ms, 0.0);
  EXPECT_FALSE(sample.critic_prepare[2].called);
  EXPECT_DOUBLE_EQ(sample.critic_prepare[2].prepare_ms, 0.0);
  EXPECT_DOUBLE_EQ(sample.debrief_ms, 0.0);
  EXPECT_DOUBLE_EQ(sample.dwb_publish_ms, 0.0);
}

TEST_F(MpcPrepareTimingTest, NoLegalTrajectoriesDebriefsWithAnEmptyCommandAndRethrows)
{
  controller_->core_result = RecordingController::CoreResult::no_legal;
  for (bool enabled : {false, true}) {
    events_.clear();
    EXPECT_THROW(run2D(enabled), dwb_core::NoLegalTrajectoriesException);
    EXPECT_EQ(events_, (std::vector<std::string>{
      "plan", "A.prepare", "B.prepare", "C.prepare", "core", "A.debrief", "B.debrief", "C.debrief"}));
    for (const auto & critic : critics_) {
      EXPECT_DOUBLE_EQ(critic->commands.back().x, 0.0);
      EXPECT_DOUBLE_EQ(critic->commands.back().y, 0.0);
      EXPECT_DOUBLE_EQ(critic->commands.back().theta, 0.0);
    }
    expectUnlocked();
  }
  const auto & sample = AckermannMPCControllerTestPeer::sample(*controller_);
  EXPECT_GT(sample.debrief_ms, 0.0);
  EXPECT_GT(sample.dwb_publish_ms, 0.0);
}

TEST_F(MpcPrepareTimingTest, UnexpectedCoreExceptionPreservesTheBaseNoDebriefBehavior)
{
  controller_->core_result = RecordingController::CoreResult::unexpected_error;
  for (bool enabled : {false, true}) {
    events_.clear();
    EXPECT_THROW(run2D(enabled), std::runtime_error);
    EXPECT_EQ(events_, (std::vector<std::string>{"plan", "A.prepare", "B.prepare", "C.prepare", "core"}));
    expectUnlocked();
  }
  EXPECT_DOUBLE_EQ(AckermannMPCControllerTestPeer::sample(*controller_).debrief_ms, 0.0);
  EXPECT_DOUBLE_EQ(AckermannMPCControllerTestPeer::sample(*controller_).dwb_publish_ms, 0.0);
}

TEST_F(MpcPrepareTimingTest, DebriefExceptionReleasesTheMutexAndSkipsLaterDebriefs)
{
  critics_[0]->throw_debrief = true;
  for (bool enabled : {false, true}) {
    events_.clear();
    EXPECT_THROW(run2D(enabled), std::runtime_error);
    EXPECT_EQ(events_, (std::vector<std::string>{
      "plan", "A.prepare", "B.prepare", "C.prepare", "core", "A.debrief"}));
    expectUnlocked();
  }
  EXPECT_GT(AckermannMPCControllerTestPeer::sample(*controller_).debrief_ms, 0.0);
  EXPECT_DOUBLE_EQ(AckermannMPCControllerTestPeer::sample(*controller_).dwb_publish_ms, 0.0);
}

TEST_F(MpcPrepareTimingTest, OuterCyclesClearOldTimingsAfterSuccessAndDifferentFailures)
{
  AckermannMPCControllerTestPeer::enableOuterTiming(*controller_, node_);
  EXPECT_DOUBLE_EQ(run3D().twist.linear.x, .23);
  EXPECT_TRUE(AckermannMPCControllerTestPeer::sample(*controller_).success);
  EXPECT_FALSE(AckermannMPCControllerTestPeer::active(*controller_));
  EXPECT_GT(AckermannMPCControllerTestPeer::sample(*controller_).critic_prepare[2].prepare_ms, 0.0);

  critics_[1]->throw_prepare = true;
  EXPECT_THROW(run3D(), std::runtime_error);
  const auto & failed = AckermannMPCControllerTestPeer::sample(*controller_);
  EXPECT_FALSE(failed.success);
  EXPECT_FALSE(AckermannMPCControllerTestPeer::active(*controller_));
  EXPECT_TRUE(failed.critic_prepare[0].called);
  EXPECT_TRUE(failed.critic_prepare[1].called);
  EXPECT_FALSE(failed.critic_prepare[2].called);
  EXPECT_DOUBLE_EQ(failed.critic_prepare[2].prepare_ms, 0.0);
  EXPECT_DOUBLE_EQ(failed.critic_prepare[2].prepare_cpu_ms, 0.0);
  EXPECT_DOUBLE_EQ(failed.debrief_ms, 0.0);
  EXPECT_DOUBLE_EQ(failed.dwb_publish_ms, 0.0);
  expectUnlocked();

  controller_->throw_plan = true;
  EXPECT_THROW(run3D(), std::runtime_error);
  const auto & plan_failed = AckermannMPCControllerTestPeer::sample(*controller_);
  EXPECT_FALSE(plan_failed.success);
  EXPECT_FALSE(AckermannMPCControllerTestPeer::active(*controller_));
  EXPECT_GT(plan_failed.plan_prepare_ms, 0.0);
  EXPECT_DOUBLE_EQ(plan_failed.critics_prepare_ms, 0.0);
  for (const auto & critic : plan_failed.critic_prepare) {
    EXPECT_FALSE(critic.called);
    EXPECT_DOUBLE_EQ(critic.prepare_ms, 0.0);
    EXPECT_DOUBLE_EQ(critic.prepare_cpu_ms, 0.0);
  }
  expectUnlocked();

  controller_->throw_plan = false;
  critics_[1]->throw_prepare = false;
  EXPECT_DOUBLE_EQ(run3D().twist.angular.z, -.17);
  EXPECT_TRUE(AckermannMPCControllerTestPeer::sample(*controller_).success);
  EXPECT_FALSE(AckermannMPCControllerTestPeer::active(*controller_));
  expectUnlocked();
}
}  // namespace
