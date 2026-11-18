#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>

#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "limo_dwb_critics/ackermann_mpc_controller.hpp"
#include "limo_dwb_critics/mppi_cost_critics.hpp"

class GoalDistanceCritic : public limo_dwb_critics::MppiPathCritic
{
public:
  explicit GoalDistanceCritic(double distance) {distance_to_goal_ = distance;}
};

class ControllerHarness : public limo_dwb_critics::AckermannMPCController
{
public:
  using AckermannMPCController::coreScoringAlgorithm;

  ControllerHarness()
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("mpc_test");
    mpc_ = std::make_unique<limo_dwb_critics::SamplingMpc>();
    debug_trajectory_details_ = false;
  }

  bool reject{false};
  bool cheap_stop{false};
  bool reject_motion{false};
  void globalGoalDistance(double distance)
  {
    critics_.push_back(std::make_shared<GoalDistanceCritic>(distance));
  }
  bool warm() const {return mpc_->hasWarmStart();}
  void advanceTime(double seconds = 0.05) {time_ns_ += static_cast<std::int64_t>(seconds * 1e9);}
  void closedLoop() {open_loop_ = false;}
  void freezeRosClock() {node_->set_parameter(rclcpp::Parameter("use_sim_time", true));}
  void nanoProfile()
  {
    limo_dwb_critics::MpcConfig config;
    config.dt = 0.1;
    config.time_steps = 25;
    config.batch_size = 96;
    config.velocity_samples = 6;
    config.curvature_samples = 9;
    config.steering_rate = 0.5;
    mpc_ = std::make_unique<limo_dwb_critics::SamplingMpc>(config);
  }
  std::int64_t controlTimeNs() const override {return time_ns_;}

  dwb_msgs::msg::TrajectoryScore scoreTrajectory(
    const dwb_msgs::msg::Trajectory2D & trajectory, double) override
  {
    const bool moving = std::abs(trajectory.velocity.x) > 0.001;
    if (reject || (reject_motion && moving)) {
      throw dwb_core::IllegalTrajectoryException("test_obstacle", "Blocked");
    }
    dwb_msgs::msg::TrajectoryScore score;
    score.traj = trajectory;
    score.total = cheap_stop ? (moving ? 100.0 : 0.0) :
      10.0 + 100.0 * std::pow(trajectory.poses.back().x - 0.8, 2);
    return score;
  }

private:
  std::int64_t time_ns_{1000000000};
};

class MpcControllerTest : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

TEST_F(MpcControllerTest, ReturnsFirstReachableCommandWithCorrectTrajectoryTimes)
{
  ControllerHarness controller;
  auto results = std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
  const auto best = controller.coreScoringAlgorithm(
    geometry_msgs::msg::Pose2D(), nav_2d_msgs::msg::Twist2D(), results);
  EXPECT_GT(best.traj.velocity.x, 0.0);
  EXPECT_LE(best.traj.velocity.x, 1.3 * 0.05 + 1e-12);
  EXPECT_DOUBLE_EQ(best.traj.velocity.y, 0.0);
  EXPECT_LE(std::abs(best.traj.velocity.theta), std::abs(best.traj.velocity.x) / 0.462 + 1e-12);
  ASSERT_EQ(best.traj.poses.size(), 51U);
  ASSERT_EQ(best.traj.time_offsets.size(), best.traj.poses.size());
  EXPECT_DOUBLE_EQ(rclcpp::Duration(best.traj.time_offsets.front()).seconds(), 0.0);
  EXPECT_DOUBLE_EQ(rclcpp::Duration(best.traj.time_offsets.back()).seconds(), 2.5);
  for (std::size_t i = 1; i < best.traj.time_offsets.size(); ++i) {
    EXPECT_GT(rclcpp::Duration(best.traj.time_offsets[i]).nanoseconds(),
      rclcpp::Duration(best.traj.time_offsets[i - 1]).nanoseconds());
  }
  ASSERT_FALSE(results->twists.empty());
  ASSERT_LT(results->best_index, results->twists.size());
  EXPECT_DOUBLE_EQ(results->twists[results->best_index].total, best.total);
  EXPECT_EQ(best.scores.back().name, "MpcEffort");
  EXPECT_TRUE(controller.warm());
}

TEST_F(MpcControllerTest, ThrowsAndDiscardsOldSequenceIfAllTrajectoriesFail)
{
  ControllerHarness controller;
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results;
  controller.coreScoringAlgorithm(
    geometry_msgs::msg::Pose2D(), nav_2d_msgs::msg::Twist2D(), results);
  ASSERT_TRUE(controller.warm());
  controller.reject = true;
  EXPECT_THROW(controller.coreScoringAlgorithm(
      geometry_msgs::msg::Pose2D(), nav_2d_msgs::msg::Twist2D(), results),
    dwb_core::NoLegalTrajectoriesException);
  EXPECT_FALSE(controller.warm());
}

TEST_F(MpcControllerTest, PrefersLegalMotionToCheaperStopFarFromGlobalGoal)
{
  ControllerHarness controller;
  controller.cheap_stop = true;
  controller.globalGoalDistance(2.0);
  auto results = std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
  const auto best = controller.coreScoringAlgorithm(
    geometry_msgs::msg::Pose2D(), nav_2d_msgs::msg::Twist2D(), results);

  EXPECT_GT(std::abs(best.traj.velocity.x), 0.001);
  EXPECT_GE(best.total, 100.0);
  bool recorded_cheaper_stop = false;
  for (const auto & candidate : results->twists) {
    if (candidate.total >= 0.0 && candidate.total < best.total &&
      std::abs(candidate.traj.velocity.x) <= 0.001)
    {
      recorded_cheaper_stop = true;
    }
  }
  EXPECT_TRUE(recorded_cheaper_stop);
  ASSERT_LT(results->best_index, results->twists.size());
  const auto & selected = results->twists[results->best_index];
  EXPECT_DOUBLE_EQ(selected.total, best.total);
  EXPECT_DOUBLE_EQ(selected.traj.velocity.x, best.traj.velocity.x);
  EXPECT_DOUBLE_EQ(selected.traj.velocity.theta, best.traj.velocity.theta);
  ASSERT_EQ(selected.traj.poses.size(), best.traj.poses.size());
  for (std::size_t i = 0; i < best.traj.poses.size(); ++i) {
    EXPECT_DOUBLE_EQ(selected.traj.poses[i].x, best.traj.poses[i].x);
    EXPECT_DOUBLE_EQ(selected.traj.poses[i].y, best.traj.poses[i].y);
    EXPECT_DOUBLE_EQ(selected.traj.poses[i].theta, best.traj.poses[i].theta);
  }
}

TEST_F(MpcControllerTest, AllowsCheaperStopInsideGlobalGoalTolerance)
{
  ControllerHarness controller;
  controller.cheap_stop = true;
  controller.globalGoalDistance(0.05);
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results;
  const auto best = controller.coreScoringAlgorithm(
    geometry_msgs::msg::Pose2D(), nav_2d_msgs::msg::Twist2D(), results);
  EXPECT_DOUBLE_EQ(best.traj.velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(best.total, 0.0);
}

TEST_F(MpcControllerTest, AllowsCheaperStopAtGlobalGoalToleranceBoundary)
{
  ControllerHarness controller;
  controller.cheap_stop = true;
  controller.globalGoalDistance(0.1);
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results;
  const auto best = controller.coreScoringAlgorithm(
    geometry_msgs::msg::Pose2D(), nav_2d_msgs::msg::Twist2D(), results);
  EXPECT_DOUBLE_EQ(best.traj.velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(best.total, 0.0);
}

TEST_F(MpcControllerTest, RetainsLegalStopWhenMovingTrajectoriesAreBlocked)
{
  ControllerHarness controller;
  controller.cheap_stop = true;
  controller.reject_motion = true;
  controller.globalGoalDistance(2.0);
  auto results = std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
  const auto best = controller.coreScoringAlgorithm(
    geometry_msgs::msg::Pose2D(), nav_2d_msgs::msg::Twist2D(), results);

  EXPECT_DOUBLE_EQ(best.traj.velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(best.total, 0.0);
  ASSERT_LT(results->best_index, results->twists.size());
  EXPECT_DOUBLE_EQ(results->twists[results->best_index].total, best.total);
  bool rejected_motion = false;
  for (const auto & candidate : results->twists) {
    if (candidate.total < 0.0 && std::abs(candidate.traj.velocity.x) > 0.001) {
      rejected_motion = true;
    }
  }
  EXPECT_TRUE(rejected_motion);
}

TEST_F(MpcControllerTest, KeepsCostSelectionWithoutKnownGlobalGoalDistance)
{
  ControllerHarness controller;
  controller.cheap_stop = true;
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results;
  const auto best = controller.coreScoringAlgorithm(
    geometry_msgs::msg::Pose2D(), nav_2d_msgs::msg::Twist2D(), results);
  EXPECT_DOUBLE_EQ(best.traj.velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(best.total, 0.0);
}

TEST_F(MpcControllerTest, KeepsCostSelectionWhenGlobalGoalDistanceIsNonFinite)
{
  ControllerHarness controller;
  controller.cheap_stop = true;
  controller.globalGoalDistance(std::numeric_limits<double>::infinity());
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results;
  const auto best = controller.coreScoringAlgorithm(
    geometry_msgs::msg::Pose2D(), nav_2d_msgs::msg::Twist2D(), results);
  EXPECT_DOUBLE_EQ(best.traj.velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(best.total, 0.0);
}

TEST_F(MpcControllerTest, CommandRampSurvivesActuatorLagAndRepeatedRosTimestamps)
{
  ControllerHarness controller;
  controller.freezeRosClock();
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results;
  nav_2d_msgs::msg::Twist2D odometry;
  double previous_speed = 0.0;
  double previous_steering = 0.0;
  double maximum_speed = 0.0;
  // Reproduce the report: odometry stays near zero while the motor cannot
  // get moving on a single 0.065 m/s acceleration increment.
  for (int cycle = 0; cycle < 10; ++cycle) {
    odometry.x = 0.002;
    odometry.theta = cycle % 2 == 0 ? 0.03 : -0.03;
    const auto best = controller.coreScoringAlgorithm(
      geometry_msgs::msg::Pose2D(), odometry, results);
    const auto & command = best.traj.velocity;
    const double steering = std::abs(command.x) > 1e-9 ?
      std::atan(0.2 * command.theta / command.x) : 0.0;
    EXPECT_LE(std::abs(command.x - previous_speed), 0.067 + 1e-9);
    EXPECT_LE(std::abs(steering - previous_steering), 0.8 * 0.05 + 1e-9);
    previous_speed = command.x;
    previous_steering = steering;
    maximum_speed = std::max(maximum_speed, command.x);
    controller.advanceTime();
  }
  EXPECT_GT(maximum_speed, 0.2);
  // An interruption resets the estimate, so resuming cannot reuse a fast
  // stale command when the measured robot is stopped.
  controller.advanceTime(1.0);
  const auto restarted = controller.coreScoringAlgorithm(
    geometry_msgs::msg::Pose2D(), odometry, results);
  EXPECT_LE(restarted.traj.velocity.x, 0.067 + 1e-9);
}

TEST_F(MpcControllerTest, ClosedLoopOptionContinuesToUseMeasuredVelocity)
{
  ControllerHarness controller;
  controller.closedLoop();
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results;
  nav_2d_msgs::msg::Twist2D odometry;
  odometry.x = 0.002;
  odometry.theta = 0.03;
  for (int cycle = 0; cycle < 4; ++cycle) {
    const auto best = controller.coreScoringAlgorithm(
      geometry_msgs::msg::Pose2D(), odometry, results);
    EXPECT_LE(best.traj.velocity.x, 0.067 + 1e-9);
    controller.advanceTime();
  }
}

TEST_F(MpcControllerTest, NanoProfileRampsAtTenHzAndResetsAfterCommandTimeout)
{
  ControllerHarness controller;
  controller.nanoProfile();
  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> results;
  double maximum_speed = 0.0;
  for (int cycle = 0; cycle < 6; ++cycle) {
    const auto best = controller.coreScoringAlgorithm(
      geometry_msgs::msg::Pose2D(), nav_2d_msgs::msg::Twist2D(), results);
    ASSERT_EQ(best.traj.poses.size(), 26U);
    EXPECT_DOUBLE_EQ(rclcpp::Duration(best.traj.time_offsets.back()).seconds(), 2.5);
    maximum_speed = std::max(maximum_speed, best.traj.velocity.x);
    controller.advanceTime(0.1);
  }
  EXPECT_GT(maximum_speed, 0.2);
  controller.advanceTime(0.5);
  const auto restarted = controller.coreScoringAlgorithm(
    geometry_msgs::msg::Pose2D(), nav_2d_msgs::msg::Twist2D(), results);
  EXPECT_LE(restarted.traj.velocity.x, 0.13 + 1e-9);
}
