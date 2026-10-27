#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "limo_inflation/path_pose_association.hpp"

namespace
{
geometry_msgs::msg::Pose2D pose(double x, double y, double yaw)
{
  geometry_msgs::msg::Pose2D result;
  result.x = x; result.y = y; result.theta = yaw;
  return result;
}
}  // namespace

TEST(PathPoseAssociation, ChoosesAlignedNearbyBranchInsteadOfOppositeHeading)
{
  const double pi = std::acos(-1.0);
  const auto points = limo_inflation::pathPosePoints(
  {
    pose(-1, 0, 0), pose(1, 0, 0), pose(1, 0.05, pi), pose(-1, 0.05, pi)});
  limo_inflation::PathPoseProjection result;
  ASSERT_TRUE(limo_inflation::projectPathPose(points, pose(0, 0.01, pi), result));
  EXPECT_EQ(result.segment, 2U);
  EXPECT_NEAR(result.heading, pi, 1e-9);
  ASSERT_TRUE(limo_inflation::projectPathPose(points, pose(0, 0.04, 0), result));
  EXPECT_EQ(result.segment, 0U);
}

TEST(PathPoseAssociation, DistantAlignedBranchCannotWinAndYawWrapsAtPi)
{
  const double pi = std::acos(-1.0);
  const auto points = limo_inflation::pathPosePoints(
  {
    pose(-1, 0, pi - 0.01), pose(1, 0, -pi + 0.01),
    pose(1, 0.5, 0), pose(-1, 0.5, 0)});
  limo_inflation::PathPoseProjection result;
  ASSERT_TRUE(limo_inflation::projectPathPose(points, pose(0, 0, 0), result));
  EXPECT_EQ(result.segment, 0U);
  ASSERT_TRUE(limo_inflation::projectPathPose(points, pose(0, 0, -pi), result));
  EXPECT_NEAR(std::abs(result.heading), pi, 1e-9);
}

TEST(PathPoseAssociation, KeepsCompletedLapAtIdenticalPositionAndOrientation)
{
  const double pi = std::acos(-1.0);
  std::vector<geometry_msgs::msg::Pose2D> poses;
  for (int i = 0; i <= 400; ++i) {
    const double angle = 2 * pi * i / 400;
    poses.push_back(pose(std::sin(angle), 1 - std::cos(angle), angle));
  }
  poses.push_back(pose(1, 0, 0));
  const auto points = limo_inflation::pathPosePoints(poses);
  limo_inflation::PathProgressTracker tracker;
  limo_inflation::PathPoseProjection result;
  ASSERT_TRUE(tracker.update(points, poses.front(), result));
  EXPECT_NEAR(tracker.progress(), 0.0, 1e-9);
  for (std::size_t i = 1; i <= 400; ++i) {
    const double previous = tracker.progress();
    ASSERT_TRUE(tracker.update(points, poses[i], result));
    EXPECT_GE(tracker.progress(), previous);
    EXPECT_NEAR(result.progress, points[i][2], 0.03);
  }
  EXPECT_GT(tracker.progress(), 6.2);
  for (int i = 0; i < 20; ++i) {
    ASSERT_TRUE(tracker.update(points, pose(0, 0, 0), result));
    EXPECT_NEAR(tracker.progress(), points[400][2], 1e-8);
  }
  ASSERT_TRUE(tracker.update(points, pose(0.3, 0, 0), result));
  EXPECT_NEAR(result.progress, points[400][2] + 0.3, 1e-8);
  // Scoring another candidate does not mutate the robot's history.
  const double progress = tracker.progress();
  ASSERT_TRUE(limo_inflation::projectPathPose(points, poses.front(), result));
  EXPECT_NEAR(result.progress, 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(tracker.progress(), progress);
  tracker.reset();
  ASSERT_TRUE(tracker.update(points, poses.front(), result));
  EXPECT_NEAR(tracker.progress(), 0.0, 1e-9);
}

TEST(PathPoseAssociation, LocalNoiseAndBriefReverseMotionDoNotEraseProgress)
{
  const auto points = limo_inflation::pathPosePoints({pose(0, 0, 0), pose(3, 0, 0)});
  limo_inflation::PathProgressTracker tracker;
  limo_inflation::PathPoseProjection result;
  ASSERT_TRUE(tracker.update(points, pose(1, 0, 0), result));
  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(tracker.update(points, pose(0.96, 0.01, 0.02), result));
    EXPECT_NEAR(tracker.progress(), 1.0, 1e-9);
  }
  ASSERT_TRUE(tracker.update(points, pose(1.1, 0, 0), result));
  EXPECT_NEAR(tracker.progress(), 1.1, 1e-9);
}

TEST(PathPoseAssociation, PreservesSingletonGoalAndRejectsEmptyReference)
{
  limo_inflation::PathPoseProjection result;
  EXPECT_FALSE(limo_inflation::projectPathPose({}, pose(0, 0, 0), result));
  const auto points = limo_inflation::pathPosePoints({pose(1, 2, 0.4)});
  ASSERT_TRUE(limo_inflation::projectPathPose(points, pose(0, 0, 0), result));
  EXPECT_DOUBLE_EQ(result.progress, 0.0);
  EXPECT_DOUBLE_EQ(result.heading, 0.4);
}
