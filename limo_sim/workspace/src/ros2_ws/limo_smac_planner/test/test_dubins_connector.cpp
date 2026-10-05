#include <gtest/gtest.h>
#include <cmath>
#include "limo_smac_planner/dubins_connector.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

TEST(DubinsConnector, ExactEndpointsForwardMotionAndRadius)
{
  geometry_msgs::msg::PoseStamped start, end;
  start.header.frame_id = end.header.frame_id = "map";
  start.pose.orientation.w = 1.0;
  const double radius = 0.7, angle = 0.6;
  end.pose.position.x = radius * std::sin(angle);
  end.pose.position.y = radius * (1.0 - std::cos(angle));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, angle);
  end.pose.orientation = tf2::toMsg(q);
  const auto path = limo_smac_planner::dubinsConnector(start, end, radius, 0.02, 2.0);
  ASSERT_GT(path.poses.size(), 2U);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, start.pose.position.x);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, end.pose.position.x);
  EXPECT_NEAR(tf2::getYaw(path.poses.back().pose.orientation), angle, 1e-9);
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose;
    const auto & b = path.poses[i].pose;
    const double ds = std::hypot(b.position.x - a.position.x, b.position.y - a.position.y);
    EXPECT_LE(ds, 0.02 + 1e-9);
    EXPECT_GT((b.position.x - a.position.x) * std::cos(tf2::getYaw(a.orientation)) +
      (b.position.y - a.position.y) * std::sin(tf2::getYaw(a.orientation)), 0.0);
    EXPECT_LE(std::abs(tf2::getYaw(b.orientation) - tf2::getYaw(a.orientation)) / ds,
      1.0 / radius + 0.001);
  }
}

TEST(DubinsConnector, RejectsExcessiveLengthAndMismatchedFrames)
{
  geometry_msgs::msg::PoseStamped start, end;
  start.header.frame_id = end.header.frame_id = "map";
  start.pose.orientation.w = end.pose.orientation.w = 1.0;
  end.pose.position.x = 3.0;
  EXPECT_THROW(limo_smac_planner::dubinsConnector(start, end, 0.7, 0.02, 2.0), std::runtime_error);
  end.header.frame_id = "odom";
  EXPECT_THROW(limo_smac_planner::dubinsConnector(start, end, 0.7, 0.02, 4.0), std::invalid_argument);
}
