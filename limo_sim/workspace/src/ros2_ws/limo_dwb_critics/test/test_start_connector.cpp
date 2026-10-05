#include <gtest/gtest.h>
#include "limo_dwb_critics/start_connector.hpp"

namespace
{
nav_msgs::msg::Path line()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i <= 30; ++i) {
    geometry_msgs::msg::PoseStamped point;
    point.header = path.header;
    point.pose.position.x = i * 0.02;
    point.pose.orientation.w = 1.0;
    path.poses.push_back(point);
  }
  return path;
}
}

TEST(StartConnector, RequiresMatchingCompletePlanAndRestoresChecksPermanently)
{
  const auto path = line();
  limo_interfaces::msg::StartConnector metadata;
  metadata.path = path;
  metadata.end_index = 20;
  limo_dwb_critics::StartConnectorGate gate;
  gate.setPlan(path);
  EXPECT_FALSE(gate.update(path.poses.front().pose, 0.01, 0.1));
  gate.setMetadata(metadata);
  EXPECT_TRUE(gate.update(path.poses.front().pose, 0.01, 0.1));
  ASSERT_EQ(gate.prefix().poses.size(), 21U);
  for (std::size_t i = 1; i < 20; ++i) {
    EXPECT_TRUE(gate.update(path.poses[i].pose, 0.01, 0.1));
  }
  EXPECT_FALSE(gate.update(path.poses[20].pose, 0.01, 0.1));
  ASSERT_EQ(gate.normalPlan().poses.size(), 11U);
  EXPECT_DOUBLE_EQ(gate.normalPlan().poses.front().pose.position.x, 0.4);
  gate.setMetadata(metadata);
  gate.setPlan(path);
  EXPECT_FALSE(gate.update(path.poses.front().pose, 0.01, 0.1));
  auto unrelated = path;
  unrelated.header.stamp.sec = 42;
  gate.setPlan(unrelated);
  EXPECT_FALSE(gate.update(unrelated.poses.front().pose, 0.01, 0.1));
  unrelated.header.stamp = path.header.stamp;
  unrelated.poses.back().pose.position.y = 1.0;
  gate.setPlan(unrelated);
  EXPECT_FALSE(gate.update(unrelated.poses.front().pose, 0.01, 0.1));
}

TEST(StartConnector, AnEndpointNearTheStartCannotSkipALoop)
{
  auto path = line();
  // End is spatially near the start but far along the reference.
  path.poses[20].pose = path.poses.front().pose;
  path.poses[20].pose.position.y = 0.01;
  limo_interfaces::msg::StartConnector metadata;
  metadata.path = path;
  metadata.end_index = 20;
  limo_dwb_critics::StartConnectorGate gate;
  gate.setPlan(path);
  gate.setMetadata(metadata);
  EXPECT_TRUE(gate.update(path.poses.front().pose, 0.05, 0.1));
  gate.setMetadata(limo_interfaces::msg::StartConnector());
  EXPECT_FALSE(gate.update(path.poses.front().pose, 0.05, 0.1));
}
