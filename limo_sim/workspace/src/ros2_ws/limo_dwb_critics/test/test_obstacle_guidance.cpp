#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "limo_dwb_critics/obstacle_guidance.hpp"

using limo_dwb_critics::ObstacleGuidance;

TEST(ObstacleGuidance, RewardsGoingAroundAnObstacleInsteadOfWaitingInFront)
{
  constexpr int size = 100;
  std::vector<unsigned char> costs(size * size, 0);
  // A metre-wide cube in a 5 m map, seen after planning a straight path.
  for (int y = 40; y <= 60; ++y) {
    for (int x = 45; x <= 65; ++x) {
      costs[y * size + x] = 254;
    }
  }
  ObstacleGuidance guidance;
  guidance.update(size, size, 0.05, costs.data(), 0.25, 2.0);
  ASSERT_TRUE(guidance.setTarget(80, 50));
  const double waiting = guidance.distance(37, 50);
  ASSERT_TRUE(std::isfinite(waiting));
  // A lateral move increases straight-line distance to the goal, but reduces
  // distance through free space. This is the previous stopping local minimum.
  EXPECT_LT(guidance.distance(37, 37), waiting);
  EXPECT_LT(guidance.distance(37, 63), waiting);
  EXPECT_TRUE(std::isinf(guidance.distance(50, 50)));
  EXPECT_FALSE(guidance.setTarget(50, 50));
}

TEST(ObstacleGuidance, RejectsUnknownCellsAndRoutesThroughNarrowGaps)
{
  std::vector<unsigned char> costs(40 * 40, 0);
  for (int y = 0; y < 40; ++y) {
    if (y < 18 || y > 22) {
      costs[y * 40 + 20] = 254;
    }
  }
  ObstacleGuidance guidance;
  guidance.update(40, 40, 0.05, costs.data(), 0.20, 0.0);
  ASSERT_TRUE(guidance.setTarget(30, 20));
  EXPECT_TRUE(std::isinf(guidance.distance(10, 20)));
  costs[20 * 40 + 30] = 255;
  guidance.update(40, 40, 0.05, costs.data(), 0.20, 0.0);
  EXPECT_FALSE(guidance.setTarget(30, 20));
}

TEST(ObstacleGuidance, DoesNotCutDiagonallyBetweenTouchingObstacles)
{
  std::vector<unsigned char> costs(5 * 5, 254);
  costs[1 * 5 + 1] = 0;
  costs[2 * 5 + 2] = 0;
  ObstacleGuidance guidance;
  guidance.update(5, 5, 0.1, costs.data(), 0.0, 0.0);
  ASSERT_TRUE(guidance.setTarget(2, 2));
  EXPECT_TRUE(std::isinf(guidance.distance(1, 1)));
  EXPECT_TRUE(std::isinf(guidance.distance(1.4, 1.4)));
}

TEST(ObstacleGuidance, ClearsOldObstaclesAndSmoothlyRewardsSubcellProgress)
{
  std::vector<unsigned char> costs(40 * 40, 0);
  for (int y = 0; y < 40; ++y) {
    costs[y * 40 + 20] = 254;
  }
  ObstacleGuidance guidance;
  guidance.update(40, 40, 0.05, costs.data(), 0.2, 0.0);
  ASSERT_TRUE(guidance.setTarget(30, 20));
  EXPECT_TRUE(std::isinf(guidance.distance(10, 20)));
  std::fill(costs.begin(), costs.end(), 0);
  guidance.update(40, 40, 0.05, costs.data(), 0.2, 0.0);
  ASSERT_TRUE(guidance.setTarget(30, 20));
  EXPECT_NEAR(guidance.distance(10, 20), 1.0, 1e-12);
  EXPECT_NEAR(guidance.distance(10.1, 20), 0.995, 1e-12);
  EXPECT_TRUE(std::isinf(guidance.distance(-1, 20)));
}
