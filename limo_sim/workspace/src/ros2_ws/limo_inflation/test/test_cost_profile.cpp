#include <gtest/gtest.h>

#include "limo_inflation/cost_profile.hpp"

using limo_inflation::computeBorderCost;
using limo_inflation::CostProfile;

TEST(CostProfileTest, CreatesZeroCostBand)
{
  const CostProfile profile;
  EXPECT_EQ(computeBorderCost(0.13, profile), 0);
  EXPECT_EQ(computeBorderCost(0.15, profile), 0);
  EXPECT_EQ(computeBorderCost(0.17, profile), 0);
}

TEST(CostProfileTest, DecreasesTowardPreferredBand)
{
  const CostProfile profile;
  EXPECT_EQ(computeBorderCost(0.0, profile), 60);
  EXPECT_GT(computeBorderCost(0.05, profile),
    computeBorderCost(0.10, profile));
}

TEST(CostProfileTest, RisesSlowlyAndSaturatesFarFromObstacle)
{
  const CostProfile profile;
  EXPECT_EQ(computeBorderCost(0.50, profile), 1);
  EXPECT_EQ(computeBorderCost(1.00, profile), 3);
  EXPECT_EQ(computeBorderCost(10.0, profile), 8);
}

TEST(CostProfileTest, CreatesSecondLaneWhenClearanceIsAvailable)
{
  const CostProfile profile;
  EXPECT_EQ(computeBorderCost(0.28, profile), 0);
  EXPECT_EQ(computeBorderCost(0.30, profile), 0);
  EXPECT_EQ(computeBorderCost(0.32, profile), 0);
}

TEST(CostProfileTest, SeparatesPreferredLanesWithBoundedRidge)
{
  const CostProfile profile;
  EXPECT_EQ(computeBorderCost(0.225, profile), 15);
  EXPECT_LT(computeBorderCost(0.18, profile), 15);
  EXPECT_LT(computeBorderCost(0.27, profile), 15);
}

TEST(CostProfileTest, SmallerSigmaNarrowsHaloWithoutMovingWells)
{
  CostProfile narrow;
  narrow.near_sigma = 0.025;
  CostProfile wide = narrow;
  wide.near_sigma = 0.05;
  EXPECT_EQ(computeBorderCost(0.0, narrow), narrow.near_max_cost);
  EXPECT_LT(computeBorderCost(0.05, narrow), computeBorderCost(0.05, wide));
  EXPECT_EQ(computeBorderCost(0.13, narrow), 0);
  EXPECT_EQ(computeBorderCost(0.15, narrow), 0);
  EXPECT_EQ(computeBorderCost(0.30, narrow), 0);
  EXPECT_EQ(computeBorderCost(0.225, narrow), computeBorderCost(0.225, wide));
}

TEST(CostProfileTest, GaussianTailStartsAtRedBoundaryAndEndsAtFirstWell)
{
  EXPECT_EQ(limo_inflation::computeNearCost(0.0, 0.165, 252, 0.025), 252);
  EXPECT_GT(limo_inflation::computeNearCost(0.025, 0.165, 252, 0.025), 0);
  EXPECT_LT(limo_inflation::computeNearCost(0.050, 0.165, 252, 0.025),
    limo_inflation::computeNearCost(0.025, 0.165, 252, 0.025));
  EXPECT_EQ(limo_inflation::computeNearCost(0.165, 0.165, 252, 0.025), 0);
}

TEST(CostProfileTest, WideIntermediateCostPlateauPreservesBothWells)
{
  CostProfile profile;
  profile.target_distance = 0.26;
  profile.second_lane_distance = 0.60;
  profile.inter_lane_peak_cost = 100;
  profile.inter_lane_plateau_ratio = 0.70;
  for (const double d : {0.33, 0.40, 0.43, 0.50, 0.53}) {
    EXPECT_EQ(computeBorderCost(d, profile), 100);
  }
  EXPECT_LT(computeBorderCost(0.30, profile), 100);
  EXPECT_LT(computeBorderCost(0.56, profile), 100);
  EXPECT_EQ(computeBorderCost(0.26, profile), 0);
  EXPECT_EQ(computeBorderCost(0.60, profile), 0);
}

TEST(CostProfileTest, NearTurquoisePlateauHasRampsAndKeepsBothWells)
{
  CostProfile profile;
  profile.target_distance = 0.41;
  profile.second_lane_distance = 0.75;
  profile.near_sigma = 0.05;
  profile.near_band_start = 0.15;
  profile.near_band_width = 0.15;
  profile.near_band_cost = 100;
  for (const double d : {0.15, 0.20, 0.25, 0.30}) {
    EXPECT_EQ(computeBorderCost(d, profile), 100);
  }
  EXPECT_GT(computeBorderCost(0.14, profile), 0);
  EXPECT_LT(computeBorderCost(0.14, profile), 100);
  EXPECT_GT(computeBorderCost(0.31, profile), 0);
  EXPECT_LT(computeBorderCost(0.31, profile), 100);
  EXPECT_EQ(computeBorderCost(0.35, profile), 0);
  EXPECT_EQ(computeBorderCost(0.41, profile), 0);
  EXPECT_EQ(computeBorderCost(0.75, profile), 0);
}

TEST(CostProfileTest, CentralTurquoisePlateauCanBeWidenedWithoutMovingItsCenter)
{
  CostProfile profile;
  profile.target_distance = 0.41;
  profile.second_lane_distance = 0.75;
  profile.inter_lane_peak_cost = 100;
  profile.inter_lane_plateau_ratio = 0.90;
  EXPECT_EQ(computeBorderCost(0.45, profile), 100);
  EXPECT_EQ(computeBorderCost(0.58, profile), 100);
  EXPECT_EQ(computeBorderCost(0.71, profile), 100);
  EXPECT_EQ(computeBorderCost(0.41, profile), 0);
  EXPECT_EQ(computeBorderCost(0.75, profile), 0);
}
