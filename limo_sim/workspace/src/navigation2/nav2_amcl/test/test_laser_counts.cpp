// Copyright (c) 2026 Giulio Cataldo
// Licensed under the Apache License, Version 2.0.
#include <gtest/gtest.h>
#include <cstdlib>
#include <limits>
#include <memory>

#include "nav2_amcl/sensors/laser/laser.hpp"

namespace
{
class LaserCounts : public ::testing::Test
{
protected:
  void SetUp() override
  {
    map = map_alloc();
    map->size_x = map->size_y = 20;
    map->scale = 1.0;
    map->cells = static_cast<map_cell_t *>(std::calloc(400, sizeof(map_cell_t)));
    // A vertical wall at world x=5.
    for (int y = 0; y < 20; ++y) {
      map->cells[MAP_INDEX(map, 15, y)].occ_state = 1;
    }
    samples[0].weight = samples[1].weight = 0.5;
    pf.sets[0].samples = samples;
    pf.sets[0].sample_count = 2;
    data.range_count = 9;
    data.range_max = 10;
    data.ranges = new double[9][2]{};
    // With 3 beams, all models select indices 0, 4, 8 or 0, 3, 6.
    for (int i = 0; i < 9; ++i) {
      data.ranges[i][0] = 5;
    }
  }
  void TearDown() override {map_free(map);}

  void initialize(nav2_amcl::Laser & model)
  {
    data.laser = &model;
    auto pose = pf_vector_zero();
    model.SetLaserPose(pose);
  }

  map_t * map;
  pf_t pf{};
  pf_sample_t samples[2]{};
  nav2_amcl::LaserData data;
};

TEST_F(LaserCounts, LikelihoodFieldCountsSubsamplingAndInvalidReturns)
{
  nav2_amcl::LikelihoodFieldModel model(0.8, 0.2, 0.2, 2.0, 3, map);
  initialize(model);
  data.ranges[4][0] = std::numeric_limits<double>::quiet_NaN();
  data.ranges[8][0] = data.range_max;
  ASSERT_TRUE(model.sensorUpdate(&pf, &data));
  EXPECT_EQ(data.candidate_points, 1u);
  EXPECT_EQ(data.used_points, 1u);
  EXPECT_EQ(data.evaluations, 2u);
  EXPECT_EQ(data.particles, 2);
  ASSERT_TRUE(model.sensorUpdate(&pf, &data));
  EXPECT_EQ(data.used_points, 1u);  // Counts reset on each update.
}

TEST_F(LaserCounts, BeamModelCountsItsMaxRangeReadings)
{
  nav2_amcl::BeamModel model(0.6, 0.1, 0.1, 0.2, 0.2, 0.1, 0, 3, map);
  initialize(model);
  data.ranges[4][0] = data.range_max;
  ASSERT_TRUE(model.sensorUpdate(&pf, &data));
  EXPECT_EQ(data.used_points, 3u);
  EXPECT_EQ(data.evaluations, 6u);
}

TEST_F(LaserCounts, ProbabilisticModelCountsOnlySelectedFiniteReturns)
{
  nav2_amcl::LikelihoodFieldModelProb model(
    0.8, 0.2, 0.2, 2.0, false, 0.5, 0.3, 0.9, 3, map);
  initialize(model);
  data.ranges[3][0] = std::numeric_limits<double>::infinity();
  ASSERT_TRUE(model.sensorUpdate(&pf, &data));
  EXPECT_EQ(data.candidate_points, 2u);
  EXPECT_EQ(data.used_points, 2u);
  EXPECT_EQ(data.evaluations, 4u);
}

TEST_F(LaserCounts, BeamSkippingSeparatesCandidatesFromUsedPoints)
{
  nav2_amcl::LikelihoodFieldModelProb model(
    0.8, 0.2, 0.2, 2.0, true, 0.5, 0.3, 0.9, 3, map);
  initialize(model);
  pf.sets[0].converged = true;
  data.ranges[3][0] = 2;  // Far from the wall: rejected by beam skipping.
  ASSERT_TRUE(model.sensorUpdate(&pf, &data));
  EXPECT_EQ(data.candidate_points, 3u);
  EXPECT_EQ(data.used_points, 2u);
  EXPECT_EQ(data.evaluations, 6u);
}
}  // namespace
