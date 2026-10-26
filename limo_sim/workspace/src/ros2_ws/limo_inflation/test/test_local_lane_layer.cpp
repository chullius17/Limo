#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "limo_inflation/local_lane_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/lifecycle_node.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

// Represents the freshly fused sensor layers. The real LayeredCostmap must
// clear the previous output before these observations and lane costs are added.
class SensorLayer : public nav2_costmap_2d::Layer
{
public:
  bool wall{true};
  std::vector<std::tuple<unsigned int, unsigned int, unsigned char>> cells;
  void reset() override {}
  void updateBounds(double, double, double, double *, double *, double *, double *) override {}
  void updateCosts(nav2_costmap_2d::Costmap2D & map, int, int, int, int) override
  {
    if (wall) {
      for (unsigned int y = 0; y < map.getSizeInCellsY(); ++y) {
        map.setCost(0, y, LETHAL_OBSTACLE);
      }
    }
    for (const auto & cell : cells) {
      map.setCost(std::get<0>(cell), std::get<1>(cell), std::get<2>(cell));
    }
  }
};

class LocalLaneTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<nav2_util::LifecycleNode>("local_lane_test");
    map = std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom", true, false);
    map->resizeMap(100, 100, 0.01, -0.5, -0.5);
    sensors = std::make_shared<SensorLayer>();
    layer = std::make_shared<limo_inflation::LocalLaneLayer>();
    layer->initialize(map.get(), "inflation_layer", nullptr, node, nullptr, nullptr);
    map->addPlugin(sensors);
    map->addPlugin(layer);
  }
  void TearDown() override
  {
    map.reset();
    layer.reset();
    node.reset();
    rclcpp::shutdown();
  }
  unsigned char cost(unsigned int x, unsigned int y = 50)
  {
    return map->getCostmap()->getCost(x, y);
  }
  std::shared_ptr<nav2_util::LifecycleNode> node;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> map;
  std::shared_ptr<SensorLayer> sensors;
  std::shared_ptr<limo_inflation::LocalLaneLayer> layer;
};

TEST_F(LocalLaneTest, MatchesTrajectoryProfileAndPreservesObstacle)
{
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(0), LETHAL_OBSTACLE);
  EXPECT_GT(cost(5), cost(10));
  EXPECT_EQ(cost(16), 0);  // 0.22 / 2 + 0.05 m.
  EXPECT_GT(cost(24), 90);
  EXPECT_EQ(cost(33), 0);
  EXPECT_GT(cost(80), 0);
  EXPECT_LE(cost(80), 8);
}

TEST_F(LocalLaneTest, DoesNotAccumulateInflationAcrossUpdates)
{
  map->updateMap(0.0, 0.0, 0.0);
  const auto * grid = map->getCostmap();
  const std::vector<unsigned char> first(grid->getCharMap(), grid->getCharMap() + 10000);
  for (int i = 0; i < 5; ++i) {
    map->updateMap(0.0, 0.0, 0.0);
    EXPECT_TRUE(std::equal(first.begin(), first.end(), grid->getCharMap()));
  }
}

TEST_F(LocalLaneTest, ClearsEntireFieldWhenObstacleDisappears)
{
  map->updateMap(0.0, 0.0, 0.0);
  ASSERT_GT(cost(24), 0);
  sensors->wall = false;
  map->updateMap(0.0, 0.0, 0.0);
  const auto * grid = map->getCostmap();
  for (unsigned int i = 0; i < 10000; ++i) {
    EXPECT_EQ(grid->getCharMap()[i], 0);
  }
}

TEST_F(LocalLaneTest, IncludesSoftSemanticCellsAndIgnoresUnknownAsSeeds)
{
  sensors->wall = false;
  sensors->cells = {{50, 50, 90}, {80, 80, NO_INFORMATION}};
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(50), 90);  // A soft source must not become lethal.
  EXPECT_GT(cost(55), 0);
  EXPECT_EQ(cost(80, 80), NO_INFORMATION);
  sensors->cells = {{80, 80, NO_INFORMATION}};
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(79, 80), 0);
  EXPECT_EQ(cost(80, 80), NO_INFORMATION);
}

TEST_F(LocalLaneTest, RebuildsForRollingWindowAndResolutionChanges)
{
  map->updateMap(0.0, 0.0, 0.0);
  map->updateMap(0.2, 0.1, 0.0);
  EXPECT_EQ(cost(16), 0);
  EXPECT_GT(cost(24), 90);
  map->resizeMap(50, 50, 0.02, -0.5, -0.5);
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(8, 25), 0);
  EXPECT_GT(cost(12, 25), 90);
}

TEST_F(LocalLaneTest, RejectsNonFiniteOrOverlappingDistances)
{
  // Re-initialize with different names to exercise actual ROS YAML parameters.
  node->declare_parameter("bad.robot_width", std::numeric_limits<double>::quiet_NaN());
  auto bad = std::make_shared<limo_inflation::LocalLaneLayer>();
  EXPECT_THROW(bad->initialize(map.get(), "bad", nullptr, node, nullptr, nullptr),
    std::invalid_argument);
  node->declare_parameter("overlap.second_lane_distance", 0.17);
  auto overlap = std::make_shared<limo_inflation::LocalLaneLayer>();
  EXPECT_THROW(overlap->initialize(map.get(), "overlap", nullptr, node, nullptr, nullptr),
    std::invalid_argument);
}

TEST_F(LocalLaneTest, AppliesNonDefaultRosParameters)
{
  node->declare_parameter("real.safety_margin", 0.04);
  node->declare_parameter("real.second_lane_distance", 0.29);
  node->declare_parameter("real.far_cost_slope", 50.0);
  node->declare_parameter("real.far_max_cost", 40);
  auto real = std::make_shared<limo_inflation::LocalLaneLayer>();
  real->initialize(map.get(), "real", nullptr, node, nullptr, nullptr);
  map->getPlugins()->back() = real;
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(15), 0);
  EXPECT_GT(cost(22), 95);
  EXPECT_EQ(cost(29), 0);
  EXPECT_GE(cost(80), 24);
  EXPECT_LE(cost(80), 25);
}

TEST_F(LocalLaneTest, ThresholdUsesRawNav2Costs)
{
  node->declare_parameter("threshold.obstacle_cost_threshold", 200);
  auto threshold = std::make_shared<limo_inflation::LocalLaneLayer>();
  threshold->initialize(map.get(), "threshold", nullptr, node, nullptr, nullptr);
  map->getPlugins()->back() = threshold;
  sensors->wall = false;
  sensors->cells = {{50, 50, 90}};
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(50), 90);
  EXPECT_EQ(cost(55), 0);
  sensors->cells = {{50, 50, 254}};
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(50), 254);
  EXPECT_GT(cost(55), 0);
}

TEST_F(LocalLaneTest, ExpandsHighCostsWithMetricRadiusAndKeepsPreferredLane)
{
  node->declare_parameter("red.high_cost_expansion_radius", 0.05);
  auto red = std::make_shared<limo_inflation::LocalLaneLayer>();
  red->initialize(map.get(), "red", nullptr, node, nullptr, nullptr);
  map->getPlugins()->back() = red;
  map->resizeMap(50, 50, 0.05, -1.25, -1.25);
  sensors->wall = false;
  sensors->cells = {{25, 25, 228}, {5, 5, 90}, {25, 24, NO_INFORMATION}};
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(25, 25), 252);  // Boardwalk repulsion stays below lethal.
  EXPECT_EQ(cost(26, 25), 252);  // One 5 cm cell from the source.
  EXPECT_LT(cost(26, 26), 252);  // Diagonal is outside the metric radius.
  EXPECT_EQ(cost(25, 24), NO_INFORMATION);
  EXPECT_LT(cost(6, 5), 252);  // Lower-cost semantic points do not seed red.
  EXPECT_EQ(cost(28, 25), 0);  // First preferred band is unchanged.
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(26, 25), 252);  // Cached field must not grow.
  sensors->cells.clear();
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(26, 25), 0);
}

TEST_F(LocalLaneTest, RedExpansionPreservesLethalSeeds)
{
  node->declare_parameter("red.high_cost_expansion_radius", 0.05);
  auto red = std::make_shared<limo_inflation::LocalLaneLayer>();
  red->initialize(map.get(), "red", nullptr, node, nullptr, nullptr);
  map->getPlugins()->back() = red;
  sensors->wall = false;
  sensors->cells = {{50, 50, 254}};
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(50), 254);
  EXPECT_EQ(cost(55), 252);
  EXPECT_LT(cost(56), 252);
}

TEST_F(LocalLaneTest, LargerRedBandHasNarrowGaussianTailAndKeepsConfiguredWells)
{
  node->declare_parameter("red.high_cost_expansion_radius", 0.075);
  node->declare_parameter("red.near_cost_sigma", 0.025);
  node->declare_parameter("red.safety_margin", 0.15);
  node->declare_parameter("red.second_lane_distance", 0.60);
  node->declare_parameter("red.inter_lane_plateau_ratio", 0.70);
  auto red = std::make_shared<limo_inflation::LocalLaneLayer>();
  red->initialize(map.get(), "red", nullptr, node, nullptr, nullptr);
  map->getPlugins()->back() = red;
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(0), 254);
  EXPECT_EQ(cost(7), 252);
  EXPECT_LT(cost(8), 252);
  EXPECT_GT(cost(10), cost(15));
  EXPECT_LT(cost(15), 5);
  EXPECT_EQ(cost(26), 0);
  EXPECT_EQ(cost(34), 100);
  EXPECT_EQ(cost(52), 100);
  EXPECT_EQ(cost(60), 0);
}

TEST_F(LocalLaneTest, YellowAddsToLaneFieldWithoutErasingExistingCosts)
{
  node->declare_parameter("add.safety_margin", 0.30);
  node->declare_parameter("add.second_lane_distance", 0.75);
  node->declare_parameter("add.near_cost_sigma", 0.05);
  node->declare_parameter("add.inter_lane_plateau_ratio", 0.70);
  auto add = std::make_shared<limo_inflation::LocalLaneLayer>();
  add->initialize(map.get(), "add", nullptr, node, nullptr, nullptr);
  map->getPlugins()->back() = add;
  map->updateMap(0.0, 0.0, 0.0);
  ASSERT_EQ(cost(58), 100);
  const auto * grid = map->getCostmap();
  const std::vector<unsigned char> baseline(grid->getCharMap(), grid->getCharMap() + 10000);
  sensors->cells = {{58, 50, 177}};
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(58), 252);  // 177 + 100 saturates below lethal.
  EXPECT_GT(cost(58, 55), 100);  // Soft halo adds to the turquoise field too.
  for (unsigned int i = 0; i < 10000; ++i) {
    EXPECT_GE(grid->getCharMap()[i], baseline[i]);
    if (baseline[i] != LETHAL_OBSTACLE) {
      EXPECT_LT(grid->getCharMap()[i], 253);
    }
  }
  // Repeated fusion must not add the same yellow contribution a second time.
  const std::vector<unsigned char> combined(grid->getCharMap(), grid->getCharMap() + 10000);
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_TRUE(std::equal(combined.begin(), combined.end(), grid->getCharMap()));
  sensors->cells.clear();
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_TRUE(std::equal(baseline.begin(), baseline.end(), grid->getCharMap()));
}

TEST_F(LocalLaneTest, YellowAloneKeepsItsSourceCostAndDoesNotCreateLaneRings)
{
  sensors->wall = false;
  sensors->cells = {{50, 50, 177}};
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(50), 177);
  EXPECT_GT(cost(55), 0);
  EXPECT_EQ(cost(74), 0);  // No inter-lane ridge generated around yellow alone.
}

TEST_F(LocalLaneTest, ShowsNearAndCentralTurquoiseBandsAlongsideRedPlateau)
{
  node->declare_parameter("bands.safety_margin", 0.30);
  node->declare_parameter("bands.second_lane_distance", 0.75);
  node->declare_parameter("bands.high_cost_expansion_radius", 0.075);
  node->declare_parameter("bands.near_cost_sigma", 0.05);
  node->declare_parameter("bands.near_band_start", 0.15);
  node->declare_parameter("bands.near_band_width", 0.15);
  node->declare_parameter("bands.near_band_cost", 100);
  node->declare_parameter("bands.inter_lane_plateau_ratio", 0.90);
  auto bands = std::make_shared<limo_inflation::LocalLaneLayer>();
  bands->initialize(map.get(), "bands", nullptr, node, nullptr, nullptr);
  map->getPlugins()->back() = bands;
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(0), 254);
  EXPECT_EQ(cost(7), 252);
  EXPECT_EQ(cost(15), 100);
  EXPECT_EQ(cost(20), 100);
  EXPECT_EQ(cost(30), 100);
  EXPECT_EQ(cost(35), 0);
  EXPECT_EQ(cost(41), 0);
  EXPECT_EQ(cost(45), 100);
  EXPECT_EQ(cost(71), 100);
  EXPECT_EQ(cost(75), 0);
}

TEST_F(LocalLaneTest, RejectsNearBandThatWouldOverlapFirstWell)
{
  node->declare_parameter("bands.near_band_start", 0.10);
  node->declare_parameter("bands.near_band_width", 0.10);
  auto bands = std::make_shared<limo_inflation::LocalLaneLayer>();
  EXPECT_THROW(bands->initialize(map.get(), "bands", nullptr, node, nullptr, nullptr),
    std::invalid_argument);
}

