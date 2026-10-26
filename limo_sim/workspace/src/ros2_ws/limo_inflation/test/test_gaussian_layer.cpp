#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "limo_inflation/gaussian_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

// Real LayeredCostmap resets its master before fusing these observations.
class GaussianSources : public nav2_costmap_2d::Layer
{
public:
  std::vector<std::tuple<unsigned int, unsigned int, unsigned char>> cells;
  void reset() override {}
  void updateBounds(double, double, double, double *, double *, double *, double *) override {}
  void updateCosts(nav2_costmap_2d::Costmap2D & map, int, int, int, int) override
  {
    for (const auto & cell : cells) {
      map.setCost(std::get<0>(cell), std::get<1>(cell), std::get<2>(cell));
    }
  }
};

class GaussianTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<nav2_util::LifecycleNode>("gaussian_test");
    map = std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom", true, false);
    map->resizeMap(250, 250, 0.02, -2.5, -2.5);
    sources = std::make_shared<GaussianSources>();
    layer = std::make_shared<limo_inflation::GaussianLayer>();
    layer->initialize(map.get(), "inflation_layer", nullptr, node, nullptr, nullptr);
    map->addPlugin(sources);
    map->addPlugin(layer);
    sources->cells = {{125, 125, 254}};
  }
  void TearDown() override
  {
    map.reset();
    layer.reset();
    node.reset();
    rclcpp::shutdown();
  }
  void configure(const std::string & name)
  {
    layer = std::make_shared<limo_inflation::GaussianLayer>();
    layer->initialize(map.get(), name, nullptr, node, nullptr, nullptr);
    map->getPlugins()->back() = layer;
  }
  unsigned char cost(unsigned int x, unsigned int y = 125)
  {
    return map->getCostmap()->getCost(x, y);
  }
  std::shared_ptr<nav2_util::LifecycleNode> node;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> map;
  std::shared_ptr<GaussianSources> sources;
  std::shared_ptr<limo_inflation::GaussianLayer> layer;
};

TEST_F(GaussianTest, UsesMetricSigmaAndKeepsLethalOnlyAtOriginalCells)
{
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(125), 254);
  EXPECT_EQ(cost(126), 154);  // 254 exp(-0.5), at 2 cm = sigma.
  EXPECT_EQ(cost(127), 34);
  EXPECT_EQ(cost(128), 3);
  EXPECT_EQ(cost(129), 0);
  EXPECT_EQ(cost(124), cost(126));
  EXPECT_EQ(cost(125, 126), cost(126));
}

TEST_F(GaussianTest, SourceServiceReturnsCoherentRawCellsBeforeBothInflations)
{
  node->declare_parameter("broad.broad_enabled", true);
  configure("broad");
  sources->cells = {{125, 125, 88}, {200, 200, 254}};
  map->updateMap(0.0, 0.0, 0.0);
  ASSERT_EQ(cost(125), 100);  // The final grid hides the original soft class cost.
  auto client = node->create_client<nav2_msgs::srv::GetCostmap>("~/broad/get_sources");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto response = client->async_send_request(std::make_shared<nav2_msgs::srv::GetCostmap::Request>());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  ASSERT_EQ(executor.spin_until_future_complete(response, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto & snapshot = response.get()->map;
  EXPECT_EQ(snapshot.header.frame_id, "odom");
  EXPECT_EQ(snapshot.metadata.size_x, 250u);
  EXPECT_EQ(snapshot.metadata.size_y, 250u);
  EXPECT_NEAR(snapshot.metadata.resolution, 0.02, 1e-8);
  EXPECT_DOUBLE_EQ(snapshot.metadata.origin.position.x, -2.5);
  EXPECT_DOUBLE_EQ(snapshot.metadata.origin.position.y, -2.5);
  ASSERT_EQ(snapshot.data.size(), 62500u);
  EXPECT_EQ(snapshot.data[125 * 250 + 125], 88);
  EXPECT_EQ(snapshot.data[125 * 250 + 126], 0);
  EXPECT_EQ(snapshot.data[200 * 250 + 200], 254);
  const auto source_count = std::count_if(snapshot.data.begin(), snapshot.data.end(),
    [](unsigned char value) {return value > 0;});
  EXPECT_EQ(source_count, 2);
}

TEST_F(GaussianTest, PreservesClassCostsAndDoesNotSeedUnknown)
{
  sources->cells = {{125, 125, 88}, {124, 125, nav2_costmap_2d::NO_INFORMATION}};
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(125), 88);
  EXPECT_EQ(cost(126), 53);
  EXPECT_EQ(cost(124), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_EQ(cost(123), 12);  // Only the real source contributes here.
  sources->cells = {{124, 125, nav2_costmap_2d::NO_INFORMATION}};
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(123), 0);
  EXPECT_EQ(cost(125), 0);
}

TEST_F(GaussianTest, CombinesOverlappingInflationsWithMaximumNeverSum)
{
  sources->cells = {{124, 125, 254}, {126, 125, 254}};
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(125), 154);  // Two identical 154 contributions remain 154.
  EXPECT_EQ(cost(124), 254);
  EXPECT_EQ(cost(126), 254);
}

TEST_F(GaussianTest, FartherStrongerSourceCanWinOverNearestSoftSource)
{
  sources->cells = {{124, 125, 20}, {127, 125, 254}};
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(125), 34);  // max(20 exp(-0.5), 254 exp(-2)).
}

TEST_F(GaussianTest, SupportsAnisotropicSigma)
{
  node->declare_parameter("wide.sigma_x", 0.04);
  configure("wide");
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(127), 154);
  EXPECT_EQ(cost(125, 127), 34);
}

TEST_F(GaussianTest, SupportsOffDiagonalCovarianceInSquareMeters)
{
  node->declare_parameter("rotated.covariance_xy", 0.0002);
  configure("rotated");
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(126, 126), 130);
  EXPECT_EQ(cost(126, 124), 34);
  EXPECT_EQ(cost(124, 124), cost(126, 126));
}

TEST_F(GaussianTest, ClearsRemovedSourcesAndNeverInflatesPreviousOutput)
{
  map->updateMap(0.0, 0.0, 0.0);
  const auto * grid = map->getCostmap();
  const std::vector<unsigned char> first(grid->getCharMap(), grid->getCharMap() + 62500);
  for (int i = 0; i < 5; ++i) {
    map->updateMap(0.0, 0.0, 0.0);
    EXPECT_TRUE(std::equal(first.begin(), first.end(), grid->getCharMap()));
  }
  sources->cells.clear();
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_TRUE(std::all_of(grid->getCharMap(), grid->getCharMap() + 62500,
      [](unsigned char value) {return value == 0;}));
}

TEST_F(GaussianTest, RebuildsForRollingWindowResolutionAndReset)
{
  map->updateMap(0.0, 0.0, 0.0);
  map->updateMap(0.2, 0.1, 0.0);
  EXPECT_EQ(cost(126), 154);
  layer->reset();
  map->updateMap(0.2, 0.1, 0.0);
  EXPECT_EQ(cost(126), 154);
  map->resizeMap(150, 150, 0.04, -3.0, -3.0);
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(126), 34);
}

TEST_F(GaussianTest, RejectsInvalidCovariance)
{
  node->declare_parameter("zero.sigma_x", 0.0);
  EXPECT_THROW(configure("zero"), std::invalid_argument);
  node->declare_parameter("nan.sigma_y", std::numeric_limits<double>::quiet_NaN());
  EXPECT_THROW(configure("nan"), std::invalid_argument);
  node->declare_parameter("singular.covariance_xy", 0.0004);
  EXPECT_THROW(configure("singular"), std::invalid_argument);
  node->declare_parameter("indefinite.covariance_xy", -0.0005);
  EXPECT_THROW(configure("indefinite"), std::invalid_argument);
}

TEST_F(GaussianTest, ThresholdPreservesButDoesNotInflateExcludedCosts)
{
  node->declare_parameter("filtered.obstacle_cost_threshold", 200);
  configure("filtered");
  sources->cells = {{125, 125, 88}};
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(125), 88);
  EXPECT_EQ(cost(126), 0);
}

TEST_F(GaussianTest, BroadFieldHasIdenticalAmplitudeForEverySourceClass)
{
  node->declare_parameter("broad.broad_enabled", true);
  configure("broad");
  sources->cells = {{50, 125, 88}, {125, 125, 176}, {200, 125, 254}};
  map->updateMap(0.0, 0.0, 0.0);
  // At 20 cm only the broad field survives, independent of class or lethal status.
  EXPECT_EQ(cost(60), 41);
  EXPECT_EQ(cost(135), 41);
  EXPECT_EQ(cost(210), 41);
  EXPECT_EQ(cost(126), 107);  // The original narrow 176-peak field still wins here.
  EXPECT_EQ(cost(200), 254);
}

TEST_F(GaussianTest, BroadFieldUsesOriginalSourcesAndMaxAcrossBothFields)
{
  node->declare_parameter("broad.broad_enabled", true);
  configure("broad");
  sources->cells = {{120, 125, 88}, {130, 125, 254}};
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(125), 80);  // Two broad contributions of 80; no summation.
  EXPECT_EQ(cost(131), 154);  // max(narrow 154, broad 99), not their sum.
  EXPECT_EQ(cost(140), 41);  // 20 cm from original source, not from its inflated halo.
}

TEST_F(GaussianTest, BroadFieldClearsRemovedSourcesAndPreservesUnknown)
{
  node->declare_parameter("broad.broad_enabled", true);
  configure("broad");
  sources->cells = {{125, 125, 88}, {126, 125, nav2_costmap_2d::NO_INFORMATION}};
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(135), 41);
  EXPECT_EQ(cost(126), nav2_costmap_2d::NO_INFORMATION);
  sources->cells = {{126, 125, nav2_costmap_2d::NO_INFORMATION}};
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(135), 0);
  EXPECT_EQ(cost(127), 0);
}

TEST_F(GaussianTest, BroadFieldRespectsConfiguredSigmaPeakAndResolution)
{
  node->declare_parameter("custom.broad_enabled", true);
  node->declare_parameter("custom.broad_sigma", 0.20);
  node->declare_parameter("custom.broad_peak_cost", 80);
  configure("custom");
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(135), 49);  // 80 exp(-0.5), at sigma = 20 cm.
  layer->reset();
  map->resizeMap(150, 150, 0.04, -3.0, -3.0);
  map->updateMap(0.0, 0.0, 0.0);
  EXPECT_EQ(cost(135), 11);  // At 40 cm = two sigma.
}

TEST_F(GaussianTest, BroadFieldMatchesDirectGaussianMaximumAtEveryCell)
{
  node->declare_parameter("broad.broad_enabled", true);
  configure("broad");
  sources->cells = {{1, 1, 10}, {11, 22, 88}, {73, 51, 176}, {100, 20, 254}};
  map->updateMap(0.0, 0.0, 0.0);
  const auto * grid = map->getCostmap();
  const std::vector<unsigned char> combined(grid->getCharMap(), grid->getCharMap() + 62500);
  configure("narrow_only");
  map->updateMap(0.0, 0.0, 0.0);
  for (unsigned int y = 0; y < 250; ++y) {
    for (unsigned int x = 0; x < 250; ++x) {
      unsigned char expected = cost(x, y);
      for (const auto & source : sources->cells) {
        const double dx = (static_cast<double>(x) - std::get<0>(source)) * 0.02;
        const double dy = (static_cast<double>(y) - std::get<1>(source)) * 0.02;
        const auto broad = static_cast<unsigned char>(std::lround(
            100.0 * std::exp(-(dx * dx + dy * dy) / (2.0 * 0.15 * 0.15))));
        expected = std::max(expected, broad);
      }
      ASSERT_EQ(combined[y * 250 + x], expected) << "cell " << x << ", " << y;
    }
  }
}

TEST_F(GaussianTest, RejectsInvalidBroadParameters)
{
  node->declare_parameter("zero.broad_sigma", 0.0);
  EXPECT_THROW(configure("zero"), std::invalid_argument);
  node->declare_parameter("nan.broad_sigma", std::numeric_limits<double>::quiet_NaN());
  EXPECT_THROW(configure("nan"), std::invalid_argument);
  node->declare_parameter("lethal.broad_peak_cost", 254);
  EXPECT_THROW(configure("lethal"), std::invalid_argument);
}

TEST_F(GaussianTest, HandlesMovingObservationsAndReportsUpdateTimings)
{
  node->declare_parameter("broad.broad_enabled", true);
  for (const int count : {700, 5000}) {
    sources->cells.clear();
    for (int i = 0; i < count; ++i) {
      const int index = (i * 79 + 500) % 62500;
      sources->cells.emplace_back(index % 250, index / 250, i % 2 ? 88 : 254);
    }
    for (const auto & name : {"narrow", "broad"}) {
      configure(name);
      std::vector<double> timings;
      for (int i = 0; i < 25; ++i) {
        std::get<0>(sources->cells[0]) = 5 + i % 2;  // Force a real rebuild, bypassing cache.
        const auto start = std::chrono::steady_clock::now();
        map->updateMap(0.0, 0.0, 0.0);
        const double milliseconds = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
        if (i >= 5) {timings.push_back(milliseconds);}
        EXPECT_EQ(cost(5 + i % 2, 2), 254);
      }
      std::sort(timings.begin(), timings.end());
      std::cout << "Gaussian " << name << ", " << count << " sources, 250x250: median="
                << timings[timings.size() / 2] << " ms, p95=" << timings[18] << " ms\n";
    }
  }
}
