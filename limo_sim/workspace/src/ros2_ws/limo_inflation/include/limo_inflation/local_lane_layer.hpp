#ifndef LIMO_INFLATION__LOCAL_LANE_LAYER_HPP_
#define LIMO_INFLATION__LOCAL_LANE_LAYER_HPP_

#include <cstdint>
#include <vector>

#include "limo_inflation/cost_profile.hpp"
#include "nav2_costmap_2d/layer.hpp"

namespace limo_inflation
{

// Run LAST, after the sensor layers. updateBounds requests a full master reset,
// so only freshly fused observations, never our previous costs, seed the field.
class LocalLaneLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;

private:
  void rebuildField(const nav2_costmap_2d::Costmap2D & source);

  CostProfile profile_;
  int obstacle_cost_threshold_{1};
  double high_cost_expansion_radius_{0.0};
  int high_cost_threshold_{200};
  int high_cost_expansion_cost_{252};
  unsigned int width_{0};
  unsigned int height_{0};
  double resolution_{0.0};
  std::vector<std::uint8_t> source_costs_;
  std::vector<std::uint8_t> lane_costs_;
};

}  // namespace limo_inflation

#endif  // LIMO_INFLATION__LOCAL_LANE_LAYER_HPP_
