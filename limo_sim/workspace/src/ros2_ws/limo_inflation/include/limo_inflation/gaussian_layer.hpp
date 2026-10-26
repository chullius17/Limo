#ifndef LIMO_INFLATION__GAUSSIAN_LAYER_HPP_
#define LIMO_INFLATION__GAUSSIAN_LAYER_HPP_

#include <array>
#include <cstdint>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"

namespace limo_inflation
{

// Run after sensor fusion. Bounds cover the full rolling window so previous
// inflation never becomes a source and removed observations leave no ghosts.
class GaussianLayer : public nav2_costmap_2d::Layer
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
  struct Offset
  {
    int x;
    int y;
    double weight;
  };
  struct CostOffset
  {
    int x;
    int y;
    std::uint8_t cost;
  };

  void rebuildKernel();
  void rebuildField(const nav2_costmap_2d::Costmap2D & source);
  void applyBroadField();
  const std::vector<CostOffset> & kernel(std::uint8_t peak);

  double sigma_x_{0.02};
  double sigma_y_{0.02};
  double covariance_xy_{0.0};
  double correlation_{0.0};
  int obstacle_cost_threshold_{1};
  bool broad_enabled_{false};
  double broad_sigma_{0.15};
  int broad_peak_cost_{100};
  unsigned int width_{0};
  unsigned int height_{0};
  double resolution_{0.0};
  std::vector<Offset> offsets_;
  std::array<std::vector<CostOffset>, 255> kernels_;
  std::vector<std::uint8_t> source_costs_;
  std::vector<std::uint8_t> inflated_costs_;
  std::vector<std::uint8_t> broad_costs_;
  rclcpp::Service<nav2_msgs::srv::GetCostmap>::SharedPtr sources_service_;
  rclcpp::Time source_stamp_{0, 0, RCL_ROS_TIME};
  double source_origin_x_{0.0};
  double source_origin_y_{0.0};
};

}  // namespace limo_inflation

#endif  // LIMO_INFLATION__GAUSSIAN_LAYER_HPP_
