#ifndef LIMO_INFLATION__PATH_DISTANCE_LAYER_HPP_
#define LIMO_INFLATION__PATH_DISTANCE_LAYER_HPP_

#include <array>
#include <cstdint>
#include <mutex>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace limo_inflation
{

// Add last, after obstacle inflation. Attraction costs must never become
// obstacle seeds on the next update, and must never create collision costs.
class PathDistanceLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void updateBounds(
    double, double, double, double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override;
  void reset() override;
  void setPath(nav_msgs::msg::Path::ConstSharedPtr path);
  bool matchesPath(const nav_msgs::msg::Path & path);
  bool containsFootprint(const geometry_msgs::msg::Pose2D & pose);
  bool projectProgress(const geometry_msgs::msg::Pose2D & pose, double & progress,
    double & heading, double minimum = -1e30, double maximum = 1e30) const;
  double tubeRadius() const {return tube_radius_;}

private:
  void rebuildField(
    const nav2_costmap_2d::Costmap2D &, const nav_msgs::msg::Path &,
    const std::array<double, 3> & transform);

  int max_cost_{50};
  double distance_for_max_cost_{0.50};
  double tube_radius_{0.60};
  bool tube_field_valid_{false};
  std::mutex path_mutex_;
  nav_msgs::msg::Path::ConstSharedPtr path_;
  nav_msgs::msg::Path::ConstSharedPtr cached_path_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;
  unsigned int width_{0}, height_{0};
  double resolution_{0.0}, origin_x_{0.0}, origin_y_{0.0};
  std::array<double, 3> cached_transform_{};
  std::vector<std::uint8_t> cached_mask_;
  std::vector<std::uint8_t> path_costs_;
  std::vector<float> path_distances_;
  std::vector<std::array<double, 3>> path_points_;  // x, y, cumulative arc length.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
};

}  // namespace limo_inflation

#endif  // LIMO_INFLATION__PATH_DISTANCE_LAYER_HPP_
