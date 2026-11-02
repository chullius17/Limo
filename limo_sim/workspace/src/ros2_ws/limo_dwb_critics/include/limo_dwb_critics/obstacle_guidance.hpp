#ifndef LIMO_DWB_CRITICS__OBSTACLE_GUIDANCE_HPP_
#define LIMO_DWB_CRITICS__OBSTACLE_GUIDANCE_HPP_

#include <array>
#include <cstddef>
#include <utility>
#include <vector>

namespace limo_dwb_critics
{

// Cost-to-go through the live grid. This guides MPC scoring only: the bicycle
// model and footprint critic still decide whether a command is executable.
class ObstacleGuidance
{
public:
  void update(
    unsigned int width, unsigned int height, double resolution,
    const unsigned char * costs, double clearance, double cost_weight);
  bool traversable(int x, int y) const;
  bool setTarget(int x, int y);
  // Coordinates are measured in cells, with integer coordinates at centers.
  double distance(double x, double y) const;
  double heading(double x, double y, double current_heading) const;

private:
  struct Offset
  {
    int x;
    int y;
  };
  struct Neighbor
  {
    int x;
    int y;
    double length;
    double half_length;
    double heading;
  };
  using QueueEntry = std::pair<double, std::size_t>;

  unsigned int width_{0};
  unsigned int height_{0};
  double resolution_{0.0};
  std::vector<unsigned char> blocked_;
  std::vector<double> penalties_;
  std::vector<double> distances_;
  // Cache only geometry and scalar arithmetic, never live costs or routes.
  double clearance_radius_{-1.0};
  double penalty_weight_{-1.0};
  std::vector<Offset> clearance_offsets_;
  std::array<double, 256> penalty_lookup_{};
  std::array<Neighbor, 8> neighbors_{};
  // Same (distance, index) heap order as priority_queue, with retained capacity.
  std::vector<QueueEntry> queue_;
};

}  // namespace limo_dwb_critics
#endif  // LIMO_DWB_CRITICS__OBSTACLE_GUIDANCE_HPP_
