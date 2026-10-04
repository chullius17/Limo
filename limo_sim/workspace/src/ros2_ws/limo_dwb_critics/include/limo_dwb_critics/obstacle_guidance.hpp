#ifndef LIMO_DWB_CRITICS__OBSTACLE_GUIDANCE_HPP_
#define LIMO_DWB_CRITICS__OBSTACLE_GUIDANCE_HPP_

#include <cstddef>
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
  unsigned int width_{0};
  unsigned int height_{0};
  double resolution_{0.0};
  std::vector<bool> blocked_;
  std::vector<double> penalties_;
  std::vector<double> distances_;
};

}  // namespace limo_dwb_critics
#endif  // LIMO_DWB_CRITICS__OBSTACLE_GUIDANCE_HPP_
