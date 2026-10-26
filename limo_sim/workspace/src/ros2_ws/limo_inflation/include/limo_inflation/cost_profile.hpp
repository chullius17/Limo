#ifndef LIMO_INFLATION__COST_PROFILE_HPP_
#define LIMO_INFLATION__COST_PROFILE_HPP_

#include <cstdint>

namespace limo_inflation
{

struct CostProfile
{
  double target_distance{0.15};
  double second_lane_distance{0.30};
  double tolerance{0.02};
  std::uint8_t near_max_cost{60};
  std::uint8_t inter_lane_peak_cost{15};
  double far_cost_slope{5.0};
  std::uint8_t far_max_cost{8};
  double near_sigma{0.0};  // 0 keeps the original linear near-obstacle ramp.
  double inter_lane_plateau_ratio{0.0};  // Fraction of the inter-lane span at peak cost.
  double near_band_start{0.0};
  double near_band_width{0.0};  // 0 disables the additional near-obstacle plateau.
  double near_band_ramp{0.025};
  std::uint8_t near_band_cost{100};
};

std::uint8_t computeNearCost(double distance, double span, std::uint8_t peak, double sigma);

std::uint8_t computeBorderCost(double distance, const CostProfile & profile);

}  // namespace limo_inflation

#endif  // LIMO_INFLATION__COST_PROFILE_HPP_
