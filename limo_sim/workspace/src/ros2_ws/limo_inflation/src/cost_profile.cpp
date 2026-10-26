#include "limo_inflation/cost_profile.hpp"

#include <algorithm>
#include <cmath>

namespace limo_inflation
{

std::uint8_t computeNearCost(
  const double distance, const double span, const std::uint8_t peak, const double sigma)
{
  if (distance <= 0.0) {
    return peak;
  }
  if (distance >= span) {
    return 0;
  }
  double ratio = 1.0 - distance / span;
  if (sigma > 0.0) {
    // Truncate and normalize the Gaussian so the preferred band stays at zero.
    // expm1 keeps this well-conditioned when sigma is large compared to span.
    const double t = distance / sigma;
    const double end = span / sigma;
    const double denominator = -std::expm1(-0.5 * end * end);
    if (denominator > 0.0) {
      ratio = 1.0 + std::expm1(-0.5 * t * t) / denominator;
    }
  }
  return static_cast<std::uint8_t>(std::lround(peak * std::clamp(ratio, 0.0, 1.0)));
}

std::uint8_t computeBorderCost(
  const double distance, const CostProfile & profile)
{
  const double first_lower = profile.target_distance - profile.tolerance;
  const double first_upper = profile.target_distance + profile.tolerance;
  const double second_lower =
    profile.second_lane_distance - profile.tolerance;
  const double second_upper =
    profile.second_lane_distance + profile.tolerance;

  if (distance < first_lower) {
    int cost = computeNearCost(distance, first_lower, profile.near_max_cost, profile.near_sigma);
    if (profile.near_band_width > 0.0) {
      const double start = profile.near_band_start;
      const double end = start + profile.near_band_width;
      double ratio = 0.0;
      if (distance >= start && distance <= end) {
        ratio = 1.0;
      } else if (profile.near_band_ramp > 0.0) {
        ratio = distance < start ?
          (distance - start + profile.near_band_ramp) / profile.near_band_ramp :
          (end + profile.near_band_ramp - distance) / profile.near_band_ramp;
      }
      const int band_cost = static_cast<int>(std::lround(
            profile.near_band_cost * std::clamp(ratio, 0.0, 1.0)));
      cost = std::max(cost, band_cost);
    }
    return static_cast<std::uint8_t>(cost);
  }
  if (distance <= first_upper) {
    return 0;
  }
  if (distance < second_lower) {
    // Flatten the center of the ridge to widen the intermediate-cost band.
    const double midpoint = 0.5 * (first_upper + second_lower);
    const double half_span = 0.5 * (second_lower - first_upper);
    const double normalized = (1.0 - std::abs(distance - midpoint) / half_span) /
      (1.0 - profile.inter_lane_plateau_ratio);
    return static_cast<std::uint8_t>(std::lround(
      static_cast<double>(profile.inter_lane_peak_cost) *
      std::clamp(normalized, 0.0, 1.0)));
  }
  if (distance <= second_upper) {
    return 0;
  }

  const double far_cost = profile.far_cost_slope * (distance - second_upper);
  return static_cast<std::uint8_t>(std::lround(std::clamp(
    far_cost, 0.0, static_cast<double>(profile.far_max_cost))));
}

}  // namespace limo_inflation
