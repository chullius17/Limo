// Copyright (c) 2026 Giulio Cataldo
// Licensed under the Apache License, Version 2.0.
#ifndef NAV2_AMCL__EXPERIMENTS__CV_DIAGNOSTICS_HPP_
#define NAV2_AMCL__EXPERIMENTS__CV_DIAGNOSTICS_HPP_

#include <cstdint>
#include <string>

namespace nav2_amcl
{
// One CV attempt. Counts describe template cells, not particle evaluations.
struct CvFusionDiagnostics
{
  bool accepted{false};
  bool reused{false};
  std::string reason;
  uint64_t input_points{0};
  uint64_t candidate_points{0};
  uint64_t non_road_points{0};
  uint64_t particles{0};
  uint64_t evaluations{0};
  double cloud_stamp{-1.0};
  double sync_error{-1.0};
};
}  // namespace nav2_amcl
#endif  // NAV2_AMCL__EXPERIMENTS__CV_DIAGNOSTICS_HPP_
