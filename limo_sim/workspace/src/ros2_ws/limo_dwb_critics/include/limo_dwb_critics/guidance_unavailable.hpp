#ifndef LIMO_DWB_CRITICS__GUIDANCE_UNAVAILABLE_HPP_
#define LIMO_DWB_CRITICS__GUIDANCE_UNAVAILABLE_HPP_

#include <string>

#include "dwb_core/exceptions.hpp"

namespace limo_dwb_critics
{

// A safe local reference may disappear between scheduled avoidance updates.
// Keep the controller action alive while stopped, so its next update can retry.
class GuidanceUnavailable : public dwb_core::IllegalTrajectoryException
{
public:
  GuidanceUnavailable(const std::string & critic, const std::string & reason)
  : dwb_core::IllegalTrajectoryException(critic, reason) {}
};

}  // namespace limo_dwb_critics
#endif  // LIMO_DWB_CRITICS__GUIDANCE_UNAVAILABLE_HPP_
