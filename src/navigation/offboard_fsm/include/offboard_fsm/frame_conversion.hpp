#pragma once

#include <cmath>

namespace offboard_fsm::frame
{
constexpr double kPiHalf = 1.57079632679489661923;
constexpr double kTwoPi = 6.28318530717958647692;

inline void enu_to_ned(double east, double north, double up,
                       float &north_out, float &east_out, float &down_out)
{
  north_out = static_cast<float>(north);
  east_out = static_cast<float>(east);
  down_out = static_cast<float>(-up);
}

inline double enu_yaw_to_ned(double yaw_enu)
{
  return std::remainder(kPiHalf - yaw_enu, kTwoPi);
}

inline double enu_yaw_rate_to_ned(double yaw_rate_enu)
{
  return -yaw_rate_enu;
}
}  // namespace offboard_fsm::frame
