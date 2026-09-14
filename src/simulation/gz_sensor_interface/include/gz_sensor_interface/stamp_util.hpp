// Shared monotonic-timestamp helper for sensor-interface relays.
//
// Gazebo simulation time can stutter or regress. Downstream consumers require
// monotonic sensor timestamps, so relays clamp duplicate/regressing stamps to
// one microsecond past the previous published stamp.
#pragma once

#include <cstdint>

#include <builtin_interfaces/msg/time.hpp>

namespace gz_sensor_interface
{

/// Maintains the last-published stamp and clamps new stamps to a strictly
/// increasing sequence. Thread-unsafe; use from a single callback.
class StampMonotonicizer
{
public:
  /// Clamp `stamp` so it is strictly after the last one seen, then record it.
  builtin_interfaces::msg::Time clamp(builtin_interfaces::msg::Time stamp)
  {
    if (has_last_) {
      const std::uint64_t cur = key(stamp);
      if (cur <= last_) {
        ++clamped_;
        last_ += 1000ULL;
        stamp.sec = static_cast<std::int32_t>(last_ / 1000000000ULL);
        stamp.nanosec = static_cast<std::uint32_t>(last_ % 1000000000ULL);
      } else {
        last_ = cur;
      }
    } else {
      last_ = key(stamp);
      has_last_ = true;
    }
    return stamp;
  }

  std::uint64_t clamped() const { return clamped_; }

private:
  static std::uint64_t key(const builtin_interfaces::msg::Time &t)
  {
    return static_cast<std::uint64_t>(t.sec) * 1000000000ULL + t.nanosec;
  }

  bool has_last_{false};
  std::uint64_t last_{0};
  std::uint64_t clamped_{0};
};

}  // namespace gz_sensor_interface
