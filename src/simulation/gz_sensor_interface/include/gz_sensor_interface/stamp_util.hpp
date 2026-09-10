// Shared monotonic-timestamp helper for the sensor_interface nodes.
//
// Gazebo's sim clock can stutter / regress, and FAST-LIO aborts on
// non-monotonic sensor stamps ("lidar loop back, clear buffer", "cannot store
// a negative time point"). These relays clamp every forwarded stamp to a
// strictly increasing sequence, nudging a regressing/duplicate stamp 1 µs
// past the last one published — mirroring the behaviour of the original
// Python/C++ nodes (imu_relay / add_time_field / lidar_sensor).
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
  /// Returns the (possibly adjusted) stamp for convenience.
  builtin_interfaces::msg::Time clamp(builtin_interfaces::msg::Time stamp)
  {
    if (has_last_) {
      const std::uint64_t cur = key(stamp);
      if (cur <= last_) {
        ++clamped_;
        // Nudge forward by 1 µs past the last published stamp.
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

}  // namespace sensor_interface
