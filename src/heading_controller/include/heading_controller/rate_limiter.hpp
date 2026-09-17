#pragma once

namespace heading_controller
{

// Limits how fast a commanded value can change per cycle (max_accel),
// and clamps the final value to an absolute ceiling (max_rate).
// Used to keep assist-driven yaw commands smooth rather than snapping.
class RateLimiter
{
public:
  RateLimiter(double max_rate, double max_accel);

  // Applies rate limiting given the previous output, a new raw target,
  // and the time elapsed since the previous cycle. Returns the limited value.
  double limit(double raw_value, double previous_value, double dt) const;

  // Reset reference point — used when re-entering assist mode so it
  // doesn't ramp from a stale value.
  void reset(double value);

  double lastValue() const { return last_value_; }

private:
  double max_rate_;
  double max_accel_;
  double last_value_{0.0};
};

}  // namespace heading_controller