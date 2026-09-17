#include "heading_controller/rate_limiter.hpp"

#include <algorithm>

namespace heading_controller
{

RateLimiter::RateLimiter(double max_rate, double max_accel)
: max_rate_(max_rate), max_accel_(max_accel)
{
}

double RateLimiter::limit(double raw_value, double previous_value, double dt) const
{
  // Cap how much the value can change this cycle based on max_accel
  const double max_delta = max_accel_ * dt;
  const double delta = std::clamp(raw_value - previous_value, -max_delta, max_delta);
  double limited = previous_value + delta;

  // Cap the absolute value based on max_rate
  limited = std::clamp(limited, -max_rate_, max_rate_);
  return limited;
}

void RateLimiter::reset(double value)
{
  last_value_ = value;
}

}  // namespace heading_controller