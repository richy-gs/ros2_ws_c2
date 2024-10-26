#include "pd.hpp"
#include <algorithm>
#include <cmath>

PD::PD(const PDParameters &params) { params_ = params; }

double PD::update(double measurement, double desired) {
  double error = desired - measurement;

  double d = (error - prev_error_) / params_.kDt;
  prev_error_ = error;

  double u = params_.kP * error + params_.kD * d;

  // If ramp rate is disabled, or if we are within ramp rate, go to U.
  if (!params_.enable_ramp_rate_limit ||
      std::abs((set_u_ - u)) < params_.ramp_rate * params_.kDt) {
        set_u_ = u;
  } else {
    // Ramp rate is enabled, and we can only increase by ramp rate.
    set_u_ += std::copysign(params_.ramp_rate * params_.kDt, u - set_u_);
  }

  return std::clamp(set_u_, params_.kUMin, params_.kUMax);
}

PDParameters PD::defaultParams() {
  PDParameters p{};
  p.kD = 0.0;
  p.kD = 0.0;
  p.kDt = 0.0;
  p.kUMax = 0.0;
  p.kUMin = 0.0;
  p.enable_ramp_rate_limit = false;
  p.ramp_rate = 0.0;
  return p;
}