// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_omni_pid_pursuit_controller/pid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

PID::PID(
  double dt, double max, double min, double kp, double kd, double ki,
  double integral_min, double integral_max)
: dt_(dt),
  max_(max),
  min_(min),
  kp_(kp),
  kd_(kd),
  ki_(ki),
  pre_error_(0),
  integral_(0),
  integral_min_(integral_min),
  integral_max_(integral_max)
{
  if (!std::isfinite(dt_) || dt_ <= 0.0) {
    throw std::invalid_argument("PID: dt must be positive and finite");
  }
  if (!std::isfinite(max_) || !std::isfinite(min_) || min_ > max_) {
    throw std::invalid_argument("PID: max/min must be finite and max >= min");
  }
  if (!std::isfinite(kp_) || !std::isfinite(ki_) || !std::isfinite(kd_)) {
    throw std::invalid_argument("PID: gains must be finite");
  }
}

double PID::calculate(double set_point, double pv)
{
  // Reject NaN/Inf inputs
  if (!std::isfinite(set_point) || !std::isfinite(pv)) {
    return 0.0;
  }

  // Calculate error
  double error = set_point - pv;

  // Handle first call: avoid derivative kick
  if (!initialized_) {
    pre_error_ = error;
    initialized_ = true;
  }

  // Proportional term
  double p_out = kp_ * error;

  // Integral term with anti-windup clamping (clamp BEFORE computing output)
  integral_ += error * dt_;
  integral_ = std::clamp(integral_, integral_min_, integral_max_);
  double i_out = ki_ * integral_;

  // Derivative term (skip on first call)
  double d_out = 0.0;
  if (initialized_) {
    double derivative = (error - pre_error_) / dt_;
    d_out = kd_ * derivative;
  }

  // Calculate total output
  double output = p_out + i_out + d_out;

  // Reject NaN/Inf output
  if (!std::isfinite(output)) {
    output = 0.0;
  }

  // Restrict to max/min
  output = std::clamp(output, min_, max_);

  // Save error for next derivative calculation
  pre_error_ = error;

  return output;
}

void PID::reset()
{
  integral_ = 0.0;
  pre_error_ = 0.0;
  initialized_ = false;
}

void PID::setGains(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PID::setSumError(double sum_error) { integral_ = sum_error; }

PID::~PID() {}
