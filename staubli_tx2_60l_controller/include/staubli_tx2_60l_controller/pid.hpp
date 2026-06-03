#pragma once
#include <algorithm>

namespace staubli_tx2_60l_controller {

struct PIDGains {
  double kp = 1.0;
  double ki = 0.0;
  double kd = 0.0;
  double max_output = 10.0;
  double max_integral = 2.0;
};

class PIDController {
public:
  PIDController() : gains_{} {};

  explicit PIDController(PIDGains gains) : gains_(gains) {}

  void reset() {
    integral_     = 0.0;
    last_error_   = 0.0;
    initialized_  = false;
  }

  double compute(double error, double dt) {
    if (dt <= 0.0) return 0.0;

    double p_term = gains_.kp * error;

    integral_ += error * dt;
    integral_  = std::clamp(integral_, -gains_.max_integral, gains_.max_integral); //saturation
    double i_term = gains_.ki * integral_;

    double d_term = 0.0;
    if (initialized_) {
      d_term = gains_.kd * (error - last_error_) / dt;
    }
    initialized_ = true;
    last_error_   = error;

    double output = p_term + i_term + d_term;
    return std::clamp(output, -gains_.max_output, gains_.max_output);
  }

private:
  PIDGains gains_;
  double integral_   = 0.0;
  double last_error_ = 0.0;
  bool   initialized_ = false;
};

}