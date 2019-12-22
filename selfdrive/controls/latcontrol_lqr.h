#ifndef LATCONTROL_LQR_H_
#define LATCONTROL_LQR_H_

#include "latcontrol_base.h"

class LatControlLQR: public LatControlBase {
public:
  LatControlLQR(const cereal::CarParams::Reader &CP, double rate=100.);
  void reset();
  void update(bool active, double v_ego, double angle_steers, double angle_steers_rate, double eps_torque,
              bool steer_override, const cereal::CarParams::Reader &CP,
              const cereal::PathPlan::Reader &path_plan, double &output_steer_v, double & output_steering_angle,
              void *log_ptr);

  double scale, ki, A_0_0, A_0_1, A_1_0, A_1_1, B_0_0, B_1_0, C_0_0, C_0_1, K_0_0, K_0_1, L_0_0, L_1_0, dc_gain;
  double x_hat_0_0, x_hat_1_0, i_unwind_rate, i_rate, i_lqr;
};

#endif