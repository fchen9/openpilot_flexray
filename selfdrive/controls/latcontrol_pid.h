#ifndef LATCONTROL_PID_H_
#define LATCONTROL_PID_H_

#include "pid.h"

class LatControlPID: public LatControlBase {
public:
  LatControlPID(const cereal::CarParams::Reader &CP);
  void reset();
  void update(bool active, double v_ego, double angle_steers, double angle_steers_rate, double eps_torque,
              bool steer_override, const cereal::CarParams::Reader &CP,
              const cereal::PathPlan::Reader&path_plan, double &output_steer_v, double & output_steering_angle,
              void *log_ptr);

  PIController pid;
};

#endif