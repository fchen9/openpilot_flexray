#ifndef LONGCONTROL_H_
#define LONGCONTROL_H_

#include "pid.h"

extern const double STARTING_TARGET_SPEED;

class LongControl {
public:
  LongControl(const cereal::CarParams::Reader &CP, convert_func compute_gb);
  void reset(double v_pid);
  void update(bool active, double v_ego, bool brake_pressed, bool standstill, bool cruise_standstill, double v_cruise,
              double v_target, double v_target_future, double a_target,
              const cereal::CarParams::Reader &CP, double &final_gas, double &final_brake);

  cereal::ControlsState::LongControlState long_control_state;
  double v_pid, last_output_gb;
  PIController pid;
};

#endif