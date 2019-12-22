#ifndef LATCONTROL_BASE_H_
#define LATCONTROL_BASE_H_

class LatControlBase {
public:
  LatControlBase(): sat_flag(false), angle_steers_des(0.f), output_steer(0.f) {}
  virtual ~LatControlBase() {}
  virtual void reset() = 0;
  virtual void update(bool active, double v_ego, double angle_steers, double angle_steers_rate, double eps_torque,
              bool steer_override, const cereal::CarParams::Reader &CP,
              const cereal::PathPlan::Reader &path_plan, double &output_steer_v, double & output_steering_angle,
              void *log_ptr) = 0;
public:
  bool sat_flag;
  double angle_steers_des, output_steer;
};

#endif