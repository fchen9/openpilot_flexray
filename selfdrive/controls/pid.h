#ifndef PID_H_
#define PID_H_

inline double apply_deadzone(double error, double deadzone) {
  if(error > deadzone)
    error -= deadzone;
  else if(error < - deadzone)
    error += deadzone;
  else
    error = 0.;
  return error;
}

typedef double (*convert_func)(double accel, double speed);

class PIController {
public:
  PIController(const capnp::List<float>::Reader &k_p_0, const capnp::List<float>::Reader &k_p_1,
              const capnp::List<float>::Reader &k_i_0, const capnp::List<float>::Reader &k_i_1,
              double k_f=1., double pos_limit=0., double neg_limit=0., double rate=100.,
              double sat_limit=0.8, convert_func convert=NULL);

  double k_p();
  double k_i();
  bool _check_saturation(double control, bool override, double error);
  void reset();
  double update(double setpoint, double measurement, double speed=0.0, bool check_saturation=true, bool override=false,
              double feedforward=0., double deadzone=0., bool freeze_integrator=false);

  const capnp::List<float>::Reader _k_p_0, _k_p_1, _k_i_0, _k_i_1;
  double k_f, pos_limit, neg_limit, sat_count_rate, i_unwind_rate, i_rate, sat_limit, speed;
  double p, i, f, sat_count, control;
  bool saturated;
  convert_func convert;
};

#endif