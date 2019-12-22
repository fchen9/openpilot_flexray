#include <capnp/serialize.h>
#include "cereal/gen/cpp/car.capnp.h"
#include "cereal/gen/cpp/log.capnp.h"
#include "drive_helpers.h"
#include "pid.h"

PIController::PIController(const capnp::List<float>::Reader &k_p_0, const capnp::List<float>::Reader &k_p_1,
            const capnp::List<float>::Reader &k_i_0, const capnp::List<float>::Reader &k_i_1,
            double k_f, double pos_limit, double neg_limit, double rate,
            double sat_limit, convert_func convert):
              _k_p_0(k_p_0), _k_p_1(k_p_1), // proportional gain
              _k_i_0(k_i_0), _k_i_1(k_i_1) // integral gain
             {
  this->k_f = k_f;  // feedforward gain
  this->pos_limit = pos_limit;
  this->neg_limit = neg_limit;

  sat_count_rate = 1.0 / rate;
  i_unwind_rate = 0.3 / rate;
  i_rate = 1.0 / rate;
  this->sat_limit = sat_limit;
  this->convert = convert;

  reset();
}

double PIController::k_p() {
  return interp(speed, _k_p_0, _k_p_1);
}

double PIController::k_i() {
  return interp(speed, _k_i_0, _k_i_1);
}

bool PIController::_check_saturation(double control, bool override, double error) {
  bool saturated = (control < neg_limit) or (control > pos_limit);

  if(saturated and !override and std::abs(error) > 0.1)
    sat_count += sat_count_rate;
  else
    sat_count -= sat_count_rate;

  sat_count = clip<double>(sat_count, 0.0, 1.0);

  return sat_count > sat_limit;
}

void PIController::reset() {
  p = 0.0;
  i = 0.0;
  f = 0.0;
  sat_count = 0.0;
  saturated = false;
  control = 0;
}

double PIController::update(double setpoint, double measurement, double speed, bool check_saturation, bool override,
            double feedforward, double deadzone, bool freeze_integrator) {
  this->speed = speed;

  double error = apply_deadzone(setpoint - measurement, deadzone);
  p = error * k_p();
  f = feedforward * k_f;

  if(override) {
    i -= i_unwind_rate * sign(i);
  } else {
    double ii = i + error * k_i() * i_rate;
    control = p + f + ii;

    if(convert != NULL)
      control = convert(control, this->speed);
    // Update when changing i will move the control away from the limits
    // or when i will move towards the sign of the error
    if(((error >= 0 and (control <= pos_limit or ii < 0.0)) or \
        (error <= 0 and (control >= neg_limit or ii > 0.0))) and \
        !freeze_integrator)
      i = ii;
  }
  control = p + f + i;
  if(convert != NULL)
    control = convert(control, this->speed);

  if(check_saturation)
    saturated = _check_saturation(control, override, error);
  else
    saturated = false;

  control = clip<double>(control, neg_limit, pos_limit);
  return control;
}
