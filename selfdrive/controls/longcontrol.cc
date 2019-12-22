#include <capnp/serialize.h>
#include "cereal/gen/cpp/car.capnp.h"
#include "cereal/gen/cpp/log.capnp.h"
#include "drive_helpers.h"
#include "longcontrol.h"

static const double STOPPING_EGO_SPEED = 0.5;
static const double MIN_CAN_SPEED = 0.3;  // TODO: parametrize this in car interface
static const double STOPPING_TARGET_SPEED = MIN_CAN_SPEED + 0.01;
const double STARTING_TARGET_SPEED = 0.5;
static const double BRAKE_THRESHOLD_TO_PID = 0.2;

static const double STOPPING_BRAKE_RATE = 0.2;  // brake_travel/s while trying to stop
static const double STARTING_BRAKE_RATE = 0.8;  // brake_travel/s while releasing on restart
static const double BRAKE_STOPPING_TARGET = 0.5;  // apply at least this amount of brake to maintain the vehicle stationary

static const double RATE = 100.0;

static cereal::ControlsState::LongControlState long_control_state_trans(bool active,
                            cereal::ControlsState::LongControlState long_control_state,
                            double v_ego, double v_target, double v_pid,
                             double output_gb, bool brake_pressed, bool cruise_standstill) {
  // Update longitudinal control state machine
  bool stopping_condition = (v_ego < 2.0 and cruise_standstill) or \
                       (v_ego < STOPPING_EGO_SPEED and \
                        ((v_pid < STOPPING_TARGET_SPEED and v_target < STOPPING_TARGET_SPEED) or \
                        brake_pressed));

  bool starting_condition = v_target > STARTING_TARGET_SPEED and !cruise_standstill;

  if(!active)
    long_control_state = cereal::ControlsState::LongControlState::OFF;
  else {
    if(long_control_state == cereal::ControlsState::LongControlState::OFF) {
      if(active)
        long_control_state = cereal::ControlsState::LongControlState::PID;
    }
    else if(long_control_state == cereal::ControlsState::LongControlState::PID) {
      if(stopping_condition)
        long_control_state = cereal::ControlsState::LongControlState::STOPPING;
    }
    else if(long_control_state == cereal::ControlsState::LongControlState::STOPPING) {
      if(starting_condition)
        long_control_state = cereal::ControlsState::LongControlState::STARTING;
    }
    else if(long_control_state == cereal::ControlsState::LongControlState::STARTING) {
      if(stopping_condition)
        long_control_state = cereal::ControlsState::LongControlState::STOPPING;
      else if(output_gb >= -BRAKE_THRESHOLD_TO_PID)
        long_control_state = cereal::ControlsState::LongControlState::PID;
    }
  }
  return long_control_state;
}

LongControl::LongControl(const cereal::CarParams::Reader &CP, convert_func compute_gb):
    long_control_state(cereal::ControlsState::LongControlState::OFF),  // initialized to off
    pid(CP.getLongitudinalTuning().getKpBP(), CP.getLongitudinalTuning().getKpV(),
                            CP.getLongitudinalTuning().getKiBP(), CP.getLongitudinalTuning().getKiV(),
                            1., 0., 0.,
                            RATE,
                            0.8,
                            compute_gb),
    v_pid(0.0),
    last_output_gb(0.0) {
}

void LongControl::reset(double v_pid) {
  // Reset PID controller and change setpoint
  pid.reset();
  this->v_pid = v_pid;
}

void LongControl::update(bool active, double v_ego, bool brake_pressed, bool standstill, bool cruise_standstill, double v_cruise,
            double v_target, double v_target_future, double a_target,
            const cereal::CarParams::Reader &CP, double &final_gas, double &final_brake) {
  // Update longitudinal control. This updates the state machine and runs a PID loop
  // Actuation limits
  double gas_max = interp(v_ego, CP.getGasMaxBP(), CP.getGasMaxV());
  double brake_max = interp(v_ego, CP.getBrakeMaxBP(), CP.getBrakeMaxV());

  // Update state machine
  double output_gb = last_output_gb;
  long_control_state = long_control_state_trans(active, long_control_state, v_ego,
                                                     v_target_future, v_pid, output_gb,
                                                     brake_pressed, cruise_standstill);

  double v_ego_pid = std::max(v_ego, MIN_CAN_SPEED);  // Without this we get jumps, CAN bus reports 0 when speed < 0.3
  if(long_control_state == cereal::ControlsState::LongControlState::OFF) {
    v_pid = v_ego_pid;
    pid.reset();
    output_gb = 0.;
  }
  // tracking objects and driving
  else if(long_control_state == cereal::ControlsState::LongControlState::PID) {
    v_pid = v_target;
    pid.pos_limit = gas_max;
    pid.neg_limit = - brake_max;

    // Toyota starts braking more when it thinks you want to stop
    // Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
    bool prevent_overshoot = !CP.getStoppingControl() and v_ego < 1.5 and v_target_future < 0.7;
    double deadzone = interp(v_ego_pid, CP.getLongitudinalTuning().getDeadzoneBP(), CP.getLongitudinalTuning().getDeadzoneV());
    output_gb = pid.update(v_pid, v_ego_pid, v_ego_pid, true, false, a_target, deadzone, prevent_overshoot);

    if(prevent_overshoot)
      output_gb = std::min(output_gb, 0.0);
  }
  // Intention is to stop, switch to a different brake control until we stop
  else if(long_control_state == cereal::ControlsState::LongControlState::STOPPING) {
    // Keep applying brakes until the car is stopped
    if(!standstill or output_gb > -BRAKE_STOPPING_TARGET)
      output_gb -= STOPPING_BRAKE_RATE / RATE;
    output_gb = clip(output_gb, -brake_max, gas_max);

    v_pid = v_ego;
    pid.reset();
  }
  // Intention is to move again, release brake fast before handing control to PID
  else if(long_control_state == cereal::ControlsState::LongControlState::STARTING) {
    if(output_gb < -0.2)
      output_gb += STARTING_BRAKE_RATE / RATE;
    v_pid = v_ego;
    pid.reset();
  }
  last_output_gb = output_gb;
  final_gas = clip<double>(output_gb, 0., gas_max);
  final_brake = -clip<double>(output_gb, -brake_max, 0.);
}
