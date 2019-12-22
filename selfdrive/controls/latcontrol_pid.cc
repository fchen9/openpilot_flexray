#include <capnp/serialize.h>
#include "cereal/gen/cpp/car.capnp.h"
#include "cereal/gen/cpp/log.capnp.h"
#include "pid.h"
#include "drive_helpers.h"
#include "latcontrol_base.h"
#include "latcontrol_pid.h"

LatControlPID::LatControlPID(const cereal::CarParams::Reader &CP):
  pid(CP.getLateralTuning().getPid().getKpBP(), CP.getLateralTuning().getPid().getKpV(),
      CP.getLateralTuning().getPid().getKiBP(), CP.getLateralTuning().getPid().getKiV(),
      CP.getLateralTuning().getPid().getKf(), 1.) {
}

void LatControlPID::reset() {
  pid.reset();
}

void LatControlPID::update(bool active, double v_ego, double angle_steers, double angle_steers_rate, double eps_torque,
            bool steer_override, const cereal::CarParams::Reader &CP, const cereal::PathPlan::Reader &path_plan,
            double &output_steer_v, double & output_steering_angle, void *log_ptr) {
  cereal::ControlsState::LateralPIDState::Builder *pid_log = (cereal::ControlsState::LateralPIDState::Builder *)log_ptr;
  pid_log->setSteerAngle(angle_steers);
  pid_log->setSteerRate(angle_steers_rate);

  if(v_ego < 0.3 or !active) {
    output_steer = 0.0;
    pid_log->setActive(false);
    pid.reset();
  } else {
    angle_steers_des = path_plan.getAngleSteers();  // get from MPC/PathPlanner

    double steers_max = get_steer_max(CP, v_ego);
    pid.pos_limit = steers_max;
    pid.neg_limit = -steers_max;
    double steer_feedforward = angle_steers_des;   // feedforward desired angle
    if(CP.getSteerControlType() == cereal::CarParams::SteerControlType::TORQUE) {
      // TODO: feedforward something based on path_plan.rateSteers
      steer_feedforward -= path_plan.getAngleOffset();   // subtract the offset, since it does not contribute to resistive torque
      steer_feedforward *= v_ego*v_ego;  // proportional to realigning tire momentum (~ lateral accel)
    }
    double deadzone = 0.0f;
    output_steer = pid.update(angle_steers_des, angle_steers, v_ego, (v_ego > 10), steer_override,
                                   steer_feedforward, deadzone);
    pid_log->setActive(true);
    pid_log->setP(pid.p);
    pid_log->setI(pid.i);
    pid_log->setF(pid.f);
    pid_log->setOutput(output_steer);
    pid_log->setSaturated(pid.saturated);
  }
  sat_flag = pid.saturated;
  output_steer_v = output_steer;
  output_steering_angle = angle_steers_des;
}
