#include <capnp/serialize.h>
#include "cereal/gen/cpp/car.capnp.h"
#include "cereal/gen/cpp/log.capnp.h"
#include "drive_helpers.h"
#include "latcontrol_lqr.h"

LatControlLQR::LatControlLQR(const cereal::CarParams::Reader &CP, double rate) {
  sat_flag = false;
  scale = CP.getLateralTuning().getLqr().getScale();
  ki = CP.getLateralTuning().getLqr().getKi();

  auto a = CP.getLateralTuning().getLqr().getA();
  A_0_0 = a[0]; A_0_1 = a[1]; A_1_0 = a[2]; A_1_1 = a[3];
  auto b = CP.getLateralTuning().getLqr().getB();
  B_0_0 = b[0]; B_1_0 = b[1];
  auto c = CP.getLateralTuning().getLqr().getC();
  C_0_0 = c[0]; C_0_1 = c[1];
  auto k = CP.getLateralTuning().getLqr().getK();
  K_0_0 = k[0]; K_0_1 = k[1];
  auto l = CP.getLateralTuning().getLqr().getL();
  L_0_0 = l[0]; L_1_0 = l[1];
  dc_gain = CP.getLateralTuning().getLqr().getDcGain();
  x_hat_0_0 = 0.; x_hat_1_0 = 0.;
  i_unwind_rate = 0.3 / rate;
  i_rate = 1.0 / rate;

  reset();
}

void LatControlLQR::reset() {
  i_lqr = 0.0;
  output_steer = 0.0;
}

void LatControlLQR::update(bool active, double v_ego, double angle_steers, double angle_steers_rate, double eps_torque,
            bool steer_override, const cereal::CarParams::Reader &CP,
            const cereal::PathPlan::Reader &path_plan, double &output_steer_v, double & output_steering_angle,
            void *log_ptr) {
  cereal::ControlsState::LateralLQRState::Builder *lqr_log = (cereal::ControlsState::LateralLQRState::Builder *)log_ptr;
  double steers_max = get_steer_max(CP, v_ego);
  double torque_scale = (0.45 + v_ego / 60.0) * (0.45 + v_ego / 60.0);  // Scale actuator model with speed

  // Subtract offset. Zero angle should correspond to zero torque
  angle_steers_des = (double)path_plan.getAngleSteers() - path_plan.getAngleOffset();
  angle_steers -= path_plan.getAngleOffset();

  // Update Kalman filter
  double angle_steers_k = C_0_0 * x_hat_0_0 + C_0_1 * x_hat_1_0;
  double e = angle_steers - angle_steers_k;
  double tmp_x_hat_0_0 = x_hat_0_0;
  double tmp_x_hat_1_0 = x_hat_1_0;
  x_hat_0_0 = A_0_0 * tmp_x_hat_0_0 + A_0_1 * tmp_x_hat_1_0 + B_0_0 * (eps_torque / torque_scale) + L_0_0 * e;
  x_hat_1_0 = A_1_0 * tmp_x_hat_0_0 + A_1_1 * tmp_x_hat_1_0 + B_1_0 * (eps_torque / torque_scale) + L_1_0 * e;
  double lqr_output;
  if(v_ego < 0.3 or !active) {
    lqr_log->setActive(false);
    lqr_output = 0.;
    reset();
  } else {
    lqr_log->setActive(true);
    // LQR
    double u_lqr = angle_steers_des / dc_gain - (K_0_0 * x_hat_0_0 + K_0_1 * x_hat_1_0);
    lqr_output = torque_scale * u_lqr / scale;

    // Integrator
    if(steer_override)
      i_lqr -= i_unwind_rate * sign(i_lqr);
    else {
      double error = angle_steers_des - angle_steers_k;
      double i = i_lqr + ki * i_rate * error;
      double control = lqr_output + i;

      if (((error >= 0 and (control <= steers_max or i < 0.0)) or \
          (error <= 0 and (control >= -steers_max or i > 0.0))))
        i_lqr = i;
    }
    output_steer = lqr_output + i_lqr;
    output_steer = clip<double>(output_steer, -steers_max, steers_max);
  }
  lqr_log->setSteerAngle(angle_steers_k + path_plan.getAngleOffset());
  lqr_log->setI(i_lqr);
  lqr_log->setOutput(output_steer);
  lqr_log->setLqrOutput(lqr_output);
  output_steer_v = output_steer;
  output_steering_angle = angle_steers_des;
}
