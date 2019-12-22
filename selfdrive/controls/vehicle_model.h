#ifndef VEHICLE_MODEL_H_
#define VEHICLE_MODEL_H_

/*
Dynamic bycicle model from "The Science of Vehicle Dynamics (2014), M. Guiggiani"

The state is x = [v, r]^T
with v lateral speed [m/s], and r rotational speed [rad/s]

The input u is the steering angle [rad]

The system is defined by
x_dot = A*x + B*u

A depends on longitudinal speed, u [m/s], and vehicle parameters CP
*/


class VehicleModel {
public:
  VehicleModel(const cereal::CarParams::Reader &CP);
  void update_params(double stiffness_factor, double steer_ratio);
  double calc_curvature(double sa, double u);
  double calc_slip_factor();
  double curvature_factor(double u);
  double get_steer_from_curvature(double curv, double u);
  double yaw_rate(double sa, double u);

private:
  double m, j, l, aF, aR, chi, cF_orig, cR_orig, cF, cR, sR;
};

#endif