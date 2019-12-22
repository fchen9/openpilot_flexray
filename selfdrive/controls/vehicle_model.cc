#include "cereal/gen/cpp/car.capnp.h"
#include "vehicle_model.h"

/*
Dynamic bycicle model from "The Science of Vehicle Dynamics (2014), M. Guiggiani"

The state is x = [v, r]^T
with v lateral speed [m/s], and r rotational speed [rad/s]

The input u is the steering angle [rad]

The system is defined by
x_dot = A*x + B*u

A depends on longitudinal speed, u [m/s], and vehicle parameters CP
*/

VehicleModel::VehicleModel(const cereal::CarParams::Reader &CP) {
  // for math readability, convert long names car params into short names
  m = CP.getMass();
  j = CP.getRotationalInertia();
  l = CP.getWheelbase();
  aF = CP.getCenterToFront();
  aR = CP.getWheelbase() - CP.getCenterToFront();
  chi = CP.getSteerRatioRear();

  cF_orig = CP.getTireStiffnessFront();
  cR_orig = CP.getTireStiffnessRear();
  update_params(1.0, CP.getSteerRatio());
}

void VehicleModel::update_params(double stiffness_factor, double steer_ratio) {
  // Update the vehicle model with a new stiffness factor and steer ratio
  cF = stiffness_factor * cF_orig;
  cR = stiffness_factor * cR_orig;
  sR = steer_ratio;
}

double VehicleModel::calc_curvature(double sa, double u) {
  /* Returns the curvature. Multiplied by the speed this will give the yaw rate.
  Args:
    sa: Steering wheel angle [rad]
    u: Speed [m/s]

  Returns:
    Curvature factor [rad/m]
  */
  return curvature_factor(u) * sa / sR;
}

double VehicleModel::calc_slip_factor() {
  /* The slip factor is a measure of how the curvature changes with speed
  it's positive for Oversteering vehicle, negative (usual case) otherwise.
  */
  return m * (cF * aF - cR * aR) / (l * l * cF * cR);
}

double VehicleModel::curvature_factor(double u) {
  /* Returns the curvature factor.
  Multiplied by wheel angle (not steering wheel angle) this will give the curvature.

  Args:
    u: Speed [m/s]

  Returns:
    Curvature factor [1/m]
  */
  double sf = calc_slip_factor();
  return (1. - chi) / (1. - sf * (u * u)) / l;
}

double VehicleModel::get_steer_from_curvature(double curv, double u) {
  /* Calculates the required steering wheel angle for a given curvature

  Args:
    curv: Desired curvature [rad/s]
    u: Speed [m/s]

  Returns:
    Steering wheel angle [rad]
  */

  return curv * sR * 1.0 / curvature_factor(u);
}

double VehicleModel::yaw_rate(double sa, double u) {
  /* Calculate yaw rate

  Args:
    sa: Steering wheel angle [rad]
    u: Speed [m/s]

  Returns:
    Yaw rate [rad/s]
  */
  return calc_curvature(sa, u) * u;
}
