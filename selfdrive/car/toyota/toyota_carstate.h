#ifndef TOYOTA_CARSTATE_H_
#define TOYOTA_CARSTATE_H_

namespace toyota {

CanParser *get_can_parser(const char *libdbc_fn, const cereal::CarParams::Reader & CP);
CanParser *get_cam_can_parser(const char *libdbc_fn, const cereal::CarParams::Reader & CP);

class CarState {
public:
  CarState(const cereal::CarParams::Reader &cp, const char *libdbc_fn);
  void update(CanParser &cp);

  const cereal::CarParams::Reader &CP;
  std::map<int, std::string> shifter_values;
  int left_blinker_on, right_blinker_on, prev_left_blinker_on, prev_right_blinker_on, can_gear, brake_error, user_brake;
  double angle_offset, v_ego, a_ego, v_ego_raw, v_wheel_fl, v_wheel_fr, v_wheel_rl, v_wheel_rr, v_ego_x_0, v_ego_x_1;
  double angle_steers_rate, angle_steers, main_on, steer_state, steer_torque_driver, steer_torque_motor, v_cruise_pcm;
  bool init_angle_offset, door_all_closed, seatbelt, standstill, is_tss2_car, is_no_dsu_car, steer_error;
  int brake_pressed;
  bool ipas_active, steer_override, pcm_acc_active, low_speed_lockout, brake_lights, generic_toggle;
  std::string car_fingerprint;
  cereal::CarState::GearShifter gear_shifter;
  KF1D v_ego_kf;
  double pedal_gas, car_gas, esp_disabled, pcm_acc_status;
};

}

#endif