#ifndef TOYOTA_INTERFACE_H_
#define TOYOTA_INTERFACE_H_

namespace toyota {

class ToyotaCarInterface: public CarInterfaceBase {
public:
  ToyotaCarInterface(cereal::CarParams::Reader &car_param, const char *libdbc_fn);
  static double compute_gb(double accel, double speed);
  double calc_accel_override(double a_ego, double a_target, double v_ego, double v_target);
  void update(cereal::CarControl::Builder &c,
              std::vector<std::string> &can_strings,
              cereal::CarState::Builder &ret,
              std::vector<event_name_types_pair> &events);
  void apply(cereal::CarControl::Builder &c, std::vector<can_frame> &can_sends);
public:
  int64_t frame;
  bool gas_pressed_prev, brake_pressed_prev, cruise_enabled_prev, forwarding_camera;
  VehicleModel VM;
  CarState CS;
  std::unique_ptr<CarController> CC;
  bool no_stop_timer;
  cereal::CarParams::Reader &CP;
  std::unique_ptr<CanParser> cp, cp_cam;
};

}
#endif