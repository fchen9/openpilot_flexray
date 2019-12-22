#ifndef TOYOTA_CARCONTROLLER_H_
#define TOYOTA_CARCONTROLLER_H_

namespace toyota {

class CarController {
public:
  CarController(const char *dbc_name, const char *libdbc_fn, const char *car_fingerprint,
                bool enable_camera, bool enable_dsu, bool enable_apg);
  void update(bool enabled, CarState &CS, int64_t frame, cereal::CarControl::Actuators::Builder actuators,
              bool pcm_cancel_cmd, cereal::CarControl::HUDControl::VisualAlert hud_alert, bool forwarding_camera,
              bool left_line, bool right_line, bool lead, bool left_lane_depart, bool right_lane_depart,
              std::vector<can_frame> &can_sends);
public:
  double accel_steady, last_accel;
  bool steer_angle_enabled;
  int ipas_reset_counter;
  bool braking;
  bool controls_allowed;
  int last_steer = 0;
  int last_angle = 0;
  std::string car_fingerprint;
  bool alert_active;
  bool last_standstill;
  bool standstill_req;
  bool angle_control;
  int last_fault_frame;
  std::unordered_set<ECU> fake_ecus;
  CANPacker packer;
  std::map<int64_t, std::vector<can_frame>> static_can_frames;
};

}
#endif