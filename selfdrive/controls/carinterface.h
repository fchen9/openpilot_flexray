#ifndef CARINTERFACE_H_
#define CARINTERFACE_H_

typedef struct {
  double STEER_MAX;
  double STEER_DELTA_UP;
  double STEER_DELTA_DOWN;
  double STEER_ERROR_MAX;
} SteerLimitParams;

class CarInterfaceBase {
public:
  CarInterfaceBase() {}
  virtual ~CarInterfaceBase() {}
  virtual void update(cereal::CarControl::Builder &c,
                      std::vector<std::string> &can_strings,
                      cereal::CarState::Builder &ret,
                      std::vector<event_name_types_pair> &events) = 0;
  virtual void apply(cereal::CarControl::Builder &c, std::vector<can_frame> &can_sends) = 0;
  virtual double calc_accel_override(double a_ego, double a_target, double v_ego, double v_target) = 0;
};


std::map<std::string, std::string> dbc_dict(std::string pt_dbc, std::string radar_dbc, std::string chassis_dbc="");
int apply_toyota_steer_torque_limits(int apply_torque, int apply_torque_last, double motor_torque, const SteerLimitParams &LIMITS);
int crc8_pedal(std::vector<uint8_t> data);
can_frame create_gas_command(CANPacker &packer, double gas_amount, int idx);

#endif