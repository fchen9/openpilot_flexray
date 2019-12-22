#include <vector>
#include <string>
#include <cmath>

#include "cereal/gen/cpp/car.capnp.h"
#include "cereal/gen/cpp/log.capnp.h"
#include "can_define.h"
#include "canpacker.h"
#include "drive_helpers.h"
#include "carinterface.h"

std::map<std::string, std::string> dbc_dict(std::string pt_dbc, std::string radar_dbc, std::string chassis_dbc) {
  std::map<std::string, std::string> m = {{"pt", pt_dbc}, {"radar", radar_dbc}, {"chassis", chassis_dbc}};
  return m;
}

int apply_toyota_steer_torque_limits(int apply_torque, int apply_torque_last, double motor_torque, const SteerLimitParams &LIMITS) {
  // limits due to comparison of commanded torque VS motor reported torque
  double max_lim = std::min(std::max(motor_torque + LIMITS.STEER_ERROR_MAX, LIMITS.STEER_ERROR_MAX), LIMITS.STEER_MAX);
  double min_lim = std::max(std::min(motor_torque - LIMITS.STEER_ERROR_MAX, -LIMITS.STEER_ERROR_MAX), -LIMITS.STEER_MAX);

  double apply_torque_f = clip((double)apply_torque, min_lim, max_lim);
  double apply_torque_last_f = (double)apply_torque_last;

  // slow rate if steer torque increases in magnitude
  if(apply_torque_last > 0)
    apply_torque_f = clip(apply_torque_f,
                        std::max(apply_torque_last_f - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP);
  else
    apply_torque_f = clip(apply_torque_f,
                        apply_torque_last_f - LIMITS.STEER_DELTA_UP,
                        std::min(apply_torque_last_f + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP));

  return std::round(apply_torque_f);
}

int crc8_pedal(std::vector<uint8_t> data) {
  int crc = 0xFF;    // standard init value
  int poly = 0xD5;   // standard crc8: x8+x7+x6+x4+x2+1
  for(int i = data.size() - 1; i >= 0;i--) {
    crc ^= data[i];
    for(int j = 0; j < 8; j++) {
      if((crc & 0x80) != 0)
        crc = ((crc << 1) ^ poly) & 0xFF;
      else
        crc <<= 1;
    }
  }
  return crc;
}

can_frame create_gas_command(CANPacker &packer, double gas_amount, int idx) {
  // Common gas pedal msg generator
  bool enable = gas_amount > 0.001;
  std::map<std::string, double> values = {
    {"ENABLE", enable},
    {"COUNTER_PEDAL", idx & 0xF}
  };

  if(enable) {
    values["GAS_COMMAND"] = gas_amount * 255.;
    values["GAS_COMMAND2"] = gas_amount * 255.;
  }

  can_frame ct = packer.make_can_msg("GAS_COMMAND", 0, values);

  std::vector<uint8_t> dat(ct.dat.begin(), ct.dat.end() - 1);
  int checksum = crc8_pedal(dat);
  values["CHECKSUM_PEDAL"] = checksum;

  return packer.make_can_msg("GAS_COMMAND", 0, values);
}
