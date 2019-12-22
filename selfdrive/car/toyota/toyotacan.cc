#include <map>
#include <vector>
#include <string>

#include "cereal/gen/cpp/car.capnp.h"
#include "drive_helpers.h"
#include "can_define.h"
#include "canpacker.h"
#include "carinterface.h"
#include "toyota/toyotacan.h"

namespace toyota {
// *** Toyota specific ***

void fix(std::vector<uint8_t> & msg, uint32_t addr) {
  int checksum = 0;
  int idh = (addr & 0xff00) >> 8;
  int idl = (addr & 0xff);

  checksum = idh + idl + msg.size() + 1;
  for(auto &d_byte: msg)
    checksum += d_byte;
  msg.push_back(checksum & 0xFF);
}

can_frame make_can_msg(uint32_t addr, std::vector<uint8_t> &dat, uint8_t alt, bool cks) {
  if(cks)
    fix(dat, addr);
  can_frame cf;
  cf.address = addr;
  cf.busTime = 0;
  cf.dat = dat;
  cf.src = alt;
  return cf;
}

can_frame create_ipas_steer_command(CANPacker &packer, double steer, bool enabled, bool apgs_enabled) {
  // Creates a CAN message for the Toyota Steer Command.
  double direction = 0;
  if(steer < 0)
    direction = 3;
  else if(steer > 0)
    direction = 1;
  else
    direction = 2;
  double mode = enabled ? 3 : 1;
  std::map<std::string, double> values = {
    {"STATE", mode},
    {"DIRECTION_CMD", direction},
    {"ANGLE", steer},
    {"SET_ME_X10", 0x10},
    {"SET_ME_X40", 0x40}
  };
  if(apgs_enabled)
    return packer.make_can_msg("STEERING_IPAS", 0, values);
  else
    return packer.make_can_msg("STEERING_IPAS_COMMA", 0, values);
}

can_frame create_steer_command(CANPacker &packer, double steer, double steer_req, double raw_cnt) {
  //Creates a CAN message for the Toyota Steer Command.
  std::map<std::string, double> values = {
    {"STEER_REQUEST", steer_req},
    {"STEER_TORQUE_CMD", steer},
    {"COUNTER", raw_cnt},
    {"SET_ME_1", 1}
  };
  return packer.make_can_msg("STEERING_LKA", 0, values);
}

can_frame create_lta_steer_command(CANPacker &packer, double steer, double steer_req, double raw_cnt, double angle) {
  // Creates a CAN message for the Toyota LTA Steer Command.
  std::map<std::string, double> values = {
    {"COUNTER", raw_cnt},
    {"SETME_X3", 3},
    {"PERCENTAGE", 100},
    {"SETME_X64", 0x64},
    {"ANGLE", angle},
    {"STEER_ANGLE_CMD", steer},
    {"STEER_REQUEST", steer_req},
    {"BIT", 0}
  };
  return packer.make_can_msg("STEERING_LTA", 0, values);
}

can_frame create_accel_command(CANPacker &packer, double accel, bool pcm_cancel, bool standstill_req, bool lead) {
  // TODO, find the exact canceling bit that does not create a chime
  std::map<std::string, double> values = {
    {"ACCEL_CMD", accel},
    {"SET_ME_X01", 1},
    {"DISTANCE", 0},
    {"MINI_CAR", lead},
    {"SET_ME_X3", 3},
    {"SET_ME_1", 1},
    {"RELEASE_STANDSTILL", !standstill_req},
    {"CANCEL_REQ", pcm_cancel}
  };
  return packer.make_can_msg("ACC_CONTROL", 0, values);
}

can_frame create_acc_cancel_command(CANPacker &packer) {
  std::map<std::string, double> values = {
    {"GAS_RELEASED", 0},
    {"CRUISE_ACTIVE", 0},
    {"STANDSTILL_ON", 0},
    {"ACCEL_NET", 0},
    {"CRUISE_STATE", 0},
    {"CANCEL_REQ", 1}
  };
  return packer.make_can_msg("PCM_CRUISE", 0, values);
}

can_frame create_fcw_command(CANPacker &packer, double fcw) {
  std::map<std::string, double> values = {
    {"FCW", fcw},
    {"SET_ME_X20", 0x20},
    {"SET_ME_X10", 0x10},
    {"SET_ME_X80", 0x80}
  };
  return packer.make_can_msg("ACC_HUD", 0, values);
}

can_frame create_ui_command(CANPacker &packer, double steer, bool chime, bool left_line, bool right_line, bool left_lane_depart, bool right_lane_depart) {
  std::map<std::string, double> values = {
    {"RIGHT_LINE", right_lane_depart? 3 : (right_line? 1 : 2)},
    {"LEFT_LINE", left_lane_depart? 3 : (left_line? 1 : 2)},
    {"BARRIERS" , (left_lane_depart or right_lane_depart)? 3 : 0},
    {"SET_ME_X0C", 0x0c},
    {"SET_ME_X2C", 0x2c},
    {"SET_ME_X38", 0x38},
    {"SET_ME_X02", 0x02},
    {"SET_ME_X01", 1},
    {"SET_ME_X01_2", 1},
    {"REPEATED_BEEPS", 0},
    {"TWO_BEEPS", chime},
    {"LDA_ALERT", steer}
  };
  return packer.make_can_msg("LKAS_HUD", 0, values);
}

}