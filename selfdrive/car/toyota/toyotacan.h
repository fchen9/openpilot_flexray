#ifndef TOYOTACAN_H_
#define TOYOTACAN_H_

namespace toyota {
// *** Toyota specific ***

void fix(std::vector<uint8_t> & msg, uint32_t addr);
can_frame make_can_msg(uint32_t addr, std::vector<uint8_t> &dat, uint8_t alt, bool cks=false);
can_frame create_ipas_steer_command(CANPacker &packer, double steer, bool enabled, bool apgs_enabled);
can_frame create_steer_command(CANPacker &packer, double steer, double steer_req, double raw_cnt);
can_frame create_lta_steer_command(CANPacker &packer, double steer, double steer_req, double raw_cnt, double angle);
can_frame create_accel_command(CANPacker &packer, double accel, bool pcm_cancel, bool standstill_req, bool lead);
can_frame create_acc_cancel_command(CANPacker &packer);
can_frame create_fcw_command(CANPacker &packer, double fcw);
can_frame create_ui_command(CANPacker &packer, double steer, bool chime, bool left_line, bool right_line, bool left_lane_depart, bool right_lane_depart);

}

#endif
