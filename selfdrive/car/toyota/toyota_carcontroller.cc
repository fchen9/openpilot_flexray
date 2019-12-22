#include <vector>
#include <cmath>
#include <stdint.h>
#include <map>
#include <string>
#include <unordered_set>
#include "cereal/gen/cpp/car.capnp.h"
#include "KF1D.h"
#include "can_define.h"
#include "canpacker.h"
#include "canparser.h"
#include "drive_helpers.h"
#include "carinterface.h"
#include "toyota/toyota_values.h"
#include "toyota/toyotacan.h"
#include "toyota/toyota_carstate.h"
#include "toyota/toyota_carcontroller.h"

// Steer torque limits
SteerLimitParams ToyotaSteerLimitParams = {
  .STEER_MAX = 1500.,
  .STEER_DELTA_UP = 10.,       // 1.5s time to peak torque
  .STEER_DELTA_DOWN = 25.,     // always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
  .STEER_ERROR_MAX = 350.     // max delta between torque cmd and torque motor
};

namespace toyota {

// Accel limits
static const double ACCEL_HYST_GAP = 0.02;  // don't change accel command for small oscilalitons within this value
static const double ACCEL_MAX = 1.5;  // 1.5 m/s2
static const double ACCEL_MIN = -3.0; // 3   m/s2
static const double ACCEL_SCALE = std::max(ACCEL_MAX, -ACCEL_MIN);

// Steer angle limits (tested at the Crows Landing track and considered ok)
static const std::vector<double> ANGLE_MAX_BP = {0., 5.};
static const std::vector<double> ANGLE_MAX_V = {510., 300.};
static const std::vector<double> ANGLE_DELTA_BP = {0., 5., 15.};
static const std::vector<double> ANGLE_DELTA_V = {5., .8, .15};     // windup limit
static const std::vector<double> ANGLE_DELTA_VU = {5., 3.5, 0.4};   // unwind limit

static const std::vector<uint32_t> TARGET_IDS = {0x340, 0x341, 0x342, 0x343, 0x344, 0x345,
                                          0x363, 0x364, 0x365, 0x370, 0x371, 0x372,
                                          0x373, 0x374, 0x375, 0x380, 0x381, 0x382,
                                          0x383};


static void accel_hysteresis(double accel, double accel_steady, bool enabled, double &out_accel, double &out_accel_steady) {
  // for small accel oscillations within ACCEL_HYST_GAP, don't change the accel command
  if(!enabled)
    // send 0 when disabled, otherwise acc faults
    accel_steady = 0.;
  else if(accel > accel_steady + ACCEL_HYST_GAP)
    accel_steady = accel - ACCEL_HYST_GAP;
  else if(accel < accel_steady - ACCEL_HYST_GAP)
    accel_steady = accel + ACCEL_HYST_GAP;
  accel = accel_steady;
  out_accel = accel;
  out_accel_steady = accel_steady;
}

static void process_hud_alert(cereal::CarControl::HUDControl::VisualAlert hud_alert, int &steer, int &fcw) {
  // initialize to no alert
  steer = 0;
  fcw = 0;
  if(hud_alert == cereal::CarControl::HUDControl::VisualAlert::FCW)
    fcw = 1;
  else if(hud_alert == cereal::CarControl::HUDControl::VisualAlert::STEER_REQUIRED)
    steer = 1;
}

static void ipas_state_transition(bool steer_angle_enabled, bool enabled, bool ipas_active, int ipas_reset_counter,
                            bool &out_steer_angle_enabled, int & out_ipas_reset_counter) {
  if (enabled and !steer_angle_enabled) {
    //ipas_reset_counter = max(0, ipas_reset_counter - 1)
    //if ipas_reset_counter == 0:
    //  steer_angle_enabled = true
    //else:
    //  steer_angle_enabled = false;
    //return steer_angle_enabled, ipas_reset_counter
    out_steer_angle_enabled = true;
    out_ipas_reset_counter = 0;
  } else if (enabled and steer_angle_enabled) {
    if (steer_angle_enabled and !ipas_active)
      ipas_reset_counter += 1;
    else
      ipas_reset_counter = 0;
    if (ipas_reset_counter > 10)  // try every 0.1s
      steer_angle_enabled = false;
    out_steer_angle_enabled = steer_angle_enabled;
    out_ipas_reset_counter = ipas_reset_counter;
  } else {
    out_steer_angle_enabled = false;
    out_ipas_reset_counter = 0;
  }
}

CarController::CarController(const char *dbc_name, const char *libdbc_fn, const char *car_fingerprint, bool enable_camera, bool enable_dsu, bool enable_apg):
  packer(dbc_name, libdbc_fn) {

  braking = false;
  // redundant safety check with the board
  controls_allowed = true;
  last_steer = 0;
  last_angle = 0;
  accel_steady = 0.;
  this->car_fingerprint = car_fingerprint;
  alert_active = false;
  last_standstill = false;
  standstill_req = false;
  angle_control = false;

  steer_angle_enabled = false;
  ipas_reset_counter = 0;
  last_fault_frame = -200;

  if(enable_camera) fake_ecus.insert(ECU::CAM);
  if(enable_dsu) fake_ecus.insert(ECU::DSU);
  if(enable_apg) fake_ecus.insert(ECU::APGS);

  // Avoid static msgs construction for every update call.
  for (auto &t: STATIC_MSGS) {
    uint32_t addr = std::get<0>(t);
    ECU ecu = std::get<1>(t);
    const std::vector<std::string> &cars(std::get<2>(t));
    uint16_t bus = std::get<3>(t);
    int64_t fr_step = std::get<4>(t);
    std::string vl = std::get<5>(t);
    if(fake_ecus.find(ecu) != fake_ecus.end() and \
      std::find(cars.begin(), cars.end(), this->car_fingerprint) != cars.end() ) {
      if(static_can_frames.find(fr_step) == static_can_frames.end())
        static_can_frames[fr_step] = std::vector<can_frame>();
      std::vector<uint8_t> vlv((uint8_t *)vl.c_str(), (uint8_t *)vl.c_str() + vl.size());
      static_can_frames[fr_step].push_back(make_can_msg(addr, vlv, bus, false));
    }
  }
}

void CarController::update(bool enabled, CarState &CS, int64_t frame, cereal::CarControl::Actuators::Builder actuators,
            bool pcm_cancel_cmd, cereal::CarControl::HUDControl::VisualAlert hud_alert, bool forwarding_camera,
            bool left_line, bool right_line, bool lead, bool left_lane_depart, bool right_lane_depart,
            std::vector<can_frame> &can_sends) {
  // *** compute control surfaces ***

  // gas and brake
  double apply_gas = clip<double>(actuators.getGas(), 0., 1.);
  double apply_accel;
  if (CS.CP.getEnableGasInterceptor())
    // send only negative accel if interceptor is detected. otherwise, send the regular value
    // +0.06 offset to reduce ABS pump usage when OP is engaged
    apply_accel = 0.06 - actuators.getBrake();
  else
    apply_accel = actuators.getGas() - actuators.getBrake();

  accel_hysteresis(apply_accel, accel_steady, enabled, apply_accel, accel_steady);
  apply_accel = clip(apply_accel * ACCEL_SCALE, ACCEL_MIN, ACCEL_MAX);

  // steer torque
  int apply_steer = std::round((double)actuators.getSteer() * ToyotaSteerLimitParams.STEER_MAX);

  apply_steer = apply_toyota_steer_torque_limits(apply_steer, last_steer, CS.steer_torque_motor, ToyotaSteerLimitParams);

  // only cut torque when steer state is a known fault
  if (CS.steer_state == 9 or CS.steer_state == 25)
    last_fault_frame = frame;

  // Cut steering for 2s after fault
  double apply_steer_req;
  if (!enabled or (frame - last_fault_frame < 200)) {
    apply_steer = 0;
    apply_steer_req = 0;
  } else
    apply_steer_req = 1;

  ipas_state_transition(steer_angle_enabled, enabled, CS.ipas_active, ipas_reset_counter,
                        steer_angle_enabled, ipas_reset_counter);
  //print("{0} {1} {2}".format(steer_angle_enabled, ipas_reset_counter, CS.ipas_active))

  // steer angle
  double apply_angle;
  if (steer_angle_enabled and CS.ipas_active) {
    apply_angle = actuators.getSteerAngle();
    double angle_lim = interp<double, std::vector<double>>(CS.v_ego, ANGLE_MAX_BP, ANGLE_MAX_V);
    apply_angle = clip(apply_angle, -angle_lim, angle_lim);

    // windup slower
    double angle_rate_lim;
    if (last_angle * apply_angle > 0. and std::abs(apply_angle) > std::abs(last_angle))
      angle_rate_lim = interp<double, std::vector<double>>(CS.v_ego, ANGLE_DELTA_BP, ANGLE_DELTA_V);
    else
      angle_rate_lim = interp<double, std::vector<double>>(CS.v_ego, ANGLE_DELTA_BP, ANGLE_DELTA_VU);

    apply_angle = clip(apply_angle, last_angle - angle_rate_lim, last_angle + angle_rate_lim);
  } else
    apply_angle = CS.angle_steers;

  if (!enabled and CS.pcm_acc_status)
    // send pcm acc cancel cmd if drive is disabled but pcm is still on, or if the system can't be activated
    pcm_cancel_cmd = 1;

  // on entering standstill, send standstill request
  if (CS.standstill and !last_standstill)
    standstill_req = true;
  if (CS.pcm_acc_status != 8)
    // pcm entered standstill or it's disabled
    standstill_req = false;

  last_steer = apply_steer;
  last_angle = apply_angle;
  last_accel = apply_accel;
  last_standstill = CS.standstill;

  //*** control msgs ***
  //print("steer {0} {1} {2} {3}".format(apply_steer, min_lim, max_lim, CS.steer_torque_motor)

  // toyota can trace shows this message at 42Hz, with counter adding alternatively 1 and 2;
  // sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
  // on consecutive messages
  if (fake_ecus.find(ECU::CAM) != fake_ecus.end()) {
    if (angle_control)
      can_sends.push_back(create_steer_command(packer, 0., 0, frame));
    else
      can_sends.push_back(create_steer_command(packer, apply_steer, apply_steer_req, frame));
  }
  if(angle_control)
    can_sends.push_back(create_ipas_steer_command(packer, apply_angle, steer_angle_enabled,
                                               fake_ecus.find(ECU::APGS) != fake_ecus.end()));
  else if (fake_ecus.find(ECU::APGS) != fake_ecus.end())
    can_sends.push_back(create_ipas_steer_command(packer, 0, 0, true));

  // accel cmd comes from DSU, but we can spam can to cancel the system even if we are using lat only control
  if (((frame % 3 == 0) and fake_ecus.find(ECU::DSU) != fake_ecus.end()) or \
      (pcm_cancel_cmd and (fake_ecus.find(ECU::CAM) != fake_ecus.end()))) {
    lead = lead or CS.v_ego < 12.;    // at low speed we always assume the lead is present do ACC can be engaged

    // Lexus IS uses a different cancellation message
    if (pcm_cancel_cmd and CS.CP.getCarFingerprint() == CAR_LEXUS_IS)
      can_sends.push_back(create_acc_cancel_command(packer));
    if (fake_ecus.find(ECU::DSU) != fake_ecus.end())
      can_sends.push_back(create_accel_command(packer, apply_accel, pcm_cancel_cmd, standstill_req, lead));
    else
      can_sends.push_back(create_accel_command(packer, 0, pcm_cancel_cmd, false, lead));
  }

  if ((frame % 2 == 0) and CS.CP.getEnableGasInterceptor())
      // send exactly zero if apply_gas is zero. Interceptor will send the max between read value and apply_gas.
      // This prevents unexpected pedal range rescaling
      can_sends.push_back(create_gas_command(packer, apply_gas, frame/2));

  // ui mesg is at 100Hz but we send asap if:
  // - there is something to display
  // - there is something to stop displaying
  int steer, fcw;
  process_hud_alert(hud_alert, steer, fcw);
  bool send_ui;
  if(((steer != 0 or fcw != 0) and !alert_active) or \
     (!(steer != 0 or fcw != 0) and alert_active)) {
    send_ui = true;
    alert_active = !alert_active;
  } else
    send_ui = false;

  // disengage msg causes a bad fault sound so play a good sound instead
  if (pcm_cancel_cmd)
    send_ui = true;

  if ((frame % 100 == 0 or send_ui) and fake_ecus.find(ECU::CAM) != fake_ecus.end())
    can_sends.push_back(create_ui_command(packer, steer, pcm_cancel_cmd, left_line, right_line, left_lane_depart, right_lane_depart));

  if (frame % 100 == 0 and fake_ecus.find(ECU::DSU) != fake_ecus.end() and \
    std::find(TSS2_CAR.begin(), TSS2_CAR.end(), car_fingerprint) == TSS2_CAR.end())
    can_sends.push_back(create_fcw_command(packer, fcw));

  //*** static msgs ***
  for (const auto &it: static_can_frames) {
    if(frame % it.first == 0) {
      can_sends.insert(can_sends.end(), it.second.begin(), it.second.end());
    }
  }
}

}