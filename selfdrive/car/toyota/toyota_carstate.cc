#include <map>
#include <vector>
#include <tuple>
#include <string>
#include <unordered_set>

#include "cereal/gen/cpp/car.capnp.h"
#include "can_define.h"
#include "canparser.h"
#include "KF1D.h"
#include "drive_helpers.h"
#include "toyota/toyota_values.h"
#include "toyota/toyota_carstate.h"

namespace toyota {

cereal::CarState::GearShifter parse_gear_shifter(int gear, std::map<int, std::string> &vals) {
  if(vals[gear] == "P")
    return cereal::CarState::GearShifter::PARK;
  else if(vals[gear] == "R")
    return cereal::CarState::GearShifter::REVERSE;
  else if(vals[gear] == "N")
    return cereal::CarState::GearShifter::NEUTRAL;
  else if(vals[gear] == "D")
    return cereal::CarState::GearShifter::DRIVE;
  else if(vals[gear] == "B")
    return cereal::CarState::GearShifter::BRAKE;
  else
    return cereal::CarState::GearShifter::UNKNOWN;
}

CanParser *get_can_parser(const char *libdbc_fn, const cereal::CarParams::Reader & CP) {
  signals_vector signals = {
    // sig_name, sig_address, default
    std::make_tuple("STEER_ANGLE", "STEER_ANGLE_SENSOR", 0),
    std::make_tuple("GEAR", "GEAR_PACKET", 0),
    std::make_tuple("BRAKE_PRESSED", "BRAKE_MODULE", 0),
    std::make_tuple("GAS_PEDAL", "GAS_PEDAL", 0),
    std::make_tuple("WHEEL_SPEED_FL", "WHEEL_SPEEDS", 0),
    std::make_tuple("WHEEL_SPEED_FR", "WHEEL_SPEEDS", 0),
    std::make_tuple("WHEEL_SPEED_RL", "WHEEL_SPEEDS", 0),
    std::make_tuple("WHEEL_SPEED_RR", "WHEEL_SPEEDS", 0),
    std::make_tuple("DOOR_OPEN_FL", "SEATS_DOORS", 1),
    std::make_tuple("DOOR_OPEN_FR", "SEATS_DOORS", 1),
    std::make_tuple("DOOR_OPEN_RL", "SEATS_DOORS", 1),
    std::make_tuple("DOOR_OPEN_RR", "SEATS_DOORS", 1),
    std::make_tuple("SEATBELT_DRIVER_UNLATCHED", "SEATS_DOORS", 1),
    std::make_tuple("TC_DISABLED", "ESP_CONTROL", 1),
    std::make_tuple("STEER_FRACTION", "STEER_ANGLE_SENSOR", 0),
    std::make_tuple("STEER_RATE", "STEER_ANGLE_SENSOR", 0),
    std::make_tuple("CRUISE_ACTIVE", "PCM_CRUISE", 0),
    std::make_tuple("CRUISE_STATE", "PCM_CRUISE", 0),
    std::make_tuple("MAIN_ON", "PCM_CRUISE_2", 0),
    std::make_tuple("SET_SPEED", "PCM_CRUISE_2", 0),
    std::make_tuple("LOW_SPEED_LOCKOUT", "PCM_CRUISE_2", 0),
    std::make_tuple("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR", 0),
    std::make_tuple("STEER_TORQUE_EPS", "STEER_TORQUE_SENSOR", 0),
    std::make_tuple("TURN_SIGNALS", "STEERING_LEVERS", 3),   // 3 is no blinkers
    std::make_tuple("LKA_STATE", "EPS_STATUS", 0),
    std::make_tuple("IPAS_STATE", "EPS_STATUS", 1),
    std::make_tuple("BRAKE_LIGHTS_ACC", "ESP_CONTROL", 0),
    std::make_tuple("AUTO_HIGH_BEAM", "LIGHT_STALK", 0),
  };

  std::vector<std::tuple<std::string, double>> checks = {
    std::make_tuple("BRAKE_MODULE", 40),
    std::make_tuple("GAS_PEDAL", 33),
    std::make_tuple("WHEEL_SPEEDS", 80),
    std::make_tuple("STEER_ANGLE_SENSOR", 80),
    std::make_tuple("PCM_CRUISE", 33),
    std::make_tuple("STEER_TORQUE_SENSOR", 50),
    std::make_tuple("EPS_STATUS", 25),
  };

  if(CP.getCarFingerprint() == CAR_LEXUS_IS) {
    signals.push_back(std::make_tuple("MAIN_ON", "DSU_CRUISE", 0));
    signals.push_back(std::make_tuple("SET_SPEED", "DSU_CRUISE", 0));
    checks.push_back(std::make_tuple("DSU_CRUISE", 5));
  } else {
    signals.push_back(std::make_tuple("MAIN_ON", "PCM_CRUISE_2", 0));
    signals.push_back(std::make_tuple("SET_SPEED", "PCM_CRUISE_2", 0));
    signals.push_back(std::make_tuple("LOW_SPEED_LOCKOUT", "PCM_CRUISE_2", 0));
    checks.push_back(std::make_tuple("PCM_CRUISE_2", 33));
  }

  if(std::find(NO_DSU_CAR.begin(), NO_DSU_CAR.end(), CP.getCarFingerprint().cStr()) != NO_DSU_CAR.end())
    signals.push_back(std::make_tuple("STEER_ANGLE", "STEER_TORQUE_SENSOR", 0));

  if(CP.getCarFingerprint() == CAR_PRIUS)
    signals.push_back(std::make_tuple("STATE", "AUTOPARK_STATUS", 0));

  // add gas interceptor reading if we are using it
  if(CP.getEnableGasInterceptor()) {
    signals.push_back(std::make_tuple("INTERCEPTOR_GAS", "GAS_SENSOR", 0));
    signals.push_back(std::make_tuple("INTERCEPTOR_GAS2", "GAS_SENSOR", 0));
    checks.push_back(std::make_tuple("GAS_SENSOR", 50));
  }
  return new CanParser(libdbc_fn, DBC[CP.getCarFingerprint()]["pt"], signals, checks, 0);
}

CanParser *get_cam_can_parser(const char *libdbc_fn, const cereal::CarParams::Reader & CP) {
  signals_vector signals = {};
  // use steering message to check if panda is connected to frc
  std::vector<std::tuple<std::string, double>> checks = {std::make_tuple("STEERING_LKA", 42)};
  return new CanParser(libdbc_fn, DBC[CP.getCarFingerprint()]["pt"], signals, checks, 2);
}

CarState::CarState(const cereal::CarParams::Reader &cp, const char *libdbc_fn): CP(cp) {
  CANDefine can_define(DBC[CP.getCarFingerprint()]["pt"].c_str(), libdbc_fn);
  shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"];
  left_blinker_on = 0;
  right_blinker_on = 0;
  angle_offset = 0.;
  init_angle_offset = false;

  // initialize can parser
  car_fingerprint = cp.getCarFingerprint();
  is_tss2_car = std::find(TSS2_CAR.begin(), TSS2_CAR.end(), cp.getCarFingerprint().cStr()) != TSS2_CAR.end();
  is_no_dsu_car = std::find(NO_DSU_CAR.begin(), NO_DSU_CAR.end(), cp.getCarFingerprint().cStr()) != NO_DSU_CAR.end();

  // vEgo kalman filter
  double dt = 0.01;
  v_ego_kf.init(0.0, 0.0,
                 1.0, dt, 0.0, 1.0,
                 1.0, 0.0,
                 0.12287673, 0.29666309);
  v_ego = 0.0;
}

void CarState::update(CanParser &cp) {
  // update prevs, update must run once per loop
  prev_left_blinker_on = left_blinker_on;
  prev_right_blinker_on = right_blinker_on;

  door_all_closed = (cp.vl["SEATS_DOORS"]["DOOR_OPEN_FL"] == 0 and cp.vl["SEATS_DOORS"]["DOOR_OPEN_FR"] == 0 and \
                    cp.vl["SEATS_DOORS"]["DOOR_OPEN_RL"] == 0 and cp.vl["SEATS_DOORS"]["DOOR_OPEN_RR"] == 0);
  seatbelt = (cp.vl["SEATS_DOORS"]["SEATBELT_DRIVER_UNLATCHED"] == 0);

  brake_pressed = cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"];
  if(CP.getEnableGasInterceptor())
    pedal_gas = (cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) / 2.;
  else
    pedal_gas = cp.vl["GAS_PEDAL"]["GAS_PEDAL"];
  car_gas = pedal_gas;
  esp_disabled = cp.vl["ESP_CONTROL"]["TC_DISABLED"];

  // calc best v_ego estimate, by averaging two opposite corners
  v_wheel_fl = cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"] * KPH_TO_MS;
  v_wheel_fr = cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"] * KPH_TO_MS;
  v_wheel_rl = cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"] * KPH_TO_MS;
  v_wheel_rr = cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"] * KPH_TO_MS;
  double v_wheel = (v_wheel_fl + v_wheel_fr + v_wheel_rl + v_wheel_rr) / 4.;

  // Kalman filter
  if(std::abs(v_wheel - v_ego) > 2.0) { // Prevent large accelerations when car starts at non zero speed
    v_ego_kf.x0_0 = v_wheel;
    v_ego_kf.x1_0 = 0.0;
  }

  v_ego_raw = v_wheel;
  v_ego_kf.update(v_wheel);
  v_ego_x_0 = v_ego_kf.x0_0;
  v_ego_x_1 = v_ego_kf.x1_0;
  v_ego = v_ego_x_0;
  a_ego = v_ego_x_1;
  standstill = !(v_wheel > 0.001);

  if(is_tss2_car)
    angle_steers = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"];
  else if(is_no_dsu_car) {
    // cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"] is zeroed to where the steering angle is at start.
    // need to apply an offset as soon as the steering angle measurements are both received
    angle_steers = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"] - angle_offset;
    double angle_wheel = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"];
    if(std::abs(angle_wheel) > 1e-3 and std::abs(angle_steers) > 1e-3 and not init_angle_offset) {
      init_angle_offset = true;
      angle_offset = angle_steers - angle_wheel;
    }
  } else
    angle_steers = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"];
  angle_steers_rate = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"];
  can_gear = (int)cp.vl["GEAR_PACKET"]["GEAR"];
  gear_shifter = parse_gear_shifter(can_gear, shifter_values);
  if(CP.getCarFingerprint() == CAR_LEXUS_IS)
    main_on = cp.vl["DSU_CRUISE"]["MAIN_ON"];
  else
    main_on = cp.vl["PCM_CRUISE_2"]["MAIN_ON"];
  left_blinker_on = cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"] == 1;
  right_blinker_on = cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"] == 2;

  // 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
  steer_state = cp.vl["EPS_STATUS"]["LKA_STATE"];
  steer_error = cp.vl["EPS_STATUS"]["LKA_STATE"] != 1 and cp.vl["EPS_STATUS"]["LKA_STATE"] != 5;
  ipas_active = cp.vl["EPS_STATUS"]["IPAS_STATE"] == 3;
  brake_error = 0;
  steer_torque_driver = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"];
  steer_torque_motor = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"];
  // we could use the override bit from dbc, but it"s triggered at too high torque values
  steer_override = std::abs(steer_torque_driver) > STEER_THRESHOLD;

  user_brake = 0;
  if(CP.getCarFingerprint() == CAR_LEXUS_IS) {
    v_cruise_pcm = cp.vl["DSU_CRUISE"]["SET_SPEED"];
    low_speed_lockout = false;
  } else {
    v_cruise_pcm = cp.vl["PCM_CRUISE_2"]["SET_SPEED"];
    low_speed_lockout = cp.vl["PCM_CRUISE_2"]["LOW_SPEED_LOCKOUT"] == 2;
  }
  pcm_acc_status = cp.vl["PCM_CRUISE"]["CRUISE_STATE"];
  pcm_acc_active = cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"] != 0;
  brake_lights = cp.vl["ESP_CONTROL"]["BRAKE_LIGHTS_ACC"] != 0 or brake_pressed;
  if(CP.getCarFingerprint() == CAR_PRIUS)
    generic_toggle = cp.vl["AUTOPARK_STATUS"]["STATE"] != 0;
  else
    generic_toggle = cp.vl["LIGHT_STALK"]["AUTO_HIGH_BEAM"] != 0;
}

}