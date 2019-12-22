#include <map>
#include <vector>
#include <tuple>
#include <string>
#include <unordered_set>

#include "cereal/gen/cpp/car.capnp.h"
#include "can_define.h"
#include "canparser.h"
#include "canpacker.h"
#include "KF1D.h"
#include "drive_helpers.h"
#include "vehicle_model.h"
#include "carinterface.h"
#include "toyota/toyota_values.h"
#include "toyota/toyota_carstate.h"
#include "toyota/toyota_carcontroller.h"
#include "toyota/toyota_interface.h"

namespace toyota {

ToyotaCarInterface::ToyotaCarInterface(cereal::CarParams::Reader &car_param, const char *libdbc_fn):
  CP(car_param), VM(car_param), frame(0),
  gas_pressed_prev(false),
  brake_pressed_prev(false),
  cruise_enabled_prev(false),
  forwarding_camera(false),
  CS(car_param, libdbc_fn),
  cp(get_can_parser(libdbc_fn, car_param)),
  cp_cam(get_cam_can_parser(libdbc_fn, car_param))
{
  CC.reset(new CarController(cp->dbc_name.c_str(), libdbc_fn, car_param.getCarFingerprint().cStr(),
                            car_param.getEnableCamera(), car_param.getEnableDsu(), car_param.getEnableApgs()));
  no_stop_timer = std::find(NO_STOP_TIMER_CAR.begin(), NO_STOP_TIMER_CAR.end(), car_param.getCarFingerprint().cStr()) != NO_STOP_TIMER_CAR.end();
}

double ToyotaCarInterface::compute_gb(double accel, double speed) {
  return accel / 3.0;
}

double ToyotaCarInterface::calc_accel_override(double a_ego, double a_target, double v_ego, double v_target) {
  return 1.0;
}
// returns a car.CarState
void ToyotaCarInterface::update(cereal::CarControl::Builder &c,
            std::vector<std::string> &can_strings,
            cereal::CarState::Builder &ret,
            std::vector<event_name_types_pair> &events) {
  // ******************* do can recv *******************
  cp->update_strings(can_strings);
  cp_cam->update_strings(can_strings);

  CS.update(*cp);

  ret.setCanValid(cp->can_valid);

  // speeds
  ret.setVEgo(CS.v_ego);
  ret.setVEgoRaw(CS.v_ego_raw);
  ret.setAEgo(CS.a_ego);
  ret.setYawRate(VM.yaw_rate(CS.angle_steers * DEG_TO_RAD, CS.v_ego));
  ret.setStandstill(CS.standstill);
  ret.getWheelSpeeds().setFl(CS.v_wheel_fl);
  ret.getWheelSpeeds().setFr(CS.v_wheel_fr);
  ret.getWheelSpeeds().setRl(CS.v_wheel_rl);
  ret.getWheelSpeeds().setRr(CS.v_wheel_rr);

  // gear shifter
  ret.setGearShifter(CS.gear_shifter);

  // gas pedal
  ret.setGas(CS.car_gas);
  if(CP.getEnableGasInterceptor())
    // use interceptor values to disengage on pedal press
    ret.setGasPressed(CS.pedal_gas > 15);
  else
    ret.setGasPressed(CS.pedal_gas > 0);

  // brake pedal
  ret.setBrake(CS.user_brake);
  ret.setBrakePressed(CS.brake_pressed != 0);
  ret.setBrakeLights(CS.brake_lights);

  // steering wheel
  ret.setSteeringAngle(CS.angle_steers);
  ret.setSteeringRate(CS.angle_steers_rate);

  ret.setSteeringTorque(CS.steer_torque_driver);
  ret.setSteeringTorqueEps(CS.steer_torque_motor);
  ret.setSteeringPressed(CS.steer_override);

  // cruise state
  ret.getCruiseState().setEnabled(CS.pcm_acc_active);
  ret.getCruiseState().setSpeed(CS.v_cruise_pcm * KPH_TO_MS);
  ret.getCruiseState().setAvailable(CS.main_on);
  ret.getCruiseState().setSpeedOffset(0.f);

  if(no_stop_timer or CP.getEnableGasInterceptor())
    // ignore standstill in hybrid vehicles, since pcm allows to restart without
    // receiving any special command
    // also if interceptor is detected
    ret.getCruiseState().setStandstill(false);
  else
    ret.getCruiseState().setStandstill(CS.pcm_acc_status == 7);

  std::vector<std::tuple<cereal::CarState::ButtonEvent::Type, bool>> buttonEvents;
  if (CS.left_blinker_on != CS.prev_left_blinker_on)
    buttonEvents.push_back(std::make_tuple(cereal::CarState::ButtonEvent::Type::LEFT_BLINKER, CS.left_blinker_on != 0));

  if(CS.right_blinker_on != CS.prev_right_blinker_on)
    buttonEvents.push_back(std::make_tuple(cereal::CarState::ButtonEvent::Type::RIGHT_BLINKER, CS.right_blinker_on != 0));

  ret.initButtonEvents(buttonEvents.size());
  for(int i =0; i < buttonEvents.size(); i++) {
    ret.getButtonEvents()[i].setType(std::get<0>(buttonEvents[i]));
    ret.getButtonEvents()[i].setPressed(std::get<1>(buttonEvents[i]));
  }
  ret.setLeftBlinker(CS.left_blinker_on);
  ret.setRightBlinker(CS.right_blinker_on);

  ret.setDoorOpen(!CS.door_all_closed);
  ret.setSeatbeltUnlatched(!CS.seatbelt);

  ret.setGenericToggle(CS.generic_toggle);

  if(cp_cam->can_valid)
    forwarding_camera = true;

  if(cp_cam->can_invalid_cnt >= 100 and CP.getEnableCamera())
    events.push_back(create_event(cereal::CarEvent::EventName::INVALID_GIRAFFE_TOYOTA, EventType::PERMANENT));
  if(ret.getGearShifter() != cereal::CarState::GearShifter::DRIVE and CP.getEnableDsu())
    events.push_back(create_event(cereal::CarEvent::EventName::WRONG_GEAR, EventType::NO_ENTRY, EventType::SOFT_DISABLE));
  if(ret.getDoorOpen())
    events.push_back(create_event(cereal::CarEvent::EventName::DOOR_OPEN, EventType::NO_ENTRY, EventType::SOFT_DISABLE));
  if(ret.getSeatbeltUnlatched())
    events.push_back(create_event(cereal::CarEvent::EventName::SEATBELT_NOT_LATCHED, EventType::NO_ENTRY, EventType::SOFT_DISABLE));
  if(CS.esp_disabled and CP.getEnableDsu())
    events.push_back(create_event(cereal::CarEvent::EventName::ESP_DISABLED, EventType::NO_ENTRY, EventType::SOFT_DISABLE));
  if(!CS.main_on and CP.getEnableDsu())
    events.push_back(create_event(cereal::CarEvent::EventName::WRONG_CAR_MODE, EventType::NO_ENTRY, EventType::USER_DISABLE));
  if(ret.getGearShifter() == cereal::CarState::GearShifter::REVERSE and CP.getEnableDsu())
    events.push_back(create_event(cereal::CarEvent::EventName::REVERSE_GEAR, EventType::NO_ENTRY, EventType::IMMEDIATE_DISABLE));
  if(CS.steer_error)
    events.push_back(create_event(cereal::CarEvent::EventName::STEER_TEMP_UNAVAILABLE, EventType::NO_ENTRY, EventType::WARNING));
  if(CS.low_speed_lockout and CP.getEnableDsu())
    events.push_back(create_event(cereal::CarEvent::EventName::LOW_SPEED_LOCKOUT, EventType::NO_ENTRY, EventType::PERMANENT));
  if(ret.getVEgo() < CP.getMinEnableSpeed() and CP.getEnableDsu()) {
    events.push_back(create_event(cereal::CarEvent::EventName::SPEED_TOO_LOW, EventType::NO_ENTRY));
    if(c.getActuators().getGas() > 0.1)
      // some margin on the actuator to not false trigger cancellation while stopping
      events.push_back(create_event(cereal::CarEvent::EventName::SPEED_TOO_LOW, EventType::IMMEDIATE_DISABLE));
    if(ret.getVEgo() < 0.001)
      // while in standstill, send a user alert
      events.push_back(create_event(cereal::CarEvent::EventName::MANUAL_RESTART, EventType::WARNING));
  }
  // enable request in prius is simple, as we activate when Toyota is active (rising edge)
  if(ret.getCruiseState().getEnabled() and !cruise_enabled_prev)
    events.push_back(create_event(cereal::CarEvent::EventName::PCM_ENABLE, EventType::ENABLE));
  else if(!ret.getCruiseState().getEnabled())
    events.push_back(create_event(cereal::CarEvent::EventName::PCM_DISABLE, EventType::USER_DISABLE));

  // disable on pedals rising edge or when brake is pressed and speed isn't zero
  if((ret.getGasPressed() and !gas_pressed_prev) or (ret.getBrakePressed() and (!brake_pressed_prev or ret.getVEgo() > 0.001)))
    events.push_back(create_event(cereal::CarEvent::EventName::PEDAL_PRESSED, EventType::NO_ENTRY, EventType::USER_DISABLE));

  if(ret.getGasPressed())
    events.push_back(create_event(cereal::CarEvent::EventName::PEDAL_PRESSED, EventType::PRE_ENABLE));

  gas_pressed_prev = ret.getGasPressed();
  brake_pressed_prev = ret.getBrakePressed();
  cruise_enabled_prev = ret.getCruiseState().getEnabled();
}

// pass in a car.CarControl
// to be called @ 100hz
void ToyotaCarInterface::apply(cereal::CarControl::Builder &c, std::vector<can_frame> &can_sends) {
  CC->update(c.getEnabled(), CS, frame,
                         c.getActuators(), c.getCruiseControl().getCancel(), c.getHudControl().getVisualAlert(),
                         forwarding_camera, c.getHudControl().getLeftLaneVisible(),
                         c.getHudControl().getRightLaneVisible(), c.getHudControl().getLeadVisible(),
                         c.getHudControl().getLeftLaneDepart(), c.getHudControl().getRightLaneDepart(),
                         can_sends);

  frame += 1;
}

}