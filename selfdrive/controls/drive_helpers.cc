#include "cereal/gen/cpp/car.capnp.h"
#include "cereal/gen/cpp/log.capnp.h"
#include "drive_helpers.h"

// kph
const double V_CRUISE_MAX = 144;
const double V_CRUISE_MIN = 8;
const int V_CRUISE_DELTA = 8;
const double V_CRUISE_ENABLE_MIN = 40;
const double MPH_TO_KPH = 1.609344;
const double KPH_TO_MPH = 1. / MPH_TO_KPH;
const double MS_TO_KPH = 3.6;
const double KPH_TO_MS = 1. / MS_TO_KPH;
const double MS_TO_MPH = MS_TO_KPH * KPH_TO_MPH;
const double LON_MPC_STEP = 0.2;  // first step is 0.2s
const double CAMERA_OFFSET = 0.06;  // m from center car to camera
const double DEG_TO_RAD = M_PI/180.;

event_name_types_pair create_event(
            cereal::CarEvent::EventName name, EventType type1, EventType type2, EventType type3) {
  std::vector<EventType> ets;
  if(type1 != NULL_EVENT_TYPE)
    ets.push_back(type1);
  if(type2 != NULL_EVENT_TYPE)
    ets.push_back(type2);
  if(type3 != NULL_EVENT_TYPE)
    ets.push_back(type3);
  return std::make_pair(name, ets);
}

std::vector<cereal::CarEvent::EventName> get_events(const std::vector<event_name_types_pair> &events, EventType type1, EventType type2) {
  std::vector<cereal::CarEvent::EventName> out;
  for (auto& e : events) {
    if(type1 != NULL_EVENT_TYPE and std::find(e.second.begin(), e.second.end(), type1) != e.second.end())
      out.push_back(e.first);
    if(type2 != NULL_EVENT_TYPE and std::find(e.second.begin(), e.second.end(), type2) != e.second.end())
      out.push_back(e.first);
  }
  return out;
}

void set_cereal_event_type(cereal::CarEvent::Builder ce, const std::vector<EventType> &etv) {
  for(auto t: etv) {
    switch(t) {
    case ENABLE:
      ce.setEnable(true);
      break;
    case PRE_ENABLE:
      ce.setPreEnable(true);
      break;
    case NO_ENTRY:
      ce.setNoEntry(true);
      break;
    case WARNING:
      ce.setWarning(true);
      break;
    case USER_DISABLE:
      ce.setUserDisable(true);
      break;
    case SOFT_DISABLE:
      ce.setSoftDisable(true);
      break;
    case IMMEDIATE_DISABLE:
      ce.setImmediateDisable(true);
      break;
    case PERMANENT:
      ce.setPermanent(true);
      break;
    default:
      break;
    }
  }
}

std::string event_name_str(cereal::CarEvent::EventName en) {
  switch(en) {
    case cereal::CarEvent::EventName::CAN_ERROR: return "canError";
    case cereal::CarEvent::EventName::STEER_UNAVAILABLE: return "steerUnavailable";
    case cereal::CarEvent::EventName::BRAKE_UNAVAILABLE: return "brakeUnavailable";
    case cereal::CarEvent::EventName::GAS_UNAVAILABLE: return "gasUnavailable";
    case cereal::CarEvent::EventName::WRONG_GEAR: return "wrongGear";
    case cereal::CarEvent::EventName::DOOR_OPEN: return "DoorOpen";
    case cereal::CarEvent::EventName::SEATBELT_NOT_LATCHED: return "seatbeltNotLatched";
    case cereal::CarEvent::EventName::ESP_DISABLED: return "espDisabled";
    case cereal::CarEvent::EventName::WRONG_CAR_MODE: return "wrongCarMode";
    case cereal::CarEvent::EventName::STEER_TEMP_UNAVAILABLE: return "steerTempUnavailable";
    case cereal::CarEvent::EventName::REVERSE_GEAR: return "reverseGear";
    case cereal::CarEvent::EventName::BUTTON_CANCEL: return "buttonCancel";
    case cereal::CarEvent::EventName::BUTTON_ENABLE: return "buttonEnable";
    case cereal::CarEvent::EventName::PEDAL_PRESSED: return "pedalPressed";
    case cereal::CarEvent::EventName::CRUISE_DISABLED: return "cruiseDisable";
    case cereal::CarEvent::EventName::RADAR_CAN_ERROR: return "radarCanError";
    case cereal::CarEvent::EventName::DATA_NEEDED: return "dataNeeded";
    case cereal::CarEvent::EventName::SPEED_TOO_LOW: return "speedTooLow";
    case cereal::CarEvent::EventName::OUT_OF_SPACE: return "outOfSpace";
    case cereal::CarEvent::EventName::OVERHEAT: return "overheat";
    case cereal::CarEvent::EventName::CALIBRATION_INCOMPLETE: return "calibrationIncomplete";
    case cereal::CarEvent::EventName::CALIBRATION_INVALID: return "calibrationInvalid";
    case cereal::CarEvent::EventName::CONTROLS_MISMATCH: return "controlsMismatch";
    case cereal::CarEvent::EventName::PCM_ENABLE: return "pcmEnable";
    case cereal::CarEvent::EventName::PCM_DISABLE: return "pcmDisable";
    case cereal::CarEvent::EventName::NO_TARGET: return "noTarget";
    case cereal::CarEvent::EventName::RADAR_FAULT: return "radarFault";
    case cereal::CarEvent::EventName::MODEL_COMM_ISSUE_D_E_P_R_E_C_A_T_E_D: return "modelCommIssue";
    case cereal::CarEvent::EventName::BRAKE_HOLD: return "brakeHold";
    case cereal::CarEvent::EventName::PARK_BRAKE: return "parkHold";
    case cereal::CarEvent::EventName::MANUAL_RESTART: return "manualRestart";
    case cereal::CarEvent::EventName::LOW_SPEED_LOCKOUT: return "lowSpeedLockout";
    case cereal::CarEvent::EventName::PLANNER_ERROR: return "plannerError";
    case cereal::CarEvent::EventName::IPAS_OVERRIDE: return "ipasOverride";
    case cereal::CarEvent::EventName::DEBUG_ALERT: return "debugAlert";
    case cereal::CarEvent::EventName::STEER_TEMP_UNAVAILABLE_MUTE: return "steerTempUnavailableMute";
    case cereal::CarEvent::EventName::RESUME_REQUIRED: return "resumeRequired";
    case cereal::CarEvent::EventName::PRE_DRIVER_DISTRACTED: return "preDriverDistracted";
    case cereal::CarEvent::EventName::PROMPT_DRIVER_DISTRACTED: return "promptDriverDistracted";
    case cereal::CarEvent::EventName::DRIVER_DISTRACTED: return "driverDistracted";
    case cereal::CarEvent::EventName::GEOFENCE: return "geofence";
    case cereal::CarEvent::EventName::DRIVER_MONITOR_ON: return "driverMonitorOn";
    case cereal::CarEvent::EventName::DRIVER_MONITOR_OFF: return "driverMonitorOff";
    case cereal::CarEvent::EventName::PRE_DRIVER_UNRESPONSIVE: return "preDriverUnresponsive";
    case cereal::CarEvent::EventName::PROMPT_DRIVER_UNRESPONSIVE: return "promptDriverUnresponsive";
    case cereal::CarEvent::EventName::DRIVER_UNRESPONSIVE: return "driverUnresponsive";
    case cereal::CarEvent::EventName::BELOW_STEER_SPEED: return "belowSteerSpeed";
    case cereal::CarEvent::EventName::CALIBRATION_PROGRESS: return "calibrationProgress";
    case cereal::CarEvent::EventName::LOW_BATTERY: return "lowBattery";
    case cereal::CarEvent::EventName::INVALID_GIRAFFE_HONDA: return "invalidGiraffeHonda";
    case cereal::CarEvent::EventName::VEHICLE_MODEL_INVALID: return "vehicleModelInvalid";
    case cereal::CarEvent::EventName::CONTROLS_FAILED: return "controlsFailed";
    case cereal::CarEvent::EventName::SENSOR_DATA_INVALID: return "sensorDataInvalid";
    case cereal::CarEvent::EventName::COMM_ISSUE: return "commIssue";
    case cereal::CarEvent::EventName::TOO_DISTRACTED: return "tooDistracted";
    case cereal::CarEvent::EventName::POSENET_INVALID: return "posenetInvalid";
    case cereal::CarEvent::EventName::SOUNDS_UNAVAILABLE: return "soundsUnavailable";
    default:
      assert(0);
      return "";
  }
}

template<typename T>
T interp(T xv, const T *xp, const T *fp, int N) {
  int hi = 0;
  while(hi < N and xv > xp[hi])
    hi += 1;
  int low = hi - 1;
  if(low == -1)
    low = 1;
  if(hi == N and xv > xp[low])
    return fp[N-1];
  else if(hi == 0)
    return fp[0];
  else
    return (xv - xp[low]) * (fp[hi] - fp[low]) / (xp[hi] - xp[low]) + fp[low];
}

double sign(double x) {
  if(x > 0)
    return 1.;
  else if(x < 0)
    return -1.;
  else
    return 0.;
}

double initialize_v_cruise(double v_ego, const capnp::List< ::cereal::CarState::ButtonEvent>::Reader &buttonEvents, int v_cruise_last) {
  int ev_count = buttonEvents.size();
  for (int i = 0; i < ev_count; i++) {
    auto b = buttonEvents[i];
    // 250kph or above probably means we never had a set speed
    if(b.getType() == cereal::CarState::ButtonEvent::Type::ACCEL_CRUISE and v_cruise_last < 250)
      return v_cruise_last;
  }
  return std::round(clip((double)(v_ego * MS_TO_KPH), V_CRUISE_ENABLE_MIN, V_CRUISE_MAX));
}

double update_v_cruise(double v_cruise_kph, const capnp::List< ::cereal::CarState::ButtonEvent>::Reader &buttonEvents, bool enabled) {
  // handle button presses. TODO: this should be in state_control, but a decelCruise press
  // would have the effect of both enabling and changing speed is checked after the state transition
  int ev_count = buttonEvents.size();
  for (int i = 0; i < ev_count; i++) {
    auto b = buttonEvents[i];
    if(enabled and !b.getPressed()) {
      if(b.getType() == cereal::CarState::ButtonEvent::Type::ACCEL_CRUISE)
        v_cruise_kph += V_CRUISE_DELTA - ((int)v_cruise_kph % V_CRUISE_DELTA);
      else if(b.getType() == cereal::CarState::ButtonEvent::Type::ACCEL_CRUISE)
        v_cruise_kph -= V_CRUISE_DELTA - ((V_CRUISE_DELTA - (int)v_cruise_kph) % V_CRUISE_DELTA);
      v_cruise_kph = clip(v_cruise_kph, V_CRUISE_MIN, V_CRUISE_MAX);
    }
  }
  return v_cruise_kph;
}

double get_steer_max(const cereal::CarParams::Reader &CP, double v_ego) {
  return interp(v_ego, CP.getSteerMaxBP(), CP.getSteerMaxV());
}
