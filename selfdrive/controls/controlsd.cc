#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <assert.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <string>
#include <vector>
#include <map>
#include <tuple>
#include <unordered_set>
#include <algorithm>
#include <memory>
#include <zmq.h>

#include "common/timing.h"
#include "common/util.h"
#include "common/swaglog.h"

#include "common/params.h"

#include <capnp/serialize.h>
#include "cereal/gen/cpp/car.capnp.h"
#include "cereal/gen/cpp/log.capnp.h"

#include "messaging.h"
#include "alertmanager.h"
#include "drive_helpers.h"
#include "gps_helpers.h"
#include "vehicle_model.h"
#include "longcontrol.h"
#include "can_define.h"
#include "canparser.h"
#include "canpacker.h"
#include "carinterface.h"
#include "latcontrol_base.h"
#include "latcontrol_lqr.h"
#include "latcontrol_pid.h"
#include "filter_simple.h"
#include "stat_live.h"
#include "driver_monitor.h"
#include "KF1D.h"
// Toyota
#include "toyota/toyotacan.h"
#include "toyota/toyota_values.h"
#include "toyota/toyota_carstate.h"
#include "toyota/toyota_carcontroller.h"
#include "toyota/toyota_interface.h"

#include "controlsd.h"

const int MIN_SPEED = 7; // m/s  (~15.5mph)

ControlsdState controlsd_state = {
  .CI = nullptr,
  .ctx = nullptr,
  .sm = nullptr,
  .pm = nullptr,
  .can_sock_raw = nullptr,
};

volatile sig_atomic_t do_exit = 0;
static void set_do_exit(int sig) {
  do_exit = 1;
}

static bool isActive(cereal::ControlsState::OpenpilotState state) {
  // Check if the actuators are enabled
  return (state == cereal::ControlsState::OpenpilotState::ENABLED or state == cereal::ControlsState::OpenpilotState::SOFT_DISABLING);
}

static bool isEnabled(cereal::ControlsState::OpenpilotState state) {
  // Check if openpilot is engaged
  return (isActive(state) or state == cereal::ControlsState::OpenpilotState::PRE_ENABLED);
}

static void read_param_bool(bool* param, const char* param_name) {
  char *s;
  const int result = read_db_value(NULL, param_name, &s, NULL);
  if (result == 0) {
    *param = s[0] == '1';
    free(s);
  }
}

void *write_rhd_thread( void *dat ){
  bool is_rhd = (bool) dat;
  std::string s_is_rhd = std::to_string(is_rhd ? 1 : 0);
  write_db_value(NULL, "IsRHD", s_is_rhd.c_str(), s_is_rhd.length());
  return NULL;
}

void put_rhd_nonblocking(bool is_rhd) {
  pthread_t thd;
  int err = pthread_create( &thd, NULL, write_rhd_thread, (void*) is_rhd);
  assert(err == 0);
}

static void init_sub_sock(ControlsdState *s, SubSockIndex si, double freq, const char *addr) {
  capnp::MallocMessageBuilder msg;
  auto event = msg.initRoot<cereal::Event>();
  event.setLogMonoTime(nanos_since_boot());
  bool b_ignore_alive = false;
  if(si == SUB_SOCK_HEALTH)
    event.initHealth();
  else if(si == SUB_SOCK_THERMAL)
    event.initThermal();
  else if(si == SUB_SOCK_LIVECALIBRATION)
    event.initLiveCalibration();
  else if(si == SUB_SOCK_DRIVERMONITORING)
    event.initDriverMonitoring();
  else if(si == SUB_SOCK_PLAN)
    event.initPlan();
  else if(si == SUB_SOCK_PATHPLAN) {
    auto path_plan = event.initPathPlan();
    path_plan.setSensorValid(true);
    path_plan.setPosenetValid(true);
  } else if(si == SUB_SOCK_GPSLOCATION) {
    event.initGpsLocation();
    b_ignore_alive = true;
  }
  s->sm->add_sock(s->ctx, addr, freq, event.getValid(), b_ignore_alive, msg);
}

static void data_sample(ControlsdState *s, cereal::CarState::Builder &CS, cereal::CarControl::Builder &CC, int8_t & cal_status, int8_t & cal_perc, bool & overtemp,
                bool & free_space, bool & low_battery, DriverStatus &driver_status, cereal::ControlsState::OpenpilotState state,
                int & mismatch_counter, std::vector<event_name_types_pair> &events) {
  // Receive data from sockets and create events for battery, temperature and disk space

  // Update carstate from CAN and create events
  std::vector<std::string> can_strs;
  s->can_sock_raw->receive(can_strs);

  s->sm->update();

  events.clear();
  s->CI->update(CC, can_strs, CS, events);
  bool enabled = isEnabled(state);

  // Check for CAN timeout
  if(can_strs.size() <= 0)
    events.push_back(create_event(cereal::CarEvent::EventName::CAN_ERROR, EventType::NO_ENTRY, EventType::IMMEDIATE_DISABLE));

  if(s->sm->updated[SUB_SOCK_THERMAL]) {
    capnp::FlatArrayMessageReader thermal_reader(s->sm->msg_bufs[SUB_SOCK_THERMAL]);
    auto thermal_root = thermal_reader.getRoot<cereal::Event>();
    auto thermal = thermal_root.getThermal();
    overtemp = (thermal.getThermalStatus() >= cereal::ThermalData::ThermalStatus::RED);
    free_space = (thermal.getFreeSpace() < 0.07);  // under 7% of space free no enable allowed
    low_battery = (thermal.getBatteryPercent()) < 1 and thermal.getChargingError();  // at zero percent battery, while discharging, OP should not allowed
  }

  // Create events for battery, temperature and disk space
  if(low_battery)
    events.push_back(create_event(cereal::CarEvent::EventName::LOW_BATTERY, EventType::NO_ENTRY, EventType::SOFT_DISABLE));
  if(overtemp)
    events.push_back(create_event(cereal::CarEvent::EventName::OVERHEAT, EventType::NO_ENTRY, EventType::SOFT_DISABLE));
  if(free_space)
    events.push_back(create_event(cereal::CarEvent::EventName::OUT_OF_SPACE, EventType::NO_ENTRY));

  // GPS coords RHD parsing, once every restart
  if (s->sm->updated[SUB_SOCK_GPSLOCATION] and !driver_status.is_rhd_region_checked) {
    capnp::FlatArrayMessageReader gps_reader(s->sm->msg_bufs[SUB_SOCK_GPSLOCATION]);
    auto gps_root = gps_reader.getRoot<cereal::Event>();
    auto gps = gps_root.getGpsLocation();
    bool is_rhd = is_rhd_region(gps.getLatitude(), gps.getLongitude());
    driver_status.is_rhd_region = is_rhd;
    driver_status.is_rhd_region_checked = true;
    put_rhd_nonblocking(is_rhd);
  }

  // Handle calibration
  if(s->sm->updated[SUB_SOCK_LIVECALIBRATION]) {
    capnp::FlatArrayMessageReader live_cali_reader(s->sm->msg_bufs[SUB_SOCK_LIVECALIBRATION]);
    auto live_cali_root = live_cali_reader.getRoot<cereal::Event>();
    auto live_cali = live_cali_root.getLiveCalibration();
    cal_status = live_cali.getCalStatus();
    cal_perc = live_cali.getCalPerc();
  }

  double cal_rpy[3] = {0,0,0};
  if(cal_status != CalibrationStatus::CALIBRATED) {
    if(cal_status == CalibrationStatus::UNCALIBRATED)
      events.push_back(create_event(cereal::CarEvent::EventName::CALIBRATION_INCOMPLETE, EventType::NO_ENTRY, EventType::SOFT_DISABLE, EventType::PERMANENT));
    else
      events.push_back(create_event(cereal::CarEvent::EventName::CALIBRATION_INVALID, EventType::NO_ENTRY, EventType::SOFT_DISABLE));
  } else {
    capnp::FlatArrayMessageReader live_cali_reader(s->sm->msg_bufs[SUB_SOCK_LIVECALIBRATION]);
    auto live_cali_root = live_cali_reader.getRoot<cereal::Event>();
    auto live_cali = live_cali_root.getLiveCalibration();
    if(live_cali.hasRpyCalib()) {
      auto rpy = live_cali.getRpyCalib();
      if(rpy.size() == 3) {
        cal_rpy[0] = rpy[0]; cal_rpy[1] = rpy[1]; cal_rpy[2] = rpy[2];
      }
    }
  }

  // When the panda and controlsd do not agree on controls_allowed
  // we want to disengage openpilot. However the status from the panda goes through
  // another socket other than the CAN messages and one can arrive earlier than the other.
  // Therefore we allow a mismatch for two samples, then we trigger the disengagement.
  if(!enabled)
    mismatch_counter = 0;

  if(s->sm->updated[SUB_SOCK_HEALTH]) {
    capnp::FlatArrayMessageReader health_reader(s->sm->msg_bufs[SUB_SOCK_HEALTH]);
    auto health_root = health_reader.getRoot<cereal::Event>();
    auto health = health_root.getHealth();
    bool controls_allowed = health.getControlsAllowed();
    if(!controls_allowed and enabled)
      mismatch_counter += 1;
    if(mismatch_counter >= 2)
      events.push_back(create_event(cereal::CarEvent::EventName::CONTROLS_MISMATCH, EventType::IMMEDIATE_DISABLE));
  }

  if (s->sm->updated[SUB_SOCK_DRIVERMONITORING]) {
    capnp::FlatArrayMessageReader dm_reader(s->sm->msg_bufs[SUB_SOCK_DRIVERMONITORING]);
    auto dm_root = dm_reader.getRoot<cereal::Event>();
    auto health = dm_root.getDriverMonitoring();
    driver_status.get_pose(dm_root.getDriverMonitoring(), cal_rpy, CS.getVEgo(), enabled);
  }
  if (driver_status.terminal_alert_cnt >= MAX_TERMINAL_ALERTS)
    events.push_back(create_event(cereal::CarEvent::EventName::TOO_DISTRACTED, EventType::NO_ENTRY));
}

const char *get_startup_alert(int car_recognized, int controller_available) {
  const char *alert = "startup";
  if(car_recognized == 0)
    alert = "startupNoCar";
  else if(car_recognized != 0 and controller_available == 0)
    alert = "startupNoControl";
  return alert;
}

void state_transition(int64_t frame, cereal::CarState::Builder &CS, cereal::CarParams::Reader&CP,
                      cereal::ControlsState::OpenpilotState &state, std::vector<event_name_types_pair> &events,
                      int &soft_disable_timer, double &v_cruise_kph, alert::AlertManager &AM, double &v_cruise_kph_last) {
  // Compute conditional state transitions and execute actions on state transitions
  bool enabled = isEnabled(state);
  v_cruise_kph_last = v_cruise_kph;

  // if stock cruise is completely disabled, then we can use our own set speed logic
  if(!CP.getEnableCruise()) {
    v_cruise_kph = update_v_cruise(v_cruise_kph, CS.getButtonEvents(), enabled);
  } else if(CP.getEnableCruise() and CS.getCruiseState().getEnabled()) {
    v_cruise_kph = (double)CS.getCruiseState().getSpeed() * MS_TO_KPH;
  }

  // decrease the soft disable timer at every step, as it's reset on
  // entrance in SOFT_DISABLING state
  soft_disable_timer = std::max(0, soft_disable_timer - 1);

  // DISABLED
  if(state == cereal::ControlsState::OpenpilotState::DISABLED) {
    if(get_events(events, EventType::ENABLE).size() > 0) {
      if(get_events(events, EventType::NO_ENTRY).size() > 0) {
        for(auto& en : get_events(events, EventType::NO_ENTRY))
          AM.add(frame, event_name_str(en) + "NoEntry", enabled);
      } else {
        if(get_events(events, EventType::PRE_ENABLE).size() > 0) {
          state = cereal::ControlsState::OpenpilotState::PRE_ENABLED;
        } else {
          state = cereal::ControlsState::OpenpilotState::ENABLED;
        }
        AM.add(frame, "enable", enabled);
        v_cruise_kph = initialize_v_cruise(CS.getVEgo(), CS.getButtonEvents(), v_cruise_kph_last);
      }
    }
  }
  // ENABLED
  else if(state == cereal::ControlsState::OpenpilotState::ENABLED) {
    if(get_events(events, EventType::USER_DISABLE).size() > 0) {
      state = cereal::ControlsState::OpenpilotState::DISABLED;
      AM.add(frame, "disable", enabled);
    } else if(get_events(events, EventType::IMMEDIATE_DISABLE).size() > 0) {
      state = cereal::ControlsState::OpenpilotState::DISABLED;
      for(auto& en : get_events(events, EventType::IMMEDIATE_DISABLE))
        AM.add(frame, event_name_str(en), enabled);
    } else if(get_events(events, EventType::SOFT_DISABLE).size() > 0) {
      state = cereal::ControlsState::OpenpilotState::SOFT_DISABLING;
      soft_disable_timer = 300;   // 3s
      for(auto& en : get_events(events, EventType::SOFT_DISABLE))
        AM.add(frame, event_name_str(en), enabled);
    }
  }
  // SOFT DISABLING
  else if(state == cereal::ControlsState::OpenpilotState::SOFT_DISABLING) {
    if(get_events(events, EventType::USER_DISABLE).size() > 0) {
      state = cereal::ControlsState::OpenpilotState::DISABLED;
      AM.add(frame, "disable", enabled);
    } else if(get_events(events, EventType::IMMEDIATE_DISABLE).size() > 0) {
      state = cereal::ControlsState::OpenpilotState::DISABLED;
      for(auto& en : get_events(events, EventType::IMMEDIATE_DISABLE))
        AM.add(frame, event_name_str(en), enabled);
    } else if(get_events(events, EventType::SOFT_DISABLE).size() <= 0) {
      // no more soft disabling condition, so go back to ENABLED
      state = cereal::ControlsState::OpenpilotState::ENABLED;
    } else if(get_events(events, EventType::SOFT_DISABLE).size() > 0 and soft_disable_timer > 0) {
      for(auto& en : get_events(events, EventType::SOFT_DISABLE))
        AM.add(frame, event_name_str(en), enabled);
    } else if(soft_disable_timer <= 0)
      state = cereal::ControlsState::OpenpilotState::DISABLED;
  }
  // PRE ENABLING
  else if(state == cereal::ControlsState::OpenpilotState::PRE_ENABLED) {
    if(get_events(events, EventType::USER_DISABLE).size() > 0) {
      state = cereal::ControlsState::OpenpilotState::DISABLED;
      AM.add(frame, "disable", enabled);
    } else if(get_events(events, EventType::IMMEDIATE_DISABLE, EventType::SOFT_DISABLE).size() > 0) {
      state = cereal::ControlsState::OpenpilotState::DISABLED;
      for(auto& en : get_events(events, EventType::IMMEDIATE_DISABLE, EventType::SOFT_DISABLE))
        AM.add(frame, event_name_str(en), enabled);
    } else if(get_events(events, EventType::PRE_ENABLE).size() <= 0)
      state = cereal::ControlsState::OpenpilotState::ENABLED;
  }
}

void state_control(int64_t frame, const std::vector<double> &rcv_frame, cereal::Plan::Reader &plan, cereal::PathPlan::Reader &path_plan,
                  cereal::CarState::Builder &CS, cereal::CarParams::Reader&CP, cereal::ControlsState::OpenpilotState &state,
                  std::vector<event_name_types_pair> &events, double v_cruise_kph, double v_cruise_kph_last,
                  alert::AlertManager &AM, DriverStatus &driver_status, LatControlBase *LaC, LongControl &LoC, VehicleModel & VM,
                  int read_only, bool is_metric, int8_t cal_perc, cereal::CarControl::Actuators::Builder &actuators,
                  double &v_acc, double &a_acc, void *lac_log) {
  // Given the state, this function returns an actuators packet

  bool enabled = isEnabled(state);
  bool active = isActive(state);

  // check if user has interacted with the car
  bool driver_engaged = (CS.hasButtonEvents() and CS.getButtonEvents().size() > 0) or v_cruise_kph != v_cruise_kph_last or CS.getSteeringPressed();

  // add eventual driver distracted events
  driver_status.update(events, driver_engaged, isActive(state), CS.getStandstill());

  // send FCW alert if triggered by planner
  if(plan.getFcw())
    AM.add(frame, "fcw", enabled);

  // State specific actions
  if(state == cereal::ControlsState::OpenpilotState::PRE_ENABLED or state == cereal::ControlsState::OpenpilotState::DISABLED) {
    LaC->reset();
    LoC.reset(CS.getVEgo());
  }
  else if(state == cereal::ControlsState::OpenpilotState::ENABLED or state == cereal::ControlsState::OpenpilotState::SOFT_DISABLING) {
    // parse warnings from car specific interface
    for(auto &e: get_events(events, EventType::WARNING)) {
      std::string extra_text = "";
      if(e == cereal::CarEvent::EventName::BELOW_STEER_SPEED) {
        if(is_metric)
          extra_text = std::to_string(std::round(CP.getMinSteerSpeed() * MS_TO_KPH)) + " kph";
        else
          extra_text = std::to_string(std::round(CP.getMinSteerSpeed() * MS_TO_MPH)) + " mph";
      }
      AM.add(frame, event_name_str(e), enabled, "", extra_text.c_str());
    }
  }
  double plan_age = DT_CTRL * (frame - rcv_frame[SUB_SOCK_PLAN]);
  double dt = std::min(plan_age, LON_MPC_STEP + DT_CTRL) + DT_CTRL;  // no greater than dt mpc + dt, to prevent too high extraps

  a_acc = (double)(plan.getAStart()) + (dt / LON_MPC_STEP) * ((double)plan.getATarget() - (double)plan.getAStart());
  v_acc = (double)(plan.getVStart()) + dt * (a_acc + (double)plan.getAStart()) / 2.0;

  // Gas/Brake PID loop
  double gas, brake;
  LoC.update(active, CS.getVEgo(), CS.getBrakePressed(), CS.getStandstill(), CS.getCruiseState().getStandstill(),
            v_cruise_kph, v_acc, plan.getVTargetFuture(), a_acc, CP, gas, brake);
  actuators.setGas(gas);
  actuators.setBrake(brake);
  // Steering PID loop and lateral MPC
  double steer, steerAngle;
  LaC->update(active, CS.getVEgo(), CS.getSteeringAngle(), CS.getSteeringRate(), CS.getSteeringTorqueEps(),
              CS.getSteeringPressed(), CP, path_plan, steer, steerAngle, lac_log);
  actuators.setSteer(steer);
  actuators.setSteerAngle(steerAngle);

  // Send a "steering required alert" if saturation count has reached the limit
  if(LaC->sat_flag and CP.getSteerLimitAlert())
    AM.add(frame, "steerSaturated", enabled);

  // Parse permanent warnings to display constantly
  for(auto &en: get_events(events, EventType::PERMANENT)) {
    std::string extra_text_1 = "", extra_text_2 = "";
    if(en == cereal::CarEvent::EventName::CALIBRATION_INCOMPLETE) {
      extra_text_1 = std::to_string(cal_perc) + "%";
      if(is_metric)
        extra_text_2 = std::to_string(int(round(MIN_SPEED * MS_TO_KPH))) + " kph";
      else
        extra_text_2 = std::to_string(int(round(MIN_SPEED * MS_TO_MPH))) + " mph";
    }
    AM.add(frame, event_name_str(en) + "Permanent", enabled, extra_text_1.c_str(), extra_text_2.c_str());
  }
  AM.process_alerts(frame);
}

void data_send(ControlsdState *s, cereal::Plan::Reader &plan, cereal::PathPlan::Reader &path_plan,
              cereal::CarState::Builder &CS, cereal::CarParams::Reader&CP,
              VehicleModel & VM, cereal::ControlsState::OpenpilotState state,
              const std::vector<event_name_types_pair> &events, cereal::CarControl::Actuators::Builder &actuators,
              double v_cruise_kph, alert::AlertManager &AM, DriverStatus &driver_status,
              LatControlBase *LaC, const LongControl &LoC, bool read_only, int64_t start_time,
              double v_acc, double a_acc, void *lac_log,
              std::vector<event_name_types_pair> &events_prev, cereal::CarControl::Builder &CC) {
  // Send actuators and hud commands to the car, send controlsstate and MPC logging
  CC.setEnabled(isEnabled(state));
  CC.setActuators(actuators);

  if(!CC.hasCruiseControl())
    CC.initCruiseControl();
  CC.getCruiseControl().setOverride(true);
  CC.getCruiseControl().setCancel(!CP.getEnableCruise() or (!isEnabled(state) and CS.getCruiseState().getEnabled()));
  // Some override values for Honda
  double brake_discount = (1.0 - clip<double>(actuators.getBrake() * 3., 0.0, 1.0));  // brake discount removes a sharp nonlinearity
  if(CP.getEnableCruise()) {
    CC.getCruiseControl().setSpeedOverride(double(std::max(0.0, (LoC.v_pid + CS.getCruiseState().getSpeedOffset()) * brake_discount)));
  } else
    CC.getCruiseControl().setSpeedOverride(0.0);

  CC.getCruiseControl().setAccelOverride(s->CI->calc_accel_override(CS.getAEgo(), plan.getATarget(), CS.getVEgo(), plan.getVTarget()));
  if(!CC.hasHudControl())
    CC.initHudControl();
  CC.getHudControl().setSetSpeed(v_cruise_kph * KPH_TO_MS);
  CC.getHudControl().setSpeedVisible(isEnabled(state));
  CC.getHudControl().setLanesVisible(isEnabled(state));
  CC.getHudControl().setLeadVisible(plan.getHasLead());

  bool right_lane_visible = path_plan.getRProb() > 0.5;
  bool left_lane_visible = path_plan.getLProb() > 0.5;
  CC.getHudControl().setRightLaneVisible(right_lane_visible);
  CC.getHudControl().setLeftLaneVisible(left_lane_visible);
  bool blinker = CS.getLeftBlinker() or CS.getRightBlinker();
  bool ldw_allowed = (CS.getVEgo() > 12.5 and !blinker);
  if(path_plan.hasRPoly() and path_plan.getRPoly().size() == 4)
    CC.getHudControl().setRightLaneDepart(ldw_allowed and path_plan.getRPoly()[3] > -(1.08 + CAMERA_OFFSET) and right_lane_visible);
  if(path_plan.hasLPoly() and path_plan.getLPoly().size() == 4)
    CC.getHudControl().setLeftLaneDepart(ldw_allowed and path_plan.getLPoly()[3] < (1.08 - CAMERA_OFFSET) and left_lane_visible);
  CC.getHudControl().setVisualAlert(AM.visual_alert);

  if(!read_only) {
    // send car controls over can
    std::vector<can_frame> can_list;
    s->CI->apply(CC, can_list);
    capnp::MallocMessageBuilder msg;
    cereal::Event::Builder event = msg.initRoot<cereal::Event>();
    event.setLogMonoTime(nanos_since_boot());
    event.setValid(CS.getCanValid());
    auto canData = event.initSendcan(can_list.size());
    int j = 0;
    for (auto it = can_list.begin(); it != can_list.end(); it++, j++) {
      canData[j].setAddress(it->address);
      canData[j].setBusTime(it->busTime);
      canData[j].setDat(kj::arrayPtr((uint8_t*)it->dat.data(), it->dat.size()));
      canData[j].setSrc(it->src);
    }
    s->pm->send(PUB_SOCK_SENDCAN, msg);
  }

  bool force_decel = driver_status.awareness < 0.;
  // controlsState
  {
    capnp::MallocMessageBuilder cs_msg;
    auto cs_event = cs_msg.initRoot<cereal::Event>();
    cs_event.setLogMonoTime(nanos_since_boot());
    cereal::ControlsState::Builder controls_state = cs_event.initControlsState();

    cs_event.setValid(CS.getCanValid());
    controls_state.setAlertText1(AM.alert_text_1);
    controls_state.setAlertText2(AM.alert_text_2);
    controls_state.setAlertSize(AM.alert_size);
    controls_state.setAlertStatus(AM.alert_status);
    controls_state.setAlertBlinkingRate(AM.alert_rate);
    controls_state.setAlertType(AM.alert_type);
    controls_state.setAlertSound(AM.audible_alert);
    controls_state.setAwarenessStatus(isEnabled(state) ? std::max(driver_status.awareness, -0.1) : 1.0);
    controls_state.setDriverMonitoringOn(driver_status.face_detected);
    controls_state.setCanMonoTimes(CS.getCanMonoTimes());
    controls_state.setCanMonoTimes(CS.getCanMonoTimes());
    controls_state.setPlanMonoTime(s->sm->logMonoTime[SUB_SOCK_PLAN]);
    controls_state.setPathPlanMonoTime(s->sm->logMonoTime[SUB_SOCK_PATHPLAN]);
    controls_state.setEnabled(isEnabled(state));
    controls_state.setActive(isActive(state));
    controls_state.setVEgo(CS.getVEgo());
    controls_state.setVEgoRaw(CS.getVEgoRaw());
    controls_state.setAngleSteers(CS.getSteeringAngle());
    controls_state.setCurvature(
      VM.calc_curvature((CS.getSteeringAngle() - path_plan.getAngleOffset()) * DEG_TO_RAD, CS.getVEgo()));
    controls_state.setSteerOverride(CS.getSteeringPressed());
    controls_state.setState(state);
    controls_state.setEngageable(get_events(events, EventType::NO_ENTRY).size() == 0);
    controls_state.setLongControlState(LoC.long_control_state);
    controls_state.setVPid(LoC.v_pid);
    controls_state.setVCruise(v_cruise_kph);
    controls_state.setUpAccelCmd(LoC.pid.p);
    controls_state.setUiAccelCmd(LoC.pid.i);
    controls_state.setUfAccelCmd(LoC.pid.f);
    controls_state.setAngleSteersDes(LaC->angle_steers_des);
    controls_state.setVTargetLead(v_acc);
    controls_state.setATarget(a_acc);
    controls_state.setJerkFactor(plan.getJerkFactor());
    controls_state.setGpsPlannerActive(plan.getGpsPlannerActive());
    controls_state.setVCurvature(plan.getVCurvature());
    controls_state.setDecelForModel(plan.getLongitudinalPlanSource() == cereal::Plan::LongitudinalPlanSource::MODEL);
    controls_state.setCumLagMs(0.); // No more lag?
    controls_state.setStartMonoTime(start_time * 1e9);
    controls_state.setMapValid(plan.getMapValid());
    controls_state.setForceDecel(force_decel);

    if(CP.getLateralTuning().which() == cereal::CarParams::LateralTuning::Which::PID)
      controls_state.getLateralControlState().setPidState(*(cereal::ControlsState::LateralPIDState::Builder *)lac_log);
    else if(CP.getLateralTuning().which() == cereal::CarParams::LateralTuning::Which::LQR)
      controls_state.getLateralControlState().setLqrState(*(cereal::ControlsState::LateralLQRState::Builder *)lac_log);
    // TODO: support INDI.
    /*
    elif CP.lateralTuning.which() == 'indi':
      dat.controlsState.lateralControlState.indiState = lac_log
    */
    s->pm->send(PUB_SOCK_CONTROLSSTATE, cs_msg);
  }
  // carState
  {
    capnp::MallocMessageBuilder carstate_msg;
    auto cs_send = carstate_msg.initRoot<cereal::Event>();
    cs_send.setValid(CS.getCanValid());
    CS.initEvents(events.size());
    capnp::List< ::cereal::CarEvent>::Builder target_events = CS.getEvents();
    int i = 0;
    for(auto &e: events) {
      target_events[i].setName(e.first);
      set_cereal_event_type(target_events[i], e.second);
      i++;
    }
    cs_send.setCarState(CS);
    s->pm->send(PUB_SOCK_CARSTATE, carstate_msg);
  }

  // carEvents - logged every second or on change
  if((s->sm->frame % ((int)(1. / DT_CTRL)) == 0) or (events != events_prev)) {
    capnp::MallocMessageBuilder ce_msg;
    auto ce_send = ce_msg.initRoot<cereal::Event>();
    auto target_events = ce_send.initCarEvents(events.size());
    int i = 0;
    for(auto &e: events) {
      target_events[i].setName(e.first);
      set_cereal_event_type(target_events[i], e.second);
      i++;
    }
    s->pm->send(PUB_SOCK_CAREVENTS, ce_msg);
  }
  // carParams - logged every 50 seconds (> 1 per segment)
  if((s->sm->frame % ((int)(50. / DT_CTRL)) == 0)) {
    capnp::MallocMessageBuilder cp_msg;
    auto cp_send = cp_msg.initRoot<cereal::Event>();
    cp_send.setCarParams(CP);
    s->pm->send(PUB_SOCK_CARPARAMS, cp_msg);
  }
  // carControl
  {
    capnp::MallocMessageBuilder cc_msg;
    auto cc_send = cc_msg.initRoot<cereal::Event>();
    cc_send.setValid(CS.getCanValid());
    cc_send.setCarControl(CC);
    s->pm->send(PUB_SOCK_CARCONTROL, cc_msg);
  }
}

void controlsd_init(int can_timeout) {
  set_realtime_priority(3);

  ControlsdState *s = &controlsd_state;
  s->ctx = zmq_ctx_new();
  if(s->pm == nullptr)
    s->pm = new PubMaster();
  if(s->sm == nullptr)
    s->sm = new SubMaster();
  if(s->can_sock_raw == nullptr)
    s->can_sock_raw = new SubSocket(s->ctx, "tcp://127.0.0.1:8006");
  s->can_sock_raw->set_timeout(can_timeout);

  init_sub_sock(s, SUB_SOCK_THERMAL, 2., "tcp://127.0.0.1:8005");
  init_sub_sock(s, SUB_SOCK_HEALTH, 2., "tcp://127.0.0.1:8011");
  init_sub_sock(s, SUB_SOCK_LIVECALIBRATION, 5., "tcp://127.0.0.1:8019");
  init_sub_sock(s, SUB_SOCK_DRIVERMONITORING, 5., "tcp://127.0.0.1:8063");
  init_sub_sock(s, SUB_SOCK_PLAN, 20., "tcp://127.0.0.1:8024");
  init_sub_sock(s, SUB_SOCK_PATHPLAN, 20., "tcp://127.0.0.1:8067");
  init_sub_sock(s, SUB_SOCK_GPSLOCATION, 20., "tcp://127.0.0.1:8026");


  s->pm->add_sock(s->ctx, "tcp://*:8017"); // sendcan
  s->pm->add_sock(s->ctx, "tcp://*:8007"); // controlsstate
  s->pm->add_sock(s->ctx, "tcp://*:8021"); //carstate
  s->pm->add_sock(s->ctx, "tcp://*:8023"); // carcontrol
  s->pm->add_sock(s->ctx, "tcp://*:8070"); // carevents
  s->pm->add_sock(s->ctx, "tcp://*:8071"); // carparams
}

extern "C" {

void init(int can_timeout) {
  signal(SIGTERM, (sighandler_t)set_do_exit);
  ControlsdState *s = &controlsd_state;
  controlsd_init(can_timeout);
}

void run(int internet_needed, int read_only, int sounds_available, unsigned char *car_param_data, int car_param_data_len,
          int car_recognized, int controller_available, const char *libdbc_fn) {
  ControlsdState *s = &controlsd_state;
  bool is_metric = false, passive = false, openpilot_enabled_toggle = false;
  read_param_bool(&is_metric, "IsMetric");
  read_param_bool(&passive, "Passive");
  read_param_bool(&openpilot_enabled_toggle, "OpenpilotEnabledToggle");

  capnp::MallocMessageBuilder cc_msg;
  auto cc_event = cc_msg.initRoot<cereal::Event>();
  cc_event.setLogMonoTime(nanos_since_boot());
  cereal::CarControl::Builder CC = cc_event.initCarControl();

  cereal::ControlsState::OpenpilotState state = cereal::ControlsState::OpenpilotState::DISABLED;
  bool overtemp = false;
  bool free_space = false;
  int8_t cal_status = CalibrationStatus::INVALID;
  int8_t cal_perc = 0;
  int mismatch_counter = 0;
  bool low_battery = false;
  int soft_disable_timer = 0;
  double v_cruise_kph = 255.;
  double v_cruise_kph_last = 0.;
  DriverStatus driver_status;
  std::vector<event_name_types_pair> events;
  std::vector<event_name_types_pair> events_prev;

  auto cp_msg = kj::heapArray<capnp::word>((car_param_data_len / sizeof(capnp::word)) + 1);
  memcpy(cp_msg.begin(), car_param_data, car_param_data_len);
  capnp::FlatArrayMessageReader cp_msg_reader(cp_msg);
  cereal::CarParams::Reader CP = cp_msg_reader.getRoot<cereal::CarParams>();
  VehicleModel VM(CP);
  LatControlBase *LaC = NULL;
  if(CP.getLateralTuning().which() == cereal::CarParams::LateralTuning::Which::LQR)
    LaC = new LatControlLQR(CP);
  else if(CP.getLateralTuning().which() == cereal::CarParams::LateralTuning::Which::PID)
    LaC = new LatControlPID(CP);
  else if(CP.getLateralTuning().which() == cereal::CarParams::LateralTuning::Which::INDI)
    LaC = new LatControlPID(CP); //TODO, support INDI

  LongControl LoC(CP, toyota::ToyotaCarInterface::compute_gb);
  // TODO: support cars other than toyota
  s->CI = new toyota::ToyotaCarInterface(CP, libdbc_fn);

  alert::AlertManager AM;
  AM.add(s->sm->frame, get_startup_alert(car_recognized, controller_available), false);

  while (!do_exit) {
    int64_t start_time = seconds_since_boot();
    capnp::MallocMessageBuilder cs_msg;
    auto event = cs_msg.initRoot<cereal::Event>();
    event.setLogMonoTime(nanos_since_boot());
    cereal::CarState::Builder CS = event.initCarState();
    data_sample(s, CS, CC, cal_status, cal_perc, overtemp, free_space, low_battery, driver_status, state, mismatch_counter, events);

    // Create alerts
    if(!s->sm->all_alive_and_valid())
      events.push_back(create_event(cereal::CarEvent::EventName::COMM_ISSUE, EventType::NO_ENTRY, EventType::SOFT_DISABLE));
    capnp::FlatArrayMessageReader path_plan_reader(s->sm->msg_bufs[SUB_SOCK_PATHPLAN]);
    auto path_plan_root = path_plan_reader.getRoot<cereal::Event>();
    auto path_plan = path_plan_root.getPathPlan();
    if(!(path_plan.getMpcSolutionValid()))
      events.push_back(create_event(cereal::CarEvent::EventName::PLANNER_ERROR, EventType::NO_ENTRY, EventType::IMMEDIATE_DISABLE));
    if(!(path_plan.getSensorValid()))
      events.push_back(create_event(cereal::CarEvent::EventName::SENSOR_DATA_INVALID, EventType::NO_ENTRY, EventType::PERMANENT));
    if(!(path_plan.getParamsValid()))
      events.push_back(create_event(cereal::CarEvent::EventName::VEHICLE_MODEL_INVALID, EventType::WARNING));
    if(!(path_plan.getPosenetValid()))
      events.push_back(create_event(cereal::CarEvent::EventName::POSENET_INVALID, EventType::NO_ENTRY, EventType::SOFT_DISABLE));
    capnp::FlatArrayMessageReader plan_reader(s->sm->msg_bufs[SUB_SOCK_PLAN]);
    auto plan_root = plan_reader.getRoot<cereal::Event>();
    auto plan = plan_root.getPlan();
    if(!(plan.getRadarValid()))
      events.push_back(create_event(cereal::CarEvent::EventName::RADAR_FAULT, EventType::NO_ENTRY, EventType::SOFT_DISABLE));
    if(plan.getRadarCanError())
      events.push_back(create_event(cereal::CarEvent::EventName::RADAR_CAN_ERROR, EventType::NO_ENTRY, EventType::SOFT_DISABLE));
    if(!(CS.getCanValid()))
      events.push_back(create_event(cereal::CarEvent::EventName::CAN_ERROR, EventType::NO_ENTRY, EventType::IMMEDIATE_DISABLE));
    if(sounds_available == 0)
      events.push_back(create_event(cereal::CarEvent::EventName::SOUNDS_UNAVAILABLE, EventType::NO_ENTRY, EventType::PERMANENT));
    if(internet_needed != 0)
      events.push_back(create_event(cereal::CarEvent::EventName::INTERNET_CONNECTIVITY_NEEDED, EventType::NO_ENTRY, EventType::PERMANENT));

    // Only allow engagement with brake pressed when stopped behind another stopped car
    if(CS.getBrakePressed() and plan.getVTargetFuture() >= STARTING_TARGET_SPEED and !CP.getRadarOffCan() and CS.getVEgo() < 0.3) {
      events.push_back(create_event(cereal::CarEvent::EventName::NO_TARGET, EventType::NO_ENTRY, EventType::IMMEDIATE_DISABLE));
    }

    if(read_only == 0)
      // update control state
      state_transition(s->sm->frame, CS, CP, state, events, soft_disable_timer, v_cruise_kph, AM, v_cruise_kph_last);

    capnp::MallocMessageBuilder actuator_msg;
    auto actuator_event = actuator_msg.initRoot<cereal::Event>();
    cereal::CarControl::Builder cc = actuator_event.initCarControl();
    cereal::CarControl::Actuators::Builder actuators = cc.initActuators();

    // Compute actuators (runs PID loops and lateral MPC)
    double v_acc, a_acc;
    capnp::MallocMessageBuilder lac_log_msg;
    auto lac_log_event = lac_log_msg.initRoot<cereal::Event>();
    lac_log_event.setLogMonoTime(nanos_since_boot());
    cereal::ControlsState::Builder lac_log_cs = lac_log_event.initControlsState();
    cereal::ControlsState::LateralControlState::Builder lac_log_cs_lcs = lac_log_cs.initLateralControlState();
    cereal::ControlsState::LateralLQRState::Builder lqr_log = lac_log_cs_lcs.initLqrState();
    cereal::ControlsState::LateralPIDState::Builder pid_log = lac_log_cs_lcs.initPidState();
    void *lac_log = NULL;
    if(CP.getLateralTuning().which() == cereal::CarParams::LateralTuning::Which::LQR)
      lac_log = &lqr_log;
    else if(CP.getLateralTuning().which() == cereal::CarParams::LateralTuning::Which::PID)
      lac_log = &pid_log;
    else if(CP.getLateralTuning().which() == cereal::CarParams::LateralTuning::Which::INDI)
      lac_log = &pid_log; // TODO, support INDI

    state_control(s->sm->frame, s->sm->rcv_frame, plan, path_plan, CS, CP, state, events,
                  v_cruise_kph, v_cruise_kph_last, AM, driver_status, LaC, LoC, VM, read_only,
                  is_metric, cal_perc, actuators, v_acc, a_acc, lac_log);
    // Publish data
    data_send(s, plan, path_plan, CS, CP, VM, state, events, actuators, v_cruise_kph, AM, driver_status,
              LaC, LoC, read_only, start_time, v_acc, a_acc, lac_log, events_prev, CC);
    events_prev = events;
    // 10ms
    usleep(10*1000);
  }
}

}
