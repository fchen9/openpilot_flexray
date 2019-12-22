#include <stdint.h>
#include <vector>
#include <cmath>
#include "cereal/gen/cpp/car.capnp.h"
#include "cereal/gen/cpp/log.capnp.h"
#include "stat_live.h"
#include "filter_simple.h"
#include "drive_helpers.h"
#include "alertmanager.h"
#include "driver_monitor.h"

static const double DT_DMON = 0.1;  // driver monitoring
static const double _AWARENESS_TIME = 100.;  // 1.6 minutes limit without user touching steering wheels make the car enter a terminal status
static const double _AWARENESS_PRE_TIME_TILL_TERMINAL = 25.;  // a first alert is issued 25s before expiration
static const double _AWARENESS_PROMPT_TIME_TILL_TERMINAL = 15.;  // a second alert is issued 15s before start decelerating the car
static const double _DISTRACTED_TIME = 11.;
static const double _DISTRACTED_PRE_TIME_TILL_TERMINAL = 8.;
static const double _DISTRACTED_PROMPT_TIME_TILL_TERMINAL = 6.;

static const double _FACE_THRESHOLD = 0.4;
static const double _EYE_THRESHOLD = 0.4;
static const double _BLINK_THRESHOLD = 0.5; // 0.225
static const double _PITCH_WEIGHT = 1.35; // 1.5  // pitch matters a lot more
static const double _METRIC_THRESHOLD = 0.4;
static const double _PITCH_POS_ALLOWANCE = 0.04; // 0.08  // rad, to not be too sensitive on positive pitch
static const double _PITCH_NATURAL_OFFSET = 0.12;  // 0.1   // people don't seem to look straight when they drive relaxed, rather a bit up
static const double _YAW_NATURAL_OFFSET = 0.08;  // people don't seem to look straight when they drive relaxed, rather a bit to the right (center of car)

static const double _DISTRACTED_FILTER_TS = 0.25;  // 0.6Hz

static const double _POSE_CALIB_MIN_SPEED = 13; // 30 mph
static const int64_t _POSE_OFFSET_MIN_COUNT = 600; // valid data counts before calibration completes, 1 seg is 600 counts
static const int64_t _POSE_OFFSET_MAX_COUNT = 3600; // stop deweighting new data after 6 min, aka "short term memory"

static const double __RECOVERY_FACTOR_MAX = 5.; // relative to minus step change
static const double __RECOVERY_FACTOR_MIN = 1.25; // relative to minus step change

const int MAX_TERMINAL_ALERTS = 3;      // not allowed to engage after 3 terminal alerts

// model output refers to center of cropped image, so need to apply the x displacement offset
static const double RESIZED_FOCAL = 320.0;
static const double H = 320, W = 160, FULL_W = 426;

static void head_orientation_from_descriptor(const capnp::List<float>::Reader &angles_desc, const capnp::List<float>::Reader  &pos_desc,
                                      const double *rpy_calib, double &roll, double &pitch, double &yaw) {
  // the output of these angles are in device frame
  // so from driver's perspective, pitch is up and yaw is right

  double pitch_prnet = angles_desc[0];
  double yaw_prnet = angles_desc[1];
  double roll_prnet = angles_desc[2];

  double face_pixel_position[2] = {(pos_desc[0] + .5)*W - W + FULL_W, (pos_desc[1]+.5)*H};
  double yaw_focal_angle = atan2(face_pixel_position[0] - FULL_W/2, RESIZED_FOCAL);
  double pitch_focal_angle = atan2(face_pixel_position[1] - H/2, RESIZED_FOCAL);

  roll = roll_prnet;
  pitch = pitch_prnet + pitch_focal_angle;
  yaw = -yaw_prnet + yaw_focal_angle;

  // no calib for roll
  pitch -= rpy_calib[1];
  yaw -= rpy_calib[2];
}

DriverPose::DriverPose(): pitch_offseter(NULL, NULL, _POSE_OFFSET_MAX_COUNT), yaw_offseter(NULL, NULL, _POSE_OFFSET_MAX_COUNT) {
  yaw = 0.;
  pitch = 0.;
  roll = 0.;
}

DriverBlink::DriverBlink() {
  left_blink = 0.;
  right_blink = 0.;
}

DriverStatus::DriverStatus(): pose(), blink(), driver_distraction_filter(0., _DISTRACTED_FILTER_TS, DT_DMON){
  pose_calibrated = pose.pitch_offseter.filtered_stat.n > _POSE_OFFSET_MIN_COUNT and \
                          pose.yaw_offseter.filtered_stat.n > _POSE_OFFSET_MIN_COUNT;
  awareness = 1.;
  awareness_active = 1.;
  awareness_passive = 1.;
  driver_distracted = false;
  face_detected = false;
  terminal_alert_cnt = 0;
  step_change = 0.;
  active_monitoring_mode = true;
  threshold_prompt = _DISTRACTED_PROMPT_TIME_TILL_TERMINAL / _DISTRACTED_TIME;

  is_rhd_region = false;
  is_rhd_region_checked = false;

  _set_timers(true);
}

void DriverStatus::_set_timers(bool active_monitoring) {
  if (active_monitoring_mode and awareness <= threshold_prompt) {
    if (active_monitoring)
      step_change = DT_CTRL / _DISTRACTED_TIME;
    else
      step_change = 0.;
    return; // no exploit after orange alert
  } else if (awareness <= 0.)
    return;

  if (active_monitoring) {
    // when falling back from passive mode to active mode, reset awareness to avoid false alert
    if (!active_monitoring_mode) {
      awareness_passive = awareness;
      awareness = awareness_active;
    }

    threshold_pre = _DISTRACTED_PRE_TIME_TILL_TERMINAL / _DISTRACTED_TIME;
    threshold_prompt = _DISTRACTED_PROMPT_TIME_TILL_TERMINAL / _DISTRACTED_TIME;
    step_change = DT_CTRL / _DISTRACTED_TIME;
    active_monitoring_mode = true;
  } else {
    if (active_monitoring_mode) {
      awareness_active = awareness;
      awareness = awareness_passive;
    }
    threshold_pre = _AWARENESS_PRE_TIME_TILL_TERMINAL / _AWARENESS_TIME;
    threshold_prompt = _AWARENESS_PROMPT_TIME_TILL_TERMINAL / _AWARENESS_TIME;
    step_change = DT_CTRL / _AWARENESS_TIME;
    active_monitoring_mode = false;
  }
}

DistractedType DriverStatus::_is_driver_distracted(DriverPose &pose, DriverBlink &blink) {
  double pitch_error, yaw_error;
  if (not pose_calibrated) {
    pitch_error = pose.pitch - _PITCH_NATURAL_OFFSET;
    yaw_error = pose.yaw - _YAW_NATURAL_OFFSET;
    // add positive pitch allowance
    if (pitch_error > 0.)
      pitch_error = std::max(pitch_error - _PITCH_POS_ALLOWANCE, 0.);
  } else {
    pitch_error = pose.pitch - this->pose.pitch_offseter.filtered_stat.mean();
    yaw_error = pose.yaw - this->pose.yaw_offseter.filtered_stat.mean();
  }
  pitch_error *= _PITCH_WEIGHT;
  double pose_metric = std::sqrt(yaw_error*yaw_error + pitch_error*pitch_error);

  if (pose_metric > _METRIC_THRESHOLD)
    return DistractedType::BAD_POSE;
  else if ((blink.left_blink + blink.right_blink)*0.5 > _BLINK_THRESHOLD)
    return DistractedType::BAD_BLINK;
  else
    return DistractedType::NOT_DISTRACTED;
}

void DriverStatus::get_pose(const cereal::DriverMonitoring::Reader &driver_monitoring, const double *cal_rpy, double car_speed, bool op_engaged) {
  // 10 Hz
  if (driver_monitoring.getFaceOrientation().size() == 0 or driver_monitoring.getFacePosition().size() == 0)
    return;

  head_orientation_from_descriptor(driver_monitoring.getFaceOrientation(), driver_monitoring.getFacePosition(),
    cal_rpy, pose.roll, pose.pitch, pose.yaw);
  blink.left_blink = driver_monitoring.getLeftBlinkProb() * (driver_monitoring.getLeftEyeProb()>_EYE_THRESHOLD);
  blink.right_blink = driver_monitoring.getRightBlinkProb() * (driver_monitoring.getRightEyeProb()>_EYE_THRESHOLD);
  face_detected = driver_monitoring.getFaceProb() > _FACE_THRESHOLD and not is_rhd_region;

  driver_distracted = _is_driver_distracted(pose, blink) > 0;
  // first order filters
  driver_distraction_filter.update(driver_distracted);
  // update offseter
  // only update when driver is actively driving the car above a certain speed
  if (face_detected and car_speed>_POSE_CALIB_MIN_SPEED and not op_engaged) {
    pose.pitch_offseter.push_and_update(pose.pitch);
    pose.yaw_offseter.push_and_update(pose.yaw);
  }

  pose_calibrated = pose.pitch_offseter.filtered_stat.n > _POSE_OFFSET_MIN_COUNT and \
                          pose.yaw_offseter.filtered_stat.n > _POSE_OFFSET_MIN_COUNT;

  _set_timers(face_detected);
}

void DriverStatus::update(std::vector<event_name_types_pair> &events, bool driver_engaged, bool ctrl_active, bool standstill) {
  if ((driver_engaged and awareness > 0) or !ctrl_active) {
    // reset only when on disengagement if red reached
    awareness = 1.;
    awareness_active = 1.;
    awareness_passive = 1.;
    return;
  }

  bool driver_attentive = driver_distraction_filter.x < 0.37;
  double awareness_prev = awareness;

  if (driver_attentive and face_detected and awareness > 0) {
    // only restore awareness when paying attention and alert is not red
    awareness = std::min(awareness + ((__RECOVERY_FACTOR_MAX-__RECOVERY_FACTOR_MIN)*(1.-awareness)+__RECOVERY_FACTOR_MIN)*step_change, 1.);
    if (awareness == 1.)
      awareness_passive = std::min(awareness_passive + step_change, 1.0);
    // don't display alert banner when awareness is recovering and has cleared orange
    if (awareness > threshold_prompt)
      return;
  }
  // should always be counting if distracted unless at standstill and reaching orange
  if ((!face_detected or (driver_distraction_filter.x > 0.63 and driver_distracted and face_detected)) and \
     !(standstill and awareness - step_change <= threshold_prompt)) {
    awareness = std::max(awareness - step_change, -0.1);
  }
  if (awareness <= 0.) {
    // terminal red alert: disengagement required
    if (active_monitoring_mode)
      events.push_back(create_event(cereal::CarEvent::EventName::DRIVER_DISTRACTED, EventType::WARNING));
    else
      events.push_back(create_event(cereal::CarEvent::EventName::DRIVER_UNRESPONSIVE, EventType::WARNING));
    if (awareness_prev > 0.)
      terminal_alert_cnt += 1;
  } else if (awareness <= threshold_prompt) {
    // prompt orange alert
    if (active_monitoring_mode)
      events.push_back(create_event(cereal::CarEvent::EventName::PROMPT_DRIVER_DISTRACTED, EventType::WARNING));
    else
      events.push_back(create_event(cereal::CarEvent::EventName::PROMPT_DRIVER_UNRESPONSIVE, EventType::WARNING));
  } else if (awareness <= threshold_pre) {
    // pre green alert
    if (active_monitoring_mode)
      events.push_back(create_event(cereal::CarEvent::EventName::PRE_DRIVER_DISTRACTED, EventType::WARNING));
    else
      events.push_back(create_event(cereal::CarEvent::EventName::PRE_DRIVER_UNRESPONSIVE, EventType::WARNING));
  }
}
