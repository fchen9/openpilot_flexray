#ifndef DRIVER_MONITOR_H_
#define DRIVER_MONITOR_H_

typedef enum {
  NOT_DISTRACTED = 0,
  BAD_POSE = 1,
  BAD_BLINK = 2,
} DistractedType;

extern const int MAX_TERMINAL_ALERTS;

class DriverPose {
public:
  DriverPose();

  double yaw, pitch, roll;
  RunningStatFilter pitch_offseter, yaw_offseter;
};

class DriverBlink {
public:
  DriverBlink();

  double left_blink, right_blink;
};

class DriverStatus {
public:
  DriverStatus();

  void _set_timers(bool active_monitoring);
  DistractedType _is_driver_distracted(DriverPose &pose, DriverBlink &blink);
  void get_pose(const cereal::DriverMonitoring::Reader &driver_monitoring, const double *cal_rpy, double car_speed, bool op_engaged);
  void update(std::vector<event_name_types_pair> &events, bool driver_engaged, bool ctrl_active, bool standstill);

  DriverBlink blink;
  DriverPose pose;
  bool pose_calibrated, driver_distracted, face_detected, active_monitoring_mode, is_rhd_region, is_rhd_region_checked;
  double awareness, awareness_active, awareness_passive, terminal_alert_cnt ,step_change, threshold_prompt, threshold_pre;
  FirstOrderFilter driver_distraction_filter;
};

#endif