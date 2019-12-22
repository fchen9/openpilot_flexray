#ifndef DRIVE_HELPER_H_
#define DRIVE_HELPER_H_

#include <stdint.h>
#include <string>
#include <vector>
#include <utility>
#define _USE_MATH_DEFINES
#include <cmath>

// kph
extern const double V_CRUISE_MAX;
extern const double V_CRUISE_MIN;
extern const int V_CRUISE_DELTA;
extern const double V_CRUISE_ENABLE_MIN;
extern const double MPH_TO_KPH;
extern const double KPH_TO_MPH;
extern const double MS_TO_KPH;
extern const double KPH_TO_MS;
extern const double MS_TO_MPH;
extern const double LON_MPC_STEP;  // first step is 0.2s
extern const double CAMERA_OFFSET;  // m from center car to camera
extern const double DEG_TO_RAD;


typedef enum {
  NULL_EVENT_TYPE = 0,
  ENABLE,
  PRE_ENABLE,
  NO_ENTRY,
  WARNING,
  USER_DISABLE,
  SOFT_DISABLE,
  IMMEDIATE_DISABLE,
  PERMANENT,
} EventType;

typedef std::pair<cereal::CarEvent::EventName, std::vector<EventType>> event_name_types_pair;

void set_cereal_event_type(cereal::CarEvent::Builder ce, const std::vector<EventType> &etv);
event_name_types_pair create_event(
            cereal::CarEvent::EventName name, EventType type1 = NULL_EVENT_TYPE, EventType type2 = NULL_EVENT_TYPE, EventType type3 = NULL_EVENT_TYPE);
std::vector<cereal::CarEvent::EventName> get_events(const std::vector<event_name_types_pair> &events,
                                                    EventType type1 = NULL_EVENT_TYPE,
                                                    EventType type2 = NULL_EVENT_TYPE);
std::string event_name_str(cereal::CarEvent::EventName en);

template<typename T>
inline T clip(T x, T lo, T hi) {
  return std::max(lo, std::min(hi, x));
}

template<typename T=double>
T interp(T xv, const T *xp, const T *fp, int N = 2);

template<typename T, typename C=capnp::List<float>::Reader>
inline T interp(T xv, const C &xp, const C &fp) {
  int N = xp.size();
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

double sign(double x);

double initialize_v_cruise(double v_ego, const capnp::List< ::cereal::CarState::ButtonEvent>::Reader &buttonEvents, int v_cruise_last);
double update_v_cruise(double v_cruise_kph, const capnp::List< ::cereal::CarState::ButtonEvent>::Reader &buttonEvents, bool enabled);
double get_steer_max(const cereal::CarParams::Reader &CP, double v_ego);


#endif