#ifndef ALERTS_H_
#define ALERTS_H_

#include <string>
#include <vector>
#include <utility>

namespace alert {
// Priority
typedef enum {
  LOWEST = 0,
  LOW_LOWEST = 1,
  LOW = 2,
  MID = 3,
  HIGH = 4,
  HIGHEST = 5,
} Priority;

class Alert {
  public:
    Alert(
      const char *alert_type,
      const char *alert_text_1,
      const char *alert_text_2,
      cereal::ControlsState::AlertStatus alert_status,
      cereal::ControlsState::AlertSize alert_size,
      Priority alert_priority,
      cereal::CarControl::HUDControl::VisualAlert visual_alert,
      cereal::CarControl::HUDControl::AudibleAlert audible_alert,
      double duration_sound,
      double duration_hud_alert,
      double duration_text,
      double alert_rate=0.) {

      alert_type_ = alert_type;
      alert_text_1_ = alert_text_1;
      alert_text_2_ = alert_text_2;
      alert_status_ = alert_status;
      alert_size_ = alert_size;
      alert_priority_ = alert_priority;
      visual_alert_ = visual_alert;
      audible_alert_ = audible_alert;
  
      duration_sound_ = duration_sound;
      duration_hud_alert_ = duration_hud_alert;
      duration_text_ = duration_text;
  
      start_time_ = 0.;
      alert_rate_ = alert_rate;
    }

    Alert(const Alert &alert2) {
      alert_type_ = alert2.alert_type_;
      alert_text_1_ = alert2.alert_text_1_;
      alert_text_2_ = alert2.alert_text_2_;
      alert_status_ = alert2.alert_status_;
      alert_size_ = alert2.alert_size_;
      alert_priority_ = alert2.alert_priority_;
      visual_alert_ = alert2.visual_alert_;
      audible_alert_ = alert2.audible_alert_;
      duration_sound_ = alert2.duration_sound_;
      duration_hud_alert_ = alert2.duration_hud_alert_;
      duration_text_ = alert2.duration_text_;
      start_time_ = alert2.start_time_;
      alert_rate_ = alert2.alert_rate_;
    }

    bool operator> (const Alert &alert2) const {
      return alert_priority_ > alert2.alert_priority_;
    }

  public:
    std::string alert_type_;
    std::string alert_text_1_;
    std::string alert_text_2_;
    cereal::ControlsState::AlertStatus alert_status_;
    cereal::ControlsState::AlertSize alert_size_;
    Priority alert_priority_;
    cereal::CarControl::HUDControl::VisualAlert visual_alert_;
    cereal::CarControl::HUDControl::AudibleAlert audible_alert_;
    double duration_sound_;
    double duration_hud_alert_;
    double duration_text_;
    double start_time_;
    double alert_rate_;
};

}

extern alert::Alert ALERTS[];
extern size_t ALERT_COUNT;

#endif