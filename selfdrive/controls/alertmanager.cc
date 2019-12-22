#include "cereal/gen/cpp/car.capnp.h"
#include "cereal/gen/cpp/log.capnp.h"
#include "common/swaglog.h"
#include "alertmanager.h"

const double DT_CTRL = 0.01;  // controlsd

namespace alert {

AlertManager::AlertManager() {
  for(int i = 0; i < ALERT_COUNT;i++)
    alerts[ALERTS[i].alert_type_] = &ALERTS[i];
}

bool AlertManager::alertPresent() {
  return activealerts.size() > 0;
}

void AlertManager::add(int64_t frame, std::string alert_type, bool enabled, const char *extra_text_1, const char *extra_text_2) {
  Alert added_alert = Alert(*alerts[alert_type]);
  added_alert.alert_text_1_ += extra_text_1;
  added_alert.alert_text_2_ += extra_text_2;
  added_alert.start_time_ = frame * DT_CTRL;

  // if new alert is higher priority, log it
  if(!alertPresent() or added_alert.alert_priority_ > activealerts[0].alert_priority_)
        LOGW("alert_add, %s, %d", alert_type.c_str(), enabled);

  activealerts.push_back(added_alert);

  // sort by priority first and then by start_time
  std::sort (activealerts.begin(), activealerts.end(),
    [](const Alert & a, const Alert & b) -> bool {
      if(a.alert_priority_ > b.alert_priority_) return true;
      else if(a.alert_priority_ < b.alert_priority_) return false;
      else return a.start_time_ > b.start_time_;
    }
  );
}

void AlertManager::process_alerts(double frame) {
  double cur_time = frame * DT_CTRL;
  // first get rid of all the expired alerts
  activealerts.erase(
    std::remove_if(
      activealerts.begin(),
      activealerts.end(),
      [cur_time](Alert const & a) {
        return (a.start_time_ + std::max(a.duration_sound_, std::max(a.duration_hud_alert_, a.duration_text_))) <= cur_time;
      }
    ),
    activealerts.end()
  );
  Alert *current_alert = alertPresent()? &activealerts[0] : nullptr;

  // start with assuming no alerts
  alert_type = "";
  alert_text_1 = "";
  alert_text_2 = "";
  alert_status = cereal::ControlsState::AlertStatus::NORMAL;
  alert_size = cereal::ControlsState::AlertSize::NONE;
  visual_alert = cereal::CarControl::HUDControl::VisualAlert::NONE;
  audible_alert = cereal::CarControl::HUDControl::AudibleAlert::NONE;
  alert_rate = 0.;

  if(current_alert != nullptr) {
    alert_type = current_alert->alert_type_;

    if(current_alert->start_time_ + current_alert->duration_sound_ > cur_time)
      audible_alert = current_alert->audible_alert_;

    if(current_alert->start_time_ + current_alert->duration_hud_alert_ > cur_time)
      visual_alert = current_alert->visual_alert_;

    if(current_alert->start_time_ + current_alert->duration_text_ > cur_time) {
      alert_text_1 = current_alert->alert_text_1_;
      alert_text_2 = current_alert->alert_text_2_;
      alert_status = current_alert->alert_status_;
      alert_size = current_alert->alert_size_;
      alert_rate = current_alert->alert_rate_;
    }
  }
}

}
