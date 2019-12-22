#ifndef ALERTMANAGER_H_
#define ALERTMANAGER_H_

#include <stdint.h>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include "alerts.h"

extern const double DT_CTRL;

namespace alert {

class AlertManager {
  public:
    AlertManager();
    bool alertPresent();
    void add(int64_t frame, std::string alert_type, bool enabled=true, const char *extra_text_1="", const char *extra_text_2="");
    void process_alerts(double frame);
  private:
    std::vector<Alert> activealerts;
    std::map<std::string, Alert *> alerts;
  public:
    std::string alert_type;
    std::string alert_text_1;
    std::string alert_text_2;
    cereal::ControlsState::AlertStatus alert_status;
    cereal::ControlsState::AlertSize alert_size;
    cereal::CarControl::HUDControl::VisualAlert visual_alert;
    cereal::CarControl::HUDControl::AudibleAlert audible_alert;
    double alert_rate;
};

}

#endif