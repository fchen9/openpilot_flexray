'''
Command line tool that convert controls/lib/alerts.py to alerts.cc
Usage:
  python3 alerts_py_to_cpp > ./alerts.cc
'''

from cereal import car, log
from selfdrive.controls.lib.alerts import ALERTS, Priority


AlertSize = log.ControlsState.AlertSize
AlertStatus = log.ControlsState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert

def alert_status_to_enum(alert_status):
  if alert_status == AlertStatus.normal:
    return 'cereal::ControlsState::AlertStatus::NORMAL'
  elif alert_status == AlertStatus.userPrompt:
    return 'cereal::ControlsState::AlertStatus::USER_PROMPT'
  elif alert_status == AlertStatus.critical:
    return 'cereal::ControlsState::AlertStatus::CRITICAL'
  else:
    raise IOError()

def alert_size_to_enum(alert_size):
  if alert_size == AlertSize.none:
    return 'cereal::ControlsState::AlertSize::NONE'
  elif alert_size == AlertSize.small:
    return 'cereal::ControlsState::AlertSize::SMALL'
  elif alert_size == AlertSize.mid:
    return 'cereal::ControlsState::AlertSize::MID'
  elif alert_size == AlertSize.full:
    return 'cereal::ControlsState::AlertSize::FULL'
  else:
    raise IOError()

def alert_priority_to_enum(alert_priority):
  if alert_priority == Priority.LOWEST:
    return 'alert::Priority::LOWEST'
  elif alert_priority == Priority.LOW_LOWEST:
    return 'alert::Priority::LOW_LOWEST'
  elif alert_priority == Priority.LOW:
    return 'alert::Priority::LOW'
  elif alert_priority == Priority.MID:
    return 'alert::Priority::MID'
  elif alert_priority == Priority.HIGH:
    return 'alert::Priority::HIGH'
  elif alert_priority == Priority.HIGHEST:
    return 'alert::Priority::HIGHEST'
  else:
    raise IOError()

def visual_alert_to_enum(visual_alert):
  if visual_alert == VisualAlert.none:
    return 'cereal::CarControl::HUDControl::VisualAlert::NONE'
  elif visual_alert == VisualAlert.fcw:
    return 'cereal::CarControl::HUDControl::VisualAlert::FCW'
  elif visual_alert == VisualAlert.steerRequired:
    return 'cereal::CarControl::HUDControl::VisualAlert::STEER_REQUIRED'
  elif visual_alert == VisualAlert.brakePressed or visual_alert == 'brakePressed':
    return 'cereal::CarControl::HUDControl::VisualAlert::BRAKE_PRESSED'
  elif visual_alert == VisualAlert.wrongGear:
    return 'cereal::CarControl::HUDControl::VisualAlert::WRONG_GEAR'
  elif visual_alert == VisualAlert.seatbeltUnbuckled:
    return 'cereal::CarControl::HUDControl::VisualAlert::SEATBELT_UNBUCKLED'
  elif visual_alert == VisualAlert.speedTooHigh:
    return 'cereal::CarControl::HUDControl::VisualAlert::SPEED_TOO_HIGH'
  else:
    print('Invalid', visual_alert)
    raise IOError()

def audible_alert_to_enum(audible_alert):
  if audible_alert == AudibleAlert.none:
    return 'cereal::CarControl::HUDControl::AudibleAlert::NONE'
  elif audible_alert == AudibleAlert.chimeEngage:
    return 'cereal::CarControl::HUDControl::AudibleAlert::CHIME_ENGAGE'
  elif audible_alert == AudibleAlert.chimeDisengage:
    return 'cereal::CarControl::HUDControl::AudibleAlert::CHIME_DISENGAGE'
  elif audible_alert == AudibleAlert.chimeError:
    return 'cereal::CarControl::HUDControl::AudibleAlert::CHIME_ERROR'
  elif audible_alert == AudibleAlert.chimeWarning1:
    return 'cereal::CarControl::HUDControl::AudibleAlert::CHIME_WARNING1'
  elif audible_alert == AudibleAlert.chimeWarning2:
    return 'cereal::CarControl::HUDControl::AudibleAlert::CHIME_WARNING2'
  elif audible_alert == AudibleAlert.chimeWarningRepeat:
    return 'cereal::CarControl::HUDControl::AudibleAlert::CHIME_WARNING_REPEAT'
  elif audible_alert == AudibleAlert.chimePrompt:
    return 'cereal::CarControl::HUDControl::AudibleAlert::CHIME_PROMPT'
  else:
    print('Invalid', audible_alert)
    raise IOError()

#AudibleAlert.none

if __name__ == "__main__":
  print('#include "cereal/gen/cpp/car.capnp.h"')
  print('#include "cereal/gen/cpp/log.capnp.h"')
  print('#include "alerts.h"\n')
  print('alert::Alert ALERTS[] = {')
  for alert in ALERTS:
    print(
      "{{ \n  \"{}\",\n  \"{}\",\n  \"{}\",\n  {},\n  {},\n  {},\n  {},\n  {},\n  {}, {}, {}, {}\n}},".format(alert.alert_type,
      alert.alert_text_1,
      alert.alert_text_2,
      alert_status_to_enum(alert.alert_status),
      alert_size_to_enum(alert.alert_size),
      alert_priority_to_enum(alert.alert_priority),
      visual_alert_to_enum(alert.visual_alert),
      audible_alert_to_enum(alert.audible_alert),
      alert.duration_sound,
      alert.duration_hud_alert,
      alert.duration_text,
      alert.alert_rate)
    )
  print('};')
  print('size_t ALERT_COUNT = sizeof(ALERTS)/sizeof(ALERTS[0]);')