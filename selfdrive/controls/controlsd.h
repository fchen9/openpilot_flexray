#ifndef CONTROLSD_H_
#define CONTROLSD_H_

typedef enum {
  UNCALIBRATED = 0,
  CALIBRATED = 1,
  INVALID = 2
} CalibrationStatus;

typedef enum {
  SUB_SOCK_THERMAL = 0,
  SUB_SOCK_HEALTH,
  SUB_SOCK_LIVECALIBRATION,
  SUB_SOCK_DRIVERMONITORING,
  SUB_SOCK_PLAN,
  SUB_SOCK_PATHPLAN,
  SUB_SOCK_GPSLOCATION,
} SubSockIndex;

typedef enum {
  PUB_SOCK_SENDCAN = 0,
  PUB_SOCK_CONTROLSSTATE,
  PUB_SOCK_CARSTATE,
  PUB_SOCK_CARCONTROL,
  PUB_SOCK_CAREVENTS,
  PUB_SOCK_CARPARAMS,
} PubSockIndex;

typedef struct ControlsdState {
  CarInterfaceBase *CI;
  void *ctx;
  SubMasterBase *sm;
  PubMasterBase* pm;
  SubSocketBase *can_sock_raw;
  bool test_mode_enabled;
} ControlsdState;

extern ControlsdState controlsd_state;


#endif