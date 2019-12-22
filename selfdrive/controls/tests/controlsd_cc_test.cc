/*
  Controlsd test utilities
*/
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

#include "common/messaging.h"
#include "common/timing.h"
#include "common/util.h"
#include "common/swaglog.h"

#include "common/params.h"

#include <capnp/serialize.h>
#include "cereal/gen/cpp/car.capnp.h"
#include "cereal/gen/cpp/log.capnp.h"

#include "KF1D.h"
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
#include "latcontrol_pid.h"
#include "latcontrol_lqr.h"
#include "filter_simple.h"
#include "stat_live.h"
#include "driver_monitor.h"
// Toyota
#include "toyota/toyotacan.h"
#include "toyota/toyota_values.h"
#include "toyota/toyota_carstate.h"
#include "toyota/toyota_carcontroller.h"
#include "toyota/toyota_interface.h"
#include "controlsd.h"

class FakeSubSocket: public SubSocketBase {
public:
  FakeSubSocket(): receive_called(false), data_ready(false) {
    pthread_mutex_init(&mutex, 0);
    pthread_cond_init(&cond, 0);
  }
  
  void receive(std::vector<std::string> &can_strs) {
    pthread_mutex_lock(&mutex);
    receive_called = true;
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mutex);
  
    pthread_mutex_lock(&mutex);
    while (!data_ready)
      pthread_cond_wait(&cond, &mutex);
    data_ready = false;
    pthread_mutex_unlock(&mutex);
    can_strs.push_back(data);
  }
  
  void wait_for_recv() {
    pthread_mutex_lock(&mutex);
    while (!receive_called)
      pthread_cond_wait(&cond, &mutex);
    pthread_mutex_unlock(&mutex);
  }
  
  void set_data(unsigned char *ptr, int sz) {
    pthread_mutex_lock(&mutex);
    while (!receive_called)
      pthread_cond_wait(&cond, &mutex);
    receive_called = false;
    pthread_mutex_unlock(&mutex);
  
    pthread_mutex_lock(&mutex);
    data.assign((char *)ptr, sz);
    data_ready = true;
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mutex);
  }

private:
  std::string data;
  pthread_mutex_t mutex;
  pthread_cond_t cond;
  bool receive_called, data_ready;
};

// For process replay tests
class FakeSubMaster: public SubMasterBase {
public:
  FakeSubMaster(): update_called(false), update_ready(false) {
    pthread_mutex_init(&mutex, 0);
    pthread_cond_init(&cond, 0);
  }
  
  void update(int timeout) {
    pthread_mutex_lock(&mutex);
    update_called = true;
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mutex);
  
    pthread_mutex_lock(&mutex);
    while (!update_ready)
      pthread_cond_wait(&cond, &mutex);
    update_ready = false;
    pthread_mutex_unlock(&mutex);
  }
  
  void update_msgs(double cur_time) {
    pthread_mutex_lock(&mutex);
    while (!update_called)
      pthread_cond_wait(&cond, &mutex);
    update_called = false;
    pthread_mutex_unlock(&mutex);
  
    std::vector<int> updated_sock_ids;
    for(std::pair<int, std::string> e:  updated_socks_data) {
      updated_sock_ids.push_back(e.first);
      msg_bufs[e.first] = kj::heapArray<capnp::word>((e.second.length() / sizeof(capnp::word)) + 1);
      memcpy(msg_bufs[e.first].begin(), e.second.data(), e.second.length());
    }
    updated_socks_data.clear();
    SubMasterBase::update_msgs(cur_time, updated_sock_ids);
  
    pthread_mutex_lock(&mutex);
    update_ready = true;
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mutex);
  }
  
  void update_sock_data(int sock_idx, char *data, int len) {
    updated_socks_data[sock_idx] = std::string(data, len);
  }
private:
  pthread_mutex_t mutex;
  pthread_cond_t cond;
  bool update_called, update_ready;
  std::map<int, std::string> updated_socks_data;
};

class FakePubMaster: public PubMasterBase {
public:
  FakePubMaster(): send_called(false), get_called(false) {
    pthread_mutex_init(&mutex, 0);
    pthread_cond_init(&cond, 0);
  }

  void add_sock(void *ctx, const char *addr) {
  }

  void send(int idx, capnp::MallocMessageBuilder &msg) {
    pthread_mutex_lock(&mutex);
    send_idx = idx;
    auto words = capnp::messageToFlatArray(msg);
    auto bytes = words.asBytes();
    send_data.assign((char *)bytes.begin(), bytes.size());
    send_called = true;
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mutex);

    pthread_mutex_lock(&mutex);
    while (!get_called)
      pthread_cond_wait(&cond, &mutex);
    get_called = false;
    pthread_mutex_unlock(&mutex);
  }

  std::string wait_for_msg() {
    pthread_mutex_lock(&mutex);
    while (!send_called)
      pthread_cond_wait(&cond, &mutex);
    send_called = false;
    pthread_mutex_unlock(&mutex);

    pthread_mutex_lock(&mutex);
    std::string data = send_data;
    get_called = true;
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&mutex);
    return data;
  }
  int send_idx;
  std::string send_data;
  pthread_mutex_t mutex;
  pthread_cond_t cond;
  bool send_called, get_called;
};

extern "C" {
void enable_process_replay_test_mode() {
  controlsd_state.pm = new FakePubMaster();
  controlsd_state.sm = new FakeSubMaster();
  controlsd_state.can_sock_raw = new FakeSubSocket();
}

void process_replay_set_sock_data(char *sock_name, char *data, int len) {
  FakeSubMaster *fsm = static_cast<FakeSubMaster *>(controlsd_state.sm);
  int sock_idx = -1;
  if(strcmp(sock_name, "thermal") == 0)
    sock_idx = SUB_SOCK_THERMAL;
  else if(strcmp(sock_name, "health") == 0)
    sock_idx = SUB_SOCK_HEALTH;
  else if(strcmp(sock_name, "liveCalibration") == 0)
    sock_idx = SUB_SOCK_LIVECALIBRATION;
  else if(strcmp(sock_name, "driverMonitoring") == 0)
    sock_idx = SUB_SOCK_DRIVERMONITORING;
  else if(strcmp(sock_name, "plan") == 0)
    sock_idx = SUB_SOCK_PLAN;
  else if(strcmp(sock_name, "pathPlan") == 0)
    sock_idx = SUB_SOCK_PATHPLAN;
  else if(strcmp(sock_name, "gpsLocation") == 0)
    sock_idx = SUB_SOCK_GPSLOCATION;
  assert(sock_idx != -1);
  fsm->update_sock_data(sock_idx, data, len);
}

void process_replay_wait_for_can_sock_recv() {
  FakeSubSocket *fss = static_cast<FakeSubSocket *>(controlsd_state.can_sock_raw);
  fss->wait_for_recv();
}

void process_replay_set_can_sock_data(unsigned char *data, int len) {
  FakeSubSocket *fss = static_cast<FakeSubSocket *>(controlsd_state.can_sock_raw);
  fss->set_data(data, len);
}

void process_replay_update_msgs(double cur_time) {
  FakeSubMaster *fsm = static_cast<FakeSubMaster *>(controlsd_state.sm);
  fsm->update_msgs(cur_time);
}

int64_t process_replay_cur_submaster_frame() {
  FakeSubMaster *fsm = static_cast<FakeSubMaster *>(controlsd_state.sm);
  return fsm->frame;
}

void process_replay_wait_for_msg(char *out_buf, int *out_len, int max_out_len) {
  FakePubMaster *fpm = static_cast<FakePubMaster *>(controlsd_state.pm);
  std::string data = fpm->wait_for_msg();
  assert(max_out_len >= data.length());
  memcpy(out_buf, data.data(), data.length());
  *out_len = data.length();
}

void *test_running_stat_filter_init(int64_t max_trackable) {
  RunningStatFilter * rsf = new RunningStatFilter(NULL, NULL, max_trackable);
  return (void *)rsf;
}

void test_running_stat_filter_push_and_update(void *inst, double new_data) {
  RunningStatFilter *rsf = static_cast<RunningStatFilter *>(inst);
  rsf->push_and_update(new_data);
}

void test_running_stat_filter_query(void *inst, double *out) {
  RunningStatFilter *rsf = static_cast<RunningStatFilter *>(inst);
  out[0] = rsf->filtered_stat.n;
  out[1] = rsf->filtered_stat.M;
  out[2] = rsf->filtered_stat.S;
  out[3] = rsf->filtered_stat.M_last;
  out[4] = rsf->filtered_stat.S_last;
  out[5] = rsf->filtered_stat.n;
}

static double compute_gb(double accel, double speed) {
  return accel / 3.0;
}

void *test_longcontrol_init(char *car_param_data, int car_param_data_len) {
  auto cp_msg = kj::heapArray<capnp::word>((car_param_data_len / sizeof(capnp::word)) + 1);
  memcpy(cp_msg.begin(), car_param_data, car_param_data_len);
  capnp::FlatArrayMessageReader cp_msg_reader(cp_msg);
  cereal::CarParams::Reader CP = cp_msg_reader.getRoot<cereal::CarParams>();

  LongControl *lc = new LongControl(CP, compute_gb);
  return (void *)lc;
}

void test_longcontrol_update(void *inst, char *car_param_data, int car_param_data_len, bool active, double v_ego,
                                bool brake_pressed, bool standstill, bool cruise_standstill, double v_cruise, double v_target,
                                double v_target_future, double a_target,
                                double *final_gas, double *final_brake, double *v_pid, double *p, double *i, double *f) {
  LongControl *lc = static_cast<LongControl *>(inst);
  auto cp_msg = kj::heapArray<capnp::word>((car_param_data_len / sizeof(capnp::word)) + 1);
  memcpy(cp_msg.begin(), car_param_data, car_param_data_len);
  capnp::FlatArrayMessageReader cp_msg_reader(cp_msg);
  cereal::CarParams::Reader CP = cp_msg_reader.getRoot<cereal::CarParams>();
  lc->update(active, v_ego, brake_pressed, standstill, cruise_standstill, v_cruise, v_target, v_target_future,
              a_target, CP, *final_gas, *final_brake);
  *v_pid = lc->v_pid;
  *p = lc->pid.p;
  *i = lc->pid.i;
  *f = lc->pid.f;
}

void *test_latcontrol_pid_init(unsigned char *car_param_data, int car_param_data_len) {
  auto cp_msg = kj::heapArray<capnp::word>((car_param_data_len / sizeof(capnp::word)) + 1);
  memcpy(cp_msg.begin(), car_param_data, car_param_data_len);
  capnp::FlatArrayMessageReader cp_msg_reader(cp_msg);
  cereal::CarParams::Reader CP = cp_msg_reader.getRoot<cereal::CarParams>();

  LatControlPID *lc = new LatControlPID(CP);
  return (void *)lc;
}

void test_latcontrol_pid_update(void *inst, unsigned char *car_param_data, int car_param_data_len,
                                unsigned char *pathplan_data, int pathplan_data_len,
                                bool active, double v_ego, double angle_steers, double angle_steers_rate, double eps_torque,
                                bool steer_override, double *output_steer_v, double *output_steering_angle,
                                bool *pid_active, float *pid_steer_angle, float *pid_steer_rate) {
  LatControlPID *lc = static_cast<LatControlPID *>(inst);
  auto cp_msg = kj::heapArray<capnp::word>((car_param_data_len / sizeof(capnp::word)) + 1);
  memcpy(cp_msg.begin(), car_param_data, car_param_data_len);
  capnp::FlatArrayMessageReader cp_msg_reader(cp_msg);
  cereal::CarParams::Reader CP = cp_msg_reader.getRoot<cereal::CarParams>();

  auto pp_msg = kj::heapArray<capnp::word>((pathplan_data_len / sizeof(capnp::word)) + 1);
  memcpy(pp_msg.begin(), pathplan_data, pathplan_data_len);
  capnp::FlatArrayMessageReader pp_msg_reader(pp_msg);
  auto pp_event = pp_msg_reader.getRoot<cereal::Event>();
  auto path_plan = pp_event.getPathPlan();

  capnp::MallocMessageBuilder lac_log_msg;
  auto lac_log_event = lac_log_msg.initRoot<cereal::Event>();
  lac_log_event.setLogMonoTime(nanos_since_boot());
  cereal::ControlsState::Builder lac_log_cs = lac_log_event.initControlsState();
  cereal::ControlsState::LateralControlState::Builder lac_log_cs_lcs = lac_log_cs.initLateralControlState();
  cereal::ControlsState::LateralPIDState::Builder pid_log = lac_log_cs_lcs.initPidState();
  lc->update(active, v_ego, angle_steers, angle_steers_rate, eps_torque, steer_override, CP, path_plan,
              *output_steer_v, *output_steering_angle, &pid_log);
  *pid_active = pid_log.getActive();
  *pid_steer_angle = pid_log.getSteerAngle();
  *pid_steer_rate = pid_log.getSteerRate();
}

void *test_latcontrol_lqr_init(unsigned char *car_param_data, int car_param_data_len) {
  auto cp_msg = kj::heapArray<capnp::word>((car_param_data_len / sizeof(capnp::word)) + 1);
  memcpy(cp_msg.begin(), car_param_data, car_param_data_len);
  capnp::FlatArrayMessageReader cp_msg_reader(cp_msg);
  cereal::CarParams::Reader CP = cp_msg_reader.getRoot<cereal::CarParams>();

  LatControlLQR *lc = new LatControlLQR(CP);
  return (void *)lc;
}

void test_latcontrol_lqr_update(void *inst, unsigned char *car_param_data, int car_param_data_len,
                                unsigned char *pathplan_data, int pathplan_data_len,
                                bool active, double v_ego, double angle_steers, double angle_steers_rate, double eps_torque,
                                bool steer_override, double *output_steer_v, double *output_steering_angle,
                                bool *lqr_active, float *lqr_steer_angle, float *lqr_output) {
  LatControlLQR *lc = static_cast<LatControlLQR *>(inst);
  auto cp_msg = kj::heapArray<capnp::word>((car_param_data_len / sizeof(capnp::word)) + 1);
  memcpy(cp_msg.begin(), car_param_data, car_param_data_len);
  capnp::FlatArrayMessageReader cp_msg_reader(cp_msg);
  cereal::CarParams::Reader CP = cp_msg_reader.getRoot<cereal::CarParams>();

  auto pp_msg = kj::heapArray<capnp::word>((pathplan_data_len / sizeof(capnp::word)) + 1);
  memcpy(pp_msg.begin(), pathplan_data, pathplan_data_len);
  capnp::FlatArrayMessageReader pp_msg_reader(pp_msg);
  auto pp_event = pp_msg_reader.getRoot<cereal::Event>();
  auto path_plan = pp_event.getPathPlan();

  capnp::MallocMessageBuilder lac_log_msg;
  auto lac_log_event = lac_log_msg.initRoot<cereal::Event>();
  lac_log_event.setLogMonoTime(nanos_since_boot());
  cereal::ControlsState::Builder lac_log_cs = lac_log_event.initControlsState();
  cereal::ControlsState::LateralControlState::Builder lac_log_cs_lcs = lac_log_cs.initLateralControlState();
  cereal::ControlsState::LateralLQRState::Builder pid_log = lac_log_cs_lcs.initLqrState();
  lc->update(active, v_ego, angle_steers, angle_steers_rate, eps_torque, steer_override, CP, path_plan,
              *output_steer_v, *output_steering_angle, &pid_log);
  *lqr_active = pid_log.getActive();
  *lqr_steer_angle = pid_log.getSteerAngle();
  *lqr_output = pid_log.getOutput();
}

void *test_canparser_init(const char *libdbc_fn, unsigned char *car_param_data, int car_param_data_len) {
  auto cp_msg = kj::heapArray<capnp::word>((car_param_data_len / sizeof(capnp::word)) + 1);
  memcpy(cp_msg.begin(), car_param_data, car_param_data_len);
  capnp::FlatArrayMessageReader cp_msg_reader(cp_msg);
  cereal::CarParams::Reader CP = cp_msg_reader.getRoot<cereal::CarParams>();

  return toyota::get_can_parser(libdbc_fn, CP);
}

void test_canparser_update_string(void *inst, unsigned char *can_event_data, int can_event_data_len, bool *can_valid,
                                  double *lka_state, double *lka_state_by_addr, uint16_t *lka_state_ts, uint16_t *lka_state_ts_by_addr) {
  CanParser *cp = static_cast<CanParser *>(inst);
  std::vector<std::string> strings;
  strings.push_back(std::string((const char *)can_event_data, can_event_data_len));
  cp->update_strings(strings);
  *can_valid = cp->can_valid;
  *lka_state = cp->vl["EPS_STATUS"]["LKA_STATE"];
  *lka_state_by_addr = cp->vl_addr[0x262]["LKA_STATE"];
  *lka_state_ts = cp->ts["EPS_STATUS"]["LKA_STATE"];
  *lka_state_ts_by_addr = cp->ts_addr[0x262]["LKA_STATE"];
}


}
