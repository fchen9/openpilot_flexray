import os
import subprocess

from cffi import FFI

controlsd_dir = os.path.dirname(os.path.abspath(__file__))
libcontrolsd_cc_fn = os.path.join(controlsd_dir, "libcontrolsd_cc.so")
subprocess.check_call(["make", "-j4"], cwd=controlsd_dir)

ffi = FFI()
ffi.cdef("""
void init(int can_timeout);
int run(int internet_needed, int read_only, int sounds_available, unsigned char *car_param_data, int car_param_data_len,
        int car_recognized, int controller_available, const char *libdbc_fn);

/* Unit test utilities */
void enable_process_replay_test_mode();
void process_replay_wait_for_can_sock_recv();
void process_replay_set_can_sock_data(unsigned char *data, int len);
void process_replay_set_sock_data(char *sock_name, char *data, int len);
void process_replay_update_msgs(double cur_time);
int64_t process_replay_cur_submaster_frame();
void process_replay_wait_for_msg(char *out_buf, int *out_len, int max_out_len);

void *test_running_stat_filter_init(int64_t max_trackable);
void test_running_stat_filter_push_and_update(void *inst, double new_data);
void test_running_stat_filter_query(void *inst, double *out);

void *test_longcontrol_init(char *car_param_data, int car_param_data_len);
void test_longcontrol_update(void *inst, unsigned char *car_param_data, int car_param_data_len, bool active, double v_ego,
                                bool brake_pressed, bool standstill, bool cruise_standstill, double v_cruise, double v_target,
                                double v_target_future, double a_target, double *final_gas, double *final_brake,
                                double *v_pid, double *p, double *i, double *f);
void *test_latcontrol_pid_init(char *car_param_data, int car_param_data_len);
void test_latcontrol_pid_update(void *inst, unsigned char *car_param_data, int car_param_data_len,
                                unsigned char *pathplan_data, int pathplan_data_len,
                                bool active, double v_ego, double angle_steers, double angle_steers_rate, double eps_torque,
                                bool steer_override, double *output_steer_v, double *output_steering_angle,
                                bool *pid_active, float *pid_steer_angle, float *pid_steer_rate);                
void *test_latcontrol_lqr_init(char *car_param_data, int car_param_data_len);
void test_latcontrol_lqr_update(void *inst, unsigned char *car_param_data, int car_param_data_len,
                                unsigned char *pathplan_data, int pathplan_data_len,
                                bool active, double v_ego, double angle_steers, double angle_steers_rate, double eps_torque,
                                bool steer_override, double *output_steer_v, double *output_steering_angle,
                                bool *pid_active, float *pid_steer_angle, float *pid_output);

void *test_canparser_init(const char *libdbc_fn, unsigned char *car_param_data, int car_param_data_len);                                                
void test_canparser_update_string(void *inst, unsigned char *can_event_data, int can_event_data_len, bool *can_valid,
                                  double *lka_state, double *lka_state_by_addr, uint16_t *lka_state_ts, uint16_t *lka_state_ts_by_addr);

""")

libcontrolsd_cc = ffi.dlopen(libcontrolsd_cc_fn)
