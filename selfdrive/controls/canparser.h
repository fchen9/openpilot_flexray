#ifndef CANPARSER_H_
#define CANPARSER_H_

#include <dlfcn.h>

typedef std::vector<std::tuple<std::string, std::string, double>> signals_vector;
typedef std::vector<std::tuple<std::string, uint32_t, double>> signals_vector_addr;
typedef std::vector<std::tuple<std::string, double>> checks_vector;
typedef std::vector<std::tuple<uint32_t, double>> checks_vector_addr;

class CanParser {
public:
  CanParser(const char *libdbc_fn, const std::string &dbc_name, const signals_vector &signals, const checks_vector &checks, int bus=0);

  void update_vl(std::unordered_set<uint32_t> &updated_val);
  void update_string(std::string &dat, std::unordered_set<uint32_t> &updated_val);
  std::unordered_set<uint32_t> update_strings(std::vector<std::string> &strings);

  bool can_valid;
  std::string dbc_name;
  void *can;
  canparser_update_string_func canparser_update_string;
  canparser_query_latest_func canparser_query_latest;
  canparser_can_valid_func canparser_can_valid;
  std::map<uint32_t, std::map<std::string, double>> vl_addr;
  std::map<uint32_t, std::map<std::string, uint16_t>> ts_addr;
  std::map<std::string, std::map<std::string, double>> vl;
  std::map<std::string, std::map<std::string, uint16_t>> ts;
  std::map<std::string, uint32_t> msg_name_to_address;
  std::map<uint32_t, std::string> address_to_msg_name;
  std::vector<SignalValue> can_values;
  int can_invalid_cnt;
};

#endif