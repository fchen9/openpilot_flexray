#include <dlfcn.h>
#include <stdint.h>
#include <map>
#include <string>
#include <unordered_set>
#include "can_define.h"
#include "canparser.h"

static const int CAN_INVALID_CNT = 5;

CanParser::CanParser(const char *libdbc_fn, const std::string &dbc_name, const signals_vector &signals, const checks_vector &checks, int bus) {
  can_valid = true;
  this->dbc_name = dbc_name;
  void *libdbc = dlopen(libdbc_fn, RTLD_LAZY);
  dbc_lookup_func dbc_lookup = (dbc_lookup_func)dlsym(libdbc, "dbc_lookup");
  const DBC *dbc = dbc_lookup(dbc_name.c_str());

  can_invalid_cnt = CAN_INVALID_CNT;

  for(size_t i = 0;i < dbc[0].num_msgs; i++) {
    const struct Msg &msg = dbc[0].msgs[i];
    msg_name_to_address[std::string(msg.name)] = msg.address;
    address_to_msg_name[msg.address] = std::string(msg.name);
    vl_addr[msg.address] = std::map<std::string, double>();
    vl[std::string(msg.name)] = std::map<std::string, double>();
    ts_addr[msg.address] = std::map<std::string, uint16_t>();
    ts[std::string(msg.name)] = std::map<std::string, uint16_t>();
  }

  // Convert message names into addresses
  // (string, string, double) -> (string, uint32_t, double)
  signals_vector_addr signals_addr;
  for(const auto &s: signals) {
    signals_addr.push_back(std::make_tuple(std::get<0>(s), msg_name_to_address[std::get<1>(s)], std::get<2>(s)));
  }
  checks_vector_addr checks_addr;
  for(const auto &c: checks)
    checks_addr.push_back(std::make_tuple(msg_name_to_address[std::get<0>(c)], std::get<1>(c)));

  std::vector<SignalParseOptions> signal_options_v;
  for(const auto &sig: signals_addr) {
    SignalParseOptions spo;
    spo.address = std::get<1>(sig);
    spo.name = std::get<0>(sig).c_str();
    spo.default_value = std::get<2>(sig);
    signal_options_v.push_back(spo);
  }
  std::map<uint32_t, double> message_options;
  for(auto &sig: signals_addr) {
    message_options[std::get<1>(sig)] = 0;
  }
  for(auto &c: checks_addr)
    message_options[std::get<0>(c)] = std::get<1>(c);

  std::vector<MessageParseOptions> message_options_v;
  for(const auto &mo: message_options) {
    MessageParseOptions mpo;
    mpo.address = mo.first;
    mpo.check_frequency = mo.second;
    message_options_v.push_back(mpo);
  }
  canparser_init_func canparser_init = (canparser_init_func)dlsym(libdbc, "canparser_init");
  canparser_update_string = (canparser_update_string_func)dlsym(libdbc, "canparser_update_string");
  canparser_query_latest = (canparser_query_latest_func)dlsym(libdbc, "canparser_query_latest");
  canparser_can_valid = (canparser_can_valid_func)dlsym(libdbc, "canparser_can_valid");
  can = canparser_init(bus, dbc_name, message_options_v, signal_options_v);
  std::unordered_set<uint32_t> updated_val;
  update_vl(updated_val);
}

void CanParser::update_vl(std::unordered_set<uint32_t> &updated_val) {
  std::vector<SignalValue> can_values;
  canparser_query_latest(can, can_values);
  bool valid = canparser_can_valid(can);
  // Update invalid flag
  can_invalid_cnt += 1;
  if(valid)
    can_invalid_cnt = 0;
  can_valid = can_invalid_cnt < CAN_INVALID_CNT;

  for(auto &cv: can_values) {
    vl_addr[cv.address][std::string(cv.name)] = cv.value;
    ts_addr[cv.address][std::string(cv.name)] = cv.ts;
    auto &sig_name = address_to_msg_name[cv.address];
    vl[sig_name][std::string(cv.name)] = cv.value;
    ts[sig_name][std::string(cv.name)] = cv.ts;
    updated_val.insert(cv.address);
  }
}

void CanParser::update_string(std::string &dat, std::unordered_set<uint32_t> &updated_val) {
  canparser_update_string(can, dat);
  update_vl(updated_val);
}

// Return value not used by car interface, so remove it.
std::unordered_set<uint32_t> CanParser::update_strings(std::vector<std::string> &strings) {
  std::unordered_set<uint32_t> updated_vals;
  for(auto&s: strings) {
    std::unordered_set<uint32_t> updated_val;
    update_string(s, updated_val);
    updated_vals.insert(updated_val.begin(), updated_val.end());
  }
  return updated_vals;
}

