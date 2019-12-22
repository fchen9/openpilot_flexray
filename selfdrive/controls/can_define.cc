#include "can_define.h"

CANDefine::CANDefine(const char *dbc_name, const char *libdbc_fn) {
  void *libdbc = dlopen(libdbc_fn, RTLD_LAZY);
  dbc_lookup_func dbc_lookup = (dbc_lookup_func)dlsym(libdbc, "dbc_lookup");
  const DBC *dbc = dbc_lookup(dbc_name);

  for(size_t i = 0;i < dbc[0].num_msgs; i++)
    address_to_msg_name[dbc[0].msgs[i].address] = std::string(dbc[0].msgs[i].name);

  for(size_t i = 0;i < dbc[0].num_vals; i++) {
    const Val *val = &(dbc[0].vals[i]);

    std::string sgname(val->name);
    uint32_t address = val->address;
    std::string def_val(val->def_val);

    // separate definition/value pairs
    std::stringstream ss(def_val);
    std::string buf;

    std::vector<int> values;
    std::vector<std::string> defs;
    int idx = 0;
    while (ss >> buf) {
      if(idx % 2 == 0)
        values.push_back(std::stoi(buf));
      else
        defs.push_back(buf);
      idx++;
    }
    if(dv_addr.find(address) == dv_addr.end()) {
      dv_addr[address] = std::map<std::string, std::map<int, std::string>>();
      dv[address_to_msg_name[address]] = std::map<std::string, std::map<int, std::string>>();
    }
    // two ways to lookup: address or msg name
    dv_addr[address][sgname] = std::map<int, std::string>();
    dv[address_to_msg_name[address]][sgname] = std::map<int, std::string>();
    for(size_t j = 0;j < values.size();j++) {
      dv_addr[address][sgname][values[j]] = defs[j];
      dv[address_to_msg_name[address]][sgname][values[j]] = defs[j];
    }
  }
}
