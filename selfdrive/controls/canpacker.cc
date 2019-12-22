#include <dlfcn.h>
#include <stdint.h>
#include <map>
#include <string>

#include "can_define.h"
#include "canpacker.h"

CANPacker::CANPacker(const char *dbc_name, const char *libdbc_fn) {
  void *libdbc = dlopen(libdbc_fn, RTLD_LAZY);
  canpack_init_func canpack_init = (canpack_init_func)dlsym(libdbc, "canpack_init");
  canpack_pack_vector = (canpack_pack_vector_func)dlsym(libdbc, "canpack_pack_vector");
  dbc_lookup_func dbc_lookup = (dbc_lookup_func)dlsym(libdbc, "dbc_lookup");
  packer = canpack_init(dbc_name);
  const DBC *dbc = dbc_lookup(dbc_name);
  for(size_t i=0; i < dbc[0].num_msgs;i++) {
    const struct Msg &msg = dbc[0].msgs[i];
    name_to_address_and_size[std::string(msg.name)] = std::make_tuple(msg.address, msg.size);
    address_to_size[msg.address] = msg.size;
  }
}

uint64_t CANPacker::pack(uint32_t addr, const std::map<std::string, double> &values, int counter) {
  std::vector<SignalPackValue> values_thing;
  for(auto &v: values) {
    SignalPackValue spv;
    spv.name = v.first.c_str();
    spv.value = v.second;
    values_thing.push_back(spv);
  }
  return canpack_pack_vector(packer, addr, values_thing, counter);
}

inline uint64_t CANPacker::ReverseBytes(uint64_t x) {
  return (((x & 0xff00000000000000ull) >> 56) |
         ((x & 0x00ff000000000000ull) >> 40) |
         ((x & 0x0000ff0000000000ull) >> 24) |
         ((x & 0x000000ff00000000ull) >> 8) |
         ((x & 0x00000000ff000000ull) << 8) |
         ((x & 0x0000000000ff0000ull) << 24) |
         ((x & 0x000000000000ff00ull) << 40) |
         ((x & 0x00000000000000ffull) << 56));
}

can_frame CANPacker::make_can_msg(uint32_t addr, uint8_t bus, const std::map<std::string, double> &values, int counter) {
  int size = address_to_size[addr];
  uint64_t val = pack(addr, values, counter);
  val = ReverseBytes(val);
  can_frame cf = {addr, 0, std::vector<uint8_t>((uint8_t *)&val, ((uint8_t *)&val) + size), bus};
  return cf;
}

can_frame CANPacker::make_can_msg(const std::string &name, uint8_t bus, const std::map<std::string, double> &values, int counter) {
  auto & as = name_to_address_and_size[name];
  uint32_t addr = std::get<0>(as);
  int size = std::get<1>(as);
  uint64_t val = pack(addr, values, counter);
  val = ReverseBytes(val);
  can_frame cf = {addr, 0, std::vector<uint8_t>((uint8_t *)&val, ((uint8_t *)&val) + size), bus};
  return cf;
}
