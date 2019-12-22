#ifndef CANPACKER_H_
#define CANPACKER_H_

class CANPacker {
public:
  CANPacker(const char *dbc_name, const char *libdbc_fn);
  uint64_t pack(uint32_t addr, const std::map<std::string, double> &values, int counter);
  inline uint64_t ReverseBytes(uint64_t x);
  can_frame make_can_msg(uint32_t addr, uint8_t bus, const std::map<std::string, double> &values, int counter=-1);
  can_frame make_can_msg(const std::string &name, uint8_t bus, const std::map<std::string, double> &values, int counter=-1);

  void *packer;
  std::map<std::string, std::tuple<int, int>> name_to_address_and_size;
  std::map<int, int> address_to_size;
  canpack_pack_vector_func canpack_pack_vector;
};

#endif
