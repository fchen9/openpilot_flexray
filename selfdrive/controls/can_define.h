#ifndef CAN_DEFINE_H_
#define CAN_DEFINE_H_

#include <dlfcn.h>
#include <string>
#include <map>
#include <sstream>
#include "common.h"

typedef struct {
	uint32_t address;
	uint16_t busTime;
	std::vector<uint8_t> dat;
	uint8_t src;
} can_frame;

typedef const DBC * (*dbc_lookup_func)(const char *dbc_name);
typedef void* (*canparser_init_func)(
                int abus, const std::string & dbc_name,
                std::vector<MessageParseOptions> &message_options,
                std::vector<SignalParseOptions> &signal_options);
typedef void (*canparser_update_string_func)(void* can, const std::string &data);
typedef void (*canparser_query_latest_func)(void* can, std::vector<SignalValue>&svs);
typedef bool (*canparser_can_valid_func)(void* can);

typedef void * (*canpack_init_func)(const char* dbc_name);
typedef uint64_t (*canpack_pack_vector_func)(void* inst, uint32_t address, const std::vector<SignalPackValue> &signals, int counter);

class CANDefine {
public:
  CANDefine(const char *dbc_name, const char *libdbc_fn);
public:
  std::map<std::string, std::map<std::string, std::map<int, std::string>>> dv;
  std::map<uint32_t, std::map<std::string, std::map<int, std::string>>> dv_addr;
  std::map<uint32_t, std::string> address_to_msg_name;
};

#endif
