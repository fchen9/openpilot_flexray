#include <stdint.h>
#include <map>
#include <vector>
#include <tuple>
#include <string>

#include "cereal/gen/cpp/car.capnp.h"
#include "can_define.h"
#include "canpacker.h"
#include "drive_helpers.h"
#include "carinterface.h"
#include "toyota/toyota_values.h"

namespace toyota {

// addr, (ecu, cars, bus, 1/freq*100, vl)
const std::vector<std::tuple<uint32_t, ECU, std::vector<std::string>, uint16_t, int64_t, std::string>> STATIC_MSGS = {
  {0x128, ECU::DSU, std::vector<std::string>({CAR_PRIUS, CAR_RAV4H, CAR_LEXUS_RXH, CAR_RAV4, CAR_COROLLA, CAR_AVALON}), 1,   3, "\xf4\x01\x90\x83\x00\x37"},
  {0x128, ECU::DSU, std::vector<std::string>({CAR_HIGHLANDER, CAR_HIGHLANDERH, CAR_SIENNA}), 1,   3, "\x03\x00\x20\x00\x00\x52"},
  {0x141, ECU::DSU, std::vector<std::string>({CAR_PRIUS, CAR_RAV4H, CAR_LEXUS_RXH, CAR_RAV4, CAR_COROLLA, CAR_HIGHLANDER, CAR_HIGHLANDERH, CAR_AVALON, CAR_SIENNA}), 1,   2, "\x00\x00\x00\x46"},
  {0x160, ECU::DSU, std::vector<std::string>({CAR_PRIUS, CAR_RAV4H, CAR_LEXUS_RXH, CAR_RAV4, CAR_COROLLA, CAR_HIGHLANDER, CAR_HIGHLANDERH, CAR_AVALON, CAR_SIENNA}), 1,   7, "\x00\x00\x08\x12\x01\x31\x9c\x51"},
  {0x161, ECU::DSU, std::vector<std::string>({CAR_PRIUS, CAR_RAV4H, CAR_LEXUS_RXH, CAR_RAV4, CAR_COROLLA, CAR_AVALON}), 1,   7, "\x00\x1e\x00\x00\x00\x80\x07"},
  {0x161, ECU::DSU, std::vector<std::string>({CAR_HIGHLANDERH, CAR_HIGHLANDER, CAR_SIENNA}), 1,  7, "\x00\x1e\x00\xd4\x00\x00\x5b"},
  {0x283, ECU::DSU, std::vector<std::string>({CAR_PRIUS, CAR_RAV4H, CAR_LEXUS_RXH, CAR_RAV4, CAR_COROLLA, CAR_HIGHLANDER, CAR_HIGHLANDERH, CAR_AVALON, CAR_SIENNA}), 0,   3, "\x00\x00\x00\x00\x00\x00\x8c"},
  {0x2E6, ECU::DSU, std::vector<std::string>({CAR_PRIUS, CAR_RAV4H, CAR_LEXUS_RXH}), 0,   3, "\xff\xf8\x00\x08\x7f\xe0\x00\x4e"},
  {0x2E7, ECU::DSU, std::vector<std::string>({CAR_PRIUS, CAR_RAV4H, CAR_LEXUS_RXH}), 0,   3, "\xa8\x9c\x31\x9c\x00\x00\x00\x02"},
  {0x33E, ECU::DSU, std::vector<std::string>({CAR_PRIUS, CAR_RAV4H, CAR_LEXUS_RXH}), 0,  20, "\x0f\xff\x26\x40\x00\x1f\x00"},
  {0x344, ECU::DSU, std::vector<std::string>({CAR_PRIUS, CAR_RAV4H, CAR_LEXUS_RXH, CAR_RAV4, CAR_COROLLA, CAR_HIGHLANDER, CAR_HIGHLANDERH, CAR_AVALON, CAR_SIENNA}), 0,   5, "\x00\x00\x01\x00\x00\x00\x00\x50"},
  {0x365, ECU::DSU, std::vector<std::string>({CAR_PRIUS, CAR_LEXUS_RXH, CAR_HIGHLANDERH}), 0,  20, "\x00\x00\x00\x80\x03\x00\x08"},
  {0x365, ECU::DSU, std::vector<std::string>({CAR_RAV4, CAR_RAV4H, CAR_COROLLA, CAR_HIGHLANDER, CAR_AVALON, CAR_SIENNA}), 0,  20, "\x00\x00\x00\x80\xfc\x00\x08"},
  {0x366, ECU::DSU, std::vector<std::string>({CAR_PRIUS, CAR_RAV4H, CAR_LEXUS_RXH, CAR_HIGHLANDERH}), 0,  20, "\x00\x00\x4d\x82\x40\x02\x00"},
  {0x366, ECU::DSU, std::vector<std::string>({CAR_RAV4, CAR_COROLLA, CAR_HIGHLANDER, CAR_AVALON, CAR_SIENNA}), 0,  20, "\x00\x72\x07\xff\x09\xfe\x00"},
  {0x470, ECU::DSU, std::vector<std::string>({CAR_PRIUS, CAR_LEXUS_RXH}), 1, 100, "\x00\x00\x02\x7a"},
  {0x470, ECU::DSU, std::vector<std::string>({CAR_HIGHLANDER, CAR_HIGHLANDERH, CAR_RAV4H, CAR_SIENNA}), 1,  100, "\x00\x00\x01\x79"},
  {0x4CB, ECU::DSU, std::vector<std::string>({CAR_PRIUS, CAR_RAV4H, CAR_LEXUS_RXH, CAR_RAV4, CAR_COROLLA, CAR_HIGHLANDERH, CAR_HIGHLANDER, CAR_AVALON, CAR_SIENNA}), 0, 100, "\x0c\x00\x00\x00\x00\x00\x00\x00"},

  {0x292, ECU::APGS, std::vector<std::string>({CAR_PRIUS}), 0,   3, "\x00\x00\x00\x00\x00\x00\x00\x9e"},
  {0x32E, ECU::APGS, std::vector<std::string>({CAR_PRIUS}), 0,  20, "\x00\x00\x00\x00\x00\x00\x00\x00"},
  {0x396, ECU::APGS, std::vector<std::string>({CAR_PRIUS}), 0, 100, "\xBD\x00\x00\x00\x60\x0F\x02\x00"},
  {0x43A, ECU::APGS, std::vector<std::string>({CAR_PRIUS}), 0, 100, "\x84\x00\x00\x00\x00\x00\x00\x00"},
  {0x43B, ECU::APGS, std::vector<std::string>({CAR_PRIUS}), 0, 100, "\x00\x00\x00\x00\x00\x00\x00\x00"},
  {0x497, ECU::APGS, std::vector<std::string>({CAR_PRIUS}), 0, 100, "\x00\x00\x00\x00\x00\x00\x00\x00"},
  {0x4CC, ECU::APGS, std::vector<std::string>({CAR_PRIUS}), 0, 100, "\x0D\x00\x00\x00\x00\x00\x00\x00"},
};


const double STEER_THRESHOLD = 100;

std::map<std::string, std::map<std::string, std::string>> DBC = {
  {CAR_RAV4H, dbc_dict("toyota_rav4_hybrid_2017_pt_generated", "toyota_adas")},
  {CAR_RAV4, dbc_dict("toyota_rav4_2017_pt_generated", "toyota_adas")},
  {CAR_PRIUS, dbc_dict("toyota_prius_2017_pt_generated", "toyota_adas")},
  {CAR_COROLLA, dbc_dict("toyota_corolla_2017_pt_generated", "toyota_adas")},
  {CAR_LEXUS_RXH, dbc_dict("lexus_rx_hybrid_2017_pt_generated", "toyota_adas")},
  {CAR_CHR, dbc_dict("toyota_nodsu_pt_generated", "toyota_adas")},
  {CAR_CHRH, dbc_dict("toyota_nodsu_hybrid_pt_generated", "toyota_adas")},
  {CAR_CAMRY, dbc_dict("toyota_nodsu_pt_generated", "toyota_adas")},
  {CAR_CAMRYH, dbc_dict("toyota_camry_hybrid_2018_pt_generated", "toyota_adas")},
  {CAR_HIGHLANDER, dbc_dict("toyota_highlander_2017_pt_generated", "toyota_adas")},
  {CAR_HIGHLANDERH, dbc_dict("toyota_highlander_hybrid_2018_pt_generated", "toyota_adas")},
  {CAR_AVALON, dbc_dict("toyota_avalon_2017_pt_generated", "toyota_adas")},
  {CAR_RAV4_TSS2, dbc_dict("toyota_nodsu_pt_generated", "toyota_tss2_adas")},
  {CAR_COROLLA_TSS2, dbc_dict("toyota_nodsu_pt_generated", "toyota_tss2_adas")},
  {CAR_COROLLAH_TSS2, dbc_dict("toyota_nodsu_hybrid_pt_generated", "toyota_tss2_adas")},
  {CAR_LEXUS_ES_TSS2, dbc_dict("toyota_nodsu_pt_generated", "toyota_tss2_adas")},
  {CAR_LEXUS_ESH_TSS2, dbc_dict("toyota_nodsu_hybrid_pt_generated", "toyota_tss2_adas")},
  {CAR_SIENNA, dbc_dict("toyota_sienna_xle_2018_pt_generated", "toyota_adas")},
  {CAR_LEXUS_IS, dbc_dict("lexus_is_2018_pt_generated", "toyota_adas")},
  {CAR_LEXUS_CTH, dbc_dict("lexus_ct200h_2018_pt_generated", "toyota_adas")},
};

const std::vector<std::string> NO_DSU_CAR = {CAR_CHR, CAR_CHRH, CAR_CAMRY, CAR_CAMRYH, CAR_RAV4_TSS2, CAR_COROLLA_TSS2, CAR_COROLLAH_TSS2, CAR_LEXUS_ES_TSS2, CAR_LEXUS_ESH_TSS2};
const std::vector<std::string> TSS2_CAR = {CAR_RAV4_TSS2, CAR_COROLLA_TSS2, CAR_COROLLAH_TSS2, CAR_LEXUS_ES_TSS2, CAR_LEXUS_ESH_TSS2};
const std::vector<std::string> NO_STOP_TIMER_CAR = {CAR_RAV4H, CAR_HIGHLANDERH, CAR_HIGHLANDER, CAR_RAV4_TSS2, CAR_COROLLA_TSS2, CAR_COROLLAH_TSS2, CAR_LEXUS_ES_TSS2, CAR_LEXUS_ESH_TSS2, CAR_SIENNA};  // no resume button press required

}
