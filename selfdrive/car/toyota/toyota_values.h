#ifndef TOYOTA_VALUES_H_
#define TOYOTA_VALUES_H_

#define CAR_PRIUS "TOYOTA PRIUS 2017"
#define CAR_RAV4H "TOYOTA RAV4 HYBRID 2017"
#define CAR_RAV4 "TOYOTA RAV4 2017"
#define CAR_COROLLA "TOYOTA COROLLA 2017"
#define CAR_LEXUS_RXH "LEXUS RX HYBRID 2017"
#define CAR_CHR "TOYOTA C-HR 2018"
#define CAR_CHRH "TOYOTA C-HR HYBRID 2018"
#define CAR_CAMRY "TOYOTA CAMRY 2018"
#define CAR_CAMRYH "TOYOTA CAMRY HYBRID 2018"
#define CAR_HIGHLANDER "TOYOTA HIGHLANDER 2017"
#define CAR_HIGHLANDERH "TOYOTA HIGHLANDER HYBRID 2018"
#define CAR_AVALON "TOYOTA AVALON 2016"
#define CAR_RAV4_TSS2 "TOYOTA RAV4 2019"
#define CAR_COROLLA_TSS2 "TOYOTA COROLLA TSS2 2019"
#define CAR_COROLLAH_TSS2 "TOYOTA COROLLA HYBRID TSS2 2019"
#define CAR_LEXUS_ES_TSS2 "LEXUS ES 2019"
#define CAR_LEXUS_ESH_TSS2 "LEXUS ES 300H 2019"
#define CAR_SIENNA "TOYOTA SIENNA XLE 2018"
#define CAR_LEXUS_IS "LEXUS IS300 2018"
#define CAR_LEXUS_CTH "LEXUS CT 200H 2018"

namespace toyota {

typedef enum {
  CAM = 0, // camera
  DSU = 1, // driving support unit
  APGS = 2 // advanced parking guidance system
} ECU;

// addr, (ecu, cars, bus, 1/freq*100, vl)
extern const std::vector<std::tuple<uint32_t, ECU, std::vector<std::string>, uint16_t, int64_t, std::string>> STATIC_MSGS;
extern const double STEER_THRESHOLD;

extern std::map<std::string, std::map<std::string, std::string>> DBC;

extern const std::vector<std::string> NO_DSU_CAR;
extern const std::vector<std::string> TSS2_CAR;
extern const std::vector<std::string> NO_STOP_TIMER_CAR;

}

namespace std {
  template<>
  struct hash<toyota::ECU> {
      inline size_t operator()(const toyota::ECU &pt) const {
          return std::hash<int>()((int)pt);
      }
  };
}

#endif