#pragma once

#include <stdint.h>

// NAV_PVT
typedef struct __attribute__((packed)) {
  uint32_t iTOW;
  uint16_t year;
  int8_t month;
  int8_t day;
  int8_t hour;
  int8_t min;
  int8_t sec;
  int8_t valid;
  uint32_t tAcc;
  int32_t nano;
  int8_t fixType;
  int8_t flags;
  int8_t flags2;
  int8_t numSV;
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t hMSL;
  uint32_t hAcc;
  uint32_t vAcc;
  int32_t velN;
  int32_t velE;
  int32_t velD;
  int32_t gSpeed;
  int32_t headMot;
  uint32_t sAcc;
  uint32_t headAcc;
  uint16_t pDOP;
  int8_t reserverd1[6];
  int32_t headVeh;
  int16_t magDec;
  uint16_t magAcc;
} nav_pvt_msg;

// RXM_RAW
typedef struct __attribute__((packed)) {
  double rcvTow;
  uint16_t week;
  int8_t leapS;
  int8_t numMeas;
  int8_t recStat;
  int8_t reserved1[3];
} rxm_raw_msg;

// Extra data count is in numMeas
typedef struct __attribute__((packed)) {
  double prMes;
  double cpMes;
  float doMes;
  int8_t gnssId;
  int8_t svId;
  int8_t sigId;
  int8_t freqId;
  uint16_t locktime;
  int8_t cno;
  int8_t prStdev;
  int8_t cpStdev;
  int8_t doStdev;
  int8_t trkStat;
  int8_t reserved3;
} rxm_raw_msg_extra;
// RXM_SFRBX
typedef struct __attribute__((packed)) {
  int8_t gnssId;
  int8_t svid;
  int8_t reserved1;
  int8_t freqId;
  int8_t numWords;
  int8_t reserved2;
  int8_t version;
  int8_t reserved3;
} rxm_sfrbx_msg;

// Extra data count is in numWords
typedef struct __attribute__((packed)) {
  uint32_t dwrd;
} rxm_sfrbx_msg_extra;
