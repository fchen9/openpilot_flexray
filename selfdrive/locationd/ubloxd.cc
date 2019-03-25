#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <sched.h>
#include <sys/time.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/time.h>
#include <assert.h>
#include <math.h>
#include <ctime>
#include <chrono>

#include <zmq.h>
#include <capnp/serialize.h>
#include "cereal/gen/cpp/log.capnp.h"

#include "common/params.h"
#include "common/swaglog.h"
#include "common/timing.h"

#include "ublox_msg.h"

#define min(x, y) ((x) <= (y) ? (x) : (y))

#define UBLOX_MSG_SIZE(hdr) (*(uint16_t *)&hdr[4])

namespace {
// protocol constants
const uint8_t PREAMBLE1 = 0xb5;
const uint8_t PREAMBLE2 = 0x62;

// message classes
const uint8_t CLASS_NAV = 0x01;
const uint8_t CLASS_RXM = 0x02;

// NAV messages
const uint8_t MSG_NAV_PVT = 0x7;

// RXM messages
const uint8_t MSG_RXM_RAW = 0x15;
const uint8_t MSG_RXM_SFRBX = 0x13;

volatile int do_exit = 0; // Flag for process exit on signal
const long ZMQ_POLL_TIMEOUT = 1000; // In miliseconds
const int UBLOX_HEADER_SIZE = 6;
const int UBLOX_CHECKSUM_SIZE = 2;
const int UBLOX_MAX_MSG_SIZE = 65536;
uint8_t msg_parse_buf[UBLOX_HEADER_SIZE + UBLOX_MAX_MSG_SIZE];
size_t bytes_in_parse_buf = 0U;

inline int needed_bytes() {
	if(bytes_in_parse_buf < UBLOX_HEADER_SIZE)
		return UBLOX_HEADER_SIZE + UBLOX_CHECKSUM_SIZE - bytes_in_parse_buf;
	return UBLOX_MSG_SIZE(msg_parse_buf) + UBLOX_HEADER_SIZE + UBLOX_CHECKSUM_SIZE - bytes_in_parse_buf;
}

inline bool valid_cheksum() {
	uint8_t ck_a = 0, ck_b = 0;
	for(int i = 2; i < bytes_in_parse_buf - UBLOX_CHECKSUM_SIZE;i++) {
		ck_a = (ck_a + msg_parse_buf[i]) & 0xFF;
		ck_b = (ck_b + ck_a) & 0xFF;
	}
	if(ck_a != msg_parse_buf[bytes_in_parse_buf - 2]) {
		LOGD("Checksum a mismtach: %02X, %02X", ck_a, msg_parse_buf[6]);
		return false;
	}
	if(ck_b != msg_parse_buf[bytes_in_parse_buf - 1]) {
		LOGD("Checksum b mismtach: %02X, %02X", ck_b, msg_parse_buf[7]);
		return false;
	}
	return true;
}

inline bool valid() {
	return bytes_in_parse_buf >= UBLOX_HEADER_SIZE + UBLOX_CHECKSUM_SIZE && 
		needed_bytes() == 0 &&
		valid_cheksum();
}

bool valid_so_far() {
	if(bytes_in_parse_buf > 0 && msg_parse_buf[0] != PREAMBLE1) {
		LOGD("PREAMBLE1 invalid, %02X.", msg_parse_buf[0]);
		std::exit(-1);
		return false;
	}
	if(bytes_in_parse_buf > 1 && msg_parse_buf[1] != PREAMBLE2) {
		LOGD("PREAMBLE2 invalid, %02X.", msg_parse_buf[1]);
		return false;
	}
	if(needed_bytes() == 0 && !valid())
		return false;
	return true;
}

inline uint8_t msg_class() {
	return msg_parse_buf[2];
}

inline uint8_t msg_id() {
	return msg_parse_buf[3];
}

void publish_nav_pvt(void *sock) {
	nav_pvt_msg *msg = (nav_pvt_msg *)&msg_parse_buf[UBLOX_HEADER_SIZE];
	capnp::MallocMessageBuilder msg_builder;
	cereal::Event::Builder event = msg_builder.initRoot<cereal::Event>();
	event.setLogMonoTime(nanos_since_boot());
	auto gpsLoc = event.initGpsLocationExternal();
	gpsLoc.setSource(cereal::GpsLocationData::SensorSource::EXTERNAL);
	gpsLoc.setFlags(msg->flags);
	gpsLoc.setLatitude(msg->lat * 1e-07);
	gpsLoc.setLongitude(msg->lon * 1e-07);
	gpsLoc.setAltitude(msg->height * 1e-03);
	gpsLoc.setSpeed(msg->gSpeed * 1e-03);
	gpsLoc.setBearing(msg->headMot * 1e-5);
	gpsLoc.setAccuracy(msg->hAcc * 1e-03);
	std::tm timeinfo = std::tm();
  timeinfo.tm_year = msg->year - 1900;
  timeinfo.tm_mon = msg->month - 1;
  timeinfo.tm_mday = msg->day;
	timeinfo.tm_hour = msg->hour;
	timeinfo.tm_min = msg->min;
	timeinfo.tm_sec = msg->sec;
	std::time_t utc_tt = timegm(&timeinfo);
	gpsLoc.setTimestamp(utc_tt * 1e+03 + msg->nano * 1e-06);
	float f[] = { msg->velN * 1e-03f, msg->velE * 1e-03f, msg->velD * 1e-03f };
	kj::ArrayPtr<const float> ap(&f[0], sizeof(f) / sizeof(f[0]));
	gpsLoc.setVNED(ap);
	gpsLoc.setVerticalAccuracy(msg->vAcc * 1e-03);
	gpsLoc.setBearingAccuracy(msg->sAcc * 1e-03);
	gpsLoc.setSpeedAccuracy(msg->headAcc * 1e-05);
	auto words = capnp::messageToFlatArray(msg_builder);
	auto bytes = words.asBytes();
	zmq_send(sock, bytes.begin(), bytes.size(), 0);
}

inline bool bit_to_bool(uint8_t val, int shifts) {
	return (val & (1 << shifts)) ? true : false;
}
void publish_rxm_raw(void *sock) {
	rxm_raw_msg *msg = (rxm_raw_msg *)&msg_parse_buf[UBLOX_HEADER_SIZE];
	if(bytes_in_parse_buf != 
		(UBLOX_HEADER_SIZE + sizeof(rxm_raw_msg) + msg->numMeas * sizeof(rxm_raw_msg_extra) + UBLOX_CHECKSUM_SIZE)) {
		LOGD("Invalid measurement size %u, %u, %u, %u", msg->numMeas, bytes_in_parse_buf, sizeof(rxm_raw_msg_extra), sizeof(rxm_raw_msg));
		return;
	}
	rxm_raw_msg_extra *measurements = (rxm_raw_msg_extra *)&msg_parse_buf[UBLOX_HEADER_SIZE + sizeof(rxm_raw_msg)];
	capnp::MallocMessageBuilder msg_builder;
	cereal::Event::Builder event = msg_builder.initRoot<cereal::Event>();
	event.setLogMonoTime(nanos_since_boot());
	auto gnss = event.initUbloxGnss();
	auto mr = gnss.initMeasurementReport();
	mr.setRcvTow(msg->rcvTow);
	mr.setGpsWeek(msg->week);
	mr.setLeapSeconds(msg->leapS);
	mr.setGpsWeek(msg->week);
	auto mb = mr.initMeasurements(msg->numMeas);
	for(int8_t i = 0; i < msg->numMeas; i++) {
		mb[i].setSvId(measurements[i].svId);
		mb[i].setSigId(measurements[i].sigId);
		mb[i].setPseudorange(measurements[i].prMes);
		mb[i].setCarrierCycles(measurements[i].cpMes);
		mb[i].setDoppler(measurements[i].doMes);
		mb[i].setGnssId(measurements[i].gnssId);
		mb[i].setGlonassFrequencyIndex(measurements[i].freqId);
		mb[i].setLocktime(measurements[i].locktime);
		mb[i].setCno(measurements[i].cno);
		mb[i].setPseudorangeStdev(0.01*(pow(2, (measurements[i].prStdev & 15)))); // weird scaling, might be wrong
		mb[i].setCarrierPhaseStdev(0.004*(measurements[i].cpStdev & 15));
		mb[i].setDopplerStdev(0.002*(pow(2, (measurements[i].doStdev & 15)))); // weird scaling, might be wrong
		auto ts = mb[i].initTrackingStatus();
		ts.setPseudorangeValid(bit_to_bool(measurements[i].trkStat, 0));
		ts.setCarrierPhaseValid(bit_to_bool(measurements[i].trkStat, 1));
		ts.setHalfCycleValid(bit_to_bool(measurements[i].trkStat, 2));
		ts.setHalfCycleSubtracted(bit_to_bool(measurements[i].trkStat, 3));
	}

	mr.setNumMeas(msg->numMeas);
	auto rs = mr.initReceiverStatus();
	rs.setLeapSecValid(bit_to_bool(msg->recStat, 0));
	rs.setClkReset(bit_to_bool(msg->recStat, 2));
	auto words = capnp::messageToFlatArray(msg_builder);
	auto bytes = words.asBytes();
	zmq_send(sock, bytes.begin(), bytes.size(), 0);
}

size_t msg_parser_handle_data(const uint8_t *incoming_data, uint16_t incoming_data_len, void *gpsLocationExternal, void *ubloxGnss) {
	size_t bytes_consumed = min(needed_bytes(), incoming_data_len );
	memcpy(msg_parse_buf + bytes_in_parse_buf, incoming_data, bytes_consumed);
	bytes_in_parse_buf += bytes_consumed;
	// Handle corrupted stream
	while(!valid_so_far() && bytes_in_parse_buf != 0) {
		bytes_in_parse_buf -= 1;
		if(bytes_in_parse_buf > 0)
			memmove(&msg_parse_buf[0], &msg_parse_buf[1], bytes_in_parse_buf);
	}

	if(bytes_in_parse_buf < UBLOX_HEADER_SIZE)
		return bytes_consumed;
	if(valid()) {
		LOGD("ublox msg total size: %u", bytes_in_parse_buf);
		if(msg_class() == CLASS_NAV) {
			if(msg_id() == MSG_NAV_PVT) {
				LOGD("MSG_NAV_PVT");
				publish_nav_pvt(gpsLocationExternal);
			} else
				LOGW("Unknown nav msg id: 0x%02X", msg_id());
		} else if(msg_class() == CLASS_RXM) {
			if(msg_id() == MSG_RXM_RAW) {
				LOGD("MSG_RXM_RAW");
				publish_rxm_raw(ubloxGnss);
			} else if(msg_id() == MSG_RXM_SFRBX) {
				LOGD("MSG_RXM_SFRBX");
			} else
				LOGW("Unknown rxm msg id: 0x%02X", msg_id());
		} else
			LOGW("Unknown msg class: 0x%02X", msg_class());
		bytes_in_parse_buf = 0;
	}
	return bytes_consumed;
}

void set_do_exit(int sig) {
  do_exit = 1;
}

}

int main() {
  int err;
  LOGW("starting ubloxd");
	signal(SIGINT, (sighandler_t) set_do_exit);
  signal(SIGTERM, (sighandler_t) set_do_exit);
  void *context = zmq_ctx_new();

  void *gpsLocationExternal = zmq_socket(context, ZMQ_PUB);
  zmq_bind(gpsLocationExternal, "tcp://*:9032");
	//zmq_bind(gpsLocationExternal, "tcp://*:8032");
  void *ubloxGnss = zmq_socket(context, ZMQ_PUB);
	zmq_bind(ubloxGnss, "tcp://*:9033");
  //zmq_bind(ubloxGnss, "tcp://*:8033");
  // ubloxRaw = 8042

  void *subscriber = zmq_socket(context, ZMQ_SUB);
  zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);
  zmq_connect(subscriber, "tcp://127.0.0.1:8042");
  while (!do_exit) {
		int err;
		zmq_pollitem_t item = {.socket = subscriber, .events = ZMQ_POLLIN};
		err = zmq_poll (&item, 1, ZMQ_POLL_TIMEOUT);
		if(err < 0) {
			LOGE_100("zmq_poll error %s in %s", strerror(errno ), __FUNCTION__);
			break;
		} else if(err == 0) {
			continue;
		}
		zmq_msg_t msg;
		zmq_msg_init(&msg);
		err = zmq_msg_recv(&msg, subscriber, 0);
		assert(err >= 0);
		// format for board, make copy due to alignment issues, will be freed on out of scope
		auto amsg = kj::heapArray<capnp::word>((zmq_msg_size(&msg) / sizeof(capnp::word)) + 1);
		memcpy(amsg.begin(), zmq_msg_data(&msg), zmq_msg_size(&msg));
		capnp::FlatArrayMessageReader cmsg(amsg);
		cereal::Event::Reader event = cmsg.getRoot<cereal::Event>();
		const uint8_t *data = event.getUbloxRaw().begin();
		size_t len = event.getUbloxRaw().size();
		int bytes_consumed = 0;
		while(bytes_consumed < len && !do_exit)
			bytes_consumed += msg_parser_handle_data(data + bytes_consumed, len - bytes_consumed, gpsLocationExternal, ubloxGnss);
		zmq_msg_close(&msg);
  }
	zmq_close(subscriber);
	zmq_close(gpsLocationExternal);
	zmq_close(ubloxGnss);
	zmq_ctx_destroy(context);
}
