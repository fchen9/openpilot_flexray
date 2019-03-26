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
#include <map>
#include <vector>

#include <zmq.h>
#include <capnp/serialize.h>
#include "cereal/gen/cpp/log.capnp.h"

#include "common/params.h"
#include "common/swaglog.h"
#include "common/timing.h"

#include "ublox_msg.h"

#define min(x, y) ((x) <= (y) ? (x) : (y))
#define UBLOX_MSG_SIZE(hdr) (*(uint16_t *)&hdr[4])
#define GET_FIELD_U(w, nb, pos) (((w) >> (pos)) & ((1<<(nb))-1))
#define GET_FIELD_S(w, nb, pos) (((int)((w) << (32-(nb)-(pos)))) >> (32-(nb)))

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

typedef std::map<uint8_t, std::vector<uint32_t>> subframes_map;
std::map<uint8_t, std::map<uint8_t, subframes_map>> nav_frame_buffer;

class EphemerisData {
	public:
		EphemerisData(uint8_t svId, subframes_map subframes) {
			this->svId = svId;
			uint32_t week_no = GET_FIELD_U(subframes[1][2+0], 10, 20);
			uint32_t t_gd = GET_FIELD_S(subframes[1][2+4], 8, 6);
			uint32_t iodc = (GET_FIELD_U(subframes[1][2+0], 2, 6) << 8) | GET_FIELD_U(
				subframes[1][2+5], 8, 22);

			uint32_t t_oc = GET_FIELD_U(subframes[1][2+5], 16, 6);
			uint32_t a_f2 = GET_FIELD_S(subframes[1][2+6], 8, 22);
			uint32_t a_f1 = GET_FIELD_S(subframes[1][2+6], 16, 6);
			uint32_t a_f0 = GET_FIELD_S(subframes[1][2+7], 22, 8);

			uint32_t c_rs = GET_FIELD_S(subframes[2][2+0], 16, 6);
			uint32_t delta_n = GET_FIELD_S(subframes[2][2+1], 16, 14);
			uint32_t m_0 = (GET_FIELD_S(subframes[2][2+1], 8, 6) << 24) | GET_FIELD_U(
				subframes[2][2+2], 24, 6);
			uint32_t c_uc = GET_FIELD_S(subframes[2][2+3], 16, 14);
			uint32_t e = (GET_FIELD_U(subframes[2][2+3], 8, 6) << 24) | GET_FIELD_U(subframes[2][2+4], 24, 6);
			uint32_t c_us = GET_FIELD_S(subframes[2][2+5], 16, 14);
			uint32_t a_powhalf = (GET_FIELD_U(subframes[2][2+5], 8, 6) << 24) | GET_FIELD_U(
				subframes[2][2+6], 24, 6);
			uint32_t t_oe = GET_FIELD_U(subframes[2][2+7], 16, 14);

			uint32_t c_ic = GET_FIELD_S(subframes[3][2+0], 16, 14);
			uint32_t omega_0 = (GET_FIELD_S(subframes[3][2+0], 8, 6) << 24) | GET_FIELD_U(
				subframes[3][2+1], 24, 6);
			uint32_t c_is = GET_FIELD_S(subframes[3][2+2], 16, 14);
			uint32_t i_0 = (GET_FIELD_S(subframes[3][2+2], 8, 6) << 24) | GET_FIELD_U(
				subframes[3][2+3], 24, 6);
			uint32_t c_rc = GET_FIELD_S(subframes[3][2+4], 16, 14);
			uint32_t w = (GET_FIELD_S(subframes[3][2+4], 8, 6) << 24) | GET_FIELD_U(subframes[3][5], 24, 6);
			uint32_t omega_dot = GET_FIELD_S(subframes[3][2+6], 24, 6);
			uint32_t idot = GET_FIELD_S(subframes[3][2+7], 14, 8);

			this->_rsvd1 = GET_FIELD_U(subframes[1][2+1], 23, 6);
			this->_rsvd2 = GET_FIELD_U(subframes[1][2+2], 24, 6);
			this->_rsvd3 = GET_FIELD_U(subframes[1][2+3], 24, 6);
			this->_rsvd4 = GET_FIELD_U(subframes[1][2+4], 16, 14);
			this->aodo = GET_FIELD_U(subframes[2][2+7], 5, 8);

			double gpsPi = 3.1415926535898;

			// now form variables in radians, meters and seconds etc
			this->Tgd = t_gd * pow(2, -31);
			this->A = pow(a_powhalf * pow(2, -19), 2.0);
			this->cic = c_ic * pow(2, -29);
			this->cis = c_is * pow(2, -29);
			this->crc = c_rc * pow(2, -5);
			this->crs = c_rs * pow(2, -5);
			this->cuc = c_uc * pow(2, -29);
			this->cus = c_us * pow(2, -29);
			this->deltaN = delta_n * pow(2, -43) * gpsPi;
			this->ecc = e * pow(2, -33);
			this->i0 = i_0 * pow(2, -31) * gpsPi;
			this->idot = idot * pow(2, -43) * gpsPi;
			this->M0 = m_0 * pow(2, -31) * gpsPi;
			this->omega = w * pow(2, -31) * gpsPi;
			this->omega_dot = omega_dot * pow(2, -43) * gpsPi;
			this->omega0 = omega_0 * pow(2, -31) * gpsPi;
			this->toe = t_oe * pow(2, 4);

			this->toc = t_oc * pow(2, 4);
			this->gpsWeek = week_no;
			this->af0 = a_f0 * pow(2, -31);
			this->af1 = a_f1 * pow(2, -43);
			this->af2 = a_f2 * pow(2, -55);

			uint32_t iode1 = GET_FIELD_U(subframes[2][2+0], 8, 22);
			uint32_t iode2 = GET_FIELD_U(subframes[3][2+7], 8, 22);
			this->valid = (iode1 == iode2) && (iode1 == (iodc & 0xff));
			this->iode = iode1;

			if (GET_FIELD_U(subframes[4][2+0], 6, 22) == 56 &&
				GET_FIELD_U(subframes[4][2+0], 2, 28) == 1 &&
				GET_FIELD_U(subframes[5][2+0], 2, 28) == 1) {
				uint32_t a0 = GET_FIELD_S(subframes[4][2], 8, 14) * pow(2, -30);
				uint32_t a1 = GET_FIELD_S(subframes[4][2], 8, 6) * pow(2, -27);
				uint32_t a2 = GET_FIELD_S(subframes[4][3], 8, 22) * pow(2, -24);
				uint32_t a3 = GET_FIELD_S(subframes[4][3], 8, 14) * pow(2, -24);
				uint32_t b0 = GET_FIELD_S(subframes[4][3], 8, 6) * pow(2, 11);
				uint32_t b1 = GET_FIELD_S(subframes[4][4], 8, 22) * pow(2, 14);
				uint32_t b2 = GET_FIELD_S(subframes[4][4], 8, 14) * pow(2, 16);
				uint32_t b3 = GET_FIELD_S(subframes[4][4], 8, 6) * pow(2, 16);
				this->ionoAlpha[0] = a0;this->ionoAlpha[1] = a1;this->ionoAlpha[2] = a2;this->ionoAlpha[3] = a3;
				this->ionoBeta[0] = b0;this->ionoBeta[1] = b1;this->ionoBeta[2] = b2;this->ionoBeta[3] = b3;
				this->ionoCoeffsValid = true;
			} else {
				this->ionoCoeffsValid = false;
			}
		}
		uint16_t svId;
		double Tgd, A, cic, cis, crc, crs, cuc, cus, deltaN, ecc, i0, idot, M0, omega, omega_dot, omega0, toe, toc;
		uint32_t gpsWeek, iode, _rsvd1, _rsvd2, _rsvd3, _rsvd4, aodo;
		double af0, af1, af2;
		bool valid;
		double ionoAlpha[4], ionoBeta[4];
		bool ionoCoeffsValid;
};

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
	gpsLoc.setSource(cereal::GpsLocationData::SensorSource::UBLOX);
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
	gpsLoc.setSpeedAccuracy(msg->sAcc * 1e-03);
	gpsLoc.setBearingAccuracy(msg->headAcc * 1e-05);
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

void publish_rxm_sfrbx(void *sock) {
	rxm_sfrbx_msg *msg = (rxm_sfrbx_msg *)&msg_parse_buf[UBLOX_HEADER_SIZE];
	if(bytes_in_parse_buf != 
		(UBLOX_HEADER_SIZE + sizeof(rxm_sfrbx_msg) + msg->numWords * sizeof(rxm_sfrbx_msg_extra) + UBLOX_CHECKSUM_SIZE)) {
		LOGD("Invalid sfrbx words size %u, %u, %u, %u", msg->numWords, bytes_in_parse_buf, sizeof(rxm_raw_msg_extra), sizeof(rxm_raw_msg));
		return;
	}
	rxm_sfrbx_msg_extra *measurements = (rxm_sfrbx_msg_extra *)&msg_parse_buf[UBLOX_HEADER_SIZE + sizeof(rxm_sfrbx_msg)];
	if(msg->gnssId  == 0) {
    uint8_t subframeId =  GET_FIELD_U(measurements[1].dwrd, 3, 8);
    std::vector<uint32_t> words;
    for(int i = 0; i < msg->numWords;i++)
      words.push_back(measurements[i].dwrd);

    if(subframeId == 1) {
      nav_frame_buffer[msg->gnssId][msg->svid] = subframes_map();
      nav_frame_buffer[msg->gnssId][msg->svid][subframeId] = words;
		} else if(nav_frame_buffer[msg->gnssId][msg->svid].find(subframeId-1) != nav_frame_buffer[msg->gnssId][msg->svid].end())
      nav_frame_buffer[msg->gnssId][msg->svid][subframeId] = words;
    
		if(nav_frame_buffer[msg->gnssId][msg->svid].size() == 5) {
      EphemerisData ephem_data(msg->svid, nav_frame_buffer[msg->gnssId][msg->svid]);
			capnp::MallocMessageBuilder msg_builder;
			cereal::Event::Builder event = msg_builder.initRoot<cereal::Event>();
			event.setLogMonoTime(nanos_since_boot());
			auto gnss = event.initUbloxGnss();
			auto eph = gnss.initEphemeris();
			eph.setSvId(ephem_data.svId);
			eph.setToc(ephem_data.toc);
			eph.setGpsWeek(ephem_data.gpsWeek);
			eph.setAf0(ephem_data.af0);
			eph.setAf1(ephem_data.af1);
			eph.setAf2(ephem_data.af2);
			eph.setIode(ephem_data.iode);
			eph.setCrs(ephem_data.crs);
			eph.setDeltaN(ephem_data.deltaN);
			eph.setM0(ephem_data.M0);
			eph.setCuc(ephem_data.cuc);
			eph.setEcc(ephem_data.ecc);
			eph.setCus(ephem_data.cus);
			eph.setA(ephem_data.A);
			eph.setToe(ephem_data.toe);
			eph.setCic(ephem_data.cic);
			eph.setOmega0(ephem_data.omega0);
			eph.setCis(ephem_data.cis);
			eph.setI0(ephem_data.i0);
			eph.setCrc(ephem_data.crc);
			eph.setOmega(ephem_data.omega);
			eph.setOmegaDot(ephem_data.omega_dot);
			eph.setIDot(ephem_data.idot);
			eph.setTgd(ephem_data.Tgd);
			eph.setIonoCoeffsValid(ephem_data.ionoCoeffsValid);
			kj::ArrayPtr<const double> apa(&ephem_data.ionoAlpha[0], sizeof(ephem_data.ionoAlpha) / sizeof(ephem_data.ionoAlpha[0]));
			eph.setIonoAlpha(apa);
			kj::ArrayPtr<const double> apb(&ephem_data.ionoBeta[0], sizeof(ephem_data.ionoBeta) / sizeof(ephem_data.ionoBeta[0]));
			eph.setIonoBeta(apb);
			auto words = capnp::messageToFlatArray(msg_builder);
			auto bytes = words.asBytes();
			zmq_send(sock, bytes.begin(), bytes.size(), 0);
		}
	}
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
				publish_rxm_sfrbx(ubloxGnss);
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

  nav_frame_buffer[0U] = std::map<uint8_t, subframes_map>();
  for(int i = 1;i < 33;i++)
    nav_frame_buffer[0U][i] = subframes_map();

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
