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
#include <sys/socket.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <assert.h>
#include <algorithm>

#include <zmq.h>
#include <capnp/serialize.h>
#include "cereal/gen/cpp/log.capnp.h"

#include "common/params.h"
#include "common/swaglog.h"
#include "common/timing.h"

#define min(x, y) ((x) <= (y) ? (x) : (y))

#define UBLOX_MSG_SIZE(hdr) (*(uint16_t *)&hdr[4])

namespace {
// protocol constants
const uint8_t PREAMBLE1 = 0xb5;
const uint8_t PREAMBLE2 = 0x62;

volatile int do_exit = 0; // Flag for process exit on signal
const long ZMQ_POLL_TIMEOUT = 1000; // In miliseconds
const int UBLOX_HEADER_SIZE = 8;
const int UBLOX_MAX_MSG_SIZE = 65536;
uint8_t msg_parse_buf[UBLOX_HEADER_SIZE + UBLOX_MAX_MSG_SIZE];
size_t bytes_in_parse_buf = 0U;

inline int needed_bytes() {
	if(bytes_in_parse_buf < UBLOX_HEADER_SIZE)
		return UBLOX_HEADER_SIZE - bytes_in_parse_buf;
	return UBLOX_MSG_SIZE(msg_parse_buf) + UBLOX_HEADER_SIZE - bytes_in_parse_buf;
}

inline bool valid_cheksum() {
	uint8_t ck_a = 0, ck_b = 0;
	for(int i = 2; i <= 5;i++) {
		ck_a = (ck_a + msg_parse_buf[i]) & 0xFF;
		ck_b = (ck_b + ck_a) & 0xFF;
	}
	if(ck_a != msg_parse_buf[6]) {
		LOGD("Checksum a mismtach: %u, %u", ck_a, msg_parse_buf[6]);
		return false;
	}
	if(ck_b != msg_parse_buf[7]) {
		LOGD("Checksum b mismtach: %u, %u", ck_b, msg_parse_buf[7]);
		return false;
	}
	return true;
}

inline bool valid() {
	return bytes_in_parse_buf >= 8 && valid_cheksum();
}

bool valid_so_far() {
	if(bytes_in_parse_buf > 0 && msg_parse_buf[0] != PREAMBLE1) {
		LOGD("PREAMBLE1 invalid, %u.", msg_parse_buf[0]);
		return false;
	}
	if(bytes_in_parse_buf > 1 && msg_parse_buf[1] != PREAMBLE2) {
		LOGD("PREAMBLE2 invalid, %u.", msg_parse_buf[1]);
		return false;
	}
	if(needed_bytes() == 0 && !valid())
		return false;
	return true;
}

size_t msg_parser_handle_data(const uint8_t *incoming_data, uint16_t incoming_data_len, void *publisher) {
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
	LOGD("ublox msg size: %u", UBLOX_MSG_SIZE(msg_parse_buf));
	if(bytes_in_parse_buf == UBLOX_MSG_SIZE(msg_parse_buf) + UBLOX_HEADER_SIZE) {
		LOGD("ublox msg total size: %u", bytes_in_parse_buf);
		#if 0
		capnp::MallocMessageBuilder msg;
		cereal::Event::Builder event = msg.initRoot<cereal::Event>();
		event.setLogMonoTime(nanos_since_boot());
		auto flexRayData = event.initFlexRay(1);
		flexRayData[0].setFrameId(EXTRACT_PACKET_FLAG_FRAME_ID(ntohs(pkt_hdr->flags)));
		flexRayData[0].setDat(kj::arrayPtr((uint8_t *)(pkt_hdr + 1), payload_len));
		auto words = capnp::messageToFlatArray(msg);
		auto bytes = words.asBytes();
		zmq_send(publisher, bytes.begin(), bytes.size(), 0);
		#endif
		bytes_in_parse_buf = 0;
	}
	return bytes_consumed;
}

void ublox_parse_and_send(void *publisher) {
  // ubloxRaw = 8042
  void *context = zmq_ctx_new();
  void *subscriber = zmq_socket(context, ZMQ_SUB);
  zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);
  zmq_connect(subscriber, "tcp://127.0.0.1:8042");

  while (!do_exit) {
		int err;
		zmq_pollitem_t item = {.socket = subscriber, .events = ZMQ_POLLIN};
		err = zmq_poll (&item, 1, ZMQ_POLL_TIMEOUT);
		if(err < 0) {
			LOGE_100("zmq_poll error %s in %s", strerror(errno ), __FUNCTION__);
			return;
		} else if(err == 0) {
			return;
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
		LOGD("ublox raw data: %u", len);
		int bytes_consumed = 0;
		while(bytes_consumed < len && !do_exit)
			bytes_consumed += msg_parser_handle_data(data + bytes_consumed, len - bytes_consumed, publisher);
		zmq_msg_close(&msg);
  }
	zmq_close(subscriber);
	zmq_ctx_destroy(context);
  return;
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
  LOG("setpriority returns %d", err);
  void *context = zmq_ctx_new();
  void *publisher = zmq_socket(context, ZMQ_PUB);
  // flexRay = 8066
  zmq_bind(publisher, "tcp://*:8066");
	// Do connect & recv in main thread
	while(!do_exit) {
		ublox_parse_and_send(publisher);
	}
	zmq_close(publisher);
	zmq_ctx_destroy(context);
}
