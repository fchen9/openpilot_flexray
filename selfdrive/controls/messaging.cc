#include <vector>
#include <string>
#include <map>
#include <pthread.h>
#include <zmq.h>
#include <capnp/serialize.h>

#include "cereal/gen/cpp/car.capnp.h"
#include "cereal/gen/cpp/log.capnp.h"
#include "common/swaglog.h"
#include "messaging.h"

static void *sub_sock(void *ctx, const char *endpoint) {
  void* sock = zmq_socket(ctx, ZMQ_SUB);
  assert(sock);
  zmq_setsockopt(sock, ZMQ_SUBSCRIBE, "", 0);
  int reconnect_ivl = 500;
  zmq_setsockopt(sock, ZMQ_RECONNECT_IVL_MAX, &reconnect_ivl, sizeof(reconnect_ivl));
  zmq_connect(sock, endpoint);
  return sock;
}

SubSocketBase::SubSocketBase(): sock(nullptr) {

}

void SubSocketBase::set_timeout(int can_timeout) {
}

SubSocket::SubSocket(void *ctx, const char *endpoint) {
  sock = sub_sock(ctx, endpoint);
}

void SubSocket::set_timeout(int can_timeout) {
  zmq_setsockopt(sock, ZMQ_RCVTIMEO, &can_timeout, sizeof(can_timeout));
}

void SubSocket::receive(std::vector<std::string> &can_strs) {
  zmq_msg_t msg;
  zmq_msg_init(&msg);
  int err = zmq_msg_recv(&msg, sock, 0);
  if(err <= 0) {
    if(err < 0 and errno != EAGAIN) {
      LOGW("zmq_msg_recv failed: %d, %d, %s", err, errno, strerror(errno));
    }
    return;
  } else {
    can_strs.push_back(std::string((char *)zmq_msg_data(&msg), zmq_msg_size(&msg)));
    while(1) {
      zmq_msg_init(&msg);
      int err = zmq_msg_recv(&msg, sock, ZMQ_DONTWAIT);
      if(err <= 0)
        break;
      can_strs.push_back(std::string((char *)zmq_msg_data(&msg), zmq_msg_size(&msg)));
    }
  }
}

SubMasterBase::SubMasterBase():frame(-1) {
}

void SubMasterBase::add_sock(void *ctx, const char *addr, float freq, bool valid, bool b_ignore_alive, capnp::MallocMessageBuilder &msg) {
  this->socks.push_back(sub_sock(ctx, addr));
  this->freq.push_back(freq);
  this->valid.push_back(valid);
  this->alive.push_back(false);
  this->updated.push_back(false);
  this->ignore_alive.push_back(b_ignore_alive);
  this->rcv_frame.push_back(frame);
  this->rcv_time.push_back(0);
  this->logMonoTime.push_back(0);
  auto words = capnp::messageToFlatArray(msg);
  auto bytes = words.asBytes();
  this->msg_bufs.push_back(kj::heapArray<capnp::word>((bytes.size() / sizeof(capnp::word)) + 1));
  memcpy(this->msg_bufs[this->msg_bufs.size() - 1].begin(), bytes.begin(), bytes.size());
}

void SubMasterBase::update_msgs(double cur_time, std::vector<int> sock_ids) {
  frame += 1;
  for(auto it = updated.begin(); it != updated.end();it++)
    *it = false;
  for(int i: sock_ids) {
    capnp::FlatArrayMessageReader cmsg(msg_bufs[i]);
    cereal::Event::Reader event = cmsg.getRoot<cereal::Event>();
    updated[i] = true;
    rcv_frame[i] = frame;
    rcv_time[i] = cur_time;
    logMonoTime[i] = event.getLogMonoTime();
    valid[i] = event.getValid();
  }
  for (int i=0; i < socks.size(); i++) {
    // arbitrary small number to avoid float comparison. If freq is 0, we can skip the check
    if(freq[i] > 1e-5)
      // alive if delay is within 10x the expected frequency
      alive[i] = (cur_time - rcv_time[i]) < (10. / freq[i]);
    else
      alive[i] = true;
  }
}

bool SubMasterBase::all_alive_and_valid() {
  for (int i=0; i < socks.size(); i++) {
    if(!valid[i]) return false;
    if(!alive[i] and !ignore_alive[i]) return false;
  }
  return true;
}

SubMaster::SubMaster() {
}

void SubMaster::update(int timeout) {
  int err;
  // peek and consume all events in the zmq queue, then return.
  std::vector<zmq_pollitem_t> polls;
  for(auto sock : socks) {
    zmq_pollitem_t pi;
    pi.socket = sock;
    pi.events = ZMQ_POLLIN;
    polls.push_back(pi);
  }

  int ret = zmq_poll(polls.data(), polls.size(), ZMQ_DONTWAIT);
  if (ret < 0) {
    LOGW("poll failed (%d)", ret);
    return;
  }
  if (ret == 0)
    return;
  double cur_time = seconds_since_boot();
  std::vector<int> updated_sock_ids;
  // zmq messages
  for (int i=0; i < polls.size(); i++) {
    if (polls[i].revents) {
      zmq_msg_t msg;
      zmq_msg_init(&msg);
      err = zmq_msg_recv(&msg, polls[i].socket, 0);
      assert(err >= 0);
      double cur_time = seconds_since_boot();
      msg_bufs[i] = kj::heapArray<capnp::word>((zmq_msg_size(&msg) / sizeof(capnp::word)) + 1);
      memcpy(msg_bufs[i].begin(), zmq_msg_data(&msg), zmq_msg_size(&msg));
      updated_sock_ids.push_back(i);
    }
  }
  update_msgs(cur_time, updated_sock_ids);
}

PubMasterBase::PubMasterBase() {
}

PubMaster::PubMaster() {
}

void PubMaster::add_sock(void *ctx, const char *addr) {
  void *pub_sock = zmq_socket(ctx, ZMQ_PUB);
  assert(zmq_bind(pub_sock, addr) == 0);
  socks.push_back(pub_sock);
}

void PubMaster::send(int idx, capnp::MallocMessageBuilder &msg) {
  auto words = capnp::messageToFlatArray(msg);
  auto bytes = words.asBytes();
  zmq_send(socks[idx], bytes.begin(), bytes.size(), 0);
}
