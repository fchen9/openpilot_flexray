#ifndef MESSAGING_H_
#define MESSAGING_H_

class SubSocketBase {
public:
  SubSocketBase();
  virtual void receive(std::vector<std::string> &can_strs) = 0;
  virtual void set_timeout(int can_timeout);
  void *sock;
};

class SubSocket: public SubSocketBase {
public:
  SubSocket(void *ctx, const char *endpoint);
  void set_timeout(int can_timeout);
  void receive(std::vector<std::string> &can_strs);
};

class SubMasterBase {
public:
  SubMasterBase();
  void add_sock(void *ctx, const char *addr, float freq, bool valid, bool b_ignore_alive, capnp::MallocMessageBuilder &msg);
  virtual void update(int timeout=-1) = 0;
  virtual void update_msgs(double cur_time, std::vector<int> sock_ids);
  bool all_alive_and_valid();

  int64_t frame;
  // Diff from python code, we use vector instead of map
  std::vector<void *> socks;
  std::vector<bool> updated;
  std::vector<kj::Array<capnp::word>> msg_bufs;
  std::vector<bool> alive;
  std::vector<bool> valid;
  std::vector<double> freq;
  std::vector<double> rcv_time;
  std::vector<double> rcv_frame;
  std::vector<uint64_t> logMonoTime;
  std::vector<bool> ignore_alive;
};

class SubMaster: public SubMasterBase {
public:
  SubMaster();

  void update(int timeout=-1);
};

class PubMasterBase {
public:
  PubMasterBase();
  virtual void add_sock(void *ctx, const char *addr) = 0;
  virtual void send(int idx, capnp::MallocMessageBuilder &msg) = 0;
};

class PubMaster: public PubMasterBase {
public:
  PubMaster();
  void add_sock(void *ctx, const char *addr);
  void send(int idx, capnp::MallocMessageBuilder &msg);

  std::vector<void *> socks;
};

#endif
