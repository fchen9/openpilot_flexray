CC = clang
CXX = clang++

ARCH := $(shell uname -m)
OS := $(shell uname -o)

BASEDIR = ../..
PHONELIBS = ../../phonelibs

WARN_FLAGS = -Werror=implicit-function-declaration \
             -Werror=incompatible-pointer-types \
             -Werror=int-conversion \
             -Werror=return-type \
             -Werror=format-extra-args

CFLAGS = -std=gnu11 -fPIC -I../ -I../../ -O2 $(WARN_FLAGS)
CXXFLAGS = -std=c++11 -fPIC -I../ -I../../ -O2 $(WARN_FLAGS)
#CFLAGS = -std=gnu11 -g -fPIC -I../ -I../../ $(WARN_FLAGS)
#CXXFLAGS = -std=c++11 -g -fPIC -I../ -I../../ $(WARN_FLAGS)

ZMQ_FLAGS = -I$(PHONELIBS)/zmq/aarch64/include
ZMQ_LIBS = -L$(PHONELIBS)/zmq/aarch64/lib \
           -l:libczmq.a -l:libzmq.a \
           -lgnustl_shared

JSON_FLAGS = -I$(PHONELIBS)/json/src

EXTRA_LIBS = -lpthread

ifeq ($(ARCH),x86_64)
ZMQ_LIBS = -L$(BASEDIR)/external/zmq/lib/ \
           -l:libczmq.a -l:libzmq.a
endif

.PHONY: all
all: ubloxd

include ../common/cereal.mk

OBJS = ubloxd.o \
       ../common/swaglog.o \
       ../common/params.o \
       ../common/util.o \
       $(PHONELIBS)/json/src/json.o \
       $(CEREAL_OBJS)

DEPS := $(OBJS:.o=.d)

ubloxd: $(OBJS)
	@echo "[ LINK ] $@"
	$(CXX) -fPIC -o '$@' $^ \
            $(CEREAL_LIBS) \
            $(ZMQ_LIBS) \
            $(EXTRA_LIBS)

ubloxd.o: ubloxd.cc
	@echo "[ CXX ] $@"
	$(CXX) $(CXXFLAGS) \
           -I$(PHONELIBS)/android_system_core/include \
           $(CEREAL_CFLAGS) \
           $(CEREAL_CXXFLAGS) \
           $(ZMQ_FLAGS) \
           -I../ \
           -I../../ \
           -c -o '$@' '$<'

%.o: %.c
	@echo "[ CC ] $@"
	$(CC) $(CFLAGS) -MMD \
          -Iinclude -I.. -I../.. \
          $(CEREAL_CFLAGS) \
          $(ZMQ_FLAGS) \
          $(JSON_FLAGS) \
          -c -o '$@' '$<'

.PHONY: clean
clean:
	rm -f ubloxd $(OBJS) $(DEPS)

-include $(DEPS)
