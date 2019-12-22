CC = clang
CXX = clang++


PHONELIBS = ../../phonelibs

WARN_FLAGS = -Werror=implicit-function-declaration \
             -Werror=incompatible-pointer-types \
             -Werror=int-conversion \
             -Werror=return-type \
             -Werror=format-extra-args

#CFLAGS = -std=gnu11 -fPIC -O2 $(WARN_FLAGS)
#CXXFLAGS = -std=c++11 -fPIC -O2 $(WARN_FLAGS)
CFLAGS = -std=gnu11 -fPIC -g $(WARN_FLAGS)
CXXFLAGS = -std=c++11 -fPIC -g $(WARN_FLAGS)

UNAME_M := $(shell uname -m)

ifeq ($(UNAME_M),x86_64)
	ZMQ_FLAGS = -I$(PHONELIBS)/zmq/x64/include
	ZMQ_LIBS = -L$(PHONELIBS)/zmq/x64/lib/ -l:libzmq.a
else
	# assume phone
	ZMQ_LIBS = -l:libzmq.a -lgnustl_shared
	EXTRA_LIBS += -llog -luuid
endif

ifeq ($(ARCH),aarch64)
CFLAGS += -mcpu=cortex-a57
CXXFLAGS += -mcpu=cortex-a57
CFLAGS += -DQCOM \
          -I$(PHONELIBS)/android_frameworks_native/include \
          -I$(PHONELIBS)/android_system_core/include \
          -I$(PHONELIBS)/android_hardware_libhardware/include
CXXFLAGS += -DQCOM \
            -I$(PHONELIBS)/android_frameworks_native/include \
            -I$(PHONELIBS)/android_system_core/include \
            -I$(PHONELIBS)/android_hardware_libhardware/include
endif

JSON_FLAGS = -I$(PHONELIBS)/json/src

DBC_FLAGS = -I../can
CAR_FLAGS = -I../car

OBJS = controlsd.o \
	alertmanager.o \
	alerts.o \
	can_define.o \
	canpacker.o \
	canparser.o \
	carinterface.o \
	drive_helpers.o \
	driver_monitor.o \
	gps_helpers.o \
	KF1D.o \
	latcontrol_lqr.o \
	latcontrol_pid.o \
	longcontrol.o \
	messaging.o \
	pid.o \
	stat_live.o \
	vehicle_model.o \
	tests/controlsd_cc_test.o \
	../car/toyota/toyota_interface.o \
	../car/toyota/toyota_carstate.o \
	../car/toyota/toyota_carcontroller.o \
	../car/toyota/toyota_values.o \
	../car/toyota/toyotacan.o \
	../common/params.o \
	../common/util.o \
	../common/swaglog.o \
	$(PHONELIBS)/json/src/json.o \
	$(CEREAL_OBJS)

DEPS := $(OBJS:.o=.d)

all: libcontrolsd_cc.so

include ../common/cereal.mk

libcontrolsd_cc.so: $(OBJS)
	@echo "[ LINK ] $@"
	$(CXX) -shared -o '$@' $^ \
	$(CEREAL_LIBS) \
	$(ZMQ_LIBS) \
	-L/system/vendor/lib64 \
	${UUID_LIBS} \
	-lm -lpthread

%.o: %.cc
	@echo "[ CXX ] $@"
	$(CXX) $(CXXFLAGS) -MMD \
   	-I. -I.. -I../.. \
	$(ZMQ_FLAGS) \
	$(DBC_FLAGS) \
	$(CAR_FLAGS) \
   -c -o '$@' '$<'

%.o: %.c
	@echo "[ CC ] $@"
	$(CC) $(CFLAGS) -MMD \
          -I.. -I../.. \
          $(ZMQ_FLAGS) \
          $(CEREAL_CFLAGS) \
          $(JSON_FLAGS) \
          -I$(PHONELIBS)/linux/include \
          -c -o '$@' '$<'

.PHONY: clean
clean:
	rm -f ui $(OBJS) $(DEPS)

-include $(DEPS)
