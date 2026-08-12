CC ?= gcc
CMAKE ?= cmake

TARGET := uvc_demo
SOURCES := main.c hik_sensor.c hik_capture.c uvc_camera.c
OBJS := $(SOURCES:.c=.o)

LIBUVC_BUILD := libuvc/build
LIBUVC_STATIC := $(LIBUVC_BUILD)/libuvc.a
LIBUVC_INPUTS := $(wildcard libuvc/src/*.c libuvc/include/libuvc/*.h) libuvc/CMakeLists.txt

CPPFLAGS := \
	-I/usr/include/libusb-1.0 \
	-I./libuvc/include \
	-I./$(LIBUVC_BUILD)/include
CFLAGS := -Wall -O2 -DDEBUG
LDLIBS := $(LIBUVC_STATIC) -lusb-1.0 -lpthread

.PHONY: all clean

all: $(TARGET)

$(LIBUVC_STATIC): $(LIBUVC_INPUTS)
	$(CMAKE) -S libuvc -B $(LIBUVC_BUILD) \
		-DCMAKE_BUILD_TARGET=Static \
		-DBUILD_EXAMPLE=OFF \
		-DBUILD_TEST=OFF \
		-DCMAKE_DISABLE_FIND_PACKAGE_JpegPkg=TRUE
	$(CMAKE) --build $(LIBUVC_BUILD) --target uvc_static -- -j$$(nproc)

$(TARGET): $(OBJS) $(LIBUVC_STATIC)
	$(CC) -o $@ $(OBJS) $(LDLIBS)

%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
	@if [ -f "$(LIBUVC_BUILD)/Makefile" ]; then \
		$(CMAKE) --build $(LIBUVC_BUILD) --target clean; \
	fi
