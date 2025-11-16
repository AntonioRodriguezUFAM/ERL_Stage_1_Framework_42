###############################################################################
#
# Lynsyn Makefile
#
# Compiles host software for all Lynsyn boards
#
# Usage:
#    make            Builds all host software
#    make clean      Cleans everything
#    make liblynsyn    Builds only the liblynsyn shared library
#
# Make sure to adjust settings at the top of this file for your system
#
###############################################################################

###############################################################################
# Settings for the host tools

export QMAKE = qmake CONFIG+=release

###############################################################################
###############################################################################
# Everything below this should not need any changes

export HOSTDIR = $(shell pwd)

ifeq ($(OS),Windows_NT)
export CFLAGS = -I$(HOSTDIR)/../argp-standalone-1.3/ -I/mingw64/include/libusb-1.0/
export LDFLAGS = -largp -L$(HOSTDIR)/../argp-standalone-1.3/
endif

export CFLAGS += -g -O2 -Wall -fPIC -I/usr/include/libusb-1.0/ -I$(HOSTDIR)/common/ -I$(HOSTDIR)/liblynsyn/ # Added -fPIC
export LDFLAGS += -lusb-1.0
export CXXFLAGS = -std=gnu++11 $(CFLAGS)

export CC = gcc
export CPP = g++
export LD = /usr/bin/g++ # Changed from g++ to /usr/bin/g++
export AR = ar
export RANLIB = ranlib

###############################################################################
# New target to build the shared library
LIBLYNSYN_TARGET = liblynsyn/liblynsyn.so # Define the output shared library name
LIBLYNSYN_OBJS = liblynsyn/lynsyn.o

.PHONY: liblynsyn
liblynsyn: $(LIBLYNSYN_TARGET)
	@echo "Building liblynsyn shared library"

$(LIBLYNSYN_TARGET): $(LIBLYNSYN_OBJS)
	$(LD) -shared -o $@ $^ $(LDFLAGS) # Use LD (g++) and -shared to create the shared library
	#RANLIB $@ # Not needed for shared libraries on Linux
	#$(CC) -shared -o $@ $^ $(LDFLAGS) # alternative

###############################################################################

.PHONY: host_software
host_software: bin/lynsyn_tester bin/lynsyn_sampler bin/lynsyn_xvc bin/lynsyn_xsvf bin/lynsyn_viewer liblynsyn # Added liblynsyn as a dependency
	@echo
	@echo "Host software compilation successful"
	@echo

.PHONY: bin/lynsyn_tester
bin/lynsyn_tester:
	mkdir -p bin
	cd lynsyn_tester && $(MAKE)
	cp lynsyn_tester/lynsyn_tester bin

.PHONY: bin/lynsyn_sampler
bin/lynsyn_sampler:
	cd lynsyn_sampler && $(MAKE)
	cp lynsyn_sampler/lynsyn_sampler bin

.PHONY: bin/lynsyn_xvc
bin/lynsyn_xvc:
	mkdir -p lynsyn_xvc/build
	cd lynsyn_xvc/build && $(QMAKE) ..
	cd lynsyn_xvc/build && $(MAKE)
ifeq ($(OS),Windows_NT)
	cp lynsyn_xvc/build/release/lynsyn_xvc bin
else
	cp lynsyn_xvc/build/lynsyn_xvc bin
endif

.PHONY: bin/lynsyn_xsvf
bin/lynsyn_xsvf:
	cd libxsvf && $(MAKE) lynsyn_xsvf
	cp libxsvf/lynsyn_xsvf bin

.PHONY: bin/lynsyn_viewer
bin/lynsyn_viewer:
	mkdir -p lynsyn_viewer/build
	cd lynsyn_viewer/build && $(QMAKE) ..
	cd lynsyn_viewer/build && $(MAKE)
ifeq ($(OS),Windows_NT)
	cp lynsyn_viewer/build/release/lynsyn_viewer bin
else
	cp lynsyn_viewer/build/lynsyn_viewer bin
endif

###############################################################################

.PHONY: install
install: host_software install_hw
	cp bin/* /usr/bin/
	cp jtagdevices /etc/jtagdevices
	@echo
	@echo "Software and hardware installed"
	@echo

###############################################################################

.PHONY: install_hw
install_hw:
	cp udev/48-lynsyn.rules /etc/udev/rules.d
	udevadm control --reload-rules

###############################################################################

.PHONY : clean
clean:
	cd lynsyn_tester && $(MAKE) clean
	cd lynsyn_sampler && $(MAKE) clean
	cd libxsvf && $(MAKE) clean
	rm -rf lynsyn_xvc/build
	rm -rf lynsyn_viewer/build
	rm -rf liblynsyn/*.o
	rm -rf liblynsyn/*.so
