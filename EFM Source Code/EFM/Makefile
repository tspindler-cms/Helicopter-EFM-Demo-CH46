SHELL := /bin/bash

ifeq ($(origin CXX), default)
CXX := x86_64-w64-mingw32-g++
endif

OUT_DIR := ../../bin
TARGET := $(OUT_DIR)/CH46D.dll

SOURCES := \
	dllmain.cpp \
	CH46D.cpp \
	FlightModel/Aero.cpp \
	FlightModel/Rotordisk.cpp \
	stdafx.cpp

INCLUDES := -I./include

DEFINES := \
	-DWIN32 \
	-D_WINDOWS \
	-D_USRDLL \
	-D_CRT_SECURE_NO_DEPRECATE \
	-D_USE_MATH_DEFINES \
	-DNOMINMAX \
	-D_HAS_ITERATOR_DEBUGGING=0 \
	-D_SECURE_SCL=0 \
	-DFM_EXPORTS

CXXFLAGS ?= -std=gnu++17 -O2
LDFLAGS := -shared
LDLIBS := \
	-lkernel32 \
	-luser32 \
	-lgdi32 \
	-lwinspool \
	-lcomdlg32 \
	-ladvapi32 \
	-lshell32 \
	-lole32 \
	-loleaut32 \
	-luuid

.PHONY: all clean check-toolchain print-config

all: check-toolchain $(TARGET)

print-config:
	@echo "CXX:      $(CXX)"
	@echo "PWD:      $(CURDIR)"
	@echo "OUT_DIR:  $(abspath $(OUT_DIR))"
	@echo "TARGET:   $(TARGET)"
	@echo "SOURCES:  $(SOURCES)"

check-toolchain:
	@command -v "$(CXX)" >/dev/null 2>&1 || { \
		echo "error: $(CXX) not found in PATH."; \
		echo "Install MinGW-w64 (macOS: brew install mingw-w64)."; \
		exit 1; \
	}

$(OUT_DIR):
	@mkdir -p "$(OUT_DIR)"

$(TARGET): $(SOURCES) | $(OUT_DIR)
	"$(CXX)" $(CXXFLAGS) $(DEFINES) $(INCLUDES) $(LDFLAGS) \
		-o "$(TARGET)" $(SOURCES) $(LDLIBS)

clean:
	@rm -f "$(TARGET)"
