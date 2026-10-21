# ============================================================================
# Makefile for Webots R2025a/R2025b C++ Controller (line_bot_cpp)
# Supports macOS (/Applications/Webots.app) and Linux (/usr/local/webots)
# ============================================================================

CXX ?= clang++
WEBOTS_HOME ?= /Applications/Webots.app

ifneq ($(wildcard $(WEBOTS_HOME)/Contents/include/controller/cpp),)
  INC_DIR = $(WEBOTS_HOME)/Contents/include/controller/cpp
  LIB_DIR = $(WEBOTS_HOME)/Contents/lib/controller
else
  INC_DIR = $(WEBOTS_HOME)/include/controller/cpp
  LIB_DIR = $(WEBOTS_HOME)/lib/controller
endif

CXXFLAGS = -std=c++11 -Wall -Wextra -O2 -I"$(INC_DIR)"
LDFLAGS  = -L"$(LIB_DIR)" -lController -lCppController -Wl,-rpath,"$(LIB_DIR)" -Wl,-rpath,"$(WEBOTS_HOME)"

TARGET = line_bot_cpp
SOURCES = line_bot_cpp.cpp

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(LDFLAGS) -o $(TARGET)

clean:
	rm -rf $(TARGET) *.o *.d
