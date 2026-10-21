# ============================================================================
# Makefile for Webots R2025a C++ Controller on macOS (line_bot_cpp)
# ============================================================================

CXX ?= clang++
WEBOTS_APP ?= /Applications/Webots.app

CXXFLAGS = -std=c++11 -Wall -Wextra -O2 -I$(WEBOTS_APP)/Contents/include/controller/cpp
LDFLAGS  = -L$(WEBOTS_APP)/Contents/lib/controller -lController -lCppController -Wl,-rpath,$(WEBOTS_APP)/Contents/lib/controller

TARGET = line_bot_cpp
SOURCES = line_bot_cpp.cpp

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(LDFLAGS) -o $(TARGET)

clean:
	rm -rf $(TARGET) *.o *.d
