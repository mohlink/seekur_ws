# Makefile - seekur_baudrate_detector
#
# Utilitaire C++ standalone de diagnostic bas niveau, pas partie du stack ROS2.

CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -pthread

TARGET := seekur_baudrate_detector
SRC    := seekur_baudrate_detector.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -f $(TARGET)

.PHONY: all clean
