# Host-side tests for Zephyr-free firmware code (no Zephyr SDK needed).
CXX ?= g++
CXXFLAGS ?= -std=c++20 -Wall -Wextra -O1 -g

all: run

awProtocolTest: awProtocolTest.cpp ../../src/sections/awProtocol.hpp
	$(CXX) $(CXXFLAGS) -o $@ awProtocolTest.cpp

joybusWiringTest: joybusWiringTest.cpp ../../src/module/joybusWiring.hpp
	$(CXX) $(CXXFLAGS) -o $@ joybusWiringTest.cpp

run: awProtocolTest joybusWiringTest
	./awProtocolTest
	./joybusWiringTest

# The JOY Bus PIO program in a PIO emulator (pip install rp2040-pio-emulator adafruit-circuitpython-pioasm)
pio:
	python3 joybusPioTest.py

clean:
	rm -f awProtocolTest joybusWiringTest

.PHONY: all run pio clean
