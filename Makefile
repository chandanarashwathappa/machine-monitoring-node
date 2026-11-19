# machine-monitoring-node - PC build (Linux / WSL)
#
#   make            build unit tests and the device simulator
#   make unit       run the C unit tests
#   make sil        run the pytest SIL suite (device + gateway + OPC UA)
#   make test       both
#   make clean

CC      ?= gcc
CFLAGS  := -std=c99 -Wall -Wextra -O2 -Icore -Isim
BUILD   := build
CORE    := core/modbus_tcp.c core/monitor.c
HEADERS := $(wildcard core/*.h sim/*.h)
PYTHON  ?= python3

.PHONY: all unit sil test clean

all: $(BUILD)/unit_tests $(BUILD)/device_sim

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/unit_tests: $(CORE) test/unit/test_core.c $(HEADERS) | $(BUILD)
	$(CC) $(CFLAGS) -o $@ $(CORE) test/unit/test_core.c

$(BUILD)/device_sim: $(CORE) sim/machine_sim.c sim/device_sim.c $(HEADERS) | $(BUILD)
	$(CC) $(CFLAGS) -o $@ $(CORE) sim/machine_sim.c sim/device_sim.c

unit: $(BUILD)/unit_tests
	./$(BUILD)/unit_tests

sil: $(BUILD)/device_sim
	$(PYTHON) -m pytest -v

test: unit sil

clean:
	rm -rf $(BUILD)
