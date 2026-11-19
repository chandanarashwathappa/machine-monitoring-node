# Industrial Machine-Monitoring Node (Modbus TCP + OPC UA)

[![ci](https://github.com/chandanarashwathappa/machine-monitoring-node/actions/workflows/ci.yml/badge.svg)](https://github.com/chandanarashwathappa/machine-monitoring-node/actions/workflows/ci.yml)

A small version of a real shop-floor data chain: a **field device** in C monitors a
production machine (temperature, motor current, cycle sensor) and serves the data
through its own **Modbus TCP server**. A Python **gateway** polls it and publishes an
**OPC UA** information model for SCADA, MES or a PLC. The whole chain is verified in
**software-in-the-loop** with pytest, with fault injection, on every push.

> **Status:** Phase 1 (C core, device simulator, gateway, SIL tests, CI) done.
> Phase 2 (ESP32 + real sensors, CODESYS soft-PLC as a client) planned. See [Roadmap](#roadmap).

## Architecture

```
  Machine signals            Field device (C)                    Gateway (Python)            Clients
 ┌───────────────┐   ┌─────────────────────────────────┐   ┌─────────────────────────┐   ┌──────────────┐
 │ temperature   │──►│ monitor.c                       │   │ pymodbus client         │   │ SCADA / MES  │
 │ motor current │──►│  debounce, cycle count/time,    │   │  polls every 0.5 s      │   │ UaExpert     │
 │ cycle sensor  │──►│  run time, OEE, alarms + ack    │   │ asyncua OPC UA server   │──►│ PLC          │
 │ running       │──►│ modbus_tcp.c  FC 03/04/06/16    │──►│  Machine/ Alarms/ Config│   │ pytest       │
 └───────────────┘   │  exceptions 01/02/03            │TCP│  AcknowledgeAlarms()    │   └──────────────┘
   sim/ (PC) or      └─────────────────────────────────┘502│  ResetCounters()        │ opc.tcp
   ESP32 (phase 2)                                         └─────────────────────────┘
```

`core/` is hardware independent: the same code runs in the PC simulator and (phase 2)
on an ESP32.

## What the device does

- **Cycle monitoring:** debounced cycle sensor (glitches shorter than 40 ms are rejected),
  cycle count, last and average cycle time
- **OEE factors:** availability (run time / observed time) and performance
  (ideal / actual cycle time)
- **Alarms:** over-temperature with hysteresis, over-current, jam (no cycle within the
  timeout while running), sensor fault. Each alarm also sets an *unacknowledged* bit that
  stays until an operator acknowledges it, as in industrial HMIs.
- **Modbus TCP server written from scratch:** function codes 0x03, 0x04, 0x06, 0x10,
  exception responses 0x01 / 0x02 / 0x03, up to 4 clients at once, configurable limits
  in holding registers.

Register map: [docs/register_map.md](docs/register_map.md)

## OPC UA model (gateway)

```
Objects/Machine/
  Temperature, MotorCurrent, State, StateCode, CycleCount, LastCycleTime,
  AverageCycleTime, RunTime, Uptime, Availability, Performance, DeviceConnected
  Alarms/   OverTemperature, OverCurrent, Jam, SensorFault (+ ...Unacknowledged)
  Config/   TemperatureLimit (writable, written through to Modbus)
  AcknowledgeAlarms(), ResetCounters()
```

## Verification

| Level | Tests |
|---|---|
| C unit tests | Modbus frame handling (MBAP echo, all exceptions, byte-count checks, atomic multi-write), debounce, cycle time, EMA, OEE, alarm hysteresis, latching and acknowledge, jam timeout, register ranges, int16 / uint32 encoding |
| SIL – Modbus conformance | pymodbus (independent implementation) plus raw frames: read/write, two's complement, illegal address / value / function, quantity limits, 3 concurrent clients |
| SIL – machine behaviour | cycle time 2.2 s, performance 90.9 %, speed change, stopped machine, jam alarm + ack, over-temperature **hysteresis band**, over-current, sensor fault, counter reset |
| SIL – end to end | OPC UA values = Modbus values, alarm → OPC UA → `AcknowledgeAlarms()` → Modbus, limit written from OPC UA reaches the device, `ResetCounters()`, device offline → `DeviceConnected = false` |

## Run it (Linux or WSL)

```sh
pip install -r requirements.txt
make            # build/unit_tests, build/device_sim
make unit       # C unit tests
make sil        # pytest: device simulator + gateway + OPC UA client
```

By hand:

```sh
./build/device_sim --port 1502 &
python3 gateway/gateway.py --device-port 1502 --opc-port 4840
# then browse opc.tcp://127.0.0.1:4840/machine/ with UaExpert, or read Modbus with any client
```

## Repository layout

```
core/            Modbus TCP protocol + monitoring logic (C99, no I/O)
sim/             PC device: machine model + TCP server, fault-injection registers
gateway/         Modbus -> OPC UA gateway (pymodbus, asyncua)
test/unit/       C unit tests
test/sil/        pytest SIL suite
firmware/esp32/  ESP32 port (phase 2)
docs/            register map
```

## Roadmap

- [x] Monitoring core: cycles, OEE factors, alarms with acknowledge
- [x] Modbus TCP server (03/04/06/16, exceptions)
- [x] Device simulator with fault injection
- [x] Modbus → OPC UA gateway with methods and write-through
- [x] Unit + SIL tests in CI
- [ ] ESP32 port (ESP-IDF, lwIP sockets) with real sensors
- [ ] CODESYS soft-PLC reading the device over Modbus TCP
- [ ] Modbus RTU (RS-485) variant

## License

MIT
