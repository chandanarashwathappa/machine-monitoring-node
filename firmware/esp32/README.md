# ESP32 firmware (phase 2, planned)

ESP-IDF project for a real ESP32 DevKit. It reuses `../../core/` unchanged:
`modbus_tcp.c` has no I/O, and ESP-IDF's lwIP offers the same BSD socket API as
the PC simulator, so the TCP server loop ports almost line for line.

## Planned hardware

| Part | Purpose |
|---|---|
| ESP32 DevKit (Wi-Fi) | field device, Modbus TCP server on port 502 |
| DS18B20 or NTC | temperature |
| INA219 or ACS712 | motor current |
| Inductive proximity sensor / button | cycle sensor (debounced in software) |
| Switch | machine-running signal |

## Planned tasks (FreeRTOS)

| Task | Period | Job |
|---|---|---|
| `sample_task` | 10 ms | read sensors, `monitor_step()` |
| `modbus_task` | event | lwIP sockets, `modbus_tcp_process()` |
| Wi-Fi / event loop | – | station mode, reconnect |

## Test with a PLC

The node can be read by a **CODESYS** soft-PLC (CODESYS Control Win, free) using a
Modbus TCP Client device, and by the gateway in this repository, which publishes
the data over OPC UA.
