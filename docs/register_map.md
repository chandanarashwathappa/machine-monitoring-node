# Modbus register map

Modbus TCP, unit id: any (echoed), addresses are **0-based** (PDU addresses).
Register values are 16-bit big-endian on the wire. 32-bit values use two registers,
**high word first**. Signed values use two's complement.

## Input registers (function code 0x04, read only)

| Addr | Name | Type / unit |
|---|---|---|
| 0 | Temperature | int16, 0.1 °C |
| 1 | Motor current | uint16, mA |
| 2 | Machine state | 0 STOPPED, 1 RUNNING, 2 ALARM |
| 3 | Active alarms | bit field (below) |
| 4 | Unacknowledged alarms | bit field (below) |
| 5–6 | Cycle count | uint32 |
| 7 | Last cycle time | uint16, ms |
| 8 | Average cycle time | uint16, ms (moving average, weight 1/8) |
| 9–10 | Run time | uint32, s |
| 11–12 | Uptime | uint32, s |
| 13 | Availability | 0–1000 ‰ (run time / observed time) |
| 14 | Performance | 0–1000 ‰ (ideal cycle time / average cycle time, max 1000) |
| 15 | Heartbeat | +1 every second |

## Holding registers (function codes 0x03, 0x06, 0x10)

| Addr | Name | Unit | Range | Default |
|---|---|---|---|---|
| 0 | Temperature alarm limit | int16, 0.1 °C | −200…1500 | 800 (80.0 °C) |
| 1 | Current alarm limit | mA | 100…20000 | 3000 |
| 2 | Temperature hysteresis | 0.1 °C | 0…200 | 50 (5.0 °C) |
| 3 | Ideal cycle time | ms | 100…60000 | 2000 |
| 4 | Cycle timeout (jam) | ms | 500…60000 | 6000 |
| 5 | Command | – | 1 = acknowledge alarms, 2 = reset counters | reads 0 |

A value outside its range is rejected with exception **0x03**.

## Alarm bits

| Bit | Alarm | On | Off |
|---|---|---|---|
| 0x01 | Over-temperature | temperature > limit | temperature < limit − hysteresis |
| 0x02 | Over-current | current > limit | current < 90 % of limit |
| 0x04 | Jam | running and no cycle for longer than the cycle timeout | next cycle, or machine stopped |
| 0x08 | Sensor fault | temperature outside −20…150 °C | back in range |

An alarm that becomes active also sets its **unacknowledged** bit. That bit stays set
until command 1 (acknowledge), even after the alarm itself has gone. Acknowledging does
not clear an alarm that is still active.

## Exceptions

| Code | When |
|---|---|
| 0x01 Illegal function | any function code other than 0x03, 0x04, 0x06, 0x10 |
| 0x02 Illegal data address | a register in the request does not exist |
| 0x03 Illegal data value | quantity 0 or > 125 (> 123 for 0x10), wrong byte count, value out of range |

## Simulator only (fault injection, holding registers)

| Addr | Effect |
|---|---|
| 100 | extra heat, int16, 0.1 °C (e.g. 400 = +40 °C) |
| 101 | extra motor current, mA |
| 102 | jam: 1 = machine stuck, no more cycles |
| 103 | machine running: 0 / 1 |
| 104 | temperature sensor open: 1 = reads −60 °C |
| 105 | cycle period, ms (default 2200) |
