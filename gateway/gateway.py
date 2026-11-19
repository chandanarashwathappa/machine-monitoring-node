"""Modbus TCP -> OPC UA gateway for the machine-monitoring node.

Polls the field device over Modbus TCP and publishes an OPC UA information
model that a SCADA system, MES or PLC can browse:

    Objects/
      Machine/
        Temperature, MotorCurrent           (Double, degC / A)
        State, StateCode                    (String, Int32)
        CycleCount, LastCycleTime, AverageCycleTime
        RunTime, Uptime, Availability, Performance
        DeviceConnected                     (Boolean, Modbus link status)
        Alarms/
          OverTemperature, OverCurrent, Jam, SensorFault            (active)
          ...Unacknowledged                                         (needs ack)
        Config/
          TemperatureLimit                  (Double, writable -> Modbus)
        AcknowledgeAlarms()                 (method -> Modbus command)
        ResetCounters()                     (method -> Modbus command)

Usage:
    python gateway/gateway.py --device-port 1502 --opc-port 4840
"""
from __future__ import annotations

import argparse
import asyncio
import logging

from asyncua import Server, ua
from asyncua.common.methods import uamethod
from pymodbus.client import AsyncModbusTcpClient

import registers as R

NAMESPACE_URI = "urn:machine-monitoring-node"
VT = ua.VariantType

log = logging.getLogger("gateway")


class Gateway:
    def __init__(self, device_host: str, device_port: int, opc_port: int,
                 poll_s: float = 0.5, unit: int = 1):
        self.endpoint = f"opc.tcp://127.0.0.1:{opc_port}/machine/"
        self.poll_s = poll_s
        self.unit = unit
        self.device_host = device_host
        self.device_port = device_port
        self.modbus = None                       # created inside the event loop (run)
        self.server = Server()
        self.vars: dict[str, tuple] = {}         # name -> (node, variant type)
        self.limit_node = None
        self.limit_last = None                   # last value we put in TemperatureLimit

    # ------------------------------------------------------------- model
    async def setup(self) -> None:
        await self.server.init()
        self.server.set_endpoint(self.endpoint)
        self.server.set_server_name("Machine Monitoring Gateway")
        self.server.set_security_policy([ua.SecurityPolicyType.NoSecurity])
        idx = await self.server.register_namespace(NAMESPACE_URI)

        machine = await self.server.nodes.objects.add_object(idx, "Machine")

        async def var(parent, name, value, vt):
            node = await parent.add_variable(idx, name, ua.Variant(value, vt))
            self.vars[name] = (node, vt)
            return node

        await var(machine, "Temperature", 0.0, VT.Double)
        await var(machine, "MotorCurrent", 0.0, VT.Double)
        await var(machine, "State", "UNKNOWN", VT.String)
        await var(machine, "StateCode", -1, VT.Int32)
        await var(machine, "CycleCount", 0, VT.Int64)
        await var(machine, "LastCycleTime", 0.0, VT.Double)
        await var(machine, "AverageCycleTime", 0.0, VT.Double)
        await var(machine, "RunTime", 0, VT.Int64)
        await var(machine, "Uptime", 0, VT.Int64)
        await var(machine, "Availability", 0.0, VT.Double)
        await var(machine, "Performance", 0.0, VT.Double)
        await var(machine, "DeviceConnected", False, VT.Boolean)

        alarms = await machine.add_object(idx, "Alarms")
        for name in R.ALARM_NAMES.values():
            await var(alarms, name, False, VT.Boolean)
            await var(alarms, name + "Unacknowledged", False, VT.Boolean)

        config = await machine.add_object(idx, "Config")
        self.limit_node = await var(config, "TemperatureLimit", 0.0, VT.Double)
        await self.limit_node.set_writable()

        @uamethod
        async def acknowledge_alarms(parent):
            await self.write_holding(R.HR_COMMAND, R.CMD_ACK_ALARMS)

        @uamethod
        async def reset_counters(parent):
            await self.write_holding(R.HR_COMMAND, R.CMD_RESET_COUNTERS)

        await machine.add_method(idx, "AcknowledgeAlarms", acknowledge_alarms, [], [])
        await machine.add_method(idx, "ResetCounters", reset_counters, [], [])

    async def set(self, name: str, value) -> None:
        node, vt = self.vars[name]
        await node.write_value(ua.Variant(value, vt))

    # ------------------------------------------------------------ modbus
    async def ensure_connected(self) -> None:
        if not self.modbus.connected:
            await self.modbus.connect()
        if not self.modbus.connected:
            raise ConnectionError("Modbus device not reachable")

    async def write_holding(self, addr: int, value: int) -> None:
        await self.ensure_connected()
        rr = await self.modbus.write_register(addr, R.u16(value), slave=self.unit)
        if rr.isError():
            raise ua.UaError(f"Modbus write to {addr} failed: {rr}")

    async def poll_once(self) -> None:
        await self.ensure_connected()
        ir = await self.modbus.read_input_registers(0, count=R.IR_COUNT, slave=self.unit)
        hr = await self.modbus.read_holding_registers(0, count=R.HR_COUNT, slave=self.unit)
        if ir.isError() or hr.isError():
            raise ConnectionError(f"Modbus read failed: {ir} / {hr}")

        d = R.decode_inputs(ir.registers)
        await self.set("Temperature", d["temperature_c"])
        await self.set("MotorCurrent", d["current_a"])
        await self.set("State", d["state_name"])
        await self.set("StateCode", d["state"])
        await self.set("CycleCount", d["cycle_count"])
        await self.set("LastCycleTime", d["last_cycle_s"])
        await self.set("AverageCycleTime", d["avg_cycle_s"])
        await self.set("RunTime", d["run_time_s"])
        await self.set("Uptime", d["uptime_s"])
        await self.set("Availability", d["availability_pct"])
        await self.set("Performance", d["performance_pct"])
        for bit, name in R.ALARM_NAMES.items():
            await self.set(name, bool(d["alarms_active"] & bit))
            await self.set(name + "Unacknowledged", bool(d["alarms_unack"] & bit))

        # TemperatureLimit: a value written by an OPC UA client goes to Modbus,
        # otherwise the device's value is shown.
        device_limit = R.s16(hr.registers[R.HR_TEMP_LIMIT]) / 10.0
        node_value = await self.limit_node.read_value()
        if self.limit_last is not None and node_value != self.limit_last:
            await self.write_holding(R.HR_TEMP_LIMIT, round(node_value * 10))
            self.limit_last = node_value
        elif device_limit != self.limit_last:
            await self.set("TemperatureLimit", device_limit)
            self.limit_last = device_limit

        await self.set("DeviceConnected", True)

    # -------------------------------------------------------------- main
    async def run(self) -> None:
        self.modbus = AsyncModbusTcpClient(self.device_host, port=self.device_port,
                                           timeout=1, retries=0)
        await self.setup()
        async with self.server:
            print(f"OPC UA gateway running at {self.endpoint}", flush=True)
            while True:
                try:
                    await self.poll_once()
                except Exception as exc:          # device down, timeout, ...
                    log.warning("poll failed: %s", exc)
                    await self.set("DeviceConnected", False)
                    self.modbus.close()
                await asyncio.sleep(self.poll_s)


def main() -> None:
    ap = argparse.ArgumentParser(description="Modbus TCP -> OPC UA gateway")
    ap.add_argument("--device-host", default="127.0.0.1")
    ap.add_argument("--device-port", type=int, default=1502)
    ap.add_argument("--opc-port", type=int, default=4840)
    ap.add_argument("--poll", type=float, default=0.5, help="poll period in seconds")
    args = ap.parse_args()

    logging.basicConfig(level=logging.WARNING)
    logging.getLogger("asyncua").setLevel(logging.ERROR)
    logging.getLogger("pymodbus").setLevel(logging.CRITICAL)

    gw = Gateway(args.device_host, args.device_port, args.opc_port, args.poll)
    try:
        asyncio.run(gw.run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
