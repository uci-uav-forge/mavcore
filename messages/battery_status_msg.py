from enum import Enum
from typing import Any
import pymavlink.dialects.v20.all as dialect

from mavcore.mav_message import MAVMessage, thread_safe


class BatteryFunction(Enum):
    UNINITIALIZED = -1
    UNKNOWN = 0
    ALL = 1
    PROPULSION = 2
    AVIONICS = 3
    PAYLOAD = 4


class BatteryType(Enum):
    UNINITIALIZED = -1
    UNKNOWN = 0
    LIPO = 1
    LIFE = 2
    LION = 3
    NIMH = 4


class BatteryStatus(MAVMessage):
    """
    Reads BATTERY_STATUS function. Uses BatteryType and BatteryFunction enums accordingly.
    """

    def __init__(self):
        super().__init__("BATTERY_STATUS")
        self.bat_id = -1
        self.bat_func = BatteryFunction(-1)
        self.bat_type = BatteryType(-1)
        self.temp = -1  # degrees in celcius
        self.voltages = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # in mV
        self.current = -1  # in centiAmps
        self.current_consumed = -1  # in mAh
        self.energy_consumed = -1  # in hJ
        self.soc = -1  # state of charge in percent

    def decode(self, msg):
        self.bat_id = msg.id
        self.bat_func = BatteryFunction(msg.battery_function)
        self.bat_type = BatteryType(msg.type)
        self.temp = msg.temperature
        self.voltages = msg.voltages
        self.current = msg.current_battery
        self.current_consumed = msg.current_consumed
        self.energy_consumed = msg.energy_consumed
        self.soc = msg.battery_remaining

    @thread_safe
    def encode(self, system_id, component_id) -> Any:
        return dialect.MAVLink_battery_status_message(
            id=self.bat_id,
            battery_function=self.bat_func.value,
            type=self.bat_type.value,
            temperature=self.temp,
            voltages=self.voltages,
            current_battery=self.current,
            current_consumed=self.current_consumed,
            energy_consumed=self.energy_consumed,
            battery_remaining=self.soc,
        )

    def __repr__(self) -> str:
        return f"(BATTERY_STATUS) timestamp: {self.timestamp} s, voltages: {self.voltages}, soc: {self.soc}%, func: {self.bat_func.name}, type: {self.bat_type.name}"
