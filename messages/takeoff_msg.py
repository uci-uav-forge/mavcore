import pymavlink.dialects.v20.all as dialect
from enum import Enum

from mavcore.mav_message import MAVMessage, thread_safe


class MAVFrame(Enum):
    GLOBAL = 0  # Altitude in MSL
    LOCAL_NED = 1  # Altitude in AGL
    GLOBAL_RELATIVE_ALTITUDE = 3  # Altitude in AGL


class Takeoff(MAVMessage):
    """
    Takeoff message. Simplified so altitude in relative (AGL) meters.
    """

    def __init__(self, target_system: int, target_component: int, alt: float):
        super().__init__("CUSTOM_TAKEOFF")
        self.target_system = target_system
        self.target_component = target_component

        self.alt = alt  # in meters (relative)

    def encode(self, system_id, component_id):
        return dialect.MAVLink_command_long_message(
            target_system=int(self.target_system),
            target_component=int(self.target_component),
            command=int(22),  # MAV_CMD_NAV_TAKEOFF
            confirmation=int(0),
            param1=float(0.0),
            param2=float(0.0),
            param3=float(0.0),
            param4=float(0.0),
            param5=float(0.0),
            param6=float(0.0),
            param7=float(self.alt),  # required to be float
        )

    @thread_safe
    def __repr__(self):
        return f"(CUSTOM_TAKEOFF) timestamp: {self.timestamp}, altitude: {self.alt}"
