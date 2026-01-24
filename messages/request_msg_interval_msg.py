import pymavlink.dialects.v20.all as dialect
from enum import Enum

from mavcore.mav_message import MAVMessage, thread_safe


class IntervalMessageID(Enum):
    SYS_STATUS = 1
    SYSTEM_TIME = 2
    GPS_RAW_INT = 24
    ATTITUDE = 30
    ATTITUDE_QUATERNION = 31
    LOCAL_POSITION_NED = 32
    GLOBAL_POSITION_INT = 33
    BATTERY_STATUS = 147


class RequestMessageInterval(MAVMessage):
    """
    Requests message interval stream from msg id at a specified rate.
    """

    def __init__(
        self,
        target_system: int,
        target_component: int,
        msg_id: IntervalMessageID,
        rate_hz: float,
    ):
        super().__init__("MAV_CMD_SET_MESSAGE_INTERVAL")
        self.target_system = target_system
        self.target_component = target_component
        self.msg_id = msg_id
        self.rate_hz = rate_hz

    def encode(self, system_id, component_id):
        return dialect.MAVLink_command_long_message(
            target_system=self.target_system,
            target_component=self.target_component,
            command=511,  # MAV_CMD_SET_MESSAGE_INTERVAL (511)
            confirmation=0,
            param1=float(self.msg_id.value),
            param2=float((1.0 / self.rate_hz) * 1000000),  # interval in microseconds
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=1.0,
        )

    @thread_safe
    def __repr__(self):
        return f"(MAV_CMD_SET_MESSAGE_INTERVAL) timestamp: {self.timestamp}, msg_id: {self.msg_id.name}"
