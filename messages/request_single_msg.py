import pymavlink.dialects.v20.all as dialect
from enum import Enum

from ..mav_message import MAVMessage, thread_safe
from .request_msg_interval_msg import IntervalMessageID


class RequestSingleMessage(MAVMessage):
    """
    Requests single message.
    """

    def __init__(
        self,
        target_system: int,
        target_component: int,
        msg_id: IntervalMessageID,
    ):
        super().__init__("MAV_CMD_REQUEST_MESSAGE")
        self.target_system = target_system
        self.target_component = target_component
        self.msg_id = msg_id

    def encode(self, system_id, component_id):
        return dialect.MAVLink_command_long_message(
            target_system=self.target_system,
            target_component=self.target_component,
            command=512,  # MAV_CMD_REQUEST_MESSAGE (512)
            confirmation=0,
            param1=float(self.msg_id.value),
            param2=0.0,
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=1.0,
        )

    @thread_safe
    def __repr__(self):
        return f"(MAV_CMD_REQUEST_MESSAGE) timestamp: {self.timestamp}, msg_id: {self.msg_id.name}"
