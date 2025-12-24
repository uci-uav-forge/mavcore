from enum import Enum

from mavcore.mav_message import MAVMessage, thread_safe
from mavcore.messages.mission_request_msg import MissionType


from enum import IntEnum


class MissionResult(IntEnum):
    UNKNOWN = -1
    ACCEPTED = 0
    ERROR = 1
    UNSUPPORTED_FRAME = 2
    UNSUPPORTED = 3
    NO_SPACE = 4
    INVALID = 5
    INVALID_PARAM1 = 6
    INVALID_PARAM2 = 7
    INVALID_PARAM3 = 8
    INVALID_PARAM4 = 9
    INVALID_PARAM5_X = 10
    INVALID_PARAM6_Y = 11
    INVALID_PARAM7_Z = 12
    INVALID_SEQUENCE = 13
    DENIED = 14
    OPERATION_CANCELLED = 15


class MissionAck(MAVMessage):
    """
    Mission acknowledgment.
    """

    def __init__(self):
        super().__init__("MISSION_ACK")
        self.result = MissionResult.UNKNOWN
        self.mission_type = MissionType.UNINITIALIZED

    def decode(self, msg):
        self.result = MissionResult(msg.type)
        self.mission_type = MissionType(msg.mission_type)

    @thread_safe
    def __repr__(self):
        return f"(COMMAND_ACK) timestamp: {self.timestamp} s, result: {self.result.name}, type: {self.mission_type.name}"
