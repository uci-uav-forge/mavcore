from mavcore.mav_message import MAVMessage, thread_safe
from enum import IntEnum


class MissionType(IntEnum):
    UNINITIALIZED = -1
    MISSION = 0
    FENCE = 1
    RALLY = 2
    ALL = 255


class MissionRequestInt(MAVMessage):
    """
    Mission request message saying which mission item is being requested.
    """

    def __init__(self):
        super().__init__(
            "MISSION_REQUEST"
        )  # Should be MISSION_REQUEST_INT but possible bug in pymavlink
        self.mission_type = MissionType(-1)
        self.seq = -1

    def decode(self, msg):
        self.mission_type = MissionType(msg.mission_type)
        self.seq = msg.seq

    @thread_safe
    def __repr__(self):
        return f"(MISSION_REQUEST_INT) timestamp: {self.timestamp} s, type: {self.mission_type.name}, seq: {self.seq}"
