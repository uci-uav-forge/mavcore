from enum import Enum

from mavcore.mav_message import MAVMessage, thread_safe


class FixType(Enum):
    NO_GPS = 0
    NO_FIX = 1
    FIX_2D = 2
    FIX_3D = 3
    DGPS = 4
    RTK_FLOAT = 5
    RTK_FIXED = 6
    STATIC = 7
    PPP_3D = 8


class GPSRaw(MAVMessage):
    """
    Message to read gps info like number of sats and fix status (FixType enum).
    """

    def __init__(self):
        super().__init__("GPS_RAW_INT")
        self.fix_type = FixType(0)
        self.sats = -1  # number of satellites visible

    def decode(self, msg):
        self.fix_type = FixType(msg.fix_type)
        self.sats = msg.satellites_visible

    @thread_safe
    def __repr__(self) -> str:
        return f"(GPS_RAW_INT) timestamp: {self.timestamp} s, fix_type: {self.fix_type.name}, sats: {self.sats}"
