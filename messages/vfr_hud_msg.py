from mavcore.mav_message import MAVMessage, thread_safe


class VFRHUD(MAVMessage):
    """
    General flight info/telemetry.
    Units are in m/s for speeds, percentage for throttle, meters for altitude, and degrees * 100 integer for heading.
    """

    def __init__(self):
        super().__init__("VFR_HUD")
        self.airspeed = 0.0  # in m/s
        self.groundspeed = 0.0  # in m/s
        self.climbspeed = 0.0  # in m/s
        self.heading_int = 0  # in degrees (0 = north)
        self.throttle = 0.0  # in percentage (0 to 100)
        self.alt_msl = 0.0  # altitude in meters, above mean sea level

    def decode(self, msg):
        self.airspeed = msg.airspeed
        self.groundspeed = msg.groundspeed
        self.climbspeed = msg.climb
        self.heading_int = msg.heading
        self.throttle = msg.throttle
        self.alt_msl = msg.alt

    @thread_safe
    def __repr__(self) -> str:
        return f"(VFR_HUD) timestamp: {self.timestamp} s, throttle: {self.throttle}"
