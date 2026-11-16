from ..mav_message import MAVMessage, thread_safe


class AutopilotVersion(MAVMessage):
    def __init__(self):
        super().__init__("AUTOPILOT_VERSION")
        self.version = 0

    def decode(self, msg):
        self.version = msg.flight_sw_version

    def get_major_version(self) -> int:
        return self.version >> 24

    @thread_safe
    def __repr__(self):
        return f"(AUTOPILOT_VERSION) timestamp: {self.timestamp}, version: {self.version}, major version: {self.version >> 24}"
