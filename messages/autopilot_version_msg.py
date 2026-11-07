from ..mav_message import MAVMessage, thread_safe


class AutopilotVersion(MAVMessage):
    def __init__(self):
        super().__init__("AUTOPILOT_VERSION")
        self.version = 0
        self.uid = 0
        self.uid2: list[int] = []

    def decode(self, msg):
        self.version = msg.flight_sw_version
        self.uid = msg.uid
        self.uid2 = list(msg.uid2)

    def get_major_version(self) -> int:
        return self.version >> 24

    @thread_safe
    def get_uid2(self) -> str:
        """Return UID2 as hexadecimal without trailing MAVLink padding."""
        return bytes(self.uid2).rstrip(b"\x00").hex()

    @thread_safe
    def get_uid(self) -> str:
        uid2 = self.get_uid2()
        if uid2:
            return uid2
        if self.uid:
            return f"{self.uid:016x}"
        return ""

    @thread_safe
    def __repr__(self):
        return f"(AUTOPILOT_VERSION) timestamp: {self.timestamp}, version: {self.version}, major version: {self.version >> 24}, uid: {self.get_uid()}"
