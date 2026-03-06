from ..mav_message import MAVMessage


class SystemTime(MAVMessage):
    """
    Gets the system time.
    """

    def __init__(self):
        super().__init__("SYSTEM_TIME")
        self.time_unix_usec = -1
        self.time_boot_ms = -1

    def decode(self, msg):
        self.time_unix_usec = msg.time_unix_usec
        self.time_boot_ms = msg.time_boot_ms

    def __repr__(self) -> str:
        return f"(SYSTEM_TIME) timestamp: {self.timestamp} s, time: {self.time_unix_usec}, boot time: {self.time_boot_ms}"
