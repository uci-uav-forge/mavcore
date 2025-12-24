from mavcore.mav_message import MAVMessage, thread_safe


class Attitude(MAVMessage):
    """
    Reads ATTITUDE mavlink message. only reads pitch and roll.
    """

    def __init__(self):
        super().__init__("ATTITUDE")
        self.pitch = 0.0  # angle in radians
        self.roll = 0.0  # angle in radians
        self.yaw = 0.0  # angle in radians
        self.rollspeed = 0.0  # angular speed in radians/sec
        self.pitchspeed = 0.0  # angular speed in radians/sec
        self.yawspeed = 0.0  # angular speed in radians/sec

    def decode(self, msg):
        self.pitch = msg.pitch
        self.roll = msg.roll
        self.yaw = msg.yaw
        self.rollspeed = msg.rollspeed
        self.pitchspeed = msg.pitchspeed
        self.yawspeed = msg.yawspeed

    @thread_safe
    def __repr__(self) -> str:
        return f"(ATTITUDE) timestamp: {self.timestamp} s, pitch: {self.pitch}, roll: {self.roll}"
