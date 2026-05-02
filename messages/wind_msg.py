from mavcore.mav_message import MAVMessage, thread_safe


class Wind(MAVMessage):
    """
    Reads WIND mavlink message.
    """

    def __init__(self):
        super().__init__("WIND")
        self.direction = 0.0
        self.speed = 0.0
        speed_z = 0.0

    def decode(self, msg):
        self.direction = msg.direction
        self.speed = msg.speed
        self.speed_z = msg.speed_z

    @thread_safe
    def __repr__(self) -> str:
        return f"(WIND) direction: {self.direction}, speed: {self.speed}, speed_z: {self.speed_z}"