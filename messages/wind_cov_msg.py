from mavcore.mav_message import MAVMessage, thread_safe


class WindCov(MAVMessage):
    """
    Reads WIND_COV mavlink message.
    """

    def __init__(self):
        super().__init__("WIND_COV")
        self.time_usec = 0.0  # Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
        self.wind_x = 0.0  # Wind in North (NED) direction (NAN if unknown)
        self.wind_y = 0.0  # Wind in East (NED) direction (NAN if unknown)
        self.wind_z = 0.0  # Wind in Down (NED) direction (NAN if unknown)
        self.var_horiz = 0.0  # Variability of wind in XY, 1-STD estimated from a 1 Hz lowpassed wind estimate (NAN if unknown)
        self.var_vert = 0.0  # Variability of wind in Z, 1-STD estimated from a 1 Hz lowpassed wind estimate (NAN if unknown)
        self.wind_alt = 0.0  # Altitude (MSL) that this measurement was taken at (NAN if unknown)
        self.horiz_accuracy = 0.0  # Horizontal speed 1-STD accuracy (0 if unknown)
        self.vert_accuracy = 0.0  # Vertical speed 1-STD accuracy (0 if unknown)

    def decode(self, msg):
        self.time_usec = msg.time_usec
        self.wind_x = msg.wind_x
        self.wind_y = msg.wind_y
        self.wind_z = msg.wind_z
        self.var_horiz = msg.var_horiz
        self.var_vert = msg.var_vert
        self.wind_alt = msg.wind_alt 
        self.horiz_accuracy = msg.horiz_accuracy
        self.vert_accuracy = msg.vert_accuracy  

    @thread_safe
    def __repr__(self) -> str:
        return f"(WIND_COV) timestamp: {self.time_usec} μs, wind_x: {self.wind_x}, wind_y: {self.wind_y}, wind_z: {self.wind_z}, var_horiz: {self.var_horiz}, var_vert: {self.var_vert}, wind_alt: {self.wind_alt}, horiz_accuracy: {self.horiz_accuracy}, vert_accuracy: {self.vert_accuracy}"