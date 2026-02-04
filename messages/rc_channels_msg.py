from mavcore.mav_message import MAVMessage, thread_safe


class RCChannels(MAVMessage):
    """
    Raw RC input channels as seen by the flight controller.

    Channel values are PWM microseconds (typically 1000-2000).
    These reflect pilot input *before* any AUX function mapping.
    """

    def __init__(self):
        super().__init__("RC_CHANNELS")

        self.time_boot_ms = -1  # time since system boot in ms

        # Raw RC channel values (PWM)
        self.chan1_raw = 0
        self.chan2_raw = 0
        self.chan3_raw = 0
        self.chan4_raw = 0
        self.chan5_raw = 0
        self.chan6_raw = 0
        self.chan7_raw = 0
        self.chan8_raw = 0
        self.chan9_raw = 0
        self.chan10_raw = 0
        self.chan11_raw = 0
        self.chan12_raw = 0
        self.chan13_raw = 0
        self.chan14_raw = 0
        self.chan15_raw = 0
        self.chan16_raw = 0
        self.chan17_raw = 0
        self.chan18_raw = 0

        self.chancount = 0  # number of valid channels

        self.rssi = 0

    def decode(self, msg):
        self.time_boot_ms = msg.time_boot_ms

        self.chan1_raw = msg.chan1_raw
        self.chan2_raw = msg.chan2_raw
        self.chan3_raw = msg.chan3_raw
        self.chan4_raw = msg.chan4_raw
        self.chan5_raw = msg.chan5_raw
        self.chan6_raw = msg.chan6_raw
        self.chan7_raw = msg.chan7_raw
        self.chan8_raw = msg.chan8_raw
        self.chan9_raw = msg.chan9_raw
        self.chan10_raw = msg.chan10_raw
        self.chan11_raw = msg.chan11_raw
        self.chan12_raw = msg.chan12_raw
        self.chan13_raw = msg.chan13_raw
        self.chan14_raw = msg.chan14_raw
        self.chan15_raw = msg.chan15_raw
        self.chan16_raw = msg.chan16_raw
        self.chan17_raw = msg.chan17_raw
        self.chan18_raw = msg.chan18_raw

        self.chancount = msg.chancount

        self.rssi = msg.rssi

    @thread_safe
    def __repr__(self):
        return f"(RC_CHANNELS) timestamp: {self.timestamp}, channels: {self.chancount}, rssi: {self.rssi}"
