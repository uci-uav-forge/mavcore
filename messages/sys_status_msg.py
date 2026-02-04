from enum import Enum

from mavcore.mav_message import MAVMessage


class SysStatus(MAVMessage):
    """
    SYS_STATUS message.
    Provides sensor presence, enabled state, and health bitmasks.
    """

    def __init__(self, cb=lambda msg: None):
        super().__init__("SYS_STATUS", callback_func=cb)

        self.present = 0
        self.enabled = 0
        self.health = 0

    def decode(self, msg):
        self.present = msg.onboard_control_sensors_present
        self.enabled = msg.onboard_control_sensors_enabled
        self.health = msg.onboard_control_sensors_health

    def accel_health(self):
        return bool((self.health & 2**1) >> 1)

    def baro_health(self):
        return bool((self.health & 2**3) >> 3)

    def compass_health(self):
        return bool((self.health & 2**2) >> 2)

    def level_health(self):
        return bool((self.health & 2**11) >> 11)

    def estop(self):
        return bool((self.health & 2**15) >> 15)
