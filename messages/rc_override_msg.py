import pymavlink.dialects.v20.all as dialect
import numpy as np
from mavcore.mav_message import MAVMessage, thread_safe


class RCOverride(MAVMessage):
    """
    Send RC channel override values.
    """

    def __init__(self, target_system: int, target_component: int, channels: np.ndarray):
        super().__init__("RC_CHANNELS_OVERRIDE")
        self.target_system = target_system
        self.target_component = target_component
        self.channels = channels

    def encode(self, system_id, component_id):
        return dialect.MAVLink_rc_channels_override_message(
            self.target_system, self.target_component, *self.channels
        )

    @thread_safe
    def __repr__(self):
        return f"(RC_CHANNELS_OVERRIDE) timestamp: {self.timestamp}, channels: {self.channels}"
