from mavcore.mav_protocol import MAVProtocol
from mavcore.messages import RCOverride
import numpy as np


class RCOverrideProtocol(MAVProtocol):
    """
    RC Override Protocol to send RC channel override commands.
    """

    def __init__(
        self,
        channels=np.zeros(18, dtype=np.uint16),
        target_system: int = 1,
        target_component: int = 0,
    ):
        super().__init__()
        self.target_system = target_system
        self.target_component = target_component
        self.channels = channels

        self.rc_override_msg = RCOverride(
            self.target_system, self.target_component, channels
        )

    def run(self, sender, receiver):
        sender.send_msg(self.rc_override_msg)

    def set_channels(self, channels):
        self.channels = channels
        self.rc_override_msg.channels = channels
