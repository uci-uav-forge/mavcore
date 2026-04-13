from ..mav_protocol import MAVProtocol
from ..messages import BatteryStatus


class StreamBatteryProtocol(MAVProtocol):
    """
    Streams the battery status from PDB to FC.
    """

    def __init__(
        self,
        batt_msg: BatteryStatus,
    ):
        super().__init__()
        self.batt_msg = batt_msg

    def run(self, sender, receiver):
        sender.send_msg(self.batt_msg)
