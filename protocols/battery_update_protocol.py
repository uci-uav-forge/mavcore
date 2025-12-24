from mavcore.mav_protocol import MAVProtocol
from mavcore.messages import BatteryStatus


class UpdateBatteryProtocol(MAVProtocol):
    """
    Updates the battery status.
    """

    def __init__(
        self,
        batt_msg: BatteryStatus,
        target_system: int = 1,
        target_component: int = 0,
    ):
        super().__init__()
        self.batt_msg = batt_msg
        self.target_system = target_system
        self.target_component = target_component

    def run(self, sender, receiver):
        sender.send_msg(self.batt_msg)
