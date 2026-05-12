import time
from ..messages.command_ack_msg import CommandAck
from pymavlink.dialects.v20 import common as mav
from ..messages import MissionAck
from ..mav_protocol import MAVProtocol
from ..messages import FenceMissionClearAll


class FenceClearProtocol(MAVProtocol):
    """
    Sends MISSION_CLEAR_ALL (mission_type = FENCE) and waits for MISSION_ACK(FENCE).
    """

    def __init__(self, target_system: int = 1, target_component: int = 0):
        super().__init__()
        self.target_system = target_system
        self.target_component = target_component

        self.clear_msg = FenceMissionClearAll(
            target_system=self.target_system,
            target_component=self.target_component,
        )

        self.ack_msg = MissionAck()

    def run(self, sender, receiver):
        future_ack = receiver.wait_for_msg(self.ack_msg, blocking=False)
        sender.send_msg(self.clear_msg)
        future_ack.wait_until_finished()
