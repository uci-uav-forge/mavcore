from mavcore.mav_protocol import MAVProtocol
from mavcore.messages import LevelCal
from mavcore.messages.command_ack_msg import CommandAck


class LevelCalProtocol(MAVProtocol):
    """
    Calibrates the Level.
    """

    def __init__(self, target_system: int = 1, target_component: int = 0):
        super().__init__()
        self.target_system = target_system
        self.target_component = target_component

        self.arm_msg = LevelCal(self.target_system, self.target_component)
        self.ack_msg = CommandAck()

    def run(self, sender, receiver):
        future_ack = receiver.wait_for_msg(self.ack_msg, blocking=False)
        sender.send_msg(self.arm_msg)
        future_ack.wait_until_finished()
