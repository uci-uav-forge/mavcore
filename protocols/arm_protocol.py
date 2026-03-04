from ..mav_protocol import MAVProtocol
from ..messages import Arm
from ..messages.command_ack_msg import CommandAck


class ArmProtocol(MAVProtocol):
    """
    Arms device.
    """

    def __init__(self, target_system: int = 1, target_component: int = 0):
        super().__init__()
        self.target_system = target_system
        self.target_component = target_component

        self.arm_msg = Arm(self.target_system, self.target_component)
        self.ack_msg = CommandAck()

    def run(self, sender, receiver):
        future_ack = receiver.wait_for_msg(self.ack_msg, blocking=False)
        sender.send_msg(self.arm_msg)
        future_ack.wait_until_finished()
