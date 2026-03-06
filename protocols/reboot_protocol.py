from ..mav_protocol import MAVProtocol
from ..messages import RebootMsg
from ..messages.command_ack_msg import CommandAck


class RebootProtocol(MAVProtocol):
    """
    Requests message to be sent at default interval given the message id.
    """

    def __init__(
        self,
        target_system: int = 1,
        target_component: int = 0,
    ):
        super().__init__()
        self.target_system = target_system
        self.target_component = target_component

        self.reboot_msg = RebootMsg(self.target_system, self.target_component)
        self.ack_msg = CommandAck()

    def run(self, sender, receiver):
        future_ack = receiver.wait_for_msg(self.ack_msg, blocking=False)
        sender.send_msg(self.reboot_msg)
        future_ack.wait_until_finished()
