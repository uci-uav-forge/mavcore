from ..mav_protocol import MAVProtocol
from ..messages import SetHome
from ..messages.command_ack_msg import CommandAck


class SetHomeProtocol(MAVProtocol):
    """
    Sends message to set RTL home to current position and then waits for ack.
    """

    def __init__(self, target_system: int = 1, target_component: int = 0):
        super().__init__()
        self.target_system = target_system
        self.target_component = target_component

        self.set_home_msg = SetHome(self.target_system, self.target_component)
        self.ack_msg = CommandAck()

    def run(self, sender, receiver):
        future_ack = receiver.wait_for_msg(self.ack_msg, blocking=False)
        sender.send_msg(self.set_home_msg)
        future_ack.wait_until_finished()
