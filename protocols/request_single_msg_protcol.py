from ..mav_protocol import MAVProtocol
from ..messages import RequestSingleMessage, IntervalMessageID
from ..messages.command_ack_msg import CommandAck


class RequestMessageProtocol(MAVProtocol):
    """
    Requests single message to be sent given the message id.
    """

    def __init__(
        self,
        msg_id: IntervalMessageID,
        target_system: int = 1,
        target_component: int = 0,
        wait_for_ack: bool = True,
    ):
        super().__init__()
        self.msg_id = msg_id
        self.target_system = target_system
        self.target_component = target_component
        self.wait_for_ack = wait_for_ack

        self.mode_msg = RequestSingleMessage(
            self.target_system, self.target_component, self.msg_id
        )
        self.ack_msg = CommandAck()

    def run(self, sender, receiver):
        future_ack = receiver.wait_for_msg(self.ack_msg, blocking=False)
        sender.send_msg(self.mode_msg)
        if self.wait_for_ack:
            future_ack.wait_until_finished()
