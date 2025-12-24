from mavcore.mav_protocol import MAVProtocol
from mavcore.messages import RequestMessageInterval, IntervalMessageID
from mavcore.messages.command_ack_msg import CommandAck


class RequestMessageProtocol(MAVProtocol):
    """
    Requests message to be sent at a specified rate given the message id.
    Defaults to 4hz request rate.
    """

    def __init__(
        self,
        msg_id: IntervalMessageID,
        target_system: int = 1,
        target_component: int = 0,
        rate_hz: float = 4.0,
    ):
        super().__init__()
        self.msg_id = msg_id
        self.target_system = target_system
        self.target_component = target_component

        self.mode_msg = RequestMessageInterval(
            self.target_system, self.target_component, self.msg_id, rate_hz
        )
        self.ack_msg = CommandAck()

    def run(self, sender, receiver):
        future_ack = receiver.wait_for_msg(self.ack_msg, blocking=False)
        sender.send_msg(self.mode_msg)
        future_ack.wait_until_finished()
