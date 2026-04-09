from ..mav_protocol import MAVProtocol
from ..messages import SetMode
from ..mavtypes import FlightMode, FlightModePlane
from ..messages.command_ack_msg import CommandAck


class SetModeProtocol(MAVProtocol):
    """
    Allows to set device mode. Use MAVMode to specify the device mode. Sends change mode then waits for ack.
    """

    def __init__(
        self, mode: FlightMode | FlightModePlane, target_system: int = 1, target_component: int = 0
    ):
        super().__init__()
        self.mode = mode
        self.target_system = target_system
        self.target_component = target_component

        self.mode_msg = SetMode(self.target_system, self.target_component, self.mode)
        self.ack_msg = CommandAck()

    def run(self, sender, receiver):
        future_ack = receiver.wait_for_msg(self.ack_msg, blocking=False)
        sender.send_msg(self.mode_msg)
        future_ack.wait_until_finished()
