from ..mav_protocol import MAVProtocol
from ..messages import Takeoff, CommandAck


class TakeoffProtocol(MAVProtocol):
    """
    Allows to set takeoff to altitude at certain rate. All in local frame meters and m/s.
    Must be in guided mode.
    """

    def __init__(
        self, altitude: float, target_system: int = 1, target_component: int = 0
    ):
        super().__init__()
        self.altitude = altitude
        self.target_system = target_system
        self.target_component = target_component

        self.takeoff_msg = Takeoff(
            self.target_system, self.target_component, self.altitude
        )
        self.ack_msg = CommandAck()

    def run(self, sender, receiver):
        future_ack = receiver.wait_for_msg(self.ack_msg, blocking=False)
        sender.send_msg(self.takeoff_msg)
        future_ack.wait_until_finished()
