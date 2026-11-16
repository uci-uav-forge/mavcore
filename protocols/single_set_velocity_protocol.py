from ..mav_protocol import MAVProtocol
from ..messages import SetpointVelocity, CommandAck, LocalPositionNED
from ..mavtypes import Waypoint
import time
import numpy as np
from jaxtyping import Float


class SingleVelocitySetpointProtocol(MAVProtocol):
    """
    Navigates to waypoints using velocity control for speed optimization.
    Sends velocity vectors instead of positions.
    """

    def __init__(
        self,
        velocity_vector: Float[np.ndarray, "3"],
        do_yaw: bool,
        boot_time_ms: int,
        log_func=lambda msg: print(msg),
        target_system: int = 1,
        target_component: int = 0,
    ):
        super().__init__()
        self.velocity_vector = self.velocity_vector
        self.do_yaw = do_yaw
        self.boot_time_ms = boot_time_ms
        self.target_system = target_system
        self.target_component = target_component
        self.log_func = log_func

        vx: float = self.velocity_vector[0]
        vy: float = self.velocity_vector[1]
        vz: float = self.velocity_vector[2]
        self.velocity_msg = SetpointVelocity(
            self.target_system,
            self.target_component,
            self.boot_time_ms,
            vx,
            vy,
            vz,
            self.do_yaw,
        )

        self.ack_msg = CommandAck()

    def update_velocity(
        self, velocity_vector: Float[np.ndarray, "3"], do_yaw: bool | None = None
    ) -> None:
        self.velocity_vector = velocity_vector
        if do_yaw is not None:
            self.do_yaw = do_yaw

        self.velocity_msg.vx = self.velocity_vector[0]
        self.velocity_msg.vy = self.velocity_vector[1]
        self.velocity_msg.vz = self.velocity_vector[2]
        self.velocity_msg.do_yaw = self.do_yaw

    def run(self, sender, receiver):
        future_ack = receiver.wait_for_msg(self.ack_msg, blocking=False)
        sender.send_msg(self.velocity_msg)
        future_ack.wait_until_finished()
