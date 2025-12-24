from mavcore.mav_protocol import MAVProtocol
from mavcore.messages.local_position_msg import LocalPosition as LocalPositionNED
from mavcore.messages.command_ack_msg import CommandAck
from mavcore.messages.setpoint_local_msg import SetpointLocal
from gnc.types import Waypoint
import time
import numpy as np


class LocalSetpointProtocol(MAVProtocol):
    """
    Navigates to a list of waypoints. Does not yaw through them.
    Waypoints are in local ned coords xyz, in meters.
    Radius in meters.
    Must be in guided mode.
    """

    def __init__(
        self,
        current_pos: LocalPositionNED,
        waypoints: list[Waypoint],
        boot_time_ms: int,
        target_system: int = 1,
        target_component: int = 0,
    ):
        super().__init__()
        self.current_pos = current_pos
        self.waypoints = waypoints
        self.boot_time_ms = boot_time_ms
        self.target_system = target_system
        self.target_component = target_component

        self.setpoint_msg = SetpointLocal(
            self.target_system, self.target_component, self.boot_time_ms, 0.0, 0.0, 0.0
        )
        self.ack_msg = CommandAck()

    def run(self, sender, receiver):
        for waypoint in self.waypoints:
            waypoint_coords = np.array([waypoint.x, waypoint.y, waypoint.z])
            self.setpoint_msg.load(target=waypoint_coords)
            while (
                np.linalg.norm(waypoint_coords - self.current_pos.get_pos_ned())
                > waypoint.radius
            ):
                sender.send_msg(self.setpoint_msg)
                time.sleep(1)
