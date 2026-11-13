"""
MovingDropProtocol – Fly a velocity-controlled trajectory and trigger
a payload drop at a specific waypoint index.
"""

import time
from collections.abc import Callable

import numpy as np

from mavcore.mav_protocol import MAVProtocol
from mavcore.mavtypes import Waypoint
from mavcore.messages import (
    AttitudeQuat,
    LocalPositionNED,
    SetpointLocal,
    SetpointVelocity,
)


class MovingDropProtocol(MAVProtocol):

    def __init__(
        self,
        current_pos: LocalPositionNED,
        waypoint: Waypoint,
        drop_time: float,
        drop_callback: Callable[[], None],
        boot_time_ms: int,
        log_func=lambda msg: print(msg),
        target_system: int = 1,
        target_component: int = 0,
        *,
        current_attitude: AttitudeQuat,
    ):
        super().__init__()
        self.current_pos = current_pos
        self.waypoint = waypoint
        self.drop_time = drop_time
        self.drop_callback = drop_callback
        self.boot_time_ms = boot_time_ms
        self.log_func = log_func
        self.target_system = target_system
        self.target_component = target_component
        self.current_attitude = current_attitude

        self.velocity_msg = SetpointVelocity(
            self.target_system,
            self.target_component,
            self.boot_time_ms,
            0.0,
            0.0,
            0.0,
        )

    def run(self, sender, receiver):
        pass
       
