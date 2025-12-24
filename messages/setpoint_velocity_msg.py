import pymavlink.dialects.v20.all as dialect
import time
import numpy as np
from mavcore.mav_message import MAVMessage


class SetpointVelocity(MAVMessage):
    """
    A velocity setpoint in local NED frame. Measured in m/s.
    Uses same MAVLink message as SetpointLocal, but different type mask.
    """

    def __init__(
        self,
        target_system: int,
        target_component: int,
        boot_time_ms: int,
        vx: float,
        vy: float,
        vz: float,
    ):
        super().__init__("CUSTOM_SETPOINT_VELOCITY")
        self.target_system = target_system
        self.target_component = target_component

        self.boot_time_ms = (
            boot_time_ms  # time since system boot in milliseconds (for sync)
        )
        self.vx = vx  # velocity North in m/s
        self.vy = vy  # velocity East in m/s
        self.vz = vz  # velocity Down in m/s

    def encode(self, system_id, component_id):
        return dialect.MAVLink_set_position_target_local_ned_message(
            time_boot_ms=int(time.time() * 1000 - self.boot_time_ms),
            target_system=int(self.target_system),
            target_component=int(self.target_component),
            coordinate_frame=int(1),  # MAV_FRAME_LOCAL_NED
            type_mask=int(
                0b0000101111000111
            ),  # Change type_mask to ignore all but velocity
            x=float(0.0),
            y=float(0.0),
            z=float(0.0),
            vx=float(self.vx),
            vy=float(self.vy),
            vz=float(self.vz),
            afx=float(0.0),
            afy=float(0.0),
            afz=float(0.0),
            yaw=float(0.0),
            yaw_rate=float(0.0),
        )

    def load(self, velocity: np.ndarray):
        self.vx = velocity[0]
        self.vy = velocity[1]
        self.vz = velocity[2]

    def __repr__(self):
        return (
            super().__repr__()
            + f", boot: {self.boot_time_ms}, vx: {self.vx:.2f}, vy: {self.vy:.2f}, vz: {self.vz:.2f}"
        )


# Duplicate class implementation removed.
