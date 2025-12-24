import pymavlink.dialects.v20.all as dialect
import time
import numpy as np
from mavcore.mav_message import MAVMessage, thread_safe


class SetpointLocal(MAVMessage):
    """
    A position setpoint in local NED frame. Measured in meters.
    Boot time ms is the time since system boot in ms.
    """

    def __init__(
        self,
        target_system: int,
        target_component: int,
        boot_time_ms: int,
        x: float,
        y: float,
        z: float,
    ):
        super().__init__("CUSTOM_SETPOINT_LOCAL")
        self.target_system = target_system
        self.target_component = target_component

        self.boot_time_ms = (
            boot_time_ms  # time since system boot in milliseconds (for sync)
        )
        self.x = x  # in meters
        self.y = y  # in meters
        self.z = z  # in meters

    def encode(self, system_id, component_id):
        return dialect.MAVLink_set_position_target_local_ned_message(
            time_boot_ms=int(time.time() * 1000 - self.boot_time_ms),
            target_system=int(self.target_system),
            target_component=int(self.target_component),
            coordinate_frame=int(1),  # MAV_FRAME_LOCAL_NED
            type_mask=int(4088 - 1024),  # ignore all but x/y/z position
            x=float(self.x),
            y=float(self.y),
            z=float(self.z),
            vx=float(0.0),
            vy=float(0.0),
            vz=float(0.0),
            afx=float(0.0),
            afy=float(0.0),
            afz=float(0.0),
            yaw=float(0.0),
            yaw_rate=float(0.0),
        )

    @thread_safe
    def load(self, target: np.ndarray):
        self.x = target[0]
        self.y = target[1]
        self.z = target[2]

    @thread_safe
    def __repr__(self):
        return (
            super().__repr__()
            + f", boot: {self.boot_time_ms}, x: {self.x}, y: {self.y}, z: {self.z}"
        )
