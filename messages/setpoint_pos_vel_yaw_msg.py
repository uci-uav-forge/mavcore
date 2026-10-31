import time

import pymavlink.dialects.v20.all as dialect

from mavcore.mav_message import MAVMessage


class SetpointPosVelYaw(MAVMessage):
    TYPE_MASK = 0b0000100000000000

    def __init__(
        self,
        target_system,
        target_component,
        boot_time_ms,
        px,
        py,
        pz,
        vx,
        vy,
        vz,
        afx,
        afy,
        afz,
        yaw,
    ):
        super().__init__("SETPOINT_POS_VEL_YAW")
        self.target_system = target_system
        self.target_component = target_component
        self.boot_time_ms = boot_time_ms
        self.px = px
        self.py = py
        self.pz = pz
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.afx = afx
        self.afy = afy
        self.afz = afz
        self.yaw = yaw

    def encode(self, system_id, component_id):
        return dialect.MAVLink_set_position_target_local_ned_message(
            time_boot_ms=int(time.time() * 1000 - self.boot_time_ms),
            target_system=int(self.target_system),
            target_component=int(self.target_component),
            coordinate_frame=1,
            type_mask=self.TYPE_MASK,
            x=float(self.px),
            y=float(self.py),
            z=float(self.pz),
            vx=float(self.vx),
            vy=float(self.vy),
            vz=float(self.vz),
            afx=float(self.afx),
            afy=float(self.afy),
            afz=float(self.afz),
            yaw=float(self.yaw),
            yaw_rate=0.0,
        )
