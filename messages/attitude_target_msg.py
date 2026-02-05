import pymavlink.dialects.v20.all as dialect
import time
import numpy as np
from mavcore.mav_message import MAVMessage


class SetpointAttitude(MAVMessage):
    """
    Attitude + thrust setpoint using MAVLink SET_ATTITUDE_TARGET.

    Quaternion is (w, x, y, z).
    Body rates are in rad/s.
    Thrust is typically normalized [0.0, 1.0] on ArduCopter, depending on GUID_OPTIONS.
    """

    def __init__(
        self,
        target_system: int,
        target_component: int,
        boot_time_ms: int,
        q: np.ndarray,
        thrust: float,
        body_roll_rate: float = 0.0,
        body_pitch_rate: float = 0.0,
        body_yaw_rate: float = 0.0,
        type_mask: int = -1,
    ):
        super().__init__("CUSTOM_SETPOINT_ATTITUDE")

        self.target_system = target_system
        self.target_component = target_component

        self.boot_time_ms = boot_time_ms  # time since system boot in milliseconds (for sync)

        # quaternion (w, x, y, z)
        self.q = np.array(q, dtype=float)
        if self.q.shape != (4,):
            raise ValueError("q must be a 4-element array: [w, x, y, z]")

        # thrust 0..1
        self.thrust = float(thrust)

        # body rates (rad/s)
        self.body_roll_rate = float(body_roll_rate)
        self.body_pitch_rate = float(body_pitch_rate)
        self.body_yaw_rate = float(body_yaw_rate)

        # If not provided, default to "use quaternion + thrust, ignore all body rates"
        # MAVLink SET_ATTITUDE_TARGET type_mask bits:
        # 0 ignore body roll rate
        # 1 ignore body pitch rate
        # 2 ignore body yaw rate
        # 6 ignore attitude (quaternion)
        # 7 ignore thrust
        self.type_mask = int(type_mask) if type_mask != -1 else int(0b00000111)

    def encode(self, system_id, component_id):
        return dialect.MAVLink_set_attitude_target_message(
            time_boot_ms=int(time.time() * 1000 - self.boot_time_ms),
            target_system=int(self.target_system),
            target_component=int(self.target_component),
            type_mask=int(self.type_mask),
            q=[float(self.q[0]), float(self.q[1]), float(self.q[2]), float(self.q[3])],
            body_roll_rate=float(self.body_roll_rate),
            body_pitch_rate=float(self.body_pitch_rate),
            body_yaw_rate=float(self.body_yaw_rate),
            thrust=float(self.thrust),
        )

    def load_quat_thrust(self, q: np.ndarray, thrust: float):
        q = np.array(q, dtype=float)
        if q.shape != (4,):
            raise ValueError("q must be a 4-element array: [w, x, y, z]")
        self.q = q
        self.thrust = float(thrust)

    def load_body_rates(self, pqr: np.ndarray):
        pqr = np.array(pqr, dtype=float)
        if pqr.shape != (3,):
            raise ValueError("pqr must be 3-element array: [p, q, r] rad/s")
        self.body_roll_rate = float(pqr[0])
        self.body_pitch_rate = float(pqr[1])
        self.body_yaw_rate = float(pqr[2])

    def set_ignore_body_rates(self, ignore: bool = True):
        if ignore:
            self.type_mask |= (1 << 0) | (1 << 1) | (1 << 2)
        else:
            self.type_mask &= ~((1 << 0) | (1 << 1) | (1 << 2))

    def set_ignore_attitude(self, ignore: bool = True):
        if ignore:
            self.type_mask |= (1 << 6)
        else:
            self.type_mask &= ~(1 << 6)

    def set_ignore_thrust(self, ignore: bool = True):
        if ignore:
            self.type_mask |= (1 << 7)
        else:
            self.type_mask &= ~(1 << 7)

    def __repr__(self):
        return (
            super().__repr__()
            + f", boot: {self.boot_time_ms}, q(wxyz): [{self.q[0]:.4f}, {self.q[1]:.4f}, {self.q[2]:.4f}, {self.q[3]:.4f}]"
            + f", thrust: {self.thrust:.3f}, pqr(rad/s): [{self.body_roll_rate:.3f}, {self.body_pitch_rate:.3f}, {self.body_yaw_rate:.3f}]"
            + f", type_mask: {bin(self.type_mask)}"
        )
