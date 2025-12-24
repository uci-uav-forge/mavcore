import numpy as np
from mavcore.mav_message import MAVMessage, thread_safe


class LocalPosition(MAVMessage):
    """
    Gets the local position in NED or ENU frame. Origin is at ardupilot origin which is often at first gps fix.
    In meters for distances and m/s for velocities.
    """

    def __init__(self):
        super().__init__("LOCAL_POSITION_NED")
        self.time_boot_ms = -1  # timestamp (time since system boot) in milliseconds
        self.x = 0.0  # in meters
        self.y = 0.0  # in meters
        self.z = 0.0  # in meters
        self.vx = 0.0  # in m/s
        self.vy = 0.0  # in m/s
        self.vz = 0.0  # in m/s

    def decode(self, msg):
        self.time_boot_ms = msg.time_boot_ms
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz

    @thread_safe
    def get_pos_ned(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    @thread_safe
    def get_pos_enu(self) -> np.ndarray:
        return np.array([self.y, self.x, -self.z])

    @thread_safe
    def get_vel_ned(self) -> np.ndarray:
        return np.array([self.vx, self.vy, self.vz])

    @thread_safe
    def get_vel_enu(self) -> np.ndarray:
        return np.array([self.vy, self.vx, -self.vz])

    @thread_safe
    def __repr__(self) -> str:
        return f"(LOCAL_POSITION) timestamp: {self.timestamp} s \n \
            position: {self.get_pos_enu()}, velocity: {self.get_vel_enu()}"
