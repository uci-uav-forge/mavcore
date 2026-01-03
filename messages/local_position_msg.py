import numpy as np
from mavcore.mav_message import MAVMessage, thread_safe


class LocalPosition(MAVMessage):
    """
    Gets the local position in NED or ENU frame. Origin is at ardupilot origin which is often at first gps fix.
    In meters for distances and m/s for velocities.
    """

    def __init__(self):
        super().__init__("LOCAL_POSITION_NED_COV")
        self.time_boot_ms = -1  # timestamp (time since system boot) in milliseconds
        self.x = 0.0  # in meters
        self.y = 0.0  # in meters
        self.z = 0.0  # in meters
        self.vx = 0.0  # in m/s
        self.vy = 0.0  # in m/s
        self.vz = 0.0  # in m/s
        self.ax = 0.0  # in m/s^2
        self.ay = 0.0  # in m/s^2
        self.az = 0.0  # in m/s^2
        self.estimator_type = 0  # type of estimator
        self.covariance: list[float] = (
            [0.0] * 45
        )  # Row-major representation of position, velocity and acceleration 9x9 cross-covariance matrix upper right triangle

    def _process_covariance(
        self, covariance: list[float], enu: bool = False
    ) -> np.ndarray:
        """
        Row-major representation of position, velocity and acceleration 9x9 cross-covariance matrix upper right triangle
        (states: x, y, z, vx, vy, vz, ax, ay, az; first nine entries are the first ROW, next eight entries are the second row, etc.)
        Converts to a 9x9 numpy array.
        """
        processed_covariance = np.zeros((9, 9))
        index = 0
        for i in range(9):
            for j in range(i, 9):
                processed_covariance[i, j] = covariance[index]
                if i != j:
                    processed_covariance[j, i] = covariance[index]
                index += 1
        if enu:
            # swap x and y, and invert z
            ned_to_enu_matrix = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
            rotation_matrix = np.block(
                [
                    [ned_to_enu_matrix, np.zeros((3, 6))],
                    [np.zeros((3, 3)), ned_to_enu_matrix, np.zeros((3, 3))],
                    [np.zeros((3, 6)), ned_to_enu_matrix],
                ]
            )
            processed_covariance = (
                rotation_matrix @ processed_covariance @ rotation_matrix.T
            )
        return processed_covariance

    def decode(self, msg):
        self.time_boot_ms = msg.time_usec // 1000  # convert to milliseconds
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz
        self.ax = msg.ax
        self.ay = msg.ay
        self.az = msg.az
        self.estimator_type = msg.estimator_type
        self.covariance = list(msg.covariance)

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
    def get_covariance_ned(self) -> np.ndarray:
        return self._process_covariance(self.covariance, enu=False)

    @thread_safe
    def get_covariance_enu(self) -> np.ndarray:
        return self._process_covariance(self.covariance, enu=True)

    @thread_safe
    def __repr__(self) -> str:
        return f"(LOCAL_POSITION) timestamp: {self.timestamp} s \n \
            position: {self.get_pos_enu()}, velocity: {self.get_vel_enu()}"
