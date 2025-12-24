from mavcore.mav_message import MAVMessage, thread_safe

import numpy as np


class AttitudeQuat(MAVMessage):
    """
    Reads ATTITUDE_QUATERNION mavlink message.
    The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
    Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).

    quat_offset: https://mavlink.io/en/messages/common.html#ATTITUDE_QUATERNION
    Rotation offset by which the attitude quaternion and angular speed vector should be rotated for user display
    (quaternion with [w, x, y, z] order, zero-rotation is [1, 0, 0, 0], send [0, 0, 0, 0] if field not supported).
    This field is intended for systems in which the reference attitude may change during flight. For example, tailsitters
    VTOLs rotate their reference attitude by 90 degrees between hover mode and fixed wing mode, thus repr_offset_q is
    equal to [1, 0, 0, 0] in hover mode and equal to [0.7071, 0, 0.7071, 0] in fixed wing mode.
    """

    def __init__(self):
        super().__init__("ATTITUDE_QUATERNION")
        self.w = 1.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.rollspeed = 0.0  # angular speed in radians/sec
        self.pitchspeed = 0.0  # angular speed in radians/sec
        self.yawspeed = 0.0  # angular speed in radians/sec
        self.quat_offset = [0.0, 0.0, 0.0, 0.0]  # Not supported in Ardupilot?

    def decode(self, msg):
        self.w = msg.q1
        self.x = msg.q2
        self.y = msg.q3
        self.z = msg.q4
        self.rollspeed = msg.rollspeed
        self.pitchspeed = msg.pitchspeed
        self.yawspeed = msg.yawspeed
        self.quat_offset = msg.repr_offset_q  # Mavlink 2 only

    @thread_safe
    def get_quat(self) -> np.ndarray:
        return np.array([self.w, self.x, self.y, self.z])

    @thread_safe
    def __repr__(self) -> str:
        return f"(ATTITUDE_QUATERNION) timestamp: {self.timestamp} s \n \
            quat: [{self.w}, {self.x}, {self.y}, {self.z}]"
