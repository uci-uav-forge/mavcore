from mavcore.mav_message import MAVMessage, thread_safe
from mavcore.messages.attitude_msg import Attitude
from mavcore.messages.attitude_quat_msg import AttitudeQuat
from mavcore.messages.local_position_msg import LocalPosition
from mavcore.messages.global_position_msg import GlobalPosition

import bisect
import numpy as np
from mavcore.types.mav_pose import Pose


class FullPose(MAVMessage):
    """
    Reads and stores full pose information from the vehicle, including:
    - Attitude (quaternion)
    - Local Position (x, y, z in NED frame and velocities)
    - Global Position (latitude, longitude, altitude)

    Contains an additonal method to get an interpolated local pose at a given timestamp.
    """

    def __init__(self):
        super().__init__("FULL_POSE")
        self.attitude = AttitudeQuat()
        self.local_position = LocalPosition()
        self.global_position = GlobalPosition()
        self.submessages = [
            self.attitude,
            self.local_position,
            self.global_position,
        ]

        # interpolation params
        self.local_position.callback_func = self.pose_callback
        self.pose_buffer: list[Pose] = []
        self.timestamp_buffer: list[float] = []
        self.buffer_size = 200

    def get_local_position(self, timestamp=None) -> Pose:
        if timestamp is not None:
            return self._get_interpolated_pose(timestamp)
        return Pose.from_array(
            position=self.local_position.get_pos_enu(),
            quat=self.attitude.get_quat(),
            order=True,
            timestamp=self.local_position.timestamp,
        )

    @thread_safe
    def get_local_velocity(self) -> np.ndarray:
        return self.local_position.get_vel_enu()

    @thread_safe
    def get_global_position(self) -> np.ndarray:
        return self.global_position.get_pos()

    @thread_safe
    def get_global_velocity(self) -> np.ndarray:
        return np.array(self.global_position.get_vel())

    def pose_callback(self, msg):
        """
        Updates the pose buffer with the current local position and attitude.
        Maintains the buffer size by removing the oldest entry if necessary.
        """
        current_pose = self.get_local_position()
        if abs(self.attitude.timestamp - self.local_position.timestamp) > 0.1:
            print(
                f"Warning: Discarding pose since Attitude({self.attitude.get_hz()}hz) and Local Position({self.local_position.get_hz()}hz) timestamps differ by more than 100 ms."
            )
            return
        self.update_timestamp(
            np.average([self.attitude.timestamp, self.local_position.timestamp])
        )

        self.pose_buffer.append(current_pose)
        self.timestamp_buffer.append(self.timestamp)
        assert len(self.pose_buffer) == len(self.timestamp_buffer)
        if len(self.pose_buffer) > self.buffer_size:
            self.pose_buffer.pop(0)
            self.timestamp_buffer.pop(0)

    def _get_interpolated_pose(self, timestamp: int) -> Pose:
        """
        Returns an interpolated pose at the given timestamp using the pose buffer.
        If the timestamp is outside the buffer range, it extrapolates using the closest two poses.
        """
        if len(self.pose_buffer) < 2:
            if len(self.pose_buffer) == 1:
                return self.pose_buffer[0]
            print(
                "Not enough data in pose buffer to interpolate. Returning identity pose."
            )
            return Pose.identity()

        # if timestamp is outside the buffer range, extrapolate
        if timestamp < self.timestamp_buffer[0]:
            pose0 = self.pose_buffer[0]
            pose1 = self.pose_buffer[1]
            proportion = (timestamp - self.timestamp_buffer[0]) / (
                self.timestamp_buffer[1] - self.timestamp_buffer[0]
            )
            return pose0.interpolate(pose1, proportion, timestamp)
        elif timestamp > self.timestamp_buffer[-1]:
            pose0 = self.pose_buffer[-2]
            pose1 = self.pose_buffer[-1]
            proportion = (timestamp - self.timestamp_buffer[-2]) / (
                self.timestamp_buffer[-1] - self.timestamp_buffer[-2]
            )
            return pose0.interpolate(pose1, proportion, timestamp)

        # find the two poses surrounding the timestamp
        idx = bisect.bisect_left(self.timestamp_buffer, timestamp)
        t0, pose0 = self.timestamp_buffer[idx - 1], self.pose_buffer[idx - 1]
        t1, pose1 = self.timestamp_buffer[idx], self.pose_buffer[idx]
        proportion = (timestamp - t0) / (t1 - t0)
        # print(f"Interpolating between t0={t0} s and t1={t1} s for requested timestamp={timestamp} with proportion={proportion}")
        return pose0.interpolate(pose1, proportion, timestamp)

    def __repr__(self):
        out = "FullPose : Timestamp: " + str(self.timestamp) + " s\n"
        for sub in self.submessages:
            out += sub.__repr__() + "\n"
        out += "\n"
        return out
