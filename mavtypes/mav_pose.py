import numpy as np
from typing import TypeVar
from typing import NamedTuple
from dataclasses import dataclass
from scipy.spatial.transform import Rotation, Slerp

# This is solely for type hinting the interpolate method that can take another Pose as an argument
Pose_T = TypeVar("Pose_T", bound="Pose")


class Pose(NamedTuple):
    """
    Position is represented as a 3D numpy array (x, y, z).
    Rotation is represented as a scipy Rotation object (quaternion internally).
    Timestamp is in seconds (optional defaults to 0.0).
    """

    position: np.ndarray
    rotation: Rotation
    timestamp: float = 0.0  # in seconds

    @staticmethod
    def identity() -> "Pose":
        return Pose(np.zeros(3), Rotation.identity())

    def to_dict(self) -> dict:
        return {
            "timestamp": self.timestamp,
            "position": self.position.tolist(),
            "rotation_quat": self.rotation.as_quat().tolist(),
        }

    def as_euler(self, seq: str = "zyx", degrees: bool = True) -> np.ndarray:
        """
        Returns the rotation as euler angles in the specified sequence.
        Default is 'zyx' (yaw, pitch, roll) in degrees.
        """
        return self.rotation.as_euler(seq, degrees=degrees)

    def as_quat(self, scalar_first: bool = True) -> np.ndarray:
        """
        Returns the rotation as a quaternion numpy array.
        Default is scalar first (w, x, y, z).
        If scalar_first is False, returns (x, y, z, w).
        """
        quat = self.rotation.as_quat()  # returns (x, y, z, w)
        if scalar_first:
            return np.array([quat[3], quat[0], quat[1], quat[2]])
        return quat

    def as_rotvec(self) -> np.ndarray:
        """
        Returns the rotation as a rotation vector (axis-angle representation).
        """
        return self.rotation.as_rotvec()

    @staticmethod
    def from_dict(d: dict) -> "Pose":
        return Pose(
            position=np.array(d["position"]),
            rotation=Rotation.from_quat(d["rotation_quat"]),
            timestamp=d.get("timestamp", 0.0),
        )

    @staticmethod
    def from_array(
        position: np.ndarray,
        quat: np.ndarray,
        order: bool = True,
        timestamp: float = 0.0,
    ) -> "Pose":
        return Pose(
            position=position,
            rotation=Rotation.from_quat(quat, scalar_first=order),
            timestamp=timestamp,
        )

    def interpolate(
        self, other: Pose_T, proportion: float, timestamp: float = 0.0
    ) -> "Pose":
        """
        Interpolates or extrapolates between two poses.
        "proportion" is effectively a bias between A and B. I
            If proportion is 0, returns self. If proportion is 1, returns other. If proportion is somewhere in between, this function will return something in between.
            If proportion < 0 or > 1, we extrapolate in the direction of self (if < 0) or other (if > 1)
                For example, if proportion is -1, we go 1 unit "under" A (one unit is the delta between self and other)
        """
        new_pos = self.position + proportion * (other.position - self.position)
        if 0 <= proportion <= 1:
            return Pose(
                position=new_pos,
                rotation=Slerp(  # This interpolates two rotations. For complicated math reasons, it isn't as simple as interpolating vectors.
                    [0, 1], Rotation.concatenate([self.rotation, other.rotation])
                )(proportion),
                timestamp=timestamp,
            )
        # extrapolate
        rel_rot = other.rotation * self.rotation.inv()
        return Pose(
            position=new_pos,
            rotation=(  # This extrapolates the rotation beyond 'self.rotation' in the direction of 'other.rotation' by 'proportion'
                # No slerp here because we can scale directly for extrapolation
                Rotation.from_rotvec(rel_rot.as_rotvec() * proportion) * self.rotation
            ),
            timestamp=timestamp,
        )

    def __eq__(self, other: "Pose"):
        return (
            np.allclose(self.position, other.position)
            and self.rotation == other.rotation
        )
