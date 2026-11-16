import time
from collections.abc import Callable
from threading import Thread

import numpy as np

from ..mav_protocol import MAVProtocol
from ..mavtypes import Waypoint
from ..messages import LocalPositionNED, SetpointVelocity


class MovingDropProtocol(MAVProtocol):
    """
    Flies waypoints with velocity setpoints and fires drop_callback without stopping.

    The release waypoint counts as reached when the vehicle crosses the line through
    it perpendicular to the leg, within its radius horizontally. All other waypoints
    are reached inside their radius.

    drop_callback returns whether the release succeeded. It runs on its own thread
    because payload commands block for seconds, far longer than the vehicle can go
    without setpoints at speed.

    Raises RuntimeError if any waypoint takes longer than leg_timeout_s to reach.
    """

    UPDATE_PERIOD_S = 0.05

    def __init__(
        self,
        current_pos: LocalPositionNED,
        waypoints: list[Waypoint],
        release_index: int,
        drop_callback: Callable[[], bool],
        boot_time_ms: int,
        log_func=lambda msg: print(msg),
        target_system: int = 1,
        target_component: int = 0,
        leg_timeout_s: float = 120.0,
    ):
        super().__init__()
        self.current_pos = current_pos
        self.waypoints = waypoints
        self.release_index = release_index
        self.drop_callback = drop_callback
        self.log_func = log_func
        self.leg_timeout_s = leg_timeout_s

        self.velocity_msg = SetpointVelocity(
            target_system, target_component, boot_time_ms, 0.0, 0.0, 0.0
        )

    def run(self, sender, receiver):
        release_thread = None

        for i, waypoint in enumerate(self.waypoints):
            is_release = i == self.release_index
            target = np.array([waypoint.x, waypoint.y, waypoint.z])
            leg_direction = self._horizontal_direction(
                self.current_pos.get_pos_ned(), target
            )
            deadline = time.monotonic() + self.leg_timeout_s

            self.log_func(
                f"[MovingDrop] Waypoint {i + 1}/{len(self.waypoints)} "
                f"@ {waypoint.speed:.1f} m/s"
            )

            while True:
                position = self.current_pos.get_pos_ned()
                if is_release:
                    reached = self._crossed_release(
                        position, target, leg_direction, waypoint.radius
                    )
                else:
                    reached = np.linalg.norm(target - position) <= waypoint.radius
                if reached:
                    break

                if time.monotonic() > deadline:
                    raise RuntimeError(
                        f"Moving drop waypoint {i + 1} not reached within "
                        f"{self.leg_timeout_s} s"
                    )

                self._send_velocity(
                    sender, self._velocity_to(position, target, waypoint.speed)
                )
                time.sleep(self.UPDATE_PERIOD_S)

            if is_release:
                self._log_release(position, target)
                release_thread = Thread(target=self._release, daemon=True)
                release_thread.start()

        self._send_velocity(sender, np.zeros(3))
        if release_thread is not None:
            release_thread.join()
        self.log_func("[MovingDrop] Trajectory complete")

    def _send_velocity(self, sender, velocity: np.ndarray) -> None:
        self.velocity_msg.load(velocity)
        sender.send_msg(self.velocity_msg)

    def _release(self) -> None:
        try:
            released = self.drop_callback()
        except Exception as e:  # noqa: BLE001 - a failed release must not stop the flight
            self.log_func(f"[MovingDrop] Release raised: {e}")
            return
        self.log_func(f"[MovingDrop] Release confirmed: {released}")

    def _log_release(self, position: np.ndarray, target: np.ndarray) -> None:
        offset = np.linalg.norm((target - position)[:2])
        speed = np.linalg.norm(self.current_pos.get_vel_ned()[:2])
        self.log_func(
            f"[MovingDrop] Releasing {offset:.2f} m from release point "
            f"at {speed:.2f} m/s ground speed, altitude {-position[2]:.2f} m"
        )

    @staticmethod
    def _velocity_to(
        position: np.ndarray, target: np.ndarray, speed: float
    ) -> np.ndarray:
        direction = target - position
        distance = np.linalg.norm(direction)
        if distance < 0.1:
            return np.zeros(3)
        return direction / distance * speed

    @staticmethod
    def _horizontal_direction(position: np.ndarray, target: np.ndarray) -> np.ndarray:
        direction = (target - position)[:2]
        distance = np.linalg.norm(direction)
        if distance < 0.1:
            return np.zeros(2)
        return direction / distance

    @staticmethod
    def _crossed_release(
        position: np.ndarray,
        target: np.ndarray,
        leg_direction: np.ndarray,
        radius: float,
    ) -> bool:
        remaining = (target - position)[:2]
        return remaining @ leg_direction <= 0.0 and np.linalg.norm(remaining) <= radius
