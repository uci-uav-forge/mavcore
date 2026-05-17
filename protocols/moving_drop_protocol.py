"""
MovingDropProtocol – Fly a velocity-controlled trajectory and trigger
a payload drop at a specific waypoint index.
"""

from mavcore.mav_protocol import MAVProtocol
from mavcore.messages import SetpointVelocity, LocalPositionNED
from mavcore.mavtypes import Waypoint
import time
import numpy as np
from typing import Callable


class MovingDropProtocol(MAVProtocol):
    """
    Velocity-controlled waypoint navigation with mid-flight payload release.
    """

    def __init__(
        self,
        current_pos: LocalPositionNED,
        waypoints: list[Waypoint],
        release_index: int,
        drop_callback: Callable[[], None],
        boot_time_ms: int,
        log_func=lambda msg: print(msg),
        target_system: int = 1,
        target_component: int = 0,
    ):
        super().__init__()
        self.current_pos = current_pos
        self.waypoints = waypoints
        self.release_index = release_index
        self.drop_callback = drop_callback
        self.boot_time_ms = boot_time_ms
        self.log_func = log_func
        self.target_system = target_system
        self.target_component = target_component

        self._dropped = False  # ensure we only drop once

        self.velocity_msg = SetpointVelocity(
            self.target_system,
            self.target_component,
            self.boot_time_ms,
            0.0,
            0.0,
            0.0,
        )

    @staticmethod
    def _velocity_vector(
        current: np.ndarray,
        target: np.ndarray,
        speed: float,
    ) -> np.ndarray:
        """Unit direction * speed.  Returns zero if on-target."""
        direction = target - current
        dist = np.linalg.norm(direction)
        if dist < 0.1:
            return np.zeros(3)
        return (direction / dist) * speed

    def run(self, sender, receiver):
        """
        Navigate through all waypoints; fire drop_callback at release_index.
        """
        for i, waypoint in enumerate(self.waypoints):
            wp_coords = np.array([waypoint.x, waypoint.y, waypoint.z])
            is_release = i == self.release_index
            phase = "RELEASE" if is_release else "NAV"

            self.log_func(
                f"[MovingDrop] WP {i + 1}/{len(self.waypoints)} "
                f"[{phase}] @ {waypoint.speed:.1f} m/s"
            )

            while True:
                current_position = self.current_pos.get_pos_ned()
                distance = np.linalg.norm(wp_coords - current_position)

                if is_release and not self._dropped and distance <= waypoint.radius:
                    self.log_func(
                        f"[MovingDrop] >>> RELEASE triggered at d={distance:.2f}m <<<"
                    )
                    self._dropped = True
                    self.drop_callback()

                if distance <= waypoint.radius:
                    if is_release:
                        # Don't stop — fly through to next waypoint
                        self.log_func("[MovingDrop] Passing through release point")
                    else:
                        self.log_func(f"[MovingDrop] Reached waypoint {i + 1}")
                        # Brief zero-velocity command on non-critical waypoints
                        self.velocity_msg.load(np.zeros(3))
                        sender.send_msg(self.velocity_msg)
                    break

                velocity = self._velocity_vector(
                    current_position, wp_coords, waypoint.speed
                )
                self.velocity_msg.load(velocity)
                sender.send_msg(self.velocity_msg)
                time.sleep(0.25)

        # Final stop
        self.velocity_msg.load(np.zeros(3))
        sender.send_msg(self.velocity_msg)
        self.log_func("[MovingDrop] Trajectory complete")
