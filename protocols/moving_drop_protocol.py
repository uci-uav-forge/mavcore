"""
MovingDropProtocol – Fly a velocity-controlled trajectory and trigger
a payload drop at a specific waypoint index.

This extends the velocity-setpoint paradigm used by VelocitySetpointProtocol
but adds drop-trigger logic:

    1.  Navigate through waypoints using velocity vectors (same as goto).
    2.  At every control tick, check whether we have reached the *release*
        waypoint (identified by ``release_index``).
    3.  When the release waypoint radius is entered, immediately call
        ``drop_callback()`` — which is wired to PayloadManager.drop_bottle()
        or drop_beacon() by the caller.
    4.  Do **not** stop or slow down at the release point; keep flying
        through to the exit waypoint so the payload inherits full velocity.

Key differences from VelocitySetpointProtocol:
    • ``slow_on_approach`` is forced OFF for the release waypoint so constant
      speed is maintained through the drop.
    • A ``drop_callback`` is invoked exactly once when the release radius is
      crossed.
    • The protocol logs timestamped drop events for post-flight analysis.
"""

from mavcore.mav_protocol import MAVProtocol
from mavcore.messages import SetpointVelocity, LocalPositionNED
from mavcore.types import Waypoint
import time
import numpy as np
from typing import Callable


class MovingDropProtocol(MAVProtocol):
    """
    Velocity-controlled waypoint navigation with mid-flight payload release.

    Parameters
    ----------
    current_pos : LocalPositionNED
        Live position listener from the flight controller.
    waypoints : list[Waypoint]
        Ordered waypoints produced by MovingDrop.get_waypoints().
    release_index : int
        Index into *waypoints* where the drop should happen (typically 2).
    drop_callback : callable
        Function to call when the release point is reached.  Expected
        signature: ``() -> bool``.  Should trigger the servo/payload.
    boot_time_ms : int
        Milliseconds since system boot (for MAVLink time sync).
    log_func : callable
        Logging function.
    target_system / target_component : int
        MAVLink target IDs.
    """

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

    # ---- helpers ----

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

    # ---- protocol entry point ----

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

                # ---- release check ----
                if is_release and not self._dropped and distance <= waypoint.radius:
                    self.log_func(
                        f"[MovingDrop] >>> RELEASE triggered at d={distance:.2f}m <<<"
                    )
                    self._dropped = True
                    self.drop_callback()

                # ---- waypoint reached ----
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

                # ---- velocity command ----
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
