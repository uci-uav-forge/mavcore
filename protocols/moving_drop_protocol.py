"""
MovingDropProtocol – Fly a velocity-controlled trajectory and trigger
a payload drop at a specific waypoint index.
"""

import time
from collections.abc import Callable

import numpy as np

from mavcore.mav_protocol import MAVProtocol
from mavcore.mavtypes import Waypoint
from mavcore.messages import (
    AttitudeQuat,
    LocalPositionNED,
    SetpointLocal,
    SetpointVelocity,
)


class MovingDropProtocol(MAVProtocol):
    """
    Velocity-controlled waypoint navigation with mid-flight payload release.

    First reach waypoint one, giving travel to drop altitude its own timeout.
    Hold until measured translation has stopped, then face waypoint two while
    holding position. Only then start velocity navigation.
    """

    STOP_SPEED_M_S = 0.2
    SETTLE_TIME_S = 1.0
    ALIGN_TOLERANCE_RAD = np.deg2rad(3.0)
    PREPARATION_TIMEOUT_S = 30.0
    # Travel gets its own budget, allowing slow descents before braking/alignment.
    # This estimates a timeout; it does not change the flight controller's speed.
    APPROACH_MIN_SPEED_M_S = 0.5
    FEEDBACK_MAX_AGE_S = 1.0
    PREPARATION_INTERVAL_S = 0.1

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
        *,
        current_attitude: AttitudeQuat,
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
        self.current_attitude = current_attitude

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

    @staticmethod
    def _angle_error(target: float, current: float) -> float:
        return (target - current + np.pi) % (2.0 * np.pi) - np.pi

    def _feedback(self):
        now = time.time()
        if any(not 0 <= now - message.timestamp <= self.FEEDBACK_MAX_AGE_S
               or message.timestamp == 0
               for message in (self.current_pos, self.current_attitude)):
            return None
        position = self.current_pos.get_pos_ned()
        velocity = self.current_pos.get_vel_ned()
        quaternion = self.current_attitude.get_quat()
        if not all(np.isfinite(value).all() for value in (position, velocity, quaternion)):
            return None
        norm = np.linalg.norm(quaternion)
        if norm < 1e-6:
            return None
        w, x, y, z = quaternion / norm
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return position, np.linalg.norm(velocity), yaw

    def _stop_and_align_first_waypoint(self, sender):
        first = self.waypoints[0]
        target = np.array([first.x, first.y, first.z])
        hold = SetpointLocal(self.target_system, self.target_component,
                             self.boot_time_ms, *target, yaw=None)
        self.log_func("[MovingDrop] Approaching first waypoint at drop altitude")
        phases = ["approach", "stop"]
        if len(self.waypoints) > 1:
            next_point = self.waypoints[1]
            direction = np.array([next_point.x - first.x, next_point.y - first.y])
            if np.linalg.norm(direction) > 1e-6:
                target_yaw = np.arctan2(direction[1], direction[0])
                phases.append("align")

        for phase in phases:
            if phase == "stop":
                self.log_func("[MovingDrop] At first waypoint; waiting for a complete stop")
            elif phase == "align":
                hold.yaw = target_yaw
                self.log_func("[MovingDrop] Stopped; aligning toward second waypoint")
            timeout = self.PREPARATION_TIMEOUT_S
            deadline = time.monotonic() + timeout
            approach_budget_set = False
            settled_since = None
            diagnostic = "fresh position/attitude feedback unavailable"
            while time.monotonic() < deadline:
                feedback = self._feedback()
                settled = False
                if feedback is not None:
                    position, speed, yaw = feedback
                    if hold.yaw is None:
                        hold.yaw = yaw
                    distance = np.linalg.norm(target - position)
                    diagnostic = (f"distance={distance:.1f} m, "
                                  f"altitude error={abs(target[2] - position[2]):.1f} m, "
                                  f"speed={speed:.2f} m/s")
                    if phase == "approach" and not approach_budget_set:
                        timeout += distance / self.APPROACH_MIN_SPEED_M_S
                        deadline = time.monotonic() + timeout
                        approach_budget_set = True
                        self.log_func(f"[MovingDrop] First-waypoint approach: {diagnostic}; "
                                      f"timeout={timeout:.0f} s")
                    settled = distance <= first.radius
                    if phase != "approach":
                        settled = settled and speed <= self.STOP_SPEED_M_S
                    if phase == "align":
                        yaw_error = abs(self._angle_error(target_yaw, yaw))
                        diagnostic += f", heading error={np.rad2deg(yaw_error):.1f} deg"
                        settled = settled and yaw_error <= self.ALIGN_TOLERANCE_RAD
                else:
                    diagnostic = "fresh position/attitude feedback unavailable"
                sender.send_msg(hold)
                if settled:
                    if phase == "approach":
                        break
                    if settled_since is None:
                        settled_since = time.monotonic()
                    if time.monotonic() - settled_since >= self.SETTLE_TIME_S:
                        break
                else:
                    settled_since = None
                time.sleep(self.PREPARATION_INTERVAL_S)
            else:
                raise RuntimeError(f"Moving-drop first waypoint {phase} not reached "
                                   f"within {timeout:g} s ({diagnostic})")
        self.log_func("[MovingDrop] First waypoint stopped and aligned; starting run-in")

    def run(self, sender, receiver):
        """
        Navigate through all waypoints; fire drop_callback at release_index.
        """
        if not self.waypoints:
            return
        
        self._stop_and_align_first_waypoint(sender)
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
                        if i != 0:
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
