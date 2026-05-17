from __future__ import annotations

from collections.abc import Callable
import math
import time

import numpy as np

from ..mav_protocol import MAVProtocol
from ..messages import CommandAck, LocalPositionNED, SetpointLocal
from ..mavtypes import Waypoint
from gnc.util.dubins_path import compute_path_points


class DubinsNavigationProtocol(MAVProtocol):
    """
    Navigates through waypoints by sampling Dubins paths between consecutive points.

    The protocol uses local position setpoints instead of velocity commands. It keeps a
    small lookahead between the current waypoint and the next waypoint so the aircraft
    follows a smooth path with bounded curvature.
    """

    def __init__(
        self,
        current_pos: LocalPositionNED,
        waypoints: list[Waypoint],
        boot_time_ms: int,
        turn_radius: float,
        log_func: Callable[[str], None] = print,
        slow_on_approach: bool = True,
        point_step: float = 2.0,
        point_radius: float = 1.0,
        max_climb_angle_deg: float | None = None,
        target_system: int = 1,
        target_component: int = 0,
    ):
        super().__init__()
        self.current_pos = current_pos
        self.waypoints = waypoints
        self.boot_time_ms = boot_time_ms
        self.turn_radius = turn_radius
        self.slow_on_approach = slow_on_approach
        self.point_step = point_step
        self.point_radius = point_radius
        self.max_climb_angle_deg = max_climb_angle_deg
        self.target_system = target_system
        self.target_component = target_component
        self.log_func = log_func

        self.follow_sleep_s = 0.25 if slow_on_approach else 0.15

        self.setpoint_msg = SetpointLocal(
            self.target_system,
            self.target_component,
            self.boot_time_ms,
            0.0,
            0.0,
            0.0,
        )
        self.ack_msg = CommandAck()

    def _waypoint_array(self, waypoint: Waypoint) -> np.ndarray:
        return np.array([waypoint.x, waypoint.y, waypoint.z])

    def _infer_heading(self, fallback_target: np.ndarray | None = None) -> float:
        velocity = self.current_pos.get_vel_ned()
        if float(np.linalg.norm(velocity[:2])) > 1e-6:
            return float(math.atan2(float(velocity[1]), float(velocity[0])))

        if fallback_target is not None:
            current_position = self.current_pos.get_pos_ned()
            delta = fallback_target[:2] - current_position[:2]
            if float(np.linalg.norm(delta)) > 1e-6:
                return float(math.atan2(float(delta[1]), float(delta[0])))

        return 0.0

    def _heading_between(self, start: np.ndarray, end: np.ndarray) -> float:
        delta = end[:2] - start[:2]
        if float(np.linalg.norm(delta)) < 1e-6:
            return 0.0
        return float(math.atan2(float(delta[1]), float(delta[0])))

    def _send_setpoint(self, sender, target: np.ndarray) -> None:
        self.setpoint_msg.load(target)
        sender.send_msg(self.setpoint_msg)

    def _follow_point(self, sender, target: Waypoint) -> None:
        target_coords = self._waypoint_array(target)
        while True:
            current_position = self.current_pos.get_pos_ned()
            if float(np.linalg.norm(target_coords - current_position)) <= target.radius:
                break

            self._send_setpoint(sender, target_coords)
            time.sleep(self.follow_sleep_s)

    def _build_segment_points(
        self,
        current_position: np.ndarray,
        current_heading: float,
        turn_waypoint: Waypoint,
        next_waypoint: Waypoint,
    ) -> list[Waypoint]:
        turn_coords = self._waypoint_array(turn_waypoint)
        next_coords = self._waypoint_array(next_waypoint)

        virtual_start = np.array(current_position, copy=True)
        if float(np.linalg.norm(virtual_start[:2] - turn_coords[:2])) < max(
            0.5, self.point_step * 0.25
        ):
            heading_vector = np.array(
                [math.cos(current_heading), math.sin(current_heading), 0.0]
            )
            virtual_start = virtual_start - heading_vector * max(0.5, self.turn_radius * 0.25)

        return compute_path_points(
            virtual_start,
            current_heading,
            turn_coords,
            next_coords,
            self.turn_radius,
            self.point_step,
            next_waypoint.speed,
            self.point_radius,
            self.max_climb_angle_deg,
        )

    def run(self, sender, receiver):
        del receiver

        if not self.waypoints:
            self.log_func("[DubinsPath] No waypoints provided.")
            return

        if len(self.waypoints) == 1:
            self.log_func("[DubinsPath] Single waypoint provided; using direct local setpoint.")
            self._follow_point(sender, self.waypoints[0])
            return

        current_heading = self._infer_heading(self._waypoint_array(self.waypoints[0]))

        for index in range(len(self.waypoints) - 1):
            turn_waypoint = self.waypoints[index]
            next_waypoint = self.waypoints[index + 1]

            current_position = self.current_pos.get_pos_ned()
            segment_points = self._build_segment_points(
                current_position,
                current_heading,
                turn_waypoint,
                next_waypoint,
            )

            if not segment_points:
                self.log_func(
                    f"[DubinsPath] Segment {index + 1}/{len(self.waypoints) - 1} "
                    "could not be planned; falling back to direct waypoint tracking."
                )
                self._follow_point(sender, next_waypoint)
                current_heading = self._infer_heading(self._waypoint_array(next_waypoint))
                continue

            self.log_func(
                f"[DubinsPath] Segment {index + 1}/{len(self.waypoints) - 1} "
                f"with {len(segment_points)} Dubins samples"
            )

            for sampled_waypoint in segment_points:
                self._follow_point(sender, sampled_waypoint)

            if len(segment_points) >= 2:
                start_point = self._waypoint_array(segment_points[-2])
                end_point = self._waypoint_array(segment_points[-1])
                current_heading = self._heading_between(start_point, end_point)
            else:
                current_heading = self._infer_heading(self._waypoint_array(next_waypoint))
