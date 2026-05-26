from __future__ import annotations

from collections.abc import Callable
import math
import time

import numpy as np

from ..mav_protocol import MAVProtocol
from ..messages import CommandAck, LocalPositionNED, SetpointVelocity
from ..mavtypes import Waypoint
from gnc.waypoint_laps.dubins_path import compute_path_points


class DubinsNavigationProtocol(MAVProtocol):
    """
    Navigates through waypoints by sampling Dubins paths between consecutive points.

    The protocol uses local velocity setpoints along sampled Dubins paths so the aircraft
    follows a smooth path with bounded curvature while preserving requested speed.
    """

    def __init__(
        self,
        current_pos: LocalPositionNED,
        waypoints: list[Waypoint],
        boot_time_ms: int,
        turn_radius: float,
        speed: float | None = None,
        slow_on_approach: bool = True,
        max_climb_angle_deg: float | None = None,
        lookahead_distance: float = 10.0,
        path_step: float = 2.0,
        hertz: float = 15.0,
        log_func: Callable[[str], None] = print,
        target_system: int = 1,
        target_component: int = 0,
    ):
        super().__init__()
        self.current_pos = current_pos
        self.waypoints = waypoints
        self.boot_time_ms = boot_time_ms
        self.turn_radius = turn_radius
        self.speed = speed
        self.lookahead_distance = lookahead_distance
        self.path_step = path_step
        self.control_period_s = 1.0 / hertz
        self.log_func = log_func
        self.slow_on_approach = slow_on_approach
        self.max_climb_angle_deg = max_climb_angle_deg
        self.target_system = target_system
        self.target_component = target_component

        self.velocity_msg = SetpointVelocity(
            self.target_system,
            self.target_component,
            self.boot_time_ms,
            0.0,
            0.0,
            0.0,
        )
        self.ack_msg = CommandAck()

    @staticmethod
    def _position_array(position: LocalPositionNED) -> np.ndarray:
        return np.array([position.x, position.y, position.z], dtype=float)

    @staticmethod
    def _waypoint_array(waypoint: Waypoint) -> np.ndarray:
        return np.array([waypoint.x, waypoint.y, waypoint.z], dtype=float)

    @staticmethod
    def _heading_from_points(start: np.ndarray, end: np.ndarray) -> float:
        delta = end[:2] - start[:2]
        return math.atan2(float(delta[1]), float(delta[0]))

    def _segment_speed(self, waypoint: Waypoint) -> float:
        if self.speed is not None:
            return float(self.speed)
        if waypoint.speed > 0.0:
            return float(waypoint.speed)
        return 10.0

    def _build_sampled_path(self) -> list[Waypoint]:
        if len(self.waypoints) < 2:
            return list(self.waypoints)

        current_pos = self._position_array(self.current_pos)
        current_heading = self._heading_from_points(current_pos, self._waypoint_array(self.waypoints[0]))
        sampled_path: list[Waypoint] = []

        for index in range(len(self.waypoints) - 1):
            waypoint = self.waypoints[index]
            next_waypoint = self.waypoints[index + 1]
            speed = self._segment_speed(waypoint)
            point_radius = max(1.0, float(waypoint.radius), float(next_waypoint.radius))

            segment = compute_path_points(
                current_pos=current_pos,
                current_heading=current_heading,
                waypoint=self._waypoint_array(waypoint),
                next_waypoint=self._waypoint_array(next_waypoint),
                turn_radius=float(self.turn_radius),
                step=float(self.path_step),
                speed=float(speed),
                point_radius=point_radius,
                max_climb_angle_deg=self.max_climb_angle_deg,
            )

            if not segment:
                continue

            sampled_path.extend(segment)
            current_pos = self._waypoint_array(segment[-1])
            if len(segment) >= 2:
                current_heading = self._heading_from_points(
                    self._waypoint_array(segment[-2]),
                    self._waypoint_array(segment[-1]),
                )

        return sampled_path

    def _lookahead_index(self, path_points: list[Waypoint], current_index: int) -> int:
        if not path_points:
            return 0

        lookahead_steps = max(1, int(round(self.lookahead_distance / max(self.path_step, 1e-6))))
        return min(current_index + lookahead_steps, len(path_points) - 1)

    def _velocity_toward(self, current_position: np.ndarray, target: Waypoint) -> np.ndarray:
        target_position = self._waypoint_array(target)
        direction = target_position - current_position
        distance = float(np.linalg.norm(direction))
        if distance < 1e-6:
            return np.zeros(3, dtype=float)

        target_speed = self._segment_speed(target)
        if self.slow_on_approach and distance < self.lookahead_distance:
            target_speed *= max(0.25, distance / max(self.lookahead_distance, 1e-6))

        return direction / distance * target_speed


    def run(self, sender, receiver):
        del receiver

        sampled_path = self._build_sampled_path()
        if not sampled_path:
            self.log_func("[DubinsNavigation] No Dubins path could be sampled")
            return

        self.log_func(
            f"[DubinsNavigation] Following {len(sampled_path)} sampled Dubins points at "
            f"lookahead {self.lookahead_distance:.1f} m"
        )

        current_index = 0
        final_point = sampled_path[-1]

        while True:
            current_position = self._position_array(self.current_pos)
            distance_to_goal = float(
                np.linalg.norm(current_position - self._waypoint_array(final_point))
            )
            if distance_to_goal <= final_point.radius:
                break

            nearest_index = current_index
            nearest_distance = float("inf")
            for index in range(current_index, len(sampled_path)):
                candidate_distance = float(
                    np.linalg.norm(current_position - self._waypoint_array(sampled_path[index]))
                )
                if candidate_distance < nearest_distance:
                    nearest_distance = candidate_distance
                    nearest_index = index

            current_index = nearest_index
            target_index = self._lookahead_index(sampled_path, nearest_index)
            target_point = sampled_path[target_index]

            velocity = self._velocity_toward(current_position, target_point)
            self.velocity_msg.load(velocity)
            sender.send_msg(self.velocity_msg)

            if target_index >= len(sampled_path) - 1 and nearest_distance <= self.lookahead_distance:
                self.log_func("[DubinsNavigation] Final Dubins point reached")

            time.sleep(self.control_period_s)

        self.velocity_msg.load(np.zeros(3, dtype=float))
        sender.send_msg(self.velocity_msg)

