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
        final_heading: float | None = None,
        slow_on_approach: bool = True,
        max_climb_angle_deg: float | None = None,
        lookahead_distance: float = 3.0,
        path_step: float = 1.0,
        hertz: float = 15.0,
        position_kp: float = 0.8,
        velocity_kd: float = 0.4,
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
        self.final_heading = final_heading
        self.lookahead_distance = lookahead_distance
        self.path_step = path_step
        self.control_period_s = 1.0 / hertz
        self.position_kp = position_kp
        self.velocity_kd = velocity_kd
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
        return np.array(position.get_pos_ned(), dtype=float)

    @staticmethod
    def _waypoint_array(waypoint: Waypoint) -> np.ndarray:
        return np.array([waypoint.x, waypoint.y, waypoint.z], dtype=float)

    @staticmethod
    def _heading_from_points(start: np.ndarray, end: np.ndarray) -> float:
        delta = end[:2] - start[:2]
        return math.atan2(float(delta[1]), float(delta[0]))

    def _initial_heading(self) -> float:
        current_velocity = np.array(self.current_pos.get_vel_ned(), dtype=float)
        horizontal_speed = float(np.linalg.norm(current_velocity[:2]))
        if horizontal_speed > 0.5:
            return math.atan2(float(current_velocity[1]), float(current_velocity[0]))

        current_position = self._position_array(self.current_pos)
        min_distance = 1e-3

        for waypoint in self.waypoints:
            waypoint_position = self._waypoint_array(waypoint)
            if float(np.linalg.norm(waypoint_position[:2] - current_position[:2])) > min_distance:
                return self._heading_from_points(current_position, waypoint_position)

        if len(self.waypoints) >= 2:
            return self._heading_from_points(
                self._waypoint_array(self.waypoints[0]),
                self._waypoint_array(self.waypoints[1]),
            )

        return 0.0

    def _terminal_heading(self) -> float:
        if self.final_heading is not None:
            return float(self.final_heading)

        if len(self.waypoints) >= 2:
            return self._heading_from_points(
                self._waypoint_array(self.waypoints[-1]),
                self._waypoint_array(self.waypoints[0]),
            )

        return self._initial_heading()

    def _terminal_waypoint(self) -> Waypoint | None:
        if not self.waypoints:
            return None

        final_waypoint = self.waypoints[-1]
        heading = self._terminal_heading()
        extension = max(
            float(self.turn_radius),
            float(self.lookahead_distance),
            float(self.path_step),
            float(final_waypoint.radius) * 4.0,
        )
        return Waypoint(
            x=float(final_waypoint.x + math.cos(heading) * extension),
            y=float(final_waypoint.y + math.sin(heading) * extension),
            z=float(final_waypoint.z),
            radius=float(final_waypoint.radius),
            speed=float(final_waypoint.speed),
        )

    def _segment_speed(self, waypoint: Waypoint) -> float:
        if self.speed is not None:
            return float(self.speed)
        if waypoint.speed > 0.0:
            return float(waypoint.speed)
        return 10.0

    def _build_sampled_path(self) -> list[Waypoint]:
        if len(self.waypoints) < 2:
            return list(self.waypoints)

        current_pos = self.current_pos.get_pos_ned()
        current_pos = np.array(current_pos, dtype=float)
        current_heading = self._initial_heading()
        sampled_path: list[Waypoint] = []
        sampled_waypoints = [
            Waypoint(
                x=float(current_pos[0]),
                y=float(current_pos[1]),
                z=float(current_pos[2]),
                radius=0.0,
                speed=self._segment_speed(self.waypoints[0]),
            ),
            *self.waypoints,
        ]
        terminal_waypoint = self._terminal_waypoint()
        if terminal_waypoint is not None:
            sampled_waypoints.append(terminal_waypoint)

        for index in range(len(sampled_waypoints) - 2):
            waypoint = sampled_waypoints[index + 1]
            next_waypoint = sampled_waypoints[index + 2]
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

    def _path_tangent(self, path_points: list[Waypoint], index: int) -> np.ndarray:
        current_point = self._waypoint_array(path_points[index])

        if len(path_points) == 1:
            return np.array([1.0, 0.0, 0.0], dtype=float)

        if index <= 0:
            next_point = self._waypoint_array(path_points[1])
            tangent = next_point - current_point
        elif index >= len(path_points) - 1:
            previous_point = self._waypoint_array(path_points[-2])
            tangent = current_point - previous_point
        else:
            previous_point = self._waypoint_array(path_points[index - 1])
            next_point = self._waypoint_array(path_points[index + 1])
            tangent = next_point - previous_point

        tangent_norm = float(np.linalg.norm(tangent))
        if tangent_norm < 1e-6:
            return np.array([1.0, 0.0, 0.0], dtype=float)

        return tangent / tangent_norm

    def _pd_velocity(
        self,
        current_position: np.ndarray,
        current_velocity: np.ndarray,
        path_points: list[Waypoint],
        target_index: int,
    ) -> np.ndarray:
        target_point = path_points[target_index]
        target_position = self._waypoint_array(target_point)
        path_tangent = self._path_tangent(path_points, target_index)

        target_speed = self._segment_speed(target_point)
        distance_to_goal = float(
            np.linalg.norm(current_position - self._waypoint_array(path_points[-1]))
        )
        if self.slow_on_approach and distance_to_goal < self.lookahead_distance:
            target_speed *= max(0.35, distance_to_goal / max(self.lookahead_distance, 1e-6))

        position_error = target_position - current_position
        along_track_error = float(np.dot(position_error, path_tangent))
        cross_track_error = position_error - path_tangent * along_track_error

        along_velocity = float(np.dot(current_velocity, path_tangent))
        lateral_velocity = current_velocity - path_tangent * along_velocity

        command = (
            path_tangent * target_speed
            + cross_track_error * self.position_kp
            - lateral_velocity * self.velocity_kd
        )

        forward_component = float(np.dot(command, path_tangent))
        if forward_component < 0.0:
            command = command - path_tangent * forward_component

        max_speed = max(target_speed * 1.5, target_speed + 1.0)
        command_norm = float(np.linalg.norm(command))
        if command_norm > max_speed and command_norm > 1e-6:
            command = command / command_norm * max_speed

        return command


    def run(self, sender, receiver):
        del receiver

        sampled_path = self._build_sampled_path()
        if not sampled_path:
            self.log_func("[DubinsNavigation] No Dubins path could be sampled")
            return

        # self.log_func(
        #     f"[DubinsNavigation] Following {len(sampled_path)} sampled Dubins points at "
        #     f"lookahead {self.lookahead_distance:.1f} m"
        # )
        # self.log_func(
        #     f"[DubinsNavigation] PD gains: kp={self.position_kp:.2f}, kd={self.velocity_kd:.2f}"
        # )

        current_index = 0
        final_point = sampled_path[-1]

        while True:
            current_position = np.array(self.current_pos.get_pos_ned(), dtype=float)
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
            current_velocity = np.array(self.current_pos.get_vel_ned(), dtype=float)
            velocity = self._pd_velocity(
                current_position,
                current_velocity,
                sampled_path,
                target_index,
            )
            self.velocity_msg.load(velocity)
            sender.send_msg(self.velocity_msg)

            # if target_index >= len(sampled_path) - 1 and nearest_distance <= self.lookahead_distance:
            #     self.log_func("[DubinsNavigation] Final Dubins point reached")

            time.sleep(self.control_period_s)

        self.velocity_msg.load(np.zeros(3, dtype=float))
        sender.send_msg(self.velocity_msg)

