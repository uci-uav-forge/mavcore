from __future__ import annotations

from collections.abc import Callable
import math
import time

import numpy as np

from ..mav_protocol import MAVProtocol
from ..messages import CommandAck, LocalPositionNED, SetpointVelocity
from ..mavtypes import Waypoint
from gnc.waypoint_laps.dubins_path import compute_turning_point, is_turning_point_unreachable

EPS = 1e-6
MIN_REACHABILITY_SPEED = 0.5

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
        final_heading: np.array | None = None,
        slow_on_approach: bool = True,
        max_climb_angle_deg: float | None = None,
        do_yaw: bool = True,
        hertz: float = 15.0,
        log_func = lambda msg: print(msg),
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
        self.slow_on_approach = slow_on_approach
        self.max_climb_angle_deg = max_climb_angle_deg
        self.do_yaw = do_yaw
        self.control_period_s = 1.0 / hertz
        self.log_func = log_func
        self.target_system = target_system
        self.target_component = target_component

        self.velocity_msg = SetpointVelocity(
            self.target_system,
            self.target_component,
            self.boot_time_ms,
            0.0,
            0.0,
            0.0,
            self.do_yaw,
        )
        self.ack_msg = CommandAck()
        
    def calculate_direct_velocity_vector(
        self,
        current_position: np.ndarray,
        waypoint_coords: np.ndarray,
        speed: float,
    ) -> np.ndarray:
        """
        Calculate direct velocity vector to waypoint without turn rate constraints.
        Used as fallback when turning point is unreachable.
        """
        direction = waypoint_coords - current_position
        distance = np.linalg.norm(direction)
        unit_direction = direction / distance
        velocity = unit_direction * speed
        
        if self.slow_on_approach:
            if distance < EPS:
                return np.array([0.0, 0.0, 0.0])
            if distance < 15.0:
                velocity /= 3.0
            
        return velocity

    def calculate_velocity_vector(
        self,
        current_position: np.ndarray,
        current_heading: np.ndarray,
        turning_point: np.ndarray,
        exit_tangent: np.ndarray,
        speed: float,
    ) -> np.ndarray:
        """
        Calculate velocity vector with max turn rate:
        1. Calculate heading error
        2. Limit heading change based on turn radius and current speed
        3. Convert to velocity vector
        """
        max_turning_speed = speed * self.control_period_s / self.turn_radius
        current_heading_2D = np.arctan2(current_heading[1], current_heading[0])
        target_heading_2D = np.arctan2(turning_point[1] - current_position[1], turning_point[0] - current_position[0])
        heading_error = (target_heading_2D - current_heading_2D + np.pi) % (2 * np.pi) - np.pi
        
        # TODO: add control gains if needed
        if heading_error > max_turning_speed:
            new_heading_2D = current_heading_2D + max_turning_speed
        elif heading_error < -max_turning_speed:
            new_heading_2D = current_heading_2D - max_turning_speed
        else:
            new_heading_2D = target_heading_2D
            
        delta_z = turning_point[2] - current_position[2]
        new_heading_z = math.atan2(delta_z, np.linalg.norm(turning_point[:2] - current_position[:2]))
        if self.max_climb_angle_deg is not None:
            max_climb_angle_rad = math.radians(self.max_climb_angle_deg)
            new_heading_z = np.clip(new_heading_z, -max_climb_angle_rad, max_climb_angle_rad)
        
        direction = np.array([math.cos(new_heading_2D), math.sin(new_heading_2D), math.tan(new_heading_z)])
        unit_direction = direction / np.linalg.norm(direction)
        velocity = unit_direction * speed

        distance = np.linalg.norm(current_position - turning_point)
        if self.slow_on_approach:
            if distance < 0.1:
                return np.array([0.0, 0.0, 0.0])
            if distance < 15.0:
                velocity /= 3.0

        return velocity

    def run(self, sender, receiver):
        if len(self.waypoints) == 0:
            self.log_func("[DubinsNavigationControl] No waypoints provided, exiting protocol.")
            return
        current_position = self.current_pos.get_pos_ned()
        prev_heading = current_position - np.array([self.waypoints[0].x, self.waypoints[0].y, self.waypoints[0].z])
        for i in range(len(self.waypoints)):
            current_position = self.current_pos.get_pos_ned()
            current_heading = prev_heading
            
            waypoint = self.waypoints[i]
            waypoint_coords = np.array([waypoint.x, waypoint.y, waypoint.z])
            if i < len(self.waypoints) - 1:
                next_waypoint = self.waypoints[i + 1]
            elif self.final_heading is None:
                if len(self.waypoints) == 1:
                    # extend final heading in the direction of the last waypoint if only one waypoint provided
                    heading = waypoint_coords - current_position
                    norm = np.linalg.norm(heading)
                    if norm < EPS:
                        heading = np.array([self.turn_radius, 0.0, 0.0])
                    else:
                        heading = (heading / norm) * self.turn_radius
                    next_waypoint = Waypoint(
                        x=waypoint.x + heading[0],
                        y=waypoint.y + heading[1],
                        z=waypoint.z + heading[2],
                        radius=0.0,
                        speed=waypoint.speed,
                    )
                else:
                    # use first waypoint as final heading
                    next_waypoint = self.waypoints[0]
            elif self.final_heading is not None:
                # if final heading is specified, create a temporary "next waypoint" in the direction of the final heading.
                next_waypoint = Waypoint(
                    x=waypoint.x + self.final_heading[0],
                    y=waypoint.y + self.final_heading[1],
                    z=waypoint.z + self.final_heading[2],
                    radius=0.0,
                    speed=waypoint.speed,
                )
            next_waypoint_coords = np.array([next_waypoint.x, next_waypoint.y, next_waypoint.z])
            
                
            speed = self.speed if self.speed is not None else waypoint.speed
            result = compute_turning_point(
                current_pos=current_position,
                current_heading=math.atan2(current_heading[1], current_heading[0]),
                waypoint=waypoint_coords,
                next_waypoint=next_waypoint_coords,
                turn_radius=self.turn_radius,
            )
            if result is None:
                self.log_func(f"[DubinsNavigationControl] Waypoint {i + 1} is unreachable with given turn radius, defaulting to direct path")
                while True:
                    current_position = self.current_pos.get_pos_ned()
                    distance_to_waypoint = np.linalg.norm(current_position - waypoint_coords)

                    if distance_to_waypoint < waypoint.radius:
                        self.log_func(f"[DubinsNavigationControl] Reached waypoint {i + 1} by radius on direct path")
                        break

                    velocity_vector = self.calculate_direct_velocity_vector(
                        current_position=current_position,
                        waypoint_coords=waypoint_coords,
                        speed=speed,
                    )

                    self.velocity_msg.load(velocity_vector)
                    sender.send_msg(self.velocity_msg)
                    time.sleep(self.control_period_s)
                continue
            
            turning_point, exit_tangent = result
            
            while True:
                current_position = self.current_pos.get_pos_ned()
                current_heading = prev_heading
                distance_to_waypoint = np.linalg.norm(current_position - waypoint_coords)
                
                if distance_to_waypoint < waypoint.radius:
                    self.log_func(f"[DubinsNavigationControl] Reached waypoint {i + 1} by radius")
                    break
                
                if is_turning_point_unreachable(
                    current_pos=current_position,
                    current_heading=math.atan2(current_heading[1], current_heading[0]),
                    turning_point=turning_point,
                    turn_radius=self.turn_radius,
                ):
                    self.log_func(f"[DubinsNavigationControl] Reached waypoint {i + 1} by turning point reachability condition")
                    break

                velocity_vector = self.calculate_velocity_vector(
                    current_position=current_position,
                    current_heading=current_heading,
                    turning_point=turning_point,
                    exit_tangent=exit_tangent,
                    speed=speed,
                )
                
                prev_heading = velocity_vector
                
                self.velocity_msg.load(velocity_vector)
                sender.send_msg(self.velocity_msg)
                time.sleep(self.control_period_s)
            
        self.velocity_msg.load(np.array([0.0, 0.0, 0.0]))
        sender.send_msg(self.velocity_msg)