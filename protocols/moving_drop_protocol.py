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

MIN_DISTANCE = 5.0

class MovingDropProtocol(MAVProtocol):

    def __init__(
        self,
        current_pos: LocalPositionNED,
        waypoints: list[Waypoint],
        drop_time: float,
        accel_dist: float,
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
        self.drop_time = drop_time
        self.accel_dist = accel_dist
        self.drop_callback = drop_callback
        self.boot_time_ms = boot_time_ms
        self.log_func = log_func
        self.target_system = target_system
        self.target_component = target_component
        self.current_attitude = current_attitude

        self.velocity_msg = SetpointVelocity(
            self.target_system,
            self.target_component,
            self.boot_time_ms,
            0.0,
            0.0,
            0.0,
        )
        
    def calculate_velocity_vector(
            self,
            current_pos: np.ndarray,
            target_pos: np.ndarray,
            target_speed: float,
        ):
            """
            Convert waypoint position to velocity vector:
            1. Calculate direction vector (target - current)
            2. Normalize to unit vector
            3. Scale by specific speed
            """
            direction = target_pos - current_pos
            distance = np.linalg.norm(direction)
            if distance < 0.1:
                return np.array([0.0, 0.0, 0.0])
            unit_direction = direction / distance
            velocity = unit_direction * target_speed

            return velocity

    def run(self, sender, receiver):
        
        if not self.waypoints:
            self.log_func("[MovingDrop] No waypoints provided.")
            return
        
        dropped = False
        
        waypoints = self.waypoints.copy()
        waypoint_coords = np.array([waypoints[0].x, waypoints[0].y, waypoints[0].z])
        current_pos = np.array([self.current_pos.x, self.current_pos.y, self.current_pos.z])
        current_distance = np.linalg.norm(waypoint_coords[:2] - current_pos[:2])

        if current_distance <= self.accel_dist:
            direction = waypoint_coords[:2] - current_pos[:2]
            horizontal_distance = np.linalg.norm(direction)
            if horizontal_distance > 0.1:
                direction /= horizontal_distance
                accel_waypoint_coords = waypoint_coords.copy()
                accel_waypoint_coords[:2] -= direction * (
                    self.accel_dist - horizontal_distance
                )
                accel_waypoint = Waypoint(
                    accel_waypoint_coords[0],
                    accel_waypoint_coords[1],
                    accel_waypoint_coords[2],
                    waypoints[0].radius,
                    waypoints[0].speed,
                )
                waypoints.insert(0, accel_waypoint)
                self.log_func(f"[MovingDrop] Adding acc waypoint at: {accel_waypoint}")
    
        for i in range(len(waypoints)):
            waypoint = waypoints[i]
            waypoint_coords = np.array([waypoint.x, waypoint.y, waypoint.z])
            target_speed = waypoint.speed

            self.log_func(f"[MovingDrop] Waypoint {i + 1}/{len(waypoints)} @ {target_speed:.1f} m/s")

            while True:
                current_position = self.current_pos.get_pos_ned()
                offset_to_waypoint = waypoint_coords - current_position
                distance_to_waypoint = np.linalg.norm(offset_to_waypoint)
                
                if not dropped and i == len(waypoints) - 1:
                    current_velocity = self.current_pos.get_vel_ned()
                    current_speed = np.linalg.norm(current_velocity[:2])
                    horizontal_distance_to_waypoint = np.linalg.norm(
                        offset_to_waypoint[:2]
                    )
                    target_distance = max(MIN_DISTANCE, current_speed * self.drop_time)
                    
                    if horizontal_distance_to_waypoint <= target_distance:
                        self.log_func(f"[MovingDrop] Dropping payload at position: {self.current_pos.get_pos_ned()}")
                        self.drop_callback()
                        dropped = True

                if distance_to_waypoint <= waypoint.radius:
                    self.log_func(f"[MovingDrop] Reached waypoint {i + 1}")
                    if i == len(waypoints) - 1:
                        if not dropped:
                            self.log_func(f"[MovingDrop] Fallback dropping payload at position: {self.current_pos.get_pos_ned()}")
                            self.drop_callback()
                        self.velocity_msg.load(np.array([0.0, 0.0, 0.0]))
                    sender.send_msg(self.velocity_msg)
                    break

                velocity_vector = self.calculate_velocity_vector(
                    current_position,
                    waypoint_coords,
                    target_speed,
                )

                self.velocity_msg.load(velocity_vector)
                sender.send_msg(self.velocity_msg)
                time.sleep(0.25)
            
        
       
