from ..mav_protocol import MAVProtocol
from ..messages import SetpointVelocity, CommandAck, LocalPositionNED
from ..mavtypes import Waypoint
from gnc.util.dubins_path import DubinsPath
from gnc.util import constants
import time
import numpy as np


class VelocitySetpointProtocol(MAVProtocol):
    """
    Navigates to waypoints using velocity control for speed optimization.
    Sends velocity vectors instead of positions.
    """

    # Speed values for different mission phases (in m/s)
    SPEED_PROFILES = {
        "cruise": 15.0,
        "approach": 8.0,  # Approaching targets
        "scan": 10.0,
        "precision": 5.0,  # Final approach to drop
    }

    def __init__(
        self,
        current_pos: LocalPositionNED,
        waypoints: list[Waypoint],
        boot_time_ms: int,
        log_func=lambda msg: print(msg),
        slow_on_approach: bool = True,
        target_system: int = 1,
        target_component: int = 0,
    ):
        super().__init__()
        self.current_pos = current_pos
        self.waypoints = waypoints
        self.boot_time_ms = boot_time_ms
        self.slow_on_approach = slow_on_approach
        self.target_system = target_system
        self.target_component = target_component
        self.log_func = log_func

        # Base speed for mission phase
        self.base_speed = 10.0

        
        self.dubins_path = DubinsPath(
            waypoints=self.waypoints,
            speed=self.base_speed,
        )

        self.velocity_msg = SetpointVelocity(
            self.target_system, self.target_component, self.boot_time_ms, 0.0, 0.0, 0.0
        )

        self.ack_msg = CommandAck()

    def calculate_velocity_vector(
        self,
        current_pos: np.ndarray,
        target_pos: np.ndarray,
        target_speed: float,
        slow_on_approach: bool = True,
    ):
        """
        Convert waypoint position to velocity vector:
        1. Calculate direction vector (target - current)
        2. Normalize to unit vector
        3. Scale by specific speed
        """
        # Calculate direction vector
        direction = target_pos - current_pos
        distance = np.linalg.norm(direction)

        if distance < 0.1:
            return np.array([0.0, 0.0, 0.0])

        # Normalize to unit vector
        unit_direction = direction / distance

        # Scale by speed
        velocity = unit_direction * target_speed

        # Slow down when close to/approaching waypoint
        if slow_on_approach and distance < 15.0:
            velocity /= 3.0

        return velocity

    def run(self, sender, receiver):
        waypoint_index = 0
        last_logged_index = -1
        while waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[waypoint_index]
            waypoint_coords = np.array([waypoint.x, waypoint.y, waypoint.z])
            optimal_speed = waypoint.speed

            if last_logged_index != waypoint_index:
                self.log_func(
                    f"[VelocityControl] Waypoint {waypoint_index + 1}/{len(self.waypoints)} @ {optimal_speed:.1f} m/s"
                )
                last_logged_index = waypoint_index

            current_position = self.current_pos.get_pos_ned()
            distance_to_waypoint = np.linalg.norm(waypoint_coords - current_position)
            if distance_to_waypoint <= waypoint.radius:
                self.log_func(f"[VelocityControl] Reached waypoint {waypoint_index + 1}")
                self.velocity_msg.load(np.array([0.0, 0.0, 0.0]))
                sender.send_msg(self.velocity_msg)
                waypoint_index += 1
                continue

            self.dubins_path.update_waypoints(self.waypoints[waypoint_index:])
            self.dubins_path.speed = optimal_speed

            path_points = self.dubins_path.generate_horizon_path(
                current_position,
            )

            target_point = self._select_lookahead_point(
                current_position, path_points, self.dubins_path.step_size
            )

            target_coords = np.array([target_point.x, target_point.y, target_point.z])
            velocity_vector = self.calculate_velocity_vector(
                current_position,
                target_coords,
                optimal_speed,
                self.slow_on_approach,
            )

            self.velocity_msg.load(velocity_vector)
            sender.send_msg(self.velocity_msg)
            time.sleep(0.25)

    @staticmethod
    def _select_lookahead_point(
        current_position: np.ndarray,
        path_points: list[Waypoint],
        min_distance: float,
    ) -> Waypoint:
        if not path_points:
            return Waypoint(
                float(current_position[0]),
                float(current_position[1]),
                float(current_position[2]),
                0.0,
                0.0,
            )

        if min_distance <= 0.0:
            return path_points[-1]

        for point in path_points:
            dist = np.linalg.norm(
                np.array([point.x, point.y, point.z]) - current_position
            )
            if dist >= min_distance * 0.5:
                return point

        return path_points[-1]
