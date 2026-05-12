from ..mav_protocol import MAVProtocol
from ..messages import SetpointVelocity, CommandAck, LocalPositionNED
from ..mavtypes import Waypoint
from gnc.util import dubins_path
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
        use_dubins: bool = False,
        target_system: int = 1,
        target_component: int = 0,
    ):
        super().__init__()
        self.current_pos = current_pos
        self.waypoints = waypoints
        self.boot_time_ms = boot_time_ms
        self.slow_on_approach = slow_on_approach
        self.use_dubins = use_dubins
        self.target_system = target_system
        self.target_component = target_component
        self.log_func = log_func

        # Base speed for mission phase
        self.base_speed = 10.0

        self.velocity_msg = SetpointVelocity(
            self.target_system, self.target_component, self.boot_time_ms, 0.0, 0.0, 0.0
        )

        self.ack_msg = CommandAck()
        self._last_heading_rad: float | None = None

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

    def _get_heading_rad(self) -> float:
        velocity = self.current_pos.get_vel_ned()
        speed = float(np.linalg.norm(velocity[:2]))
        if speed > 0.5:
            self._last_heading_rad = float(np.arctan2(velocity[1], velocity[0]))
        if self._last_heading_rad is None:
            return 0.0
        return self._last_heading_rad

    def _goto_waypoint(self, sender, waypoint: Waypoint, waypoint_idx: int):
        waypoint_coords = np.array([waypoint.x, waypoint.y, waypoint.z])
        optimal_speed = waypoint.speed

        self.log_func(
            f"[VelocityControl] Waypoint {waypoint_idx + 1}/{len(self.waypoints)} @ {optimal_speed:.1f} m/s"
        )

        while True:
            current_position = self.current_pos.get_pos_ned()
            distance_to_waypoint = np.linalg.norm(waypoint_coords - current_position)

            if distance_to_waypoint <= waypoint.radius:
                self.log_func(f"[VelocityControl] Reached waypoint {waypoint_idx + 1}")
                self.velocity_msg.load(np.array([0.0, 0.0, 0.0]))
                sender.send_msg(self.velocity_msg)
                break

            velocity_vector = self.calculate_velocity_vector(
                current_position,
                waypoint_coords,
                optimal_speed,
                self.slow_on_approach,
            )

            self.velocity_msg.load(velocity_vector)
            sender.send_msg(self.velocity_msg)
            time.sleep(0.25)

    def _run_linear(self, sender):
        for i in range(len(self.waypoints)):
            self._goto_waypoint(sender, self.waypoints[i], i)

    def _run_dubins(self, sender):
        if not self.waypoints:
            return

        i = 0
        while i < len(self.waypoints):
            if i >= len(self.waypoints) - 1:
                self._goto_waypoint(sender, self.waypoints[i], i)
                return

            waypoint = self.waypoints[i]
            next_waypoint = self.waypoints[i + 1]
            waypoint_coords = np.array([waypoint.x, waypoint.y, waypoint.z])
            next_coords = np.array([next_waypoint.x, next_waypoint.y, next_waypoint.z])

            while True:
                current_position = self.current_pos.get_pos_ned()
                heading = self._get_heading_rad()

                result = dubins_path.compute_turning_point(
                    current_position,
                    heading,
                    waypoint_coords,
                    next_coords,
                    constants.TURN_RADIUS,
                )

                if result is None:
                    self.log_func("[DWN] Dubins solve failed, falling back to linear leg.")
                    self._goto_waypoint(sender, waypoint, i)
                    i += 1
                    break

                if dubins_path.is_turning_point_unreachable(
                    current_position,
                    heading,
                    result.turning_point,
                    constants.TURN_RADIUS,
                ):
                    self.log_func(f"[DWN] Turning point unreachable, advancing to waypoint {i + 2}.")
                    i += 1
                    break

                target_z = dubins_path.interpolate_z(
                    waypoint_coords,
                    next_coords,
                    result.path_length_to_turn,
                    result.path_length_total,
                )
                target_pos = np.array([result.turning_point[0], result.turning_point[1], target_z])

                velocity_vector = self.calculate_velocity_vector(
                    current_position,
                    target_pos,
                    waypoint.speed,
                    self.slow_on_approach,
                )

                self.velocity_msg.load(velocity_vector)
                sender.send_msg(self.velocity_msg)
                time.sleep(0.25)

    def run(self, sender, receiver):
        if self.use_dubins:
            self._run_dubins(sender)
        else:
            self._run_linear(sender)