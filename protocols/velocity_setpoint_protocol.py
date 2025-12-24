from mavcore.mav_protocol import MAVProtocol
from mavcore.messages import SetpointVelocity, CommandAck, LocalPositionNED
from gnc.types import Waypoint
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
        mission_phase: str = "cruise",
        target_system: int = 1,
        target_component: int = 0,
    ):
        super().__init__()
        self.current_pos = current_pos
        self.waypoints = waypoints
        self.boot_time_ms = boot_time_ms
        self.mission_phase = mission_phase
        self.target_system = target_system
        self.target_component = target_component

        # Base speed for mission phase
        self.base_speed = self.SPEED_PROFILES.get(mission_phase, 10.0)

        self.velocity_msg = SetpointVelocity(
            self.target_system, self.target_component, self.boot_time_ms, 0.0, 0.0, 0.0
        )

        self.ack_msg = CommandAck()

    def calculate_velocity_vector(
        self, current_pos: np.ndarray, target_pos: np.ndarray, target_speed: float
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
        if distance < 45.0:
            velocity /= 3.0

        return velocity

    def calculate_turn_angle(self, wp1: Waypoint, wp2: Waypoint, wp3: Waypoint):
        """
        Calculate the turn angle at wp2 going wp1 -> wp2 -> wp3
        """
        # Vectors
        v1 = np.array([wp2.x - wp1.x, wp2.y - wp1.y])
        v2 = np.array([wp3.x - wp2.x, wp3.y - wp2.y])

        # Normalize
        v1_norm = v1 / (np.linalg.norm(v1))
        v2_norm = v2 / (np.linalg.norm(v2))

        # Calculate angle
        dot_product = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
        angle_rad = np.arccos(dot_product)
        return np.degrees(angle_rad)

    def get_optimal_speed_for_waypoint(self, waypoint_idx: int) -> float:
        """
        Cornering optimization

        """
        # Slows down if last waypoint
        if waypoint_idx >= len(self.waypoints) - 1:
            return self.base_speed * 0.7

        # Checka for upcoming turn
        if waypoint_idx + 2 < len(self.waypoints):
            wp_curr = self.waypoints[waypoint_idx]
            wp_next = self.waypoints[waypoint_idx + 1]
            wp_after = self.waypoints[waypoint_idx + 2]

            turn_angle = self.calculate_turn_angle(wp_curr, wp_next, wp_after)

            # Reduce speed based on sharpness of turn
            if turn_angle > 90:
                return self.base_speed * 0.6
            elif turn_angle > 45:
                return self.base_speed * 0.7
            elif turn_angle > 20:  # May delete 20 degree check
                return self.base_speed * 0.9

        return self.base_speed

    def run(self, sender, receiver):
        for i in range(len(self.waypoints)):
            waypoint = self.waypoints[i]
            waypoint_coords = np.array([waypoint.x, waypoint.y, waypoint.z])
            optimal_speed = self.get_optimal_speed_for_waypoint(i)

            print(
                f"[VelocityControl] Waypoint {i + 1}/{len(self.waypoints)} @ {optimal_speed:.1f} m/s"
            )

            while True:
                current_position = self.current_pos.get_pos_ned()
                distance_to_waypoint = np.linalg.norm(
                    waypoint_coords - current_position
                )

                if distance_to_waypoint <= waypoint.radius:
                    print(f"[VelocityControl] Reached waypoint {i + 1}")
                    self.velocity_msg.load(np.array([0.0, 0.0, 0.0]))
                    sender.send_msg(self.velocity_msg)
                    break

                velocity_vector = self.calculate_velocity_vector(
                    current_position, waypoint_coords, optimal_speed
                )

                self.velocity_msg.load(velocity_vector)
                sender.send_msg(self.velocity_msg)
                time.sleep(0.25)
