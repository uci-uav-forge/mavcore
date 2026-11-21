import time
from collections.abc import Callable

import numpy as np

from ..mav_protocol import MAVProtocol
from ..messages import MissionAck, MissionItemReached, MissionRequestInt, MissionType
from ..messages.mission_ack_msg import MissionResult
from ..messages.mission_item_msg import MissionCount, MissionItemInt
from ..mavtypes import Waypoint


class AutoMissionProtocol(MAVProtocol):
    """Upload a mission, run it in AUTO, and return to Guided when its last item is reached.

    Waypoints use local NED coordinates relative to ``global_home``. Mission
    items are converted to global latitude/longitude and relative altitude.
    """

    def __init__(
        self,
        waypoints: list[Waypoint],
        global_home: np.ndarray,
        local_to_gps: Callable[[np.ndarray, np.ndarray], np.ndarray],
        set_auto: Callable[[], None],
        log_func: Callable[[str], None] = lambda msg: print(msg),
        target_system: int = 1,
        target_component: int = 0,
        upload_timeout_s: float = 10.0,
    ) -> None:
        super().__init__()
        self.waypoints = waypoints
        self.global_home = global_home
        self.local_to_gps = local_to_gps
        self.set_auto = set_auto
        self.log_func = log_func
        self.target_system = target_system
        self.target_component = target_component
        self.upload_timeout_s = upload_timeout_s
        self.ack_msg = MissionAck()
        self.request_msg = MissionRequestInt()
        
    def _upload_mission(self, sender, receiver) -> None:
        mission = [self._waypoint_to_mission_item(wp) for wp in self.waypoints]
        future = receiver.wait_for_msg(self.request_msg, blocking=False)
        sender.send_msg(
            MissionCount(
                len(mission), self.target_system, self.target_component
            )
        )
        sent: set[int] = set()
        deadline = time.monotonic() + self.upload_timeout_s
        ack_future = None

        while len(sent) < len(mission) and time.monotonic() < deadline:
            future.wait_until_finished()
            if self.request_msg.mission_type != MissionType.MISSION:
                future = receiver.wait_for_msg(self.request_msg, blocking=False)
                continue

            seq = self.request_msg.seq
            if not 0 <= seq < len(mission):
                raise RuntimeError(f"Autopilot requested invalid mission item {seq}")
            lat, lon, altitude, radius = mission[seq]

            will_complete_upload = len(sent | {seq}) == len(mission)
            if will_complete_upload:
                ack_future = receiver.wait_for_msg(self.ack_msg, blocking=False)
            else:
                future = receiver.wait_for_msg(self.request_msg, blocking=False)

            sender.send_msg(
                MissionItemInt(
                    seq,
                    lat,
                    lon,
                    altitude,
                    radius,
                    self.target_system,
                    self.target_component,
                )
            )
            sent.add(seq)

        if len(sent) != len(mission):
            raise TimeoutError(
                f"Mission upload timed out after {len(sent)}/{len(mission)} items"
            )
        if ack_future is None:
            raise RuntimeError("Mission upload ended without waiting for an ACK")
        ack_future.wait_until_finished()
        if self.ack_msg.result != MissionResult.ACCEPTED:
            raise RuntimeError(f"Mission upload rejected: {self.ack_msg}")

    def _waypoint_to_mission_item(
        self, waypoint: Waypoint
    ) -> tuple[float, float, float, float]:
        lat, lon, relative_altitude = self.local_to_gps(
            np.array([waypoint.x, waypoint.y, waypoint.z], dtype=float),
            self.global_home,
        )
        return float(lat), float(lon), float(relative_altitude), float(waypoint.radius)


    def run(self, sender, receiver) -> None:
        if not self.waypoints:
            return

        self._upload_mission(sender, receiver)

        reached_msg = MissionItemReached(
            callback_func=lambda msg: self.log_func(
                f"[AutoMission] Reached Waypoint {msg.seq + 1}/{len(self.waypoints)}"
            )
        )
        receiver.add_listener(reached_msg)
        try:
            self.set_auto()
            final_item = len(self.waypoints) - 1
            while reached_msg.get_seq() != final_item:
                time.sleep(0.1)
        finally:
            receiver.remove_listener(reached_msg)
            reached_msg.end = True
