from collections.abc import Iterable

from pymavlink.dialects.v20 import common as mav

from ..mav_protocol import MAVProtocol
from ..messages import MissionAck, MissionRequestInt
from ..messages.mission_ack_msg import MissionResult
from ..messages.mission_plan_msgs import MissionCount, MissionItemInt
from ..messages.mission_request_msg import MissionType


class MissionUploadProtocol(MAVProtocol):
    def __init__(
        self,
        waypoints: Iterable[tuple[float, float, float] | Iterable[float]],
        target_system: int = 1,
        target_component: int = 0,
    ):
        super().__init__()
        self.target_system = target_system
        self.target_component = target_component
        self.waypoints = [
            self._coerce_waypoint(waypoint, seq)
            for seq, waypoint in enumerate(waypoints)
        ]
        self.count_msg = MissionCount(len(self.waypoints), target_system, target_component)
        self.request_msg = MissionRequestInt()
        self.ack_msg = MissionAck()
        self.request_timeout_s = 1.5
        self.ack_timeout_s = 3.0
        self.max_retries = 5

    def _coerce_waypoint(
        self, waypoint: tuple[float, float, float] | Iterable[float], seq: int
    ) -> MissionItemInt:
        values = list(waypoint)
        if len(values) < 2:
            raise ValueError("Waypoint must contain at least latitude and longitude")

        latitude = float(values[0])
        longitude = float(values[1])
        altitude = float(values[2]) if len(values) > 2 else 0.0

        return MissionItemInt(
            seq=seq,
            command=mav.MAV_CMD_NAV_WAYPOINT,
            x=int(latitude * 1e7),
            y=int(longitude * 1e7),
            z=altitude,
            target_system=self.target_system,
            target_component=self.target_component,
            frame=mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            current=0,
            autocontinue=1,
            mission_type=MissionType.MISSION,
        )

    def run(self, sender, receiver):
        if not self.waypoints:
            raise ValueError("Cannot upload an empty mission")

        request_msg = MissionRequestInt()

        for _ in range(self.max_retries):
            sender.send_msg(self.count_msg)
            future_request = receiver.wait_for_msg(
                request_msg, timeout_seconds=self.request_timeout_s
            )
            if future_request.timestamp != 0.0:
                break
        else:
            raise TimeoutError("MISSION_COUNT did not receive MISSION_REQUEST_INT")

        if request_msg.mission_type != MissionType.MISSION or request_msg.seq != 0:
            raise RuntimeError(
                f"Unexpected mission request after count: type={request_msg.mission_type}, seq={request_msg.seq}"
            )

        last_sent_item = None
        for seq, item in enumerate(self.waypoints):
            last_sent_item = item
            for _ in range(self.max_retries):
                sender.send_msg(item)
                future_request = receiver.wait_for_msg(
                    request_msg, timeout_seconds=self.request_timeout_s
                )

                if seq == len(self.waypoints) - 1:
                    break

                if future_request.timestamp == 0.0:
                    continue

                if (
                    request_msg.mission_type == MissionType.MISSION
                    and request_msg.seq == seq + 1
                ):
                    break
            else:
                raise TimeoutError(f"Timed out waiting for request seq {seq}")

        for _ in range(self.max_retries):
            future_ack = receiver.wait_for_msg(
                self.ack_msg, timeout_seconds=self.ack_timeout_s
            )
            if future_ack.timestamp != 0.0:
                if self.ack_msg.result != MissionResult.ACCEPTED:
                    raise RuntimeError(
                        f"Mission upload rejected: {self.ack_msg.result.name}"
                    )
                return self.ack_msg
            if last_sent_item is not None:
                sender.send_msg(last_sent_item)

        raise TimeoutError("MISSION_ITEM_INT upload finished without MISSION_ACK")