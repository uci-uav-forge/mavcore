import time
from ..messages import MissionAck
from pymavlink.dialects.v20 import common as mav
from ..mav_protocol import MAVProtocol
from ..messages import (
    FenceMissionCount,
    FenceMissionItemInt,
    MissionRequestInt,
    MissionType,
)


class FenceUploadProtocol(MAVProtocol):
    """
    Uploads polygon vertices via the Mission sub-protocol (MAV_MISSION_TYPE_FENCE).

    Handshake:
      1. MISSION_COUNT (FENCE)
      2. Repeated:
           FC -> MISSION_REQUEST_INT (FENCE, seq=i)
           Us -> MISSION_ITEM_INT (FENCE, seq=i)
      3. MISSION_ACK (FENCE)
    """

    def __init__(
        self,
        vertices: list[tuple[float, float]],
        target_system: int = 1,
        target_component: int = 0,
    ):
        super().__init__()
        self.vertices = vertices
        self.target_system = target_system
        self.target_component = target_component

        self.handshake_timeout_s: float = 10.0
        self.ack_timeout_s: float = 3.0

        self.ack_msg = MissionAck()
        self.mission_req_msg = MissionRequestInt()

    def run(self, sender, receiver):
        # 1) Send MISSION_COUNT(FENCE)
        count_msg = FenceMissionCount(
            count=len(self.vertices),
            target_system=self.target_system,
            target_component=self.target_component,
        )
        future_req = receiver.wait_for_msg(self.mission_req_msg, blocking=False)
        sender.send_msg(count_msg)

        # 2) Serve MISSION_REQUEST_INT(FENCE) with MISSION_ITEM_INT(FENCE)
        sent = 0
        deadline = time.time() + self.handshake_timeout_s

        while sent < len(self.vertices) and time.time() < deadline:
            future_req.wait_until_finished()
            if self.mission_req_msg.mission_type != MissionType.FENCE:
                continue

            seq = self.mission_req_msg.seq
            if seq < 0 or seq >= len(self.vertices):
                continue

            lat, lon = self.vertices[seq]

            item_msg = FenceMissionItemInt(
                seq=seq,
                lat_deg=float(lat),
                lon_deg=float(lon),
                total_vertices=len(self.vertices),
                inclusion=True,
                target_system=self.target_system,
                target_component=self.target_component,
            )
            if sent + 1 < len(self.vertices):
                future_req = receiver.wait_for_msg(self.mission_req_msg, blocking=False)
            else:
                future_ack = receiver.wait_for_msg(self.ack_msg, blocking=False)

            sender.send_msg(item_msg)
            sent += 1

        if sent != len(self.vertices):
            print(f"ERROR: sent {sent}/{len(self.vertices)} fence vertices")
            return

        # 3) Wait for MISSION_ACK(FENCE)
        future_ack.wait_until_finished()
