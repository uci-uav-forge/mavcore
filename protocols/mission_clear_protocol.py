from ..mav_protocol import MAVProtocol
from ..messages import MissionAck
from ..messages.mission_ack_msg import MissionResult
from ..messages.mission_plan_msgs import MissionClearAll


class MissionClearProtocol(MAVProtocol):
    def __init__(self, target_system: int = 1, target_component: int = 0):
        super().__init__()
        self.target_system = target_system
        self.target_component = target_component
        self.clear_msg = MissionClearAll(target_system, target_component)
        self.ack_msg = MissionAck()
        self.request_timeout_s = 1.5
        self.max_retries = 5

    def run(self, sender, receiver):
        for _ in range(self.max_retries):
            sender.send_msg(self.clear_msg)
            future_ack = receiver.wait_for_msg(
                self.ack_msg, timeout_seconds=self.request_timeout_s
            )
            if future_ack.timestamp == 0.0:
                continue
            if self.ack_msg.result != MissionResult.ACCEPTED:
                raise RuntimeError(
                    f"Mission clear rejected: {self.ack_msg.result.name}"
                )
            return self.ack_msg

        raise TimeoutError("MISSION_CLEAR_ALL did not receive MISSION_ACK")
