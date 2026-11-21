from ..mav_message import MAVMessage, thread_safe
from typing import Any, Callable


class MissionItemReached(MAVMessage):
    """Reports the sequence number of a mission item just completed."""

    def __init__(self, callback_func: Callable[[Any], None] = lambda msg: None):
        super().__init__("MISSION_ITEM_REACHED", callback_func=callback_func)
        self.seq = -1

    def decode(self, msg):
        self.seq = msg.seq

    @thread_safe
    def get_seq(self) -> int:
        return self.seq
