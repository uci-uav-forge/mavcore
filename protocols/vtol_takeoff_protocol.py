import time

import pymavlink.dialects.v20.all as dialect

from ..mav_message import MAVMessage, thread_safe
from ..mav_protocol import MAVProtocol
from ..messages import MissionAck, MissionResult, MissionType


class _MissionRequest(MAVMessage):
    def __init__(self, message_name: str):
        super().__init__(message_name)
        self.seq = -1
        self.mission_type = MissionType.MISSION

    def decode(self, msg):
        self.seq = msg.seq
        self.mission_type = MissionType(getattr(msg, "mission_type", 0))

    @thread_safe
    def __repr__(self):
        return f"({self.name}) seq={self.seq} type={self.mission_type.name}"


class _MissionCount(MAVMessage):
    def __init__(self, target_system: int, target_component: int):
        super().__init__("MISSION_COUNT")
        self.target_system = target_system
        self.target_component = target_component

    def encode(self, system_id, component_id):
        return dialect.MAVLink_mission_count_message(
            target_system=self.target_system,
            target_component=self.target_component,
            # ArduPilot reserves sequence 0 for HOME, so the takeoff command
            # is uploaded at sequence 1.
            count=2,
            mission_type=dialect.MAV_MISSION_TYPE_MISSION,
        )


class _MissionItemInt(MAVMessage):
    def __init__(
        self,
        target_system: int,
        target_component: int,
        seq: int,
        command: int,
        altitude: float,
    ):
        super().__init__("MISSION_ITEM_INT")
        self.target_system = target_system
        self.target_component = target_component
        self.seq = seq
        self.command = command
        self.altitude = altitude

    def encode(self, system_id, component_id):
        return dialect.MAVLink_mission_item_int_message(
            target_system=self.target_system,
            target_component=self.target_component,
            seq=self.seq,
            frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=self.command,
            current=0,
            autocontinue=1,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            x=0,
            y=0,
            z=float(self.altitude),
            mission_type=dialect.MAV_MISSION_TYPE_MISSION,
        )


class VTOLTakeoffProtocol(MAVProtocol):
    """Upload a VTOL takeoff mission for ArduPlane to execute."""

    def __init__(
        self, altitude: float, target_system: int = 1, target_component: int = 0
    ):
        super().__init__()
        self.target_system = target_system
        self.target_component = target_component
        self.altitude = altitude
        self.ack_msg = MissionAck()
        self.request_int_msg = _MissionRequest("MISSION_REQUEST_INT")
        self.request_msg = _MissionRequest("MISSION_REQUEST")

    def run(self, sender, receiver):
        self.request_int_msg = _MissionRequest("MISSION_REQUEST_INT")
        self.request_msg = _MissionRequest("MISSION_REQUEST")
        self.ack_msg = MissionAck()
        receiver.add_listener(self.request_int_msg)
        receiver.add_listener(self.request_msg)
        receiver.add_listener(self.ack_msg)
        sender.send_msg(_MissionCount(self.target_system, self.target_component))
        try:
            for seq, command, altitude in (
                (0, dialect.MAV_CMD_NAV_WAYPOINT, 0.0),
                (1, dialect.MAV_CMD_NAV_VTOL_TAKEOFF, self.altitude),
            ):
                deadline = time.monotonic() + 10.0
                while time.monotonic() < deadline:
                    request = next(
                        (
                            candidate
                            for candidate in (self.request_int_msg, self.request_msg)
                            if candidate._decoded
                            and candidate.mission_type == MissionType.MISSION
                            and candidate.seq == seq
                        ),
                        None,
                    )
                    if request is not None:
                        break
                    if self.ack_msg._decoded:
                        raise RuntimeError(
                            f"VTOL takeoff mission upload rejected: {self.ack_msg}"
                        )
                    # Ignore retries for the previous sequence while waiting
                    # for the next item request.
                    for candidate in (self.request_int_msg, self.request_msg):
                        if candidate._decoded:
                            candidate.timestamp = 0.0
                            candidate._decoded = False
                            candidate.seq = -1
                    time.sleep(0.01)
                else:
                    raise TimeoutError(
                        f"ArduPilot did not request mission item {seq}"
                    )

                # Reset both request listeners before sending, so the next
                # request cannot race with listener registration.
                for candidate in (self.request_int_msg, self.request_msg):
                    candidate.timestamp = 0.0
                    candidate._decoded = False
                    candidate.seq = -1
                sender.send_msg(
                    _MissionItemInt(
                        self.target_system,
                        self.target_component,
                        seq,
                        command,
                        altitude,
                    )
                )

            deadline = time.monotonic() + 10.0
            while not self.ack_msg._decoded and time.monotonic() < deadline:
                time.sleep(0.01)
            if not self.ack_msg._decoded:
                raise TimeoutError(
                    "ArduPilot did not acknowledge the VTOL takeoff mission"
                )
            if self.ack_msg.result != MissionResult.ACCEPTED:
                raise RuntimeError(f"VTOL takeoff mission rejected: {self.ack_msg}")
        finally:
            receiver.remove_listener(self.request_int_msg)
            receiver.remove_listener(self.request_msg)
            receiver.remove_listener(self.ack_msg)
