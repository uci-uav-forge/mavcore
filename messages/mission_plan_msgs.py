import pymavlink.dialects.v20.all as dialect

from ..mav_message import MAVMessage
from .mission_request_msg import MissionType


class MissionClearAll(MAVMessage):
    def __init__(self, target_system: int = 1, target_component: int = 0):
        super().__init__("MISSION_CLEAR_ALL")
        self.target_system = target_system
        self.target_component = target_component

    def encode(self, system_id, component_id):
        return dialect.MAVLink_mission_clear_all_message(
            target_system=self.target_system,
            target_component=self.target_component,
            mission_type=dialect.MAV_MISSION_TYPE_MISSION,
        )


class MissionCount(MAVMessage):
    def __init__(self, count: int, target_system: int = 1, target_component: int = 0):
        super().__init__("MISSION_COUNT")
        self.count = count
        self.target_system = target_system
        self.target_component = target_component

    def encode(self, system_id, component_id):
        return dialect.MAVLink_mission_count_message(
            target_system=self.target_system,
            target_component=self.target_component,
            count=self.count,
            mission_type=dialect.MAV_MISSION_TYPE_MISSION,
        )


class MissionItemInt(MAVMessage):
    def __init__(
        self,
        seq: int,
        command: int,
        x: int,
        y: int,
        z: float,
        target_system: int = 1,
        target_component: int = 0,
        frame: int = dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        current: int = 0,
        autocontinue: int = 1,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        mission_type: MissionType = MissionType.MISSION,
    ):
        super().__init__("MISSION_ITEM_INT")
        self.seq = seq
        self.command = command
        self.x = x
        self.y = y
        self.z = z
        self.target_system = target_system
        self.target_component = target_component
        self.frame = frame
        self.current = current
        self.autocontinue = autocontinue
        self.param1 = param1
        self.param2 = param2
        self.param3 = param3
        self.param4 = param4
        self.mission_type = mission_type

    def encode(self, system_id, component_id):
        return dialect.MAVLink_mission_item_int_message(
            target_system=self.target_system,
            target_component=self.target_component,
            seq=self.seq,
            frame=self.frame,
            command=self.command,
            current=self.current,
            autocontinue=self.autocontinue,
            param1=self.param1,
            param2=self.param2,
            param3=self.param3,
            param4=self.param4,
            x=self.x,
            y=self.y,
            z=self.z,
            mission_type=self.mission_type.value,
        )