from ..mav_message import MAVMessage
import pymavlink.dialects.v20.all as dialect


class MissionCount(MAVMessage):
    def __init__(self, count: int, target_system: int = 1, target_component: int = 0):
        super().__init__("MISSION_COUNT")
        self.count = count
        self.target_system = target_system
        self.target_component = target_component

    def encode(self, system_id, component_id):
        return dialect.MAVLink_mission_count_message(
            self.target_system, self.target_component, self.count,
            mission_type=dialect.MAV_MISSION_TYPE_MISSION,
        )


class MissionItemInt(MAVMessage):
    def __init__(self, seq: int, lat_deg: float, lon_deg: float, altitude_m: float,
                 acceptance_radius: float = 0.0,
                 target_system: int = 1, target_component: int = 0):
        super().__init__("MISSION_ITEM_INT")
        self.seq, self.lat_deg, self.lon_deg, self.altitude_m = seq, lat_deg, lon_deg, altitude_m
        self.acceptance_radius = acceptance_radius
        self.target_system, self.target_component = target_system, target_component

    def encode(self, system_id, component_id):
        return dialect.MAVLink_mission_item_int_message(
            target_system=self.target_system, target_component=self.target_component,
            seq=self.seq, frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            command=dialect.MAV_CMD_NAV_WAYPOINT, current=0, autocontinue=1,
            param1=0.0, param2=self.acceptance_radius, param3=0.0, param4=0.0,
            x=int(self.lat_deg * 1e7), y=int(self.lon_deg * 1e7), z=self.altitude_m,
            mission_type=dialect.MAV_MISSION_TYPE_MISSION,
        )
