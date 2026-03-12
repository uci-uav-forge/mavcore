from ..mav_message import MAVMessage
import pymavlink.dialects.v20.all as dialect


class FenceMissionClearAll(MAVMessage):
    """
    Wrapper for MISSION_CLEAR_ALL (mission_type = FENCE).
    """

    def __init__(self, target_system: int = 1, target_component: int = 0):
        super().__init__("MISSION_CLEAR_ALL")
        self.target_system = target_system
        self.target_component = target_component

    def encode(self, system_id, component_id):
        return dialect.MAVLink_mission_clear_all_message(
            target_system=self.target_system,
            target_component=self.target_component,
            mission_type=dialect.MAV_MISSION_TYPE_FENCE,
        )


class FenceMissionCount(MAVMessage):
    """
    Wrapper for MISSION_COUNT (mission_type = FENCE).
    """

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
            mission_type=dialect.MAV_MISSION_TYPE_FENCE,
        )


class FenceMissionItemInt(MAVMessage):
    """
    Wrapper for MISSION_ITEM_INT (mission_type = FENCE) for a single polygon vertex.
    """

    def __init__(
        self,
        seq: int,
        lat_deg: float,
        lon_deg: float,
        total_vertices: int,
        inclusion: bool = True,  # False => exclusion polygon
        target_system: int = 1,
        target_component: int = 0,
    ):
        super().__init__("MISSION_ITEM_INT")
        self.seq = seq
        self.lat_deg = lat_deg
        self.lon_deg = lon_deg
        self.total_vertices = total_vertices
        self.inclusion = inclusion
        self.target_system = target_system
        self.target_component = target_component

    def encode(self, system_id, component_id):
        command = (
            dialect.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
            if self.inclusion
            else dialect.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
        )

        return dialect.MAVLink_mission_item_int_message(
            target_system=self.target_system,
            target_component=self.target_component,
            seq=self.seq,
            frame=dialect.MAV_FRAME_GLOBAL,
            command=command,
            current=0,
            autocontinue=0,
            param1=float(self.total_vertices),
            param2=0.0,
            param3=0.0,
            param4=0.0,
            x=int(self.lat_deg * 1e7),
            y=int(self.lon_deg * 1e7),
            z=0.0,
            mission_type=dialect.MAV_MISSION_TYPE_FENCE,
        )
