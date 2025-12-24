import pymavlink.dialects.v20.all as dialect

from mavcore.mav_message import MAVMessage, thread_safe
from mavcore.messages.heartbeat_msg import FlightMode


class SetMode(MAVMessage):
    """
    Allows to set mode of device. Uses FlightMode defined in heartbeat_msg.
    """

    def __init__(self, target_system: int, target_component: int, mode: FlightMode):
        super().__init__("CUSTOM_SET_MODE")
        self.target_system = target_system
        self.target_component = target_component
        self.mode = mode

    def encode(self, system_id, component_id):
        return dialect.MAVLink_command_long_message(
            target_system=self.target_system,
            target_component=self.target_component,
            command=176,  # MAV_CMD_DO_SET_MODE (176)
            confirmation=0,
            param1=1.0,
            param2=float(self.mode.value),
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=0.0,
        )

    @thread_safe
    def __repr__(self):
        return f"(CUSTOM_SET_MODE) timestamp: {self.timestamp}, mode: {self.mode.name}"
