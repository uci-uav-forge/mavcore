import pymavlink.dialects.v20.all as dialect
from mavcore.mav_message import MAVMessage, thread_safe


class SetHome(MAVMessage):
    """
    Sets the ArduPilot home to the current global position to set where it will RTL.
    """

    def __init__(self, target_system: int, target_component: int):
        super().__init__("SET_HOME")
        self.target_system = target_system
        self.target_component = target_component

    def encode(self, system_id, component_id):
        return dialect.MAVLink_command_long_message(
            target_system=self.target_system,
            target_component=self.target_component,
            command=179,  # MAV_CMD_DO_SET_HOME (179)
            confirmation=0,
            param1=1.0,  # Use current location
            param2=0.0,
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=0.0,
        )

    @thread_safe
    def __repr__(self):
        return f"(SET_HOME) timestamp: {self.timestamp}"
