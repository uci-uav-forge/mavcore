import pymavlink.dialects.v20.all as dialect

from mavcore.mav_message import MAVMessage, thread_safe


class Arm(MAVMessage):
    """
    Arms device.
    """

    def __init__(self, target_system: int, target_component: int):
        super().__init__("CUSTOM_SET_MODE")
        self.target_system = target_system
        self.target_component = target_component

    def encode(self, system_id, component_id):
        return dialect.MAVLink_command_long_message(
            target_system=self.target_system,
            target_component=self.target_component,
            command=400,  # MAV_CMD_COMPONENT_ARM_DISARM (400)
            confirmation=0,
            param1=1.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=0.0,
        )
