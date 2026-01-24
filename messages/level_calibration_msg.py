import pymavlink.dialects.v20.all as dialect
from mavcore.mav_message import MAVMessage


class LevelCal(MAVMessage):
    """
    Calibrates the level.
    """

    def __init__(self, target_system: int, target_component: int):
        super().__init__("ACCELEROMETER_CALIBRATION")
        self.target_system = target_system
        self.target_component = target_component

    def encode(self, target_system: int, target_component: int):
        return dialect.MAVLink_command_long_message(
            target_system=self.target_system,
            target_component=self.target_component,
            command=241,  # MAV_CMD_PREFLIGHT_CALIBRATION (241)
            confirmation=0,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            param5=2.0,
            param6=0.0,
            param7=0.0,
        )
