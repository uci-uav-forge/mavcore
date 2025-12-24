import pymavlink.dialects.v20.all as dialect
from enum import IntEnum

from mavcore.mav_message import MAVMessage, thread_safe


class ShutdownAction(IntEnum):
    NO_ACTION = 0
    REBOOT = 1
    SHUTDOWN = 2
    REBOOT_BOOTLOADER = 3
    REBOOT_SHUTDOWN_POWER_ON = 4


class RebootMsg(MAVMessage):
    """
    Requests reboot of fc.
    """

    def __init__(self, target_system: int, target_component: int):
        super().__init__("MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN")
        self.target_system = target_system
        self.target_component = target_component

    def encode(self, system_id, component_id):
        return dialect.MAVLink_command_long_message(
            target_system=self.target_system,
            target_component=self.target_component,
            command=246,  # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246)
            confirmation=0,
            param1=float(ShutdownAction.REBOOT),
            param2=0.0,
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=0.0,
        )

    @thread_safe
    def __repr__(self):
        return f"(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN) timestamp: {self.timestamp}"
