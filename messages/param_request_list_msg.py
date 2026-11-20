import pymavlink.dialects.v20.all as dialect

from ..mav_message import MAVMessage


class ParamRequestList(MAVMessage):
    """
    PARAM_REQUEST_LIST: asks the target to stream all of its parameters as PARAM_VALUE.
    """

    def __init__(self, target_system: int = 1, target_component: int = 1):
        super().__init__("PARAM_REQUEST_LIST")
        self.target_system = target_system
        self.target_component = target_component

    def encode(self, system_id, component_id):
        return dialect.MAVLink_param_request_list_message(
            target_system=self.target_system,
            target_component=self.target_component,
        )
