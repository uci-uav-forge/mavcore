import pymavlink.dialects.v20.all as dialect

from ..mav_message import MAVMessage
from ..mavtypes.param_types import _encode_param_id


class ParamRequestRead(MAVMessage):
    """
    PARAM_REQUEST_READ: asks for one parameter by name, or by index if param_index >= 0.
    """

    def __init__(
        self,
        param_id: str = "",
        param_index: int = -1,
        target_system: int = 1,
        target_component: int = 1,
    ):
        super().__init__("PARAM_REQUEST_READ")
        if param_index < 0:
            self._param_id = _encode_param_id(param_id)
        else:
            self._param_id = b""
        self.param_id = param_id
        self.param_index = param_index
        self.target_system = target_system
        self.target_component = target_component

    def encode(self, system_id, component_id):
        return dialect.MAVLink_param_request_read_message(
            target_system=self.target_system,
            target_component=self.target_component,
            param_id=self._param_id,
            param_index=self.param_index,
        )
