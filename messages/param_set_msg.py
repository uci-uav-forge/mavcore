import pymavlink.dialects.v20.all as dialect

from ..mav_message import MAVMessage
from ..mavtypes.param_types import (
    MAVParamType,
    ParamEncoding,
    _encode_param_id,
    encode_param_value,
)


class ParamSet(MAVMessage):
    """
    PARAM_SET: sets one parameter. Raises ValueError if value cannot be sent as param_type.
    """

    def __init__(
        self,
        param_id: str,
        value: int | float,
        param_type: MAVParamType,
        encoding: ParamEncoding = ParamEncoding.C_CAST,
        target_system: int = 1,
        target_component: int = 1,
    ):
        super().__init__("PARAM_SET")
        self._param_id = _encode_param_id(param_id)
        self._param_value = encode_param_value(value, param_type, encoding)
        self.param_id = param_id
        self.value = value
        self.param_type = param_type
        self.encoding = encoding
        self.target_system = target_system
        self.target_component = target_component

    def encode(self, system_id, component_id):
        return dialect.MAVLink_param_set_message(
            target_system=self.target_system,
            target_component=self.target_component,
            param_id=self._param_id,
            param_value=self._param_value,
            param_type=int(self.param_type),
        )
