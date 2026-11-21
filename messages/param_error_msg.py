from ..mav_message import MAVMessage, thread_safe
from ..mavtypes import MAVParamError
from ..mavtypes.param_types import _decode_param_id


class ParamError(MAVMessage):
    """
    PARAM_ERROR: the vehicle could not read or set a parameter. ArduPilot does not send this,
    it ignores requests for parameters that do not exist.
    """

    def __init__(self, cb=lambda x: None):
        super().__init__("PARAM_ERROR", callback_func=cb)
        self.param_id = ""
        self.param_index = -1
        self.error = MAVParamError.UNKNOWN
        self.system = self.component = -1

    def decode(self, msg):
        self.param_id = _decode_param_id(msg.param_id)
        self.param_index = msg.param_index
        self.system = msg.get_srcSystem()
        self.component = msg.get_srcComponent()
        try:
            self.error = MAVParamError(msg.error)
        except ValueError:
            self.error = MAVParamError.UNKNOWN

    @thread_safe
    def __repr__(self):
        return f"(PARAM_ERROR) timestamp: {self.timestamp}, param_id: '{self.param_id}', index: {self.param_index}, error: {self.error.name}"
