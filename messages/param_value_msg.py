from ..mav_message import MAVMessage, thread_safe
from ..mavtypes import Param, ParamEncoding


class ParamValue(MAVMessage):
    """
    PARAM_VALUE: one parameter value reported by the flight controller.
    """

    def __init__(
        self, encoding: ParamEncoding = ParamEncoding.C_CAST, cb=lambda x: None
    ):
        super().__init__("PARAM_VALUE", callback_func=cb)
        self.encoding = encoding
        self.param: Param | None = None

    def decode(self, msg):
        self.param = Param.from_msg(msg, self.encoding)

    @thread_safe
    def __repr__(self):
        return f"(PARAM_VALUE) timestamp: {self.timestamp}, param: {self.param}"
