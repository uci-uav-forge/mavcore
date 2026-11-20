from ..messages.param_request_read_msg import ParamRequestRead
from ..mavtypes import MAVParamError, Param, ParamEncoding
from ._param_protocol import _ParamProtocol


class ParamReadProtocol(_ParamProtocol):
    """
    Reads one parameter by name (or by index if param_index >= 0) with PARAM_REQUEST_READ.
    ArduPilot does not reply for a parameter that does not exist, so that times out.

    After running:
        success: whether the parameter was read
        param: the parameter, None if not read
        error: MAVParamError from the vehicle, UNKNOWN on timeout, None on success
    """

    def __init__(
        self,
        param_id: str = "",
        param_index: int = -1,
        target_system: int = 1,
        target_component: int = 1,
        encoding: ParamEncoding = ParamEncoding.C_CAST,
        timeout_s: float = 1.0,
        retries: int = 3,
    ):
        super().__init__(target_system, target_component, encoding, timeout_s, retries)
        ParamRequestRead(param_id, param_index)  # Raises early on a bad param_id
        self.param_id = param_id
        self.param_index = param_index

        self.success = False
        self.param: Param | None = None
        self.error: MAVParamError | None = None

    def run(self, sender, receiver):
        self.success, self.param, self.error = False, None, None
        self._start_listening(receiver)
        try:
            result = self._read(sender, self.param_id, self.param_index)
        finally:
            self._stop_listening(receiver)

        if isinstance(result, Param):
            self.success, self.param = True, result
        else:
            self.error = result
