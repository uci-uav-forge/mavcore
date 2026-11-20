from ..messages.param_request_read_msg import ParamRequestRead
from ..messages.param_set_msg import ParamSet
from ..mavtypes.param_types import (
    MAVParamError,
    MAVParamType,
    Param,
    ParamEncoding,
    encode_param_value,
    param_values_match,
)
from ._param_protocol import _ParamProtocol


class ParamWriteProtocol(_ParamProtocol):
    """
    Sets one parameter with PARAM_SET and waits for the PARAM_VALUE that confirms it.
    If param_type is None the parameter is read first to find its type.

    The vehicle replies with the parameter's value after the set, unchanged if it rejected
    the value. The set is resent if no reply arrives or the value does not match.

    After running:
        success: whether the vehicle reported the new value
        param: the last value the vehicle reported, None if it never replied
        error: MAVParamError from the vehicle, VALUE_OUT_OF_RANGE if value cannot be sent as
            the parameter's type, UNKNOWN on timeout or rejected value, None on success
    """

    def __init__(
        self,
        param_id: str,
        value: int | float,
        param_type: MAVParamType | None = None,
        target_system: int = 1,
        target_component: int = 1,
        encoding: ParamEncoding = ParamEncoding.C_CAST,
        timeout_s: float = 1.0,
        retries: int = 3,
    ):
        super().__init__(target_system, target_component, encoding, timeout_s, retries)
        ParamRequestRead(param_id)  # Raises early on a bad param_id
        if param_type is not None:
            encode_param_value(value, param_type, encoding)  # and on a bad value
        self.param_id = param_id
        self.value = value
        self.param_type = param_type

        self.success = False
        self.param: Param | None = None
        self.error: MAVParamError | None = None

    def run(self, sender, receiver):
        self.success, self.param, self.error = False, None, None
        self._start_listening(receiver)
        try:
            self._write(sender)
        finally:
            self._stop_listening(receiver)

    def _write(self, sender):
        param_type = self.param_type
        if param_type is None:
            current = self._read(sender, self.param_id)
            if not isinstance(current, Param):
                self.error = current
                return
            param_type = current.param_type

        try:
            set_msg = ParamSet(
                self.param_id,
                self.value,
                param_type,
                self.encoding,
                self.target_system,
                self.target_component,
            )
        except ValueError as e:
            print(f"ERROR: cannot set {self.param_id}: {e}")
            self.error = MAVParamError.VALUE_OUT_OF_RANGE
            return

        name = self.param_id
        for _ in range(self.retries + 1):
            with self._cond:
                self._by_name.pop(name, None)
            self._send(sender, set_msg)
            self._wait(
                lambda: name in self._by_name or name in self._errors_by_name,
                self.timeout_s,
            )
            with self._cond:
                self.param = self._by_name.get(name, self.param)
                if name in self._errors_by_name:
                    self.error = self._errors_by_name[name]
                    return
            if self.param is not None and param_values_match(
                self.value, self.param.value, param_type
            ):
                self.success = True
                return
        self.error = MAVParamError.UNKNOWN
