import threading
from typing import Callable

from ..mav_protocol import MAVProtocol
from ..messages.param_error_msg import ParamError
from ..messages.param_request_read_msg import ParamRequestRead
from ..messages.param_value_msg import ParamValue
from ..mavtypes import MAVParamError, Param, ParamEncoding


class _ParamProtocol(MAVProtocol):
    """
    Shared PARAM_VALUE / PARAM_ERROR bookkeeping for the parameter protocols.

    target_component: 1 (autopilot) by default so replies from other components are ignored.
        0 accepts replies from every component of target_system.
    timeout_s: how long to wait for a reply before resending a request
    retries: how many times a request is resent after a timeout
    """

    def __init__(
        self,
        target_system: int,
        target_component: int,
        encoding: ParamEncoding,
        timeout_s: float,
        retries: int,
    ):
        super().__init__()
        self.target_system = target_system
        self.target_component = target_component
        self.encoding = encoding
        self.timeout_s = timeout_s
        self.retries = retries

        self._cond = threading.Condition()
        self._by_name: dict[str, Param] = {}
        self._by_index: dict[int, Param] = {}
        self._errors_by_name: dict[str, MAVParamError] = {}
        self._errors_by_index: dict[int, MAVParamError] = {}
        self._replies = ()

    def _from_target(self, system, component) -> bool:
        return system == self.target_system and self.target_component in (0, component)

    def _on_param_value(self, reply):
        param = reply.param
        if not self._from_target(param.system, param.component):
            return
        with self._cond:
            self._record_param(param)
            self._cond.notify_all()

    def _record_param(self, param: Param) -> None:
        self._by_name[param.name] = param
        if 0 <= param.index < param.count:
            self._by_index[param.index] = param

    def _on_param_error(self, reply):
        if not self._from_target(reply.system, reply.component):
            return
        with self._cond:
            if reply.param_id:
                self._errors_by_name[reply.param_id] = reply.error
            if reply.param_index >= 0:
                self._errors_by_index[reply.param_index] = reply.error
            self._cond.notify_all()

    def _start_listening(self, receiver):
        with self._cond:
            self._by_name.clear()
            self._by_index.clear()
            self._errors_by_name.clear()
            self._errors_by_index.clear()
        self._replies = (
            ParamValue(self.encoding, self._on_param_value),
            ParamError(self._on_param_error),
        )
        for reply in self._replies:
            receiver.add_listener(reply)

    def _stop_listening(self, receiver):
        for reply in self._replies:
            receiver.remove_listener(reply)
            # MAVCore's stop method holds the decode lock while joining.
            reply.end = True
        for reply in self._replies:
            if reply._decodethread is not None:
                reply._decodethread.join()
        self._replies = ()

    def _wait(self, predicate: Callable[[], bool], timeout_s: float) -> bool:
        with self._cond:
            return self._cond.wait_for(predicate, timeout_s)

    @staticmethod
    def _send(sender, request):
        if sender.connection.portdead:
            raise OSError("MAVLink connection is down")
        sender.send_msg(request)

    def _read(self, sender, name: str = "", index: int = -1) -> Param | MAVParamError:
        """
        Requests one parameter until it arrives, PARAM_ERROR is received or retries run out.
        Returns the parameter, or the error (UNKNOWN on timeout).
        """
        request = ParamRequestRead(
            name, index, self.target_system, self.target_component
        )
        if index < 0:
            values, errors, key = self._by_name, self._errors_by_name, name
        else:
            values, errors, key = self._by_index, self._errors_by_index, index

        for _ in range(self.retries + 1):
            self._send(sender, request)
            if self._wait(lambda: key in values or key in errors, self.timeout_s):
                break
        with self._cond:
            if key in values:
                return values[key]
            return errors.get(key, MAVParamError.UNKNOWN)
