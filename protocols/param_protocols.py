import threading
import time
from typing import Callable

from ..mav_message import MAVMessage
from ..mav_protocol import MAVProtocol
from ..messages.param_msgs import (
    MAVParamError,
    Param,
    ParamEncoding,
    ParamRequestRead,
)


class _InlineListener(MAVMessage):
    """
    Hands every received message to handler on the receiver thread. A parameter list streams
    hundreds of PARAM_VALUE messages, more than the 50 Hz, 15 message deep listener queue keeps.
    """

    def __init__(self, name: str, handler: Callable):
        super().__init__(name)
        self._handler = handler

    def _start_callback_thread(self):
        pass  # Messages are handled in process_message, no thread needed

    def process_message(self, msg):
        self._handler(msg)


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
        self._count = 0
        self._last_rx = 0.0
        self._listeners = [
            _InlineListener("PARAM_VALUE", self._on_param_value),
            _InlineListener("PARAM_ERROR", self._on_param_error),
        ]

    def _from_target(self, msg) -> bool:
        if msg.get_srcSystem() != self.target_system:
            return False
        return (
            self.target_component == 0
            or msg.get_srcComponent() == self.target_component
        )

    def _on_param_value(self, msg):
        if not self._from_target(msg):
            return
        param = Param.from_msg(msg, self.encoding)
        with self._cond:
            self._by_name[param.name] = param
            if 0 <= param.index < param.count:
                self._by_index[param.index] = param
            self._count = param.count
            self._last_rx = time.monotonic()
            self._cond.notify_all()

    def _on_param_error(self, msg):
        if not self._from_target(msg):
            return
        try:
            error = MAVParamError(msg.error)
        except ValueError:
            error = MAVParamError.UNKNOWN
        with self._cond:
            if msg.param_id:
                self._errors_by_name[msg.param_id] = error
            if msg.param_index >= 0:
                self._errors_by_index[msg.param_index] = error
            self._cond.notify_all()

    def _start_listening(self, receiver):
        with self._cond:
            self._by_name.clear()
            self._by_index.clear()
            self._errors_by_name.clear()
            self._errors_by_index.clear()
            self._count = 0
            self._last_rx = 0.0
        for listener in self._listeners:
            receiver.add_listener(listener)

    def _stop_listening(self, receiver):
        for listener in self._listeners:
            receiver.remove_listener(listener)

    def _wait(self, predicate: Callable[[], bool], timeout_s: float) -> bool:
        with self._cond:
            return self._cond.wait_for(predicate, timeout_s)

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
            sender.send_msg(request)
            if self._wait(lambda: key in values or key in errors, self.timeout_s):
                break
        with self._cond:
            if key in values:
                return values[key]
            return errors.get(key, MAVParamError.UNKNOWN)


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
