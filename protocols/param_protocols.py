import threading
import time
from typing import Callable

from ..mav_message import MAVMessage
from ..mav_protocol import MAVProtocol
from ..messages.param_msgs import (
    MAVParamError,
    MAVParamType,
    Param,
    ParamEncoding,
    ParamRequestList,
    ParamRequestRead,
    ParamSet,
    encode_param_value,
    param_values_match,
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
            sender.send_msg(set_msg)
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


class ParamListProtocol(_ParamProtocol):
    """
    Downloads all parameters with PARAM_REQUEST_LIST. The vehicle streams every parameter as
    PARAM_VALUE with its index, the stream is over when all indices arrived or nothing arrives
    for idle_timeout_s. Missing indices are then requested with PARAM_REQUEST_READ, batch_size
    at a time (ArduPilot queues up to 20 requests), for up to retries rounds.

    After running:
        success: whether every parameter was received
        params: parameters by name, in index order
        param_count: total parameters the vehicle reported, 0 if it never replied
        missing: indices that were never received
    """

    def __init__(
        self,
        target_system: int = 1,
        target_component: int = 1,
        encoding: ParamEncoding = ParamEncoding.C_CAST,
        timeout_s: float = 2.0,
        idle_timeout_s: float = 1.0,
        retries: int = 3,
        batch_size: int = 10,
    ):
        super().__init__(target_system, target_component, encoding, timeout_s, retries)
        self.idle_timeout_s = idle_timeout_s
        self.batch_size = batch_size

        self.success = False
        self.params: dict[str, Param] = {}
        self.param_count = 0
        self.missing: list[int] = []

    def _missing(self) -> list[int]:
        return [i for i in range(self._count) if i not in self._by_index]

    def run(self, sender, receiver):
        self.success, self.params, self.param_count, self.missing = False, {}, 0, []
        self._start_listening(receiver)
        try:
            self._stream(sender)
            self._fill_missing(sender)
        finally:
            self._stop_listening(receiver)

        with self._cond:
            self.param_count = self._count
            self.missing = self._missing()
            self.params = {
                p.name: p
                for i, p in sorted(self._by_index.items())
                if i < self.param_count
            }
        self.success = self.param_count > 0 and not self.missing

    def _stream(self, sender):
        request = ParamRequestList(self.target_system, self.target_component)
        for _ in range(self.retries + 1):
            sender.send_msg(request)
            if self._wait(lambda: self._count > 0, self.timeout_s):
                break
        else:
            return

        with self._cond:
            while self._missing():
                idle_left = self._last_rx + self.idle_timeout_s - time.monotonic()
                if idle_left <= 0:
                    return
                self._cond.wait(idle_left)

    def _fill_missing(self, sender):
        for _ in range(self.retries):
            with self._cond:
                missing = self._missing()
            if not missing:
                return
            for start in range(0, len(missing), self.batch_size):
                batch = missing[start : start + self.batch_size]
                for index in batch:
                    sender.send_msg(
                        ParamRequestRead(
                            param_index=index,
                            target_system=self.target_system,
                            target_component=self.target_component,
                        )
                    )
                self._wait(
                    lambda: all(i in self._by_index for i in batch), self.timeout_s
                )
