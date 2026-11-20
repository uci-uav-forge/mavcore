import time

from ..messages.param_request_list_msg import ParamRequestList
from ..messages.param_request_read_msg import ParamRequestRead
from ..mavtypes import Param, ParamEncoding
from ._param_protocol import _ParamProtocol


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
        transfer_timeout_s: float | None = None,
    ):
        super().__init__(target_system, target_component, encoding, timeout_s, retries)
        self.idle_timeout_s = idle_timeout_s
        self.batch_size = batch_size
        self.transfer_timeout_s = transfer_timeout_s

        self.success = False
        self.params: dict[str, Param] = {}
        self.param_count = 0
        self.missing: list[int] = []

    def _missing(self) -> list[int]:
        return [i for i in range(self._count) if i not in self._by_index]

    def run(self, sender, receiver):
        self.success, self.params, self.param_count, self.missing = False, {}, 0, []
        deadline = (time.monotonic() + self.transfer_timeout_s
                    if self.transfer_timeout_s is not None else float("inf"))
        self._start_listening(receiver)
        try:
            self._stream(sender, deadline)
            self._fill_missing(sender, deadline)
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
        self.success = self.param_count > 0 and len(self.params) == self.param_count and not self.missing

    def _stream(self, sender, deadline):
        request = ParamRequestList(self.target_system, self.target_component)
        for _ in range(self.retries + 1):
            if time.monotonic() >= deadline:
                return
            self._send(sender, request)
            if self._wait(lambda: self._count > 0, min(self.timeout_s, deadline - time.monotonic())):
                break
        else:
            return

        with self._cond:
            while self._missing():
                idle_left = min(deadline, self._last_rx + self.idle_timeout_s) - time.monotonic()
                if idle_left <= 0:
                    return
                self._cond.wait(idle_left)

    def _fill_missing(self, sender, deadline):
        for _ in range(self.retries):
            with self._cond:
                missing = self._missing()
            if not missing:
                return
            for start in range(0, len(missing), self.batch_size):
                batch = missing[start : start + self.batch_size]
                for index in batch:
                    if time.monotonic() >= deadline:
                        return
                    self._send(
                        sender,
                        ParamRequestRead(
                            param_index=index,
                            target_system=self.target_system,
                            target_component=self.target_component,
                        )
                    )
                self._wait(
                    lambda: all(i in self._by_index for i in batch),
                    min(self.timeout_s, deadline - time.monotonic()),
                )
