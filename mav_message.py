from typing import Any, Callable
import time
import threading
import queue


def thread_safe(func):
    def wrapper(*args, **kwargs) -> Any:
        self: MAVMessage = args[0]
        self._lock.acquire()
        output = func(*args, **kwargs)
        self._lock.release()
        return output

    return wrapper


class MAVMessage:
    def __init__(
        self,
        name: str,
        timestamp: float = 0.0,
        priority=0,
        repeat_period: float = 0.0,
        callback_func: Callable[[Any], None] = lambda msg: None,
    ):
        """
        Designed to be an interface/template for a MAVMessage:

        name: the mavlink message name to look for
        timestamp: the time in seconds the message was recieved
        priority: unused for now
        repeat_period: the interval at which the message will be repeatedly sent
        callback_func: a function that will be executed when this message is recieved and processed,
            this message instance is passed in to the first and only argument.
        non_blocking: if true, the callback function will be executed in a new thread
        """
        self.name = name
        self.timestamp = timestamp
        self.priority = priority
        self.repeat_period = repeat_period
        self.callback_func = callback_func
        self._lock = threading.RLock()
        self._thread: None | threading.Thread = None
        self.submessages: list[MAVMessage] = []
        # For calculating receive rate
        self.hz: float = 0.0
        self._pastdt: list[float] = []
        """
        Callback processing. Similar to ROS each listener has its own thread for processing messages so that
        one slow listener does not block others from being processed. There will be a queue of up to 15 messages
        for each listener. If the queue is full, the oldest message will be dropped.
        """
        self._msg_queue: "queue.Queue[Any]" = queue.Queue(maxsize=15)
        self._decodethread: None | threading.Thread = None
        self._queuelock = threading.Lock()
        self.end = False

        self._decoded = False  # Set False on wait_for_msg, True after decoded

        # Logging util
        self.logging_callback: Callable | None = None
        self.logging_rate = 10  # Hz
        self._log_thread = threading.Thread(target=self._log_loop, daemon=True)

    def __del__(self):
        self.stop_callback_thread()

    def set_logging_callback(self, callback: Callable, rate: float = 10.0):
        """
        Sets a logging callback function that will be called at a fixed rate in a separate thread.
        The function should take no arguments.

        callback: the function to call
        rate: the rate in Hz to call the function
        """
        if not callable(callback):
            print("Logging callback is not callable. Skipping.")
            return
        self.logging_callback = callback
        self.logging_rate = rate
        if not self._log_thread.is_alive():
            self._log_thread.start()

    def _log_loop(self):
        """
        Internal method for logging at a fixed rate.
        """
        while not self.end:
            try:
                start_time = time.time()
                self.logging_callback()
                time.sleep(
                    max(0, (1.0 / self.logging_rate) - (time.time() - start_time))
                )
            except Exception:
                time.sleep(0.1)
                continue

    @thread_safe
    def update_timestamp(self, timestamp: float):
        """
        Thread-safe for updating the timestamp and calculating the receive rate (hz).
        Do not override this method.
        """
        if self.timestamp == 0.0:
            self.timestamp = timestamp
            return
        dt = timestamp - self.timestamp
        self._pastdt.append(dt)
        if len(self._pastdt) > 10:
            self._pastdt.pop(0)
        self.hz = len(self._pastdt) / sum(self._pastdt)
        self.timestamp = timestamp

    @thread_safe
    def get_hz(self) -> float:
        """
        Thread-safe for getting the receive rate (hz).
        Do not override this method.
        """
        return int(self.hz)

    @thread_safe
    def _start_callback_thread(self):
        """
        Starts the internal thread for processing the decode and callback function. <br>
        Do not override this method.
        """
        if self._decodethread is not None and self._decodethread.is_alive():
            return  # Thread is already running
        self.end = False
        self._decodethread = threading.Thread(target=self._process, daemon=True)
        self._decodethread.start()

    @thread_safe
    def stop_callback_thread(self):
        """
        Stops the internal thread for processing the decode and callback function. <br>
        Do not override this method.
        """
        self.end = True
        if self._log_thread.is_alive():
            self._log_thread.join()
        if self._decodethread is not None:
            self._decodethread.join()
            self._decodethread = None

    @thread_safe
    def process_message(self, msg: Any):
        """
        Adds a message to the internal queue for processing. <br>
        Do not override this method.
        """
        if self._msg_queue.full():
            try:
                self._msg_queue.get_nowait()
            except queue.Empty:
                pass
        self._msg_queue.put(msg)

    def _process(self):
        """
        Internal method for processing the decode and callback function. <br>
        Do not override this method.
        """
        while not self.end:
            try:
                msg = self._msg_queue.get(timeout=0.1)
                self._decode(msg)
            except queue.Empty:
                time.sleep(0.02)
                continue
            time.sleep(0.02)  # Assumes no message > 50 Hz

    @thread_safe
    def _encode(self, system_id, component_id) -> Any:
        """
        Thread-safe wrapper for encode. Do not override this method.
        """
        return self.encode(system_id, component_id)

    def encode(self, system_id, component_id) -> Any:
        """
        Returns this MAVMessage as a pymavlink friendly message.
        """
        pass

    @thread_safe
    def _decode(self, msg):
        """
        Thread-safe wrapper for decode. Do not override this method.
        """
        self.decode(msg)
        self._decoded = True
        self.callback_func(self)

    def decode(self, msg):
        """
        Transforms this MAVMessage based off of a pymavlink message that was received.
        """
        pass

    @thread_safe
    def __repr__(self) -> str:
        return f"({self.name}) timestamp: {self.timestamp} ms"

    def __str__(self) -> str:
        return self.__repr__()

    def wait_until_finished(self):
        """
        Blocks until the internal wait thread is finished executing.
        """
        if type(self._thread) is not threading.Thread:
            return

        while self._thread.is_alive() or not self._decoded:
            time.sleep(0.1)
        self._thread = None

    def is_finished(self) -> bool:
        """
        Returns whether the internal wait thread is finished executing.
        """
        return self._thread is None
