import time
import threading
import pymavlink.mavutil as utility

from mavcore.mav_message import MAVMessage


class Sender:
    def __init__(
        self,
        sys_id: int,
        component_id: int,
        connection: utility.mavudp | utility.mavserial,
    ):
        self.sys_id = sys_id
        self.component_id = component_id
        self.connection = connection
        self._lock = threading.Lock()
        self._owner = None
        self.repeating_msgs: list[tuple[MAVMessage, int | None, int | None]] = []
        self._thread = threading.Thread(target=self.repeat_loop, daemon=True)
        self._thread.start()

    def acquire(self):
        """
        Acquires send lock.
        """
        self._lock.acquire()
        self._owner = threading.get_ident()

    def release(self):
        """
        Releases send lock.
        """
        if not self._is_owned():
            raise RuntimeError("Current thread does not own the lock")
        self._owner = None
        self._lock.release()

    def _is_owned(self):
        """Checks if the current thread owns the lock"""
        return threading.get_ident() == self._owner

    def send_msg(self, msg: MAVMessage, system_id=None, component_id=None):
        """
        Sends a mavlink message. Must have aquired the lock (automatic if using run protocol).
        Optional specified system and component ids otherwise connection defaults used.
        """
        if not self._is_owned():
            raise RuntimeError("Current thread does not own the lock")

        self._check_disconnect()

        mav_msg = msg._encode(
            self.sys_id if not system_id else system_id,
            self.component_id if not component_id else component_id,
        )
        self.connection.mav.send(mav_msg)
        msg.timestamp = time.time() * 1000
        if msg.repeat_period != 0.0:
            self.repeating_msgs.append((msg, system_id, component_id))

    def repeat_loop(self):
        """
        Loop in charge of running repeated messages.
        """
        while True:
            for payload in self.repeating_msgs:
                msg, sys_id, comp_id = payload
                if time.time() - msg.timestamp > msg.repeat_period:
                    self.acquire()
                    self.send_msg(msg, sys_id, comp_id)
                    self.release()
            time.sleep(0.001)

    def _check_disconnect(self):
        while self.connection.portdead:
            time.sleep(0.25)
