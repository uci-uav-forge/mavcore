import time
import threading
from queue import Queue
from typing import Any
from mavcore.mav_message import MAVMessage


class Receiver:
    def __init__(self, history_size: int = 100):
        self.history_dict: dict[str, list] = {}
        self.queue = Queue()
        self.listeners: dict[str, list[MAVMessage]] = {}
        self.waiting: dict[str, list[MAVMessage]] = {}
        self.history_size = history_size
        self.receiving = False

    def __add_to_dict(
        self, target_dict: dict[str, list[MAVMessage]], msg: MAVMessage
    ) -> MAVMessage:
        if len(msg.submessages) > 0:
            for submsg in msg.submessages:
                self.__add_to_dict(target_dict, submsg)
        else:
            if msg.name in target_dict:
                target_dict[msg.name].append(msg)
            else:
                target_dict[msg.name] = [msg]
            msg._start_callback_thread()
        return msg

    def add_listener(self, msg: MAVMessage) -> MAVMessage:
        return self.__add_to_dict(self.listeners, msg)

    def _add_waiter(self, msg: MAVMessage) -> MAVMessage:
        return self.__add_to_dict(self.waiting, msg)

    def remove_listener(self, msg: MAVMessage | str) -> bool:
        if isinstance(msg, str):
            res = self.listeners.pop(msg, None)  # removes all with that message name
            return res is not None
        else:
            if len(msg.submessages) > 0:
                removed_all = True
                for submsg in msg.submessages:
                    removed = self.remove_listener(submsg)
                    removed_all = removed_all and removed
                return removed_all
            elif msg.name in self.listeners and msg in self.listeners[msg.name]:
                self.listeners[msg.name].remove(msg)
                return True
        return False

    def start_receiving(self):
        self.receiving = True
        self._thread = threading.Thread(target=self.process, daemon=True)
        self._thread.start()

    def stop_receiving(self):
        self.receiving = False

    def update_queue(self, timestamp_ms: float, msg: Any):
        self.queue.put((timestamp_ms, msg))

    def process(self):
        while self.receiving:
            timestamp, msg = self.queue.get()
            msg_name = msg.get_type()

            # Check if waiting for this message
            if msg_name in self.waiting:
                for wait_msg in self.waiting[msg_name]:
                    wait_msg.update_timestamp(timestamp)
                    wait_msg.process_message(msg)
                self.waiting.pop(msg_name)

            # Update listeners
            if msg_name in self.listeners:
                for listener in self.listeners[msg_name]:
                    if listener.timestamp < timestamp:
                        listener.update_timestamp(timestamp)
                        listener.process_message(msg)

            # Manage message history
            if msg_name in self.history_dict:
                self.history_dict[msg_name].insert(0, (timestamp, msg))

                # Manage history length
                if len(self.history_dict[msg_name]) > self.history_size:
                    self.history_dict[msg_name].pop()
            else:
                # Brand new message type
                self.history_dict[msg_name] = [(timestamp, msg)]

    def wait_for_msg(
        self, msg: MAVMessage, timeout_seconds: float = -1.0, blocking=True
    ) -> MAVMessage:
        """
        Will wait for msg to occur. Once it does, will return the updated object. <br>
        If blocking will return a FutureMsg.
        """
        msg._decoded = False
        if not blocking:
            msg._thread = threading.Thread(
                target=lambda: self.wait_for_msg(msg), daemon=True
            )
            msg._thread.start()
            return msg

        timeout_timer = time.time()
        msg.timestamp = 0.0
        self._add_waiter(msg)
        while (msg.timestamp == 0.0 or not msg._decoded) and (
            timeout_seconds < 0 or time.time() - timeout_timer < timeout_seconds
        ):
            time.sleep(0.01)
        return msg
