import pymavlink.mavutil as utility
import threading
import time

from mavcore.mav_protocol import MAVProtocol
from mavcore.mav_receiver import Receiver
from mavcore.mav_sender import Sender
from mavcore.mav_message import MAVMessage


class MAVDevice:
    """
    Primary class for MAVCore. Manages drone connection, sending, and receiving mavlink messages.
    """

    def __init__(
        self,
        device_address: str,
        baud_rate: int = 115200,
        source_system: int = 255,
        source_component: int = 0,
        attempt_reconnect: bool = True,
    ):
        self.attempt_reconnect = attempt_reconnect
        self.receiver = Receiver()
        self.connection: utility.mavudp | utility.mavserial = self._connect(
            device_address, baud_rate, source_system, source_component
        )
        self.sender = Sender(
            self.connection.target_system,
            self.connection.target_component,
            self.connection,
        )

        self.receiver.start_receiving()

        self.reading = True
        self.thread = threading.Thread(target=self._main_loop, daemon=True)
        self.thread.start()
        time.sleep(1)

    def _connect(
        self,
        device_address: str,
        baud_rate: int,
        source_system: int,
        source_component: int,
    ) -> utility.mavudp | utility.mavserial:
        """
        device_address follows by pymavlink standards: 'udp:127.0.0.1:14550' or '/dev/ttyACM0'
        """
        connection = utility.mavlink_connection(
            device=device_address,
            baud=baud_rate,
            source_system=source_system,
            source_component=source_component,
            autoreconnect=self.attempt_reconnect,
        )
        if (
            type(connection) is not utility.mavudp
            and type(connection) is not utility.mavserial
        ):
            raise RuntimeError("Connection is not udp or serial.")
        return connection

    def stop_reading(self):
        """
        Will stop the thread that reads messages.
        """
        self.reading = False

    def add_listener(self, listener: MAVMessage) -> MAVMessage:
        """
        Pass in a MAVMessage to listen for. Will save occurences of this message and update this message object accordingly.
        """
        self.receiver.add_listener(listener)
        return listener

    def run_protocol(self, protocol: MAVProtocol) -> MAVProtocol:
        """
        Runs a MAVProtocol object that sends and receives messages to complete the protcol.
        """
        self.sender.acquire()
        protocol.run(self.sender, self.receiver)
        self.sender.release()
        return protocol

    def _main_loop(self):
        while self.reading:
            msg = self.connection.recv_match(blocking=True, timeout=1)
            if msg:
                self.receiver.update_queue(time.time(), msg)
