from .mav_sender import Sender
from .mav_receiver import Receiver


class MAVProtocol:
    """
    Meant to be an interface/template for a mavlink message protocol that sends and or receives messages.
    """

    def __init__(self):
        pass

    def run(self, sender: Sender, receiver: Receiver):
        """
        Uses the sender and receiver to compelete a task according to a protcol. Automatically run when MAVDevice.run_protocol is used.
        """
        pass
