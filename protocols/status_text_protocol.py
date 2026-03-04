from ..mav_protocol import MAVProtocol
from ..messages.status_text_msg import MAVSeverity, StatusText


class StatusTextProtocol(MAVProtocol):
    """
    Sends status text with MAVSeverity specified for diferent msg destinations. Note msg must be < 30 characters.
    """

    def __init__(self, text: str, severity: MAVSeverity):
        super().__init__()
        self.text = text
        self.severity = severity

        self.status_text_msg = StatusText(self.text, self.severity)

    def run(self, sender, receiver):
        sender.send_msg(self.status_text_msg)
