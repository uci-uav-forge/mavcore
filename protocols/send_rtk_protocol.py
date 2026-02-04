from mavcore.mav_protocol import MAVProtocol
from mavcore.messages.rtk_msg import RTKData
from mavcore.messages.command_ack_msg import CommandAck

GPS_RTCM_MAX_LENGTH = 180


class SendRTKProtocol(MAVProtocol):
    """
    Sends RTK message and splits it into up to 4 fragments, since payloads have a
    maximum size of 180 bytes.
    """

    def __init__(self, target_system: int = 1, target_component: int = 0):
        super().__init__()
        self.target_system = target_system
        self.target_component = target_component
        self.sequence_num = 0
        self.payload = []

    def update(self, data):
        self.payload = list(data)

    def run(self, sender, receiver):
        fragment_id = 0
        while self.payload and fragment_id < 4:
            msg_length = min(len(self.payload), GPS_RTCM_MAX_LENGTH)
            rtk_msg = RTKData(
                msg_length > GPS_RTCM_MAX_LENGTH,
                fragment_id,
                self.sequence_num,
                self.payload[:msg_length],
            )
            sender.send_msg(rtk_msg)
            fragment_id += 1
            self.payload = self.payload[msg_length:]
        self.sequence_num = (self.sequence_num + 1) % 32
