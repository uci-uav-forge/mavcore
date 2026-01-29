from mavcore.mav_protocol import MAVProtocol
from mavcore.messages.rtk_msg import RTKData
from mavcore.messages.command_ack_msg import CommandAck

GPS_RTCM_MAX_LENGTH = 180

class SendRTKProtocol(MAVProtocol):
    def __init__(self, target_system: int = 1, target_component: int = 0):
        super().__init__()
        self.target_system = target_system
        self.target_component = target_component
        self.sequence_num = 0
    
    def update(self, data):
        if len(data) < 180:
            self.rtk_msg = RTKData(self.sequence_num, data)
            self.sequence_num = (self.sequence_num + 1) % 32
    
    def run(self, sender, receiver):
        sender.send_msg(self.rtk_msg)

"""
class SendRTKProtocol(MAVProtocol):
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
            rtk_msg = RTKData(msg_length > GPS_RTCM_MAX_LENGTH, fragment_id, self.sequence_num, self.payload[:msg_length])
            fragment_id += 1
            self.sequence_num = (self.sequence_num + 1) % 32
            self.payload = self.payload[msg_length:]
"""
