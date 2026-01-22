from mavcore.mav_protocol import MAVProtocol
from mavcore.messages.rtk_msg import RTKData
from mavcore.messages.command_ack_msg import CommandAck


class SendRTKProtocol(MAVProtocol):
    def __init__(self, data, target_system: int = 1, target_component: int = 0):
        super().__init__()
        self.target_system = target_system
        self.target_component = target_component

        self.rtk_msg = RTKData(data)
    
    def run(self, sender, receiver):
        sender.send_msg(self.rtk_msg)
