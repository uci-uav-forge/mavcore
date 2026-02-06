import numpy as np
from mavcore.mav_message import MAVMessage, thread_safe


class RawIMU(MAVMessage):

    def __init__(self):
        super().__init__("RAW_IMU")
        self.xac = 0.0
        self.yac = 0.0
        self.zac = 0.0

    def decode(self, msg):
        self.xac = msg.xacc
        self.yac = msg.yacc
        self.zac = msg.zacc
