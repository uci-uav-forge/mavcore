from mavcore.mav_protocol import MAVProtocol
from mavcore.messages.local_position_msg import LocalPosition as LocalPositionNED
from mavcore.messages.raw_imu_msg import RawIMU
from mavcore.messages.attitude_target_msg import SetpointAttitude
import time
import numpy as np


class AttitudeSetpointProtocol(MAVProtocol):

    def __init__(
        self,
        current_pos: LocalPositionNED,
        imu: RawIMU,
        boot_time_ms: int,
        target_system: int = 1,
        target_component: int = 0,
    ):
        super().__init__()
        self.current_pos = current_pos
        self.imu = imu
        self.boot_time_ms = boot_time_ms
        self.target_system = target_system
        self.target_component = target_component

        self.q = np.array([0.0, 1.0, 0.0, 0.0])
        self.thrust = 0.1

        self.setpoint_msg = SetpointAttitude(
            self.target_system, self.target_component, self.boot_time_ms, self.q, self.thrust
        )

    def run(self, sender, receiver):
        highest = 0.0
        while self.current_pos.get_pos_ned()[2] < -200.0:
            sender.send_msg(self.setpoint_msg)
            time.sleep(0.02)
            gs = np.linalg.norm([self.imu.xac, self.imu.yac, self.imu.zac])
            if gs > highest:
                highest = gs
            #print(f"highest G: {highest}", flush=True)

        
