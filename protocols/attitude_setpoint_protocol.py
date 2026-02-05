from mavcore.mav_protocol import MAVProtocol
from mavcore.messages.local_position_msg import LocalPosition as LocalPositionNED
from mavcore.messages.attitude_target_msg import SetpointAttitude
import time
import numpy as np


class AttitudeSetpointProtocol(MAVProtocol):

    def __init__(
        self,
        current_pos: LocalPositionNED,
        boot_time_ms: int,
        target_system: int = 1,
        target_component: int = 0,
    ):
        super().__init__()
        self.current_pos = current_pos
        self.boot_time_ms = boot_time_ms
        self.target_system = target_system
        self.target_component = target_component

        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.thrust = 0.005

        self.setpoint_msg = SetpointAttitude(
            self.target_system, self.target_component, self.boot_time_ms, self.q, self.thrust
        )

    def run(self, sender, receiver):
        while self.current_pos.get_pos_ned()[2] < -25.0:
            sender.send_msg(self.setpoint_msg)
            time.sleep(0.02)
            print(f"velocity: {np.linalg.norm(self.current_pos.get_vel_ned())} m/s and {self.current_pos.get_pos_ned()}", flush=True)

        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.thrust = 0.75
        self.setpoint_msg.load_quat_thrust(self.q, self.thrust)
        while True:      
            sender.send_msg(self.setpoint_msg)
            time.sleep(0.02)
        
