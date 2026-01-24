from mavcore.mav_protocol import MAVProtocol
from mavcore.messages import AccelCal, BaroCal, CompassCal, LevelCal
from mavcore.messages.command_ack_msg import CommandAck


class CalibrationProtocol(MAVProtocol):
    """
    Calibrates the Level.
    0 = Accelerometer Calibration
    1 = Barometer Calibration
    2 = Compass Calibration
    3 = Level Calibration
    """

    def __init__(
        self, type_cal: str, target_system: int = 1, target_component: int = 0
    ):
        super().__init__()
        self.target_system = target_system
        self.target_component = target_component

        if type_cal == "accelerometer":
            self.arm_msg = AccelCal(self.target_system, self.target_component)
        elif type_cal == "barometer":
            self.arm_msg = BaroCal(self.target_system, self.target_component)
        elif type_cal == "compass":
            self.arm_msg = CompassCal(self.target_system, self.target_component)
        elif type_cal == "level":
            self.arm_msg = LevelCal(self.target_system, self.target_component)

        self.ack_msg = CommandAck()

    def run(self, sender, receiver):
        future_ack = receiver.wait_for_msg(self.ack_msg, blocking=False)
        sender.send_msg(self.arm_msg)
        future_ack.wait_until_finished()
        print(self.ack_msg)
