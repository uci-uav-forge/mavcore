import time

import mavcore
import mavcore.messages as messages
import mavcore.protocols as protocols


device = mavcore.MAVDevice("udp:127.0.0.1:14550")

boot_time_ms = int(time.time() * 1000)

local_pos = messages.LocalPositionNED()

request_local_pos = protocols.RequestMessageProtocol(messages.IntervalMessageID.LOCAL_POSITION_NED, rate_hz=50)
device.run_protocol(request_local_pos)

request_arm = protocols.ArmProtocol()
device.run_protocol(request_arm)

request_guided = protocols.SetModeProtocol(messages.FlightMode.GUIDED)
device.run_protocol(request_guided)

takeoff = protocols.TakeoffProtocol(100.0)
device.run_protocol(takeoff)

while local_pos.get_pos_ned()[2] > -95.0:
    print(f"Altitude: {local_pos.get_pos_ned()[2]} m", flush=True)
    time.sleep(1)

dive = protocols.AttitudeSetpointProtocol(local_pos, boot_time_ms)
device.run_protocol(dive)

request_brake = protocols.SetModeProtocol(messages.FlightMode.BRAKE)
device.run_protocol(request_brake)
