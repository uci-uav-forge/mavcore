import time
import numpy as np
import mavcore
import mavcore.messages as messages
import mavcore.protocols as protocols


device = mavcore.MAVDevice("udp:127.0.0.1:14550")

boot_time_ms = int(time.time() * 1000)

local_pos = messages.LocalPositionCov()
device.add_listener(local_pos)

request_local_pos = protocols.RequestMessageProtocol(
    messages.IntervalMessageID.LOCAL_POSITION_NED_COV, rate_hz=5.0
)
device.run_protocol(request_local_pos)

print("starting...")
while True:
    print(local_pos)