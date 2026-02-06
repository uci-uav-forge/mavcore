import time
import numpy as np
import mavcore
import mavcore.messages as messages
import mavcore.protocols as protocols


device = mavcore.MAVDevice("udp:127.0.0.1:14550")

boot_time_ms = int(time.time() * 1000)

local_pos = messages.LocalPositionNED()
device.add_listener(local_pos)

request_local_pos = protocols.RequestMessageProtocol(messages.IntervalMessageID.LOCAL_POSITION_NED, rate_hz=50.0)
device.run_protocol(request_local_pos)

imu = messages.RawIMU()
device.add_listener(imu)

request_imu = protocols.RequestMessageProtocol(messages.IntervalMessageID.RAW_IMU, rate_hz=50.0)
device.run_protocol(request_imu)

time.sleep(1)

gs = np.linalg.norm([imu.xac, imu.yac, imu.zac])
highest = gs
print(f"G: {highest}", flush=True)

time.sleep(1)

request_arm = protocols.ArmProtocol()
device.run_protocol(request_arm)

request_guided = protocols.SetModeProtocol(messages.FlightMode.GUIDED)
device.run_protocol(request_guided)

takeoff = protocols.TakeoffProtocol(300.0)
device.run_protocol(takeoff)

while local_pos.get_pos_ned()[2] > -295.0:
    print(f"Altitude: {local_pos.get_pos_ned()[2]} m", flush=True)
    time.sleep(1)

dive = protocols.AttitudeSetpointProtocol(local_pos, imu, boot_time_ms)
device.run_protocol(dive)

request_brake = protocols.SetModeProtocol(messages.FlightMode.BRAKE)
device.run_protocol(request_brake)

highest = 0.0
i = 1000

while i > 0:
    gs = np.linalg.norm([imu.xac, imu.yac, imu.zac])
    if gs > highest:
        highest = gs
    i -= 1
    time.sleep(0.01)
print(f"highest G: {highest}", flush=True)
