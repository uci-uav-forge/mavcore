import mavcore.mav_device as mav_device
import mavcore.messages as messages
import mavcore.protocols as protocols
import time

device = mav_device.MAVDevice("udp:127.0.0.1:14550")

request_pos = protocols.RequestMessageProtocol(
    messages.IntervalMessageID.LOCAL_POSITION_NED, rate_hz=30.0
)
request_att = protocols.RequestMessageProtocol(
    messages.IntervalMessageID.ATTITUDE_QUATERNION, rate_hz=30.0
)

fp = messages.FullPose()
device.add_listener(fp)

while True:
    device.run_protocol(request_att)
    device.run_protocol(request_pos)
    time.sleep(5.0)
    print(fp.local_position.get_hz(), fp.attitude.get_hz())
    if fp.local_position.get_hz() > 25.0:
        break
# while True:
time.sleep(2)
set_mode_protocol = protocols.SetModeProtocol(
    messages.FlightMode.GUIDED
)  # 15/AUTOTUNE is GUIDED in plane
print("--Setting GUIDED mode")
device.run_protocol(set_mode_protocol)
print(f"Set GUIDED mode ack: {set_mode_protocol.ack_msg}")
time.sleep(2)
arm_protocol = protocols.ArmProtocol()
device.run_protocol(arm_protocol)
print(f"Requesting arm protocol Ack: {arm_protocol.ack_msg}")
time.sleep(2)
takeoff_protocol = protocols.TakeoffProtocol(altitude=(20.0))
device.run_protocol(takeoff_protocol)
while abs(fp.get_local_position().position[2] - 20.0) > 1.0:
    print(f"Current altitude: {fp.get_local_position().position[2]}")
    time.sleep(0.2)
print("Takeoff complete")
time.sleep(2)
set_mode_protocol = protocols.SetModeProtocol(messages.FlightMode.RTL)
print("--Setting RTL mode")
device.run_protocol(set_mode_protocol)
print(f"Set RTL mode ack: {set_mode_protocol.ack_msg}")
while fp.get_local_position().position[2] > 0.5:
    print(f"Current altitude: {fp.get_local_position().position[2]}")
    time.sleep(0.2)
print("Land complete")

print("Final Hz rates: [Full_Pose, Local_Position, Attitude]")
print(fp.get_hz(), fp.local_position.get_hz(), fp.attitude.get_hz())
