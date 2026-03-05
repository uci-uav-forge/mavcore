import mavcore.mav_device as mav_device
import mavcore.messages as messages
import mavcore.protocols as protocols
import time

device = mav_device.MAVDevice(device_address = "udp:127.0.0.1:14551", source_system = 255, source_component = 10)

# add a listener for the batter status message

bs_msg = messages.BatteryStatus()
device.add_listener(bs_msg)

while True:
    print(f"temp: {bs_msg.temp}")
    time.sleep(1.0)