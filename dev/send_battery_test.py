import mavcore.mav_device as mav_device
import mavcore.messages as messages
import mavcore.protocols as protocols
import time

device = mav_device.MAVDevice(device_address = "udp:127.0.0.1:14550", source_system = 1, source_component = 10)


#create device and send a battery status message protocol

battery_msg = messages.BatteryStatus()
battery_msg.temp = -12345
send_battery_status = protocols.UpdateBatteryProtocol(battery_msg)


while True:
    device.run_protocol(send_battery_status)
    time.sleep(1.0)
