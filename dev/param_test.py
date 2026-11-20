import sys

import mavcore.mav_device as mav_device
import mavcore.protocols as protocols

########## NOTE THIS WILL ONLY WORK FOR COPTER ##########
# Usage: python dev/param_test.py [address], default udp:127.0.0.1:14550

address = sys.argv[1] if len(sys.argv) > 1 else "udp:127.0.0.1:14550"
device = mav_device.MAVDevice(address)

print("--Reading WPNAV_SPEED")
read = device.run_protocol(protocols.ParamReadProtocol("WPNAV_SPEED"))
print(f"success: {read.success}, param: {read.param}, error: {read.error!r}")
if not read.success:
    sys.exit(1)
original = read.param.value

print("--Downloading all parameters")
param_list = device.run_protocol(protocols.ParamListProtocol())
print(
    f"success: {param_list.success}, received {len(param_list.params)}/{param_list.param_count}, "
    f"missing: {param_list.missing}"
)

new_value = original + 50.0
print(f"--Setting WPNAV_SPEED to {new_value}")
try:
    write = device.run_protocol(protocols.ParamWriteProtocol("WPNAV_SPEED", new_value))
    print(f"success: {write.success}, param: {write.param}, error: {write.error!r}")

    check = device.run_protocol(protocols.ParamReadProtocol("WPNAV_SPEED"))
    print(f"read back: {check.param.value if check.success else repr(check.error)}")
finally:
    print(f"--Restoring WPNAV_SPEED to {original}")
    restore = device.run_protocol(protocols.ParamWriteProtocol("WPNAV_SPEED", original))
    print(f"success: {restore.success}")

print("--Setting a parameter that does not exist (times out)")
missing = device.run_protocol(
    protocols.ParamWriteProtocol("NOT_A_PARAM", 1, timeout_s=0.5, retries=1)
)
print(f"success: {missing.success}, error: {missing.error!r}")
