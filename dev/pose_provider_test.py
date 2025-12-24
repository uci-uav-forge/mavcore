import mavcore.mav_device as mav_device
import mavcore.messages as messages
import mavcore.protocols as protocols
import time
import bisect
import numpy as np

device = mav_device.MAVDevice("udp:127.0.0.1:14550")

full_pose = messages.FullPose()
device.add_listener(full_pose)

request_pos = protocols.RequestMessageProtocol(
    messages.IntervalMessageID.LOCAL_POSITION_NED, rate_hz=30.0
)
request_pos_global = protocols.RequestMessageProtocol(
    messages.IntervalMessageID.GLOBAL_POSITION_INT, rate_hz=30.0
)
request_att = protocols.RequestMessageProtocol(
    messages.IntervalMessageID.ATTITUDE_QUATERNION, rate_hz=30.0
)

while True:
    device.run_protocol(request_pos)
    device.run_protocol(request_pos_global)
    device.run_protocol(request_att)
    time.sleep(5.0)
    print(full_pose.local_position.hz)
    if full_pose.local_position.hz > 25.0:
        break

ts = []
save = []
timeout = 3.0
start = time.time()
print("Collecting ground truth poses...")
while time.time() - start < timeout:
    pose = full_pose.get_local_position()
    save.append(pose)
    ts.append(pose.timestamp)
    time.sleep(0.05)

# ts2 = []
# save2 = []
# timeout = 3.0
# start = time.time()
# for t in ts:
#     pose = full_pose.get_local_position(t)
#     save2.append(pose)
#     ts2.append(pose.timestamp)
#     time.sleep(0.05)

ts2 = []
save2 = []
timeout = 3.0
start = time.time()
print("Collecting delayed poses...")
past = 3.0
while time.time() - start < timeout:
    pose = full_pose.get_local_position(time.time() - past)
    save2.append(pose)
    ts2.append(pose.timestamp)
    time.sleep(0.05)

print("Comparing interpolated poses to ground truth...")

for t, p in zip(ts2, save2):
    i = bisect.bisect_left(ts, t)
    if i == 0:
        p_ref = save[0]
    elif i == len(ts):
        p_ref = save[-1]
    else:
        t0, t1 = ts[i - 1], ts[i]
        p0, p1 = save[i - 1], save[i]
        prop = (t - t0) / (t1 - t0)
        p_ref = p0.interpolate(p1, prop, timestamp=t)

    pos_err = np.linalg.norm(p.position - p_ref.position)
    rot_err = np.linalg.norm(p.as_rotvec() - p_ref.as_rotvec())
    # assert pos_err < 0.05, f"Position error too high at t={t}: {pos_err} m"
    # assert rot_err < 0.1, f"Rotation error too high at t={t}: {rot_err} rad"
    print(
        f"t={t:.2f}: Position error: {pos_err:.3f} m, Rotation error: {rot_err:.3f} rad"
    )

print("Current pose:")
print(full_pose)
print("Pose interpolation test completed.")
