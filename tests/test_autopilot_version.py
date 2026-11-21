import pymavlink.dialects.v20.all as dialect
import pytest

from mavcore.messages import AutopilotVersion


@pytest.mark.parametrize(
    "uid2, uid",
    [(b"\x12\x00\x34" + bytes(15), 0x1234), (bytes(18), 0x1234), (bytes(18), 0)],
)
def test_autopilot_uid2_preserves_hardware_bytes_or_falls_back(uid2, uid):
    mav = dialect.MAVLink(None)
    message = dialect.MAVLink_autopilot_version_message(
        capabilities=0,
        flight_sw_version=1,
        middleware_sw_version=0,
        os_sw_version=0,
        board_version=0,
        flight_custom_version=bytes(8),
        middleware_custom_version=bytes(8),
        os_custom_version=bytes(8),
        vendor_id=0,
        product_id=0,
        uid=uid,
        uid2=uid2,
    )
    version = AutopilotVersion()
    version.decode(mav.decode(bytearray(message.pack(mav))))

    expected_uid2 = uid2.hex() if any(uid2) else ""
    assert version.get_uid2() == expected_uid2
    assert version.get_uid() == (expected_uid2 or (f"{uid:016x}" if uid else ""))
