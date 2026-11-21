"""Exercise the moving-drop protocol with simulated position and velocity."""

from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from mavcore.mavtypes import Waypoint
from mavcore.messages import SetpointVelocity
from mavcore.protocols import MovingDropProtocol


class SimulatedMotion:
    def __init__(self):
        self.position = np.array([0.0, 0.0, -15.0])
        self.velocity = np.zeros(3)
        self.elapsed = 0.0
        self.move = True

    def get_pos_ned(self):
        return self.position.copy()

    def get_vel_ned(self):
        return self.velocity.copy()

    def send_msg(self, message):
        assert isinstance(message, SetpointVelocity)
        self.velocity = np.array([message.vx, message.vy, message.vz])

    def sleep(self, duration):
        if self.move:
            self.position += self.velocity * duration
        self.elapsed += duration


def make_protocol(monkeypatch, motion, *, leg_timeout_s=120.0):
    clock = SimpleNamespace(monotonic=lambda: motion.elapsed, sleep=motion.sleep)
    monkeypatch.setattr("mavcore.protocols.moving_drop_protocol.time", clock)
    drop_callback = Mock(return_value=True)
    points = [
        Waypoint(0.0, east, -15.0, radius=5.0, speed=5.0)
        for east in (0.0, 40.0, 60.0, 80.0)
    ]
    protocol = MovingDropProtocol(
        motion,
        points,
        2,
        drop_callback,
        0,
        log_func=lambda _: None,
        leg_timeout_s=leg_timeout_s,
    )
    return protocol, drop_callback


def test_follows_waypoints_and_releases_once(monkeypatch):
    motion = SimulatedMotion()
    protocol, drop_callback = make_protocol(monkeypatch, motion)

    protocol.run(motion, None)

    drop_callback.assert_called_once_with()
    assert motion.position[1] >= 75.0
    np.testing.assert_array_equal(motion.velocity, np.zeros(3))


def test_unreachable_waypoint_times_out_before_release(monkeypatch):
    motion = SimulatedMotion()
    motion.move = False
    protocol, drop_callback = make_protocol(monkeypatch, motion, leg_timeout_s=1.0)

    with pytest.raises(RuntimeError, match=r"waypoint 2 not reached within 1\.0 s"):
        protocol.run(motion, None)

    assert motion.elapsed == pytest.approx(1.0, abs=protocol.UPDATE_PERIOD_S)
    drop_callback.assert_not_called()
