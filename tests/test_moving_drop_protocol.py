"""Moving-drop preparation with rate-limited motion and real protocol commands."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from mavcore.mavtypes import Waypoint
from mavcore.messages import SetpointLocal, SetpointVelocity
from mavcore.protocols import MovingDropProtocol


class SimulatedMotion:
    def __init__(self, altitude=60.0):
        self.position = np.array([0.0, 0.0, -altitude])
        self.velocity = np.zeros(3)
        self.yaw = math.pi
        self.elapsed = 0.0
        self.command = None
        self.commands = []
        self.fresh = True
        self.move = True
        self.turn = True
        self.keep_moving = False

    @property
    def timestamp(self):
        return 1000.0 + self.elapsed if self.fresh else 0.0

    def get_pos_ned(self):
        return self.position.copy()

    def get_vel_ned(self):
        return np.ones(3) if self.keep_moving else self.velocity.copy()

    def get_quat(self):
        return np.array([math.cos(self.yaw / 2), 0.0, 0.0, math.sin(self.yaw / 2)])

    def send_msg(self, message):
        self.commands.append((self.elapsed, self.velocity.copy(), self.yaw, message.encode(255, 0)))
        if isinstance(message, SetpointLocal):
            self.command = ("hold", np.array([message.x, message.y, message.z]), message.yaw)
        else:
            assert isinstance(message, SetpointVelocity)
            self.command = ("velocity", np.array([message.vx, message.vy, message.vz]), None)

    def sleep(self, duration):
        mode, target, yaw = self.command
        self.velocity = np.zeros(3)
        if self.move:
            if mode == "hold":
                # 1 m/s descent: the Main Field West 60 -> 15 m change takes 45 s.
                change = np.clip(target - self.position, -duration, duration)
                self.velocity = change / duration
            else:
                self.velocity = target.copy()
            self.position += self.velocity * duration
        if self.turn and yaw is not None:
            error = MovingDropProtocol._angle_error(yaw, self.yaw)
            self.yaw += np.clip(error, -0.5 * duration, 0.5 * duration)
        self.elapsed += duration

    def clock(self):
        return SimpleNamespace(time=lambda: 1000.0 + self.elapsed,
                               monotonic=lambda: self.elapsed, sleep=self.sleep)


def make_protocol(monkeypatch, motion):
    monkeypatch.setattr("mavcore.protocols.moving_drop_protocol.time", motion.clock())
    releases = []
    points = [Waypoint(0.0, east, -15.0, radius=5.0, speed=5.0)
              for east in (0.0, 40.0, 60.0, 80.0)]
    protocol = MovingDropProtocol(
        motion, points, 2, lambda: releases.append(motion.position.copy()), 0,
        current_attitude=motion, log_func=lambda _: None,
    )
    return protocol, releases


def test_long_descent_stops_and_aligns_before_run_in_and_releases_once(monkeypatch):
    motion = SimulatedMotion()
    protocol, releases = make_protocol(monkeypatch, motion)
    protocol.run(motion, None)

    # Velocity masks ignore position; the first motion command follows the stop
    # and heading checks, even though descending alone exceeded the old timeout.
    first_velocity = next(command for command in motion.commands
                          if command[3].type_mask & 7 == 7
                          and np.linalg.norm([command[3].vx, command[3].vy, command[3].vz]) > 0)
    elapsed, velocity, yaw, _ = first_velocity
    assert elapsed > 45.0 + protocol.SETTLE_TIME_S
    assert np.linalg.norm(velocity) <= protocol.STOP_SPEED_M_S
    assert abs(protocol._angle_error(math.pi / 2, yaw)) <= protocol.ALIGN_TOLERANCE_RAD
    assert len(releases) == 1
    assert np.linalg.norm(releases[0] - np.array([0.0, 60.0, -15.0])) <= 5.0
    assert motion.position[1] >= 75.0


def test_unreachable_first_waypoint_has_bounded_timeout_and_diagnostics(monkeypatch):
    motion = SimulatedMotion()
    motion.move = False
    protocol, releases = make_protocol(monkeypatch, motion)
    with pytest.raises(RuntimeError, match=r"approach.*120 s.*altitude error=45.0 m"):
        protocol.run(motion, None)
    assert motion.elapsed == pytest.approx(120.0, abs=0.2)
    assert releases == []


def test_missing_feedback_cannot_start_run_in_or_extend_timeout(monkeypatch):
    motion = SimulatedMotion(altitude=15.0)
    motion.fresh = False
    protocol, releases = make_protocol(monkeypatch, motion)
    with pytest.raises(RuntimeError, match=r"approach.*30 s.*feedback unavailable"):
        protocol.run(motion, None)
    assert motion.elapsed == pytest.approx(30.0, abs=0.2)
    assert releases == []


def test_reaching_first_waypoint_without_stopping_does_not_start_run_in(monkeypatch):
    motion = SimulatedMotion(altitude=15.0)
    motion.keep_moving = True
    protocol, releases = make_protocol(monkeypatch, motion)
    with pytest.raises(RuntimeError, match=r"waypoint stop.*30 s.*speed=1.73 m/s"):
        protocol.run(motion, None)
    assert releases == []


def test_heading_must_settle_before_run_in(monkeypatch):
    motion = SimulatedMotion(altitude=15.0)
    motion.turn = False
    protocol, releases = make_protocol(monkeypatch, motion)
    with pytest.raises(RuntimeError, match=r"waypoint align.*30 s.*heading error=90.0 deg"):
        protocol.run(motion, None)
    assert releases == []
