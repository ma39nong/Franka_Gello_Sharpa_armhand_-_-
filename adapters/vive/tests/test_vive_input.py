import numpy as np

from vive_tracker_teleop.config import LocalTransformConfig, ViveTrackerConfig
from vive_tracker_teleop.input import ViveTrackerInput
from vive_tracker_teleop.openvr_source import TrackerReading


class Operator:
    def __init__(self):
        self.active = {"left": False, "right": False}
        self.messages = []

    def poll(self):
        return dict(self.active)

    def disable_all(self, reason):
        self.active = {"left": False, "right": False}
        self.messages.append(reason)

    def deny(self, side, reason):
        self.active[side] = False
        self.messages.append(f"{side}: {reason}")

    def show(self, message):
        self.messages.append(message)


class Client:
    def __init__(self, readings):
        self.readings = readings
        self.opened = 0
        self.closed = 0

    def open(self):
        self.opened += 1

    def close(self):
        self.closed += 1

    def read(self, serials):
        return dict(self.readings)


def reading(serial, position):
    transform = np.eye(4)
    transform[:3, 3] = position
    return TrackerReading(
        serial=serial,
        transform=transform,
        velocity=np.zeros(3),
        angular_velocity=np.zeros(3),
        tracking_result=200,
    )


def config():
    identity = LocalTransformConfig(np.zeros(3), np.eye(3))
    return ViveTrackerConfig(
        serials={"left": "LEFT", "right": "RIGHT"},
        world_to_control_rotation=np.array(
            [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
        ),
        tracker_to_control={"left": identity, "right": identity},
        ready_timeout=0.01,
        frozen_timeout=10.0,
        max_position_jump=0.2,
        max_rotation_jump=1.0,
        max_linear_speed=1000.0,
        max_angular_speed=1000.0,
    )


def both_readings():
    return {
        "left": reading("LEFT", [0.1, 0.2, 0.3]),
        "right": reading("RIGHT", [0.4, 0.5, 0.6]),
    }


def test_vive_poses_use_the_common_control_axes_and_serial_roles():
    operator = Operator()
    client = Client(both_readings())
    source = ViveTrackerInput(config(), operator, client=client)
    operator.active = {"left": True, "right": True}

    sample = source.sample()

    assert sample is not None
    np.testing.assert_allclose(sample.poses["left"].position, [-0.3, -0.1, 0.2])
    np.testing.assert_allclose(sample.poses["right"].position, [-0.6, -0.4, 0.5])
    assert sample.activations == {"left": True, "right": True}
    source.close()
    assert client.opened == 1
    assert client.closed == 1


def test_missing_disengaged_tracker_denies_only_that_side():
    operator = Operator()
    readings = both_readings()
    client = Client(readings)
    source = ViveTrackerInput(config(), operator, client=client)
    operator.active = {"left": True, "right": False}
    assert source.sample() is not None

    del readings["right"]
    operator.active["right"] = True
    sample = source.sample()

    assert sample is not None
    assert sample.activations == {"left": True, "right": False}
    assert operator.active == {"left": True, "right": False}


def test_tracker_loss_while_engaged_disengages_everything():
    operator = Operator()
    readings = both_readings()
    client = Client(readings)
    source = ViveTrackerInput(config(), operator, client=client)
    operator.active = {"left": True, "right": True}
    assert source.sample() is not None

    del readings["left"]

    assert source.sample() is None
    assert operator.active == {"left": False, "right": False}


def test_jump_while_engaged_disengages_everything():
    operator = Operator()
    readings = both_readings()
    source = ViveTrackerInput(config(), operator, client=Client(readings))
    operator.active = {"left": True, "right": True}
    assert source.sample() is not None

    readings["left"] = reading("LEFT", [1.0, 0.2, 0.3])

    assert source.sample() is None
    assert operator.active == {"left": False, "right": False}
    assert any("jumped" in message for message in operator.messages)
