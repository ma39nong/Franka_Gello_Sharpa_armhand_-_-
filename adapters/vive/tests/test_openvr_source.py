from types import SimpleNamespace

import numpy as np

from vive_tracker_teleop.openvr_source import OpenVRClient


class Matrix:
    m = [[1.0, 0.0, 0.0, 0.1], [0.0, 1.0, 0.0, 0.2], [0.0, 0.0, 1.0, 0.3]]


class Pose:
    bDeviceIsConnected = True
    bPoseIsValid = True
    eTrackingResult = 200
    mDeviceToAbsoluteTracking = Matrix()
    vVelocity = SimpleNamespace(v=[0.0, 0.0, 0.0])
    vAngularVelocity = SimpleNamespace(v=[0.0, 0.0, 0.0])


class Runtime:
    k_unMaxTrackedDeviceCount = 4
    TrackingUniverseStanding = 1
    TrackedDeviceClass_GenericTracker = 3
    TrackedDeviceClass_Invalid = 0
    TrackingResult_Running_OK = 200
    Prop_SerialNumber_String = 1000


class VR:
    def __init__(self):
        self.pose_queries = 0

    def getTrackedDeviceClass(self, index):
        return 3 if index in (1, 2) else 0

    def getStringTrackedDeviceProperty(self, index, _property):
        return {1: "RIGHT", 2: "LEFT"}[index]

    def getDeviceToAbsoluteTrackingPose(self, *_args):
        self.pose_queries += 1
        invalid = SimpleNamespace(
            bDeviceIsConnected=False,
            bPoseIsValid=False,
            eTrackingResult=0,
        )
        return [invalid, Pose(), Pose(), invalid]


def test_requested_serials_are_read_from_one_pose_batch_regardless_of_index():
    client = OpenVRClient()
    client.openvr = Runtime()
    client.vr = VR()

    readings = client.read({"left": "LEFT", "right": "RIGHT"})

    assert readings["left"].serial == "LEFT"
    assert readings["right"].serial == "RIGHT"
    np.testing.assert_allclose(readings["left"].transform[:3, 3], [0.1, 0.2, 0.3])
    assert client.vr.pose_queries == 1
