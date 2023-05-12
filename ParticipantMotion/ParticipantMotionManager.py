import csv
import json
import threading
import time

import numpy as np

# ----- Custom class ----- #
from OptiTrack.OptiTrackStreamingManager import OptiTrackStreamingManager
from Sensor.SensorManager import GripperSensorManager


class ParticipantMotionManager:
    optiTrackStreamingManager = OptiTrackStreamingManager(mocapServer = "133.68.35.155", mocapLocal = "133.68.35.155")
    streamingThread = threading.Thread(target = optiTrackStreamingManager.stream_run)
    streamingThread.setDaemon(True)
    streamingThread.start()

    def __init__(self, ParticipantConfig: dict) -> None:
        self.participantConfig = ParticipantConfig
        self.InitPosition = {}
        self.InitQuaternion = {}
        self.InitInverseMatrix = {}
        self.InitGripper = {}

        for Config in self.participantConfig:
            self.optiTrackStreamingManager.position[str(Config['RigidBody'])] = np.zeros(3)
            self.optiTrackStreamingManager.rotation[str(Config['RigidBody'])] = np.zeros(4)

        self.sensorManagers = {}
        for Config in self.participantConfig:
            self.sensorManagers[Config['Mount']] = (GripperSensorManager(Config['SerialCOM'], BandRate = 9600))
            SensorThread = threading.Thread(target = self.sensorManagers[Config['Mount']].StartReceiving)
            SensorThread.setDaemon(True)
            SensorThread.start()

    def GetMotion(self):
        participantMotion = {}
        position = self.GetRelativePosition()
        rotation = self.GetRelativeRotation()
        gripper = self.GetGripperControlValue()
        for Config in self.participantConfig:
            participantMotion[Config['Mount']] = {'position': position[Config['Mount']], 'rotation': rotation[Config['Mount']], 'gripper': gripper[Config['Mount']], 'weight': Config['Weight']}

        return participantMotion

    def GetPosition(self):
        relativePosition = {}
        for Config in self.participantConfig:
            position = (ParticipantMotionManager.optiTrackStreamingManager.position[ str(Config['RigidBody'])]) * 1000
            relativePosition[Config['Mount']] = self.ConvertAxis_Position(position, Config['Mount'])

        return relativePosition

    def GetQuaternion(self):
        relativeRotation = {}
        for Config in self.participantConfig:
            rotation = ParticipantMotionManager.optiTrackStreamingManager.rotation[str(Config['RigidBody'])]
            relativeRotation[Config['Mount']] = self.CnvertAxis_Rotation(rotation, Config['Mount'])
        return relativeRotation

    def SetInitPosition(self):
        for Config in self.participantConfig:
            self.InitPosition[Config['Mount']] = ParticipantMotionManager.optiTrackStreamingManager.position[str(Config['RigidBody'])] * 1000

    def SetInitQuaternion(self) -> None:
        for Config in self.participantConfig:
            q = self.InitQuaternion[Config['Mount']] = ParticipantMotionManager.optiTrackStreamingManager.rotation[str(Config['RigidBody'])]
            qw, qx, qy, qz = q[3], q[1], q[2], q[0]
            mat4x4 = np.array([ [qw, -qy, qx, qz],
                                [qy, qw, -qz, qx],
                                [-qx, qz, qw, qy],
                                [-qz,-qx, -qy, qw]])
            self.InitInverseMatrix[Config['Mount']] = np.linalg.inv(mat4x4)

    def ConvertAxis_Position(self, position, axis):
        if axis == 'vertical':
            position = [position[2], position[0], position[1]]
        elif axis == 'left':
            position = [position[2], -1 * position[1], position[0]]
        elif axis == 'right':
            position = [position[2], position[1], -1 * position[0]]

        return position

    def CnvertAxis_Rotation(self, rotation, axis):
        if axis == 'vertical':
            rotation = [rotation[2], rotation[0], rotation[1], rotation[3]]
        elif axis == 'left':
            rotation = [rotation[2], -1 * rotation[1], rotation[0], rotation[3]]
        elif axis == 'right':
            rotation = [rotation[2], rotation[1], -1 * rotation[0], rotation[3]]

        return rotation

    def GetGripperControlValue(self):
        gripper = {}
        for Config in self.participantConfig:
            gripper[Config['Mount']] = self.ConvertSensorToGripper(self.sensorManagers[Config['Mount']].sensorValue)
        return gripper

    def ConvertSensorToGripper(self, sensorValue, InputMax = 1, InputMin = 0, TargetMax = 850, TargetMin = 0):
        gripperValue = ((sensorValue - InputMin) / (InputMax - InputMin)) * (TargetMax - TargetMin) + TargetMin

        if gripperValue > TargetMax:
            gripperValue = TargetMax
        elif gripperValue < TargetMin:
            gripperValue = TargetMin

        return gripperValue
