import csv
import json
import threading
import time

import numpy as np

# ----- Custom class ----- #
from OptiTrack.OptiTrackStreamingManager import OptiTrackStreamingManager
from Sensor.SensorManager import GripperSensorManager

class ParticipantManager:
    def __init__(self, ParticipantConfig: dict) -> None:
        self.participantConfig = ParticipantConfig

        self.motionManagers= {}
        for Config in self.participantConfig:
            self.motionManagers[Config['Mount']] = MotionManager(Config)

    def GetParticipantMotion(self):
        participantMotions = {}
        for Config in self.participantConfig:
            participantMotions[Config['Mount']] = self.motionManagers[Config['Mount']].GetMotionData()

        return participantMotions
    
    def SetParticipantInitPosition(self):
        for Config in self.participantConfig:
            self.motionManagers[Config['Mount']].SetInitPosition()

    def SetParticipantInitRotation(self):
        for Config in self.participantConfig:
            self.motionManagers[Config['Mount']].SetInitRotation()


class MotionManager:
    optiTrackStreamingManager = OptiTrackStreamingManager(mocapServer = "133.68.35.155", mocapLocal = "133.68.35.155")
    streamingThread = threading.Thread(target = optiTrackStreamingManager.stream_run)
    streamingThread.setDaemon(True)
    streamingThread.start()

    def __init__(self, Config) -> None:
        self.mount = Config['Mount']
        self.rigidBody = Config['RigidBody']
        self.weight = Config['Weight']
        self.initPosition = []
        self.initQuaternion = []
        self.initInverseMatrix = []

        MotionManager.optiTrackStreamingManager.position[str(self.rigidBody)] = np.zeros(3)
        MotionManager.optiTrackStreamingManager.rotation[str(self.rigidBody)] = np.zeros(4)

        self.sensorManager = GripperSensorManager(Config['SerialCOM'], BandRate = 9600)
        sensorThread = threading.Thread(target = self.sensorManager.StartReceiving)
        sensorThread.setDaemon(True)
        sensorThread.start()
    
    def GetMotionData(self):
        return {'position': self.GetPosition(), 'rotation': self.GetRotation(), 'gripper': self.GetGripperValue(), 'weight': self.weight}

    def GetPosition(self):
        return self.ConvertAxis_Position(MotionManager.optiTrackStreamingManager.position[self.rigidBody] - self.initPosition, self.mount) * 1000
    
    def GetRotation(self):
        return [self.CnvertAxis_Rotation(MotionManager.optiTrackStreamingManager.rotation[self.rigidBody], self.mount), self.initQuaternion, self.initInverseMatrix]
    
    def GetGripperValue(self):
        return self.ConvertSensorToGripper(self.sensorManager.sensorValue)
    
    def SetInitPosition(self):
        self.initPosition = MotionManager.optiTrackStreamingManager.position[self.rigidBody]

    def SetInitRotation(self) -> None:
        q = self.initQuaternion = self.CnvertAxis_Rotation(MotionManager.optiTrackStreamingManager.rotation[self.rigidBody], self.mount)
        qw, qx, qy, qz = q[3], q[1], q[2], q[0]
        mat4x4 = np.array([ [qw, -qy, qx, qz],
                            [qy, qw, -qz, qx],
                            [-qx, qz, qw, qy],
                            [-qz,-qx, -qy, qw]])
        self.initInverseMatrix = np.linalg.inv(mat4x4)
    
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
    
    def ConvertSensorToGripper(self, sensorValue, InputMax = 1, InputMin = 0, TargetMax = 850, TargetMin = 0):
        gripperValue = ((sensorValue - InputMin) / (InputMax - InputMin)) * (TargetMax - TargetMin) + TargetMin

        if gripperValue > TargetMax:
            gripperValue = TargetMax
        elif gripperValue < TargetMin:
            gripperValue = TargetMin

        return gripperValue