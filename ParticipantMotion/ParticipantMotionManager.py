import csv
import json
import threading
import time

import numpy as np

# ----- Custom class ----- #
from OptiTrack.OptiTrackStreamingManager import OptiTrackStreamingManager
from Sensor.SensorManager import GripperSensorManager
from ParticipantMotion.MinimunJerk import MinimumJerk


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
        self.rigidBody = str(Config['RigidBody'])
        self.weight = Config['Weight']
        self.initPosition = []
        self.initQuaternion = []
        self.initInverseMatrix = []
        self.isMoving_Pos = self.isMoving_Rot = self.isMoving_Grip = False

        self.automation = MinimumJerk(Config['Target'])

        MotionManager.optiTrackStreamingManager.position[self.rigidBody] = np.zeros(3)
        MotionManager.optiTrackStreamingManager.rotation[self.rigidBody] = np.zeros(4)

        self.sensorManager = GripperSensorManager(Config['SerialCOM'], BandRate = 9600)
        sensorThread = threading.Thread(target = self.sensorManager.StartReceiving)
        sensorThread.setDaemon(True)
        sensorThread.start()

    def GetMotionData(self):
        position, rotation, gripper = self.GetPosition(), self.GetRotation(), self.GetGripperValue()
        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            if self.automation.MonitoringMotion(position, rotation, gripper):
                self.isMoving_Pos = self.isMoving_Rot = self.isMoving_Grip = True

        return {'position': position, 'rotation': rotation, 'gripper': gripper, 'weight': self.weight}

    def GetPosition(self):
        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            position = self.ConvertAxis_Position(MotionManager.optiTrackStreamingManager.position[self.rigidBody] - self.initPosition, self.mount) * 1000
        else:
            position, isMoving = self.automation.GetPosition()
            if isMoving == False:
                self.isMoving_Pos = isMoving
                self.SetInitPosition(Adjust = True, position = position)

        return position
    
    def GetRotation(self):
        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            rotation = self.CnvertAxis_Rotation(MotionManager.optiTrackStreamingManager.rotation[self.rigidBody], self.mount)
        else:
            rotation, isMoving = self.automation.GetRotation()
            if isMoving == False:
                self.isMoving_Rot = isMoving
                self.SetInitRotation(Adjust = True, rotation = rotation)

        return [rotation, self.initQuaternion, self.initInverseMatrix]
    
    def GetGripperValue(self):
        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            gripper = self.ConvertSensorToGripper(self.sensorManager.sensorValue)
        else:
            gripper, isMoving = self.automation.GetGripperValue()
            if isMoving == False:
                self.isMoving_Grip = isMoving

        return gripper
    
    def SetInitPosition(self, Adjust = False, position = None):
        if Adjust:
            self.initPosition -= (position - MotionManager.optiTrackStreamingManager.position[self.rigidBody])
        else:
            self.initPosition = MotionManager.optiTrackStreamingManager.position[self.rigidBody]

    def SetInitRotation(self, Adjust = False, rotation = None) -> None:
        if Adjust:
            qw, qx, qy, qz = rotation[3], rotation[1], rotation[2], rotation[0]
            mat4x4 = np.array([ [qw, -qy, qx, qz],
                                [qy, qw, -qz, qx],
                                [-qx, qz, qw, qy],
                                [-qz,-qx, -qy, qw]])
            mat4x4_inverse = np.linalg.inv(mat4x4)
            q = self.CnvertAxis_Rotation(MotionManager.optiTrackStreamingManager.rotation[self.rigidBody], self.mount)
            qw, qx, qy, qz = q[3], q[1], q[2], q[0]
            mat4x4 = np.array([ [qw, -qy, qx, qz],
                                [qy, qw, -qz, qx],
                                [-qx, qz, qw, qy],
                                [-qz,-qx, -qy, qw]])
            q = self.initQuaternion = np.dot(mat4x4, np.dot(mat4x4_inverse, self.initQuaternion))
            qw, qx, qy, qz = q[3], q[1], q[2], q[0]
            mat4x4 = np.array([ [qw, -qy, qx, qz],
                                [qy, qw, -qz, qx],
                                [-qx, qz, qw, qy],
                                [-qz,-qx, -qy, qw]])
            self.initInverseMatrix = np.linalg.inv(mat4x4)
            
        else:
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