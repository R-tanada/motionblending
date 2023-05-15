import csv
import json
import threading
import time

import numpy as np

# ----- Custom class ----- #
from OptiTrack.OptiTrackStreamingManager import OptiTrackStreamingManager
from Sensor.SensorManager import GripperSensorManager
from ParticipantMotion.MinimunJerk import MinimumJerk
import CustomFunction.CustomFunction as cf


class ParticipantManager:
    def __init__(self, ParticipantConfig: dict) -> None:
        self.participantConfig = ParticipantConfig
        self.position = []
        self.rotation = []
        self.initFlag = False

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
        self.initFlag = False

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
            if self.initFlag == True:
                self.SetInitPosition(Adjust = True, position = self.position)
                self.SetInitRotation(Adjust = True, rotation = self.rotation)
                self.initFlag = False
            if self.automation.MonitoringMotion(position, rotation, gripper):
                self.isMoving_Pos = self.isMoving_Rot = self.isMoving_Grip = True

        return {'position': position, 'rotation': rotation, 'gripper': gripper, 'weight': self.weight}

    def GetPosition(self):
        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            self.position = cf.ConvertAxis_Position(MotionManager.optiTrackStreamingManager.position[self.rigidBody] * 1000, self.mount) - np.array(self.initPosition)
        else:
            self.position, self.isMoving_Pos = self.automation.GetPosition()

        return self.position
    
    def GetRotation(self):
        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            self.rotation = cf.CnvertAxis_Rotation(MotionManager.optiTrackStreamingManager.rotation[self.rigidBody], self.mount)
        else:
            self.rotation, self.isMoving_Rot = self.automation.GetRotation()

        return [self.rotation, self.initQuaternion, self.initInverseMatrix]
    
    def GetGripperValue(self):
        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            gripper = cf.ConvertSensorToGripper(self.sensorManager.sensorValue)
        else:
            gripper, isMoving = self.automation.GetGripperValue()
            if isMoving == False:
                self.isMoving_Grip = isMoving
                self.initFlag = True

        return gripper
    
    def SetInitPosition(self, Adjust = False, position = None):
        if Adjust:
            self.initPosition -= (np.array(position) - cf.ConvertAxis_Position((MotionManager.optiTrackStreamingManager.position[self.rigidBody] * 1000), self.mount))
        else:
            self.initPosition = cf.ConvertAxis_Position(MotionManager.optiTrackStreamingManager.position[self.rigidBody] * 1000, self.mount)

    def SetInitRotation(self, Adjust = False, rotation = None) -> None:
        if Adjust:
            mat4x4_inverse = cf.Convert2Matrix_Quaternion(quaternion = rotation, inverse = True)
            q = cf.CnvertAxis_Rotation(MotionManager.optiTrackStreamingManager.rotation[self.rigidBody], self.mount)
            mat4x4 = cf.Convert2Matrix_Quaternion(quaternion = q)
            q = self.initQuaternion = np.dot(mat4x4, np.dot(mat4x4_inverse, self.initQuaternion))
            self.initInverseMatrix = cf.Convert2Matrix_Quaternion(quaternion = q, inverse = True)
            
        else:
            q = self.initQuaternion = cf.CnvertAxis_Rotation(MotionManager.optiTrackStreamingManager.rotation[self.rigidBody], self.mount)
            self.initInverseMatrix = cf.Convert2Matrix_Quaternion(quaternion = q, inverse = True)
    