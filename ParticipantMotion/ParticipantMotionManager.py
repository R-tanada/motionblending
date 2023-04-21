# -----------------------------------------------------------------------
# Author:   Takayoshi Hagiwara (KMD)
# Created:  2021/10/6
# Summary:  操作者の動きマネージャー
# -----------------------------------------------------------------------

import threading
import time
import numpy as np
import csv

# ----- Custom class ----- #
from OptiTrack.OptiTrackStreamingManager import OptiTrackStreamingManager
from Sensor.SensorManager import GripperSensorManager


class ParticipantMotionManager:
    rigitBodyCount = 0

    optiTrackStreamingManager = OptiTrackStreamingManager(rigid_body_num = 4, mocapServer = "133.68.35.155", mocapLocal = "133.68.35.155")
    streamingThread = threading.Thread(target = optiTrackStreamingManager.stream_run)
    streamingThread.setDaemon(True)
    streamingThread.start()

    def __init__(self, ArmToUse: list = ['left', 'right'], GripperToUse: list = ['left', 'right'], SensorPort, RigidBodyNum :int, RecordedMotion: bool = False, RecordedMotionPath: str = '', RecordedGripperPath: str = '') -> None:
        self.GripperToUse               = GripperToUse
        self.recordedMotion             = {}
        self.recordedGripperValue       = {}
        self.recordedMotionLength       = []
        self.InitSensorValues    = {}
        self.GripperValueMax = 850
        self.GripperValueMin = 0
        self.SensorInputValueMax = 1
        self.SensorInputValueMin = 0
        self.rigitBodyNums = []

        for i in len(ArmToUse):
            self.rigitBodyNums.append(ParticipantMotionManager.rigitBodyCount)
            ParticipantMotionManager.rigitBodyCount += 1

        self.sensorManagers = {} 
        for Gripper in self.GripperToUse:
            self.sensorManagers[Gripper] = (GripperSensorManager(ip=self.ip[i], port=self.port[i]))

            # ----- Start receiving bending sensor value from UDP socket ----- #
            SensorThread = threading.Thread(target = self.sensorManagers[Gripper].StartReceiving)
            SensorThread.setDaemon(True)
            SensorThread.start()
        
        if RecordedMotion == True:
            with open(RecordedMotionPath) as f:
                reader = csv.reader(f)
                data = [row for row in reader][1:]
                data = [[float(v) for v in row] for row in data]
                self.recordedMotion = iter(data) 

            with open(RecordedGripperPath) as f:
                reader = csv.reader(f)
                data = [row for row in reader][1:] 
                data = [[float(v) for v in row] for row in data]  
                self.recordedGripperValue = iter(data)

    def GetParticipantMotion(self):
        return dict(self.LocalPosition, )
        

    def GetLocalPosition(self):
        for rigidBodyNum in self.rigitBodyNums:
            rigidBodyPos = ParticipantMotionManager.optiTrackStreamingManager.position

        # If the data is ended, the last value is returned.
        for i in range(self.recordedParticipantNum):
            recordedParticipantNum = self.defaultParticipantNum + i+1
            if loopCount >= self.recordedMotionLength[i]:
                dictPos['participant'+str(recordedParticipantNum)] = np.array(self.recordedMotion['participant'+str(recordedParticipantNum)][-1][0:3])
                continue

        return dictPos

    def GetLocalRotation(self):
        dictRot = {}
        dictRot = self.optiTrackStreamingManager.rotation

        # If the data is ended, the last value is returned.
        for i in range(self.recordedParticipantNum):
            recordedParticipantNum = self.defaultParticipantNum + i+1
            if loopCount >= self.recordedMotionLength[i]:
                dictRot['participant'+str(recordedParticipantNum)] = np.array(self.recordedMotion['participant'+str(recordedParticipantNum)][-1][3:8])
                continue

            dictRot['participant'+str(recordedParticipantNum)] = np.array(self.recordedMotion['participant'+str(recordedParticipantNum)][loopCount][3:8])

        return dictRot
    
    def SetInitialBendingValue(self):
        for Gripper in self.GripperToUse:
            self.InitSensorValues[Gripper] = self.sensorManagers[Gripper].sensorValue

    def GripperControlValue(self, loopCount: int = 0):
        dictGripperValue = {}
        for Gripper in range(self.GripperToUse):
            bendingVal = self.sensorManagers[Gripper].sensorValue - self.InitSensorValues[Gripper]
            bendingValueNorm = bendingVal * (self.SensorInputValueMax - self.GripperValueMin) + self.GripperValueMin

            if bendingValueNorm > self.SensorInputValueMax:
                bendingValueNorm = self.SensorInputValueMax
            elif bendingValueNorm < self.GripperValueMin:
                bendingValueNorm = self.GripperValueMin

            dictGripperValue['gripperValue'+str(i+1)] = bendingValueNorm

        return dictGripperValue
