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

    def __init__(self, ParticipantConfig, SensorPort, RigidBodyNum :int) -> None:
        self.recordedMotion             = {}
        self.recordedGripperValue       = {}
        self.recordedMotionLength       = []
        self.InitSensorValues    = {}
        self.GripperValueMax = 850
        self.GripperValueMin = 0
        self.SensorInputValueMax = 1
        self.SensorInputValueMin = 0
        self.rigitBodyNums = {}
        self.participantConfig = ParticipantConfig

        {"Arm": ["left", "right"], "Gripper": ["left", "right"], "weight": 0.5}

        for mount in len(self.participantConfig['Arm']):
            self.rigitBodyNums[mount] = (ParticipantMotionManager.rigitBodyCount)
            ParticipantMotionManager.rigitBodyCount += 1

        self.sensorManagers = {} 
        for Gripper in self.participantConfig['Gripper']:
            self.sensorManagers[Gripper] = (GripperSensorManager(ip=self.ip[i], port=self.port[i]))

            # ----- Start receiving bending sensor value from UDP socket ----- #
            SensorThread = threading.Thread(target = self.sensorManagers[Gripper].StartReceiving)
            SensorThread.setDaemon(True)
            SensorThread.start()

    def GetParticipantMotion(self):
        participantMotion = {}
        for config in self.participantConfig['Arm']:
            participantMotion[config] = {'positon': self.GetLocalPosition(config), 'rotation': self.GetLocalRotation(config), 'gripper': self.GetGripperControlValue[config]}
        
        return participantMotion

    def GetLocalPosition(self, config):
        position = ParticipantMotionManager.optiTrackStreamingManager.position[self.rigitBodyNums[config]]
        return position * 1000

    def GetLocalRotation(self, config):
        rotation = ParticipantMotionManager.optiTrackStreamingManager.rotation[self.rigitBodyNums[config]]
        return rotation
    
    def SetInitialBendingValue(self):
        for Gripper in self.Gripper:
            self.InitSensorValues[Gripper] = self.sensorManagers[Gripper].sensorValue

    def GetGripperControlValue(self, config):
        dictGripperValue = {}
        bendingVal = self.sensorManagers[config].sensorValue - self.InitSensorValues[config]
        bendingValueNorm = bendingVal * (self.SensorInputValueMax - self.GripperValueMin) + self.GripperValueMin

        if bendingValueNorm > self.SensorInputValueMax:
            bendingValueNorm = self.SensorInputValueMax
        elif bendingValueNorm < self.GripperValueMin:
            bendingValueNorm = self.GripperValueMin

        dictGripperValue['gripperValue'+str(i+1)] = bendingValueNorm

        return bendingVal
    
    # def GripperControlValue(self, loopCount: int = 0):
    #     dictGripperValue = {}
    #     for Gripper in range(self.GripperToUse):
    #         bendingVal = self.sensorManagers[Gripper].sensorValue - self.InitSensorValues[Gripper]
    #         bendingValueNorm = bendingVal * (self.SensorInputValueMax - self.GripperValueMin) + self.GripperValueMin

    #         if bendingValueNorm > self.SensorInputValueMax:
    #             bendingValueNorm = self.SensorInputValueMax
    #         elif bendingValueNorm < self.GripperValueMin:
    #             bendingValueNorm = self.GripperValueMin

    #         dictGripperValue['gripperValue'+str(i+1)] = bendingValueNorm

    #     return dictGripperValue
