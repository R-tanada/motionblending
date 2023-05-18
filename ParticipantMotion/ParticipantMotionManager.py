import json
import threading

import numpy as np

# # ----- Custom class ----- #
from OptiTrack.OptiTrackStreamingManager import OptiTrackStreamingManager
from Sensor.SensorManager import GripperSensorManager
from ParticipantMotion.MinimunJerk import MinimumJerk
import CustomFunction.CustomFunction as cf


class ParticipantManager:
    with open('SettingFile/settings_single.json', 'r') as settings_file:
        settings = json.load(settings_file)
    xArmConfig = {}
    for xArm in settings['xArmConfigs'].keys():
        xArmConfig[settings['xArmConfigs'][xArm]['Mount']] = settings['xArmConfigs'][xArm]

    def __init__(self, ParticipantConfig: dict) -> None:
        self.participantConfig = ParticipantConfig
        self.position = []
        self.rotation = []

        self.motionManagers= {}
        for Config in self.participantConfig:
            self.motionManagers[Config['Mount']] = MotionManager(Config, ParticipantManager.xArmConfig[Config['Mount']])

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

    def __init__(self, Config, xArmConfig) -> None:
        self.mount = Config['Mount']
        self.rigidBody = str(Config['RigidBody'])
        self.weight = Config['Weight']
        self.initPosition = []
        self.initQuaternion = []
        self.initInverseMatrix = []
        self.ConvertFlag = False
        self.shiftInitPosition = []
        self.shiftInitQuaternion = []
        self.shiftInitInverseMatrix = []
        self.iter_initPos = self.iter_initRot = []
        self.isMoving_Pos = self.isMoving_Rot = self.isMoving_Grip = self.isMoving = False

        self.automation = MinimumJerk(Config['Target'], xArmConfig)
        self.initRot = xArmConfig['InitRot']

        MotionManager.optiTrackStreamingManager.position[self.rigidBody] = np.zeros(3)
        MotionManager.optiTrackStreamingManager.rotation[self.rigidBody] = np.zeros(4)

        self.sensorManager = GripperSensorManager(Config['SerialCOM'], BandRate = 9600)
        sensorThread = threading.Thread(target = self.sensorManager.StartReceiving)
        sensorThread.setDaemon(True)
        sensorThread.start()

    def GetMotionData(self):
        position, rotation, gripper = self.GetPosition(), self.GetRotation(), self.GetGripperValue()

        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            if self.isMoving == True:
                self.ShiftInitPosition(position)
                self.ShiftInitRotation(rotation)
                self.isMoving = False
            
            if self.isMoving == False and self.initFlag == True:
                posFlag = self.AdjustInitPosition()
                rotFlag = self.AdjustInitRotation()
                if posFlag == rotFlag == False:
                    self.initFlag = False
                
            if self.automation.MonitoringMotion(position, rotation, gripper):
                self.isMoving_Pos = self.isMoving_Rot = self.isMoving_Grip = self.isMoving = self.ConvertFlag = True

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
    
    def SetInitPosition(self):
        self.initPosition = cf.ConvertAxis_Position(MotionManager.optiTrackStreamingManager.position[self.rigidBody] * 1000, self.mount)

    def SetInitRotation(self) -> None:
        q = self.initQuaternion = self.automation.q_init = cf.CnvertAxis_Rotation(MotionManager.optiTrackStreamingManager.rotation[self.rigidBody], self.mount)
        self.initInverseMatrix = cf.Convert2InverseMatrix(quaternion = q)

    def ShiftInitPosition(self, position):
        self.shiftInitPosition -= (np.array(position) - self.GetPosition())
        p_list = np.linspace(self.shiftInitPosition, self.initPosition, 100)
        self.iter_initPos = iter(p_list)

    def ShiftInitRotation(self, rotation):
        q_zero = [0, 0, 0, 1]
        quaternion, initQuaternion, initInveseMatrix = self.GetRotation()
        q_inverse = np.dot(cf.Convert2InverseMatrix(quaternion), q_zero)
        self.shiftInitQuaternion = np.dot(initInveseMatrix, np.dot(cf.Convert2Matrix(rotation), q_inverse))
        weight_list = np.linspace(0, 1, 100)
        q_list = []
        for weight in weight_list:
            q_list.append(cf.Slerp_Quaternion(self.initQuaternion, self.shiftInitQuaternion, weight))

        self.iter_initRot = iter(q_list)

    def AdjustInitPosition(self):
        try:
            self.initPosition, flag = next(self.iter_initPos), True
        except StopIteration:
            flag = False

        return flag
    
    def AdjustInitRotation(self):
        try:
            self.initQuaternion, flag = next(self.iter_initRot), True
        except StopIteration:
            flag = False

        return flag

    