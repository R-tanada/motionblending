import json
import threading
import time
import numpy as np

# # ----- Custom class ----- #
from OptiTrack.OptiTrackStreamingManager import OptiTrackStreamingManager
from Sensor.SensorManager import GripperSensorManager
from ParticipantMotion.MinimunJerk import MinimumJerk
import CustomFunction.CustomFunction as cf
from Data.DataManager import DataRecordManager, DataLoadManager
from CustomFunction.FilterManager import RealTimeLowpassFilter


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

    def ExportCSV(self):
        for Config in self.participantConfig:
            self.motionManagers[Config['Mount']].ExportCSV()

class MotionManager:
    optiTrackStreamingManager = OptiTrackStreamingManager(mocapServer = "133.68.108.109", mocapLocal = "133.68.108.109")
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
        self.initFlag = False
        self.updateInitPosition = []
        self.updateInitQuaternion = []
        self.updateInitInverseMatrix = []
        self.iter_initPos = self.iter_initRot = []
        self.isMoving_Pos = self.isMoving_Rot = self.isMoving_Grip = self.isMoving = False
        self.pos_list = []
        self.pos_norm_list = []
        self.dt = 1/ 200
        self.startTime = time.perf_counter()
        self.before_time = 0
        self.recording = True
        self.Simulation = True
        path_pos = ''
        path_rot = ''
        path_grip = ''

        self.automation = MinimumJerk(Config['Target'], xArmConfig)
        self.initRot = xArmConfig['InitRot']

        MotionManager.optiTrackStreamingManager.position[self.rigidBody] = np.zeros(3)
        MotionManager.optiTrackStreamingManager.rotation[self.rigidBody] = np.zeros(4)

        self.sensorManager = GripperSensorManager(Config['SerialCOM'], BandRate = 9600)
        sensorThread = threading.Thread(target = self.sensorManager.StartReceiving)
        sensorThread.setDaemon(True)
        sensorThread.start()

        if self.recording:
            self.recorder_pos = DataRecordManager(header = ['x', 'y', 'z'])
            self.recorder_rot = DataRecordManager(header = ['x', 'y', 'z', 'w'])
            self.recorder_grip = DataRecordManager(header = ['grip'])

        if self.Simulation:
            self.data_pos = DataLoadManager(path_pos)
            self.data_rot = DataLoadManager(path_rot)
            self.data_grip = DataLoadManager(path_grip)

    def GetMotionData(self):
        position, rotation, gripper = self.GetPosition(), self.GetRotation(), self.GetGripperValue()
        velocity, accelaration = self.GetParticipnatMotionInfo(position)

        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            if self.isMoving == True:
                self.UpdateInitPosition(position)
                self.UpdateInitRotation(rotation)
                self.isMoving = False
            
            if self.isMoving == False and self.initFlag == True:
                posFlag = self.LerpInitPosition()
                rotFlag = self.SlerpInitRotation()
                if posFlag == rotFlag == False:
                    self.initFlag = False
                    
            if self.initFlag == False:
                if self.automation.MonitoringMotion(position, rotation, gripper, velocity, accelaration):
                    self.isMoving_Pos = self.isMoving_Rot = self.isMoving_Grip = self.isMoving = self.initFlag = True

        return {'position': position, 'rotation': rotation, 'gripper': gripper, 'weight': self.weight}

    def GetPosition(self):
        if self.Simulation:
            position = self.data_pos.getdata()
        else:
            position = cf.ConvertAxis_Position(MotionManager.optiTrackStreamingManager.position[self.rigidBody] * 1000, self.mount) - np.array(self.initPosition)
            if self.recording:
                self.recorder_pos.record(position)

        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            self.position = position
        else:
            self.position, self.isMoving_Pos = self.automation.GetPosition()

        return self.position
    
    def GetRotation(self):
        if self.Simulation:
            rotation = self.data_rot.getdata()
        else:
            rotation = cf.CnvertAxis_Rotation(MotionManager.optiTrackStreamingManager.rotation[self.rigidBody], self.mount)
            if self.recording:
                self.recorder_rot.record(rotation)

        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            quaternion = rotation
            if quaternion[3] < 0:
                quaternion = -np.array(quaternion)
            self.rotation = quaternion
        else:
            self.rotation, self.isMoving_Rot = self.automation.GetRotation()

        return [self.rotation, self.initQuaternion, self.initInverseMatrix]
    
    def GetGripperValue(self):
        if self.Simulation:
            grip = self.data_grip.getdata()
        else:
            grip = cf.ConvertSensorToGripper(self.sensorManager.sensorValue)
            if self.recording:
                self.recorder_grip.record(grip)

        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            gripper = grip
        else:
            gripper, self.isMoving_Grip = self.automation.GetGripperValue()

        return gripper
    
    def SetInitPosition(self):
        self.initPosition = cf.ConvertAxis_Position(MotionManager.optiTrackStreamingManager.position[self.rigidBody] * 1000, self.mount)

    def SetInitRotation(self):
        quaternion = cf.CnvertAxis_Rotation(MotionManager.optiTrackStreamingManager.rotation[self.rigidBody], self.mount)
        if quaternion[3] < 0:
            quaternion = -np.array(quaternion)
        self.initQuaternion = self.automation.q_init = quaternion
        self.initInverseMatrix = cf.Convert2InverseMatrix(quaternion)

    def UpdateInitPosition(self, position):
        self.updateInitPosition = self.initPosition - (np.array(position) - self.GetPosition())
        p_list = np.linspace(self.updateInitPosition, self.initPosition, 500)
        self.iter_initPos = iter(p_list)

    def UpdateInitRotation(self, rotation):
        q_zero = [0, 0, 0, 1]
        quaternion, initQuaternion, initInveseMatrix = self.GetRotation()
        q_inverse = np.dot(cf.Convert2InverseMatrix(quaternion), q_zero)
        self.updateInitQuaternion = np.dot(initInveseMatrix, np.dot(cf.Convert2Matrix(rotation[0]), q_inverse))
        weight_list = np.linspace(0, 1, 500)
        q_list = []
        for weight in weight_list:
            q_list.append(cf.Slerp_Quaternion(self.initQuaternion, self.updateInitQuaternion, weight))

        self.iter_initRot = iter(q_list)

    def LerpInitPosition(self):
        try:
            self.initPosition, flag = next(self.iter_initPos), True
        except StopIteration:
            flag = False

        return flag
    
    def SlerpInitRotation(self):
        try:
            rot = next(self.iter_initRot)
            self.initQuaternion, self.initInverseMatrix, flag = rot, cf.Convert2Matrix(rot), True
        except StopIteration:
            flag = False
            self.initQuaternion = self.automation.q_init
            self.initInverseMatrix = cf.Convert2InverseMatrix(self.automation.q_init)

        return flag

    def GetParticipnatMotionInfo(self, position):
        self.pos_list.append(np.linalg.norm(position))

        # if len(self.pos_list) == 21:
        #     vel = (self.pos_list[10] - self.pos_list[0])/ (self.dt * 10)
        #     acc = ((self.pos_list[20] - self.pos_list[10]) - (self.pos_list[10] - self.pos_list[0]))/ (self.dt * 10)**2

        if len(self.pos_list) == 7:
            vel = (self.pos_list[6] - self.pos_list[3])/ (self.dt * 3)
            acc = (vel - ((self.pos_list[3] - self.pos_list[0])/ (self.dt * 3)))/ (self.dt * 3)

            # self.recorder.Record([self.pos_list[6], vel, acc])
            # if len(self.recorder.dataRecorded['data']) == 500:
            #     self.recorder.ExportAsCSV('Recorder/RecordedData/mocap_raw_data.csv')
            
            del self.pos_list[0]

        else:
            vel = acc = 0

        return vel, acc
    
    def ExportCSV(self):
        self.recorder_pos.exportAsCSV()
        self.recorder_rot.exportAsCSV()
        self.recorder_grip.exportAsCSV()
