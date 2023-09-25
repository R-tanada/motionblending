import json
import threading
import time

import numpy as np

import lib.self.CustomFunction as cf
from src.DataManager import DataLoadManager, DataPlotManager, DataRecordManager
from MotionBlending import RecordedMotion
# # ----- Custom class ----- #
from src.OptiTrackStreamingManager import OptiTrackStreamingManager
from src.SensorManager import GripperSensorManager


class ParticipantManager:
    def __init__(self, ParticipantConfig: dict) -> None:
        self.participantConfig = ParticipantConfig
        self.position = []
        self.rotation = []

        self.motionManagers= {}
        for Config in self.participantConfig:
            self.motionManagers[Config['Mount']] = MotionManager(Config, ParticipantManager.xArmConfig[Config['Mount']])

    def get_motions(self):
        participantMotions = {}
        for Config in self.participantConfig:
            participantMotions[Config['Mount']] = self.motionManagers[Config['Mount']].get_motion()

        return participantMotions

    def SetParticipantInitPosition(self):
        for Config in self.participantConfig:
            self.motionManagers[Config['Mount']].SetInitPosition()

    def SetParticipantInitRotation(self):
        for Config in self.participantConfig:
            self.motionManagers[Config['Mount']].SetInitRotation()

class MotionManager:
    optiTrackStreamingManager = OptiTrackStreamingManager(mocapServer = "127.0.0.1", mocapLocal = "127.0.0.1")
    streamingThread = threading.Thread(target = optiTrackStreamingManager.stream_run)
    streamingThread.setDaemon(True)
    streamingThread.start()

    def __init__(self, Config, xArmConfig, is_Simulation, is_Recording) -> None:
        self.mount, self.rigidBody = Config['Mount'], str(Config['RigidBody'])
        self.init_position = self.init_rotation = self.init_inverse_matrix = []
        self.updated_init_position = self.updated_init_rotation = self.updated_init_inverse_matrix = []
        self.iter_init_position = self.iter_init_rotation = []
        self.is_recording, self.is_simulation = is_Recording, is_Simulation

        self.recorder_motion = DataRecordManager('motion_' + self.mount)
        self.sensorManager = GripperSensorManager(Config['SerialCOM'], BandRate = 9600)

        MotionManager.optiTrackStreamingManager.position[self.rigidBody] = np.zeros(3)
        MotionManager.optiTrackStreamingManager.rotation[self.rigidBody] = np.zeros(4)

    def get_motion(self):
        motion = {'position': self.get_position(), 'rotation': self.get_rotation(), 'gripper': self.get_gripper}
        if self.is_recording == True and self.is_simulation == False:
            self.recorder_motion.record(motion)
        return motion

    def get_position(self):
        return cf.ConvertAxis_Position(MotionManager.optiTrackStreamingManager.position[self.rigidBody] * 1000, self.mount) - np.array(self.initPosition)

    def get_rotation(self):
        # rotationはquternionで取得
        rotation = cf.CnvertAxis_Rotation(MotionManager.optiTrackStreamingManager.rotation[self.rigidBody], self.mount)
        if quaternion[3] < 0:
            quaternion = -np.array(quaternion)
        return np.dot(self.initInverseMatrix, quaternion)

    def get_gripper(self):
        return cf.ConvertSensorToGripper(self.sensorManager.sensorValue)
    
    def set_init_motion(self):
        self.set_init_position()
        self.set_init_rotation()

    def set_init_position(self):
        self.init_position = cf.ConvertAxis_Position(MotionManager.optiTrackStreamingManager.position[self.rigidBody] * 1000, self.mount)

    def set_init_rotation(self):
        quaternion = cf.CnvertAxis_Rotation(MotionManager.optiTrackStreamingManager.rotation[self.rigidBody], self.mount)
        if quaternion[3] < 0:
            quaternion = -np.array(quaternion)
        self.init_rotation = quaternion
        self.init_inverse_matrix = cf.Convert2InverseMatrix(quaternion)

    def update_init_position(self, position):
        self.updated_init_position = self.initPosition - (np.array(position) - self.get_position())
        p_list = np.linspace(self.updated_init_position, self.init_position, 500)
        self.iter_init_position = iter(p_list)

    def update_init_rotation(self, rotation):
        q_zero = [0, 0, 0, 1]
        quaternion, initQuaternion, initInveseMatrix = self.get_rotation()
        q_inverse = np.dot(cf.Convert2InverseMatrix(quaternion), q_zero)
        self.updateInitQuaternion = np.dot(initInveseMatrix, np.dot(cf.Convert2Matrix(rotation[0]), q_inverse))
        self.updateInitQuaternion = np.dot(cf.Convert2InverseMatrix(self.updateInitQuaternion), q_zero)
        weight_list = np.linspace(0, 1, 300)
        q_list = []
        for weight in weight_list:
            q_list.append(cf.Slerp_Quaternion(self.initQuaternion, self.updateInitQuaternion, weight))

        self.iter_initRot = iter(q_list)

    def lerp_init_position(self):
        try:
            self.initPosition, flag = next(self.iter_initPos), True
        except StopIteration:
            flag = False

        return flag

    def slerp_init_rotation(self):
        try:
            rot = next(self.iter_initRot)
            print(rot)
            self.initQuaternion, self.initInverseMatrix, flag = rot, cf.Convert2InverseMatrix(rot), True
        except StopIteration:
            flag = False
            self.initQuaternion = self.automation.q_init
            self.initInverseMatrix = cf.Convert2InverseMatrix(self.automation.q_init)

        return flag