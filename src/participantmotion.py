import json
import threading
import time

import numpy as np

import lib.self.CustomFunction as cf
from src.datamanage import DataLoadManager, DataPlotManager, DataRecordManager
# # ----- Custom class ----- #
from src.optitrack import OptiTrackStreamingManager
from src.sensor import GripperSensorManager
from src.automation import RecordedMotion

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

    def set_init_motions(self):
        for Config in self.participantConfig:
            self.motionManagers[Config['Mount']].set_init_motion()

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
        self.automation = RecordedMotion()

        MotionManager.optiTrackStreamingManager.position[self.rigidBody] = np.zeros(3)
        MotionManager.optiTrackStreamingManager.rotation[self.rigidBody] = np.zeros(4)

    def get_motion(self):
        motion = {'position': self.get_position(), 'rotation': self.get_rotation(), 'gripper': self.get_gripper(), 'velocity': self.get_velocity()}

        motion_automated = self.automation.MonitoringMotion()

        return motion

    def get_position(self):
        return cf.ConvertAxis_Position(MotionManager.optiTrackStreamingManager.position[self.rigidBody] * 1000, self.mount) - np.array(self.initPosition)

    def get_rotation(self):
        # rotationはquternionで取得
        quaternion = cf.CnvertAxis_Rotation(MotionManager.optiTrackStreamingManager.rotation[self.rigidBody], self.mount)
        if quaternion[3] < 0:
            quaternion = -np.array(quaternion)
        return np.dot(self.init_inverse_matrix, quaternion)

    def get_gripper(self):
        return cf.ConvertSensorToGripper(self.sensorManager.sensorValue)
    
    def get_velocity(self):
        pass
    
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