import threading
import numpy as np

# ----- Custom class ----- #
from OptiTrack.OptiTrackStreamingManager import OptiTrackStreamingManager
from Sensor.SensorManager import GripperSensorManager
import CustomFunction.CustomFunction as cf

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

        MotionManager.optiTrackStreamingManager.position[self.rigidBody] = np.zeros(3)
        MotionManager.optiTrackStreamingManager.rotation[self.rigidBody] = np.zeros(4)

        self.sensorManager = GripperSensorManager(Config['SerialCOM'], BandRate = 9600)
        sensorThread = threading.Thread(target = self.sensorManager.StartReceiving)
        sensorThread.setDaemon(True)
        sensorThread.start()
    
    def GetMotionData(self):
        return {'position': self.GetPosition(), 'rotation': self.GetRotation(), 'gripper': self.GetGripperValue(), 'weight': self.weight}

    def GetPosition(self):
        return cf.ConvertAxis_Position(MotionManager.optiTrackStreamingManager.position[self.rigidBody] * 1000 - self.initPosition, self.mount)
    
    def GetRotation(self):
        return [cf.CnvertAxis_Rotation(MotionManager.optiTrackStreamingManager.rotation[self.rigidBody], self.mount), self.initQuaternion, self.initInverseMatrix]
    
    def GetGripperValue(self):
        return cf.ConvertSensorToGripper(self.sensorManager.sensorValue)
    
    def SetInitPosition(self):
        self.initPosition = MotionManager.optiTrackStreamingManager.position[self.rigidBody] * 1000

    def SetInitRotation(self) -> None:
        self.initQuaternion = cf.CnvertAxis_Rotation(MotionManager.optiTrackStreamingManager.rotation[self.rigidBody], self.mount)
        self.initInverseMatrix = cf.Convert2Matrix_Quaternion(quaternion = self.initQuaternion, inverse = True)
    