import numpy as np
from ParticipantMotion.ParticipantMotionManager import ParticipantManager
import CustomFunction.CustomFunction as cf


class CyberneticAvatarMotionManager:
    def __init__(self, ParticipantConfigs: dict, xArmConfigs: dict) -> None:
        self.xArmConfigs = xArmConfigs
        self.preMotion = {}

        self.participantManagers = {}
        for participant in ParticipantConfigs.keys():
            self.participantManagers[participant] = ParticipantManager(ParticipantConfigs[participant])

        self.sharedMotions = {}
        for xArm in self.xArmConfigs.keys():
            self.sharedMotions[self.xArmConfigs[xArm]['Mount']] = {'position': [0, 0, 0], 'rotation': [0, 0, 0, 1], 'gripper': 0}

    def GetSharedTransform(self):
        return self.IntegrationWithWeight(self.GetParticipantMotion())

    def IntegrationWithWeight(self, participantMotions: dict):
        for participant in participantMotions.keys():
            for mount in participantMotions[participant].keys():
                motions, premotions = participantMotions[participant][mount], self.preMotion[participant][mount]
                self.sharedMotions[mount]['position'] += (np.array(motions['position']) - premotions['position'])  * motions['weight']
                self.sharedMotions[mount]['rotation'] = np.dot(cf.Convert2Matrix_Quaternion(np.dot(cf.Convert2Matrix_Quaternion(premotions['rotation'], inverse = True), cf.Slerp_Quaternion(motions['rotation'], premotions['rotation'], motions['weight']))), self.sharedMotions[mount]['rotation'])
                self.sharedMotions[mount]['gripper'] += 0
                premotions['position'], premotions['rotation'] = motions['position'], motions['rotation']

        return self.sharedMotions

    def GetParticipantMotion(self):
        motions = {}
        for participant in self.participantManagers.keys():
            motions[participant] = self.participantManagers[participant].GetParticipantMotion()

        return motions

    def SetParticipantInitMotion(self):
        for participant in self.participantManagers.keys():
            self.preMotion[participant] = self.participantManagers[participant].SetParticipantInitMotion()





