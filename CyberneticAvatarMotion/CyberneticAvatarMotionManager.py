import math

import numpy as np
import quaternion
import scipy.spatial.transform as scitransform

from ParticipantMotion.ParticipantMotionManager import ParticipantManager
import CustomFunction.CustomFunction as cf


class CyberneticAvatarMotionManager:
    def __init__(self, ParticipantConfigs: dict, xArmConfigs: dict) -> None:
        self.xArmConfigs = xArmConfigs

        self.participantManagers = {}
        for participant in ParticipantConfigs.keys():
            self.participantManagers[participant] = ParticipantManager(ParticipantConfigs[participant])

    def GetSharedTransform(self):
        return self.IntegrationWithWeight(self.GetParticipantMotion())

    def IntegrationWithWeight(self, participantMotions: dict):
        sharedMotions = {}
        for xArm in self.xArmConfigs.keys():
            sharedMotions[self.xArmConfigs[xArm]['Mount']] = {'position': [0, 0, 0], 'rotation': [0, 0, 0], 'gripper': 0}

        for participant in participantMotions.keys():
            for mount in participantMotions[participant].keys():
                sharedMotions[mount]['position'] += np.array(participantMotions[participant][mount]['position']) * participantMotions[participant][mount]['weight']
                sharedMotions[mount]['rotation'] += cf.Quaternion2Euler(np.dot(participantMotions[participant][mount]['rotation'][2], cf.Slerp_Quaternion(participantMotions[participant][mount]['rotation'][0], participantMotions[participant][mount]['rotation'][1], participantMotions[participant][mount]['weight'])))
                sharedMotions[mount]['gripper'] += np.array(participantMotions[participant][mount]['gripper']) * participantMotions[participant][mount]['weight']

        return sharedMotions

    def GetParticipantMotion(self):
        motions = {}
        for participant in self.participantManagers.keys():
            motions[participant] = self.participantManagers[participant].GetParticipantMotion()

        return motions

    def SetParticipantInitMotion(self):
        for participant in self.participantManagers.keys():
            self.participantManagers[participant].SetParticipantInitPosition()
            self.participantManagers[participant].SetParticipantInitRotation()

