import math

import numpy as np
import quaternion
import scipy.spatial.transform as scitransform

from ParticipantMotion.ParticipantMotionManager import ParticipantManager


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
                sharedMotions[mount]['rotation'] += self.Quaternion2Euler(np.dot(participantMotions[participant]['rotation'][2], self.SlerpFunction(participantMotions[participant]['rotation'][0], participantMotions[participant]['rotation'][0], participantMotions[participant][mount]['weight'])))
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

    def Quaternion2Euler(self, q, isDeg: bool = True):
        qx, qy, qz, qw = q[0], q[1], q[2], q[3]

        m00 = 1 - (2 * qy**2) - (2 * qz**2)
        m01 = (2 * qx * qy) + (2 * qw * qz)
        m02 = (2 * qx * qz) - (2 * qw * qy)
        m10 = (2 * qx * qy) - (2 * qw * qz)
        m11 = 1 - (2 * qx**2) - (2 * qz**2)
        m12 = (2 * qy * qz) + (2 * qw * qx)
        m20 = (2 * qx * qz) + (2 * qw * qy)
        m21 = (2 * qy * qz) - (2 * qw * qx)
        m22 = 1 - (2 * qx**2) - (2 * qy**2)

        if m02 == 1:
            tx = math.atan2(m10, m11)
            ty = math.pi/2
            tz = 0
        elif m02 == -1:
            tx = math.atan2(m21, m20)
            ty = -math.pi/2
            tz = 0
        else:
            tx = -math.atan2(-m12, m22)
            ty = -math.asin(m02)
            tz = -math.atan2(-m01, m00)

        if isDeg:
            tx = np.rad2deg(tx)
            ty = np.rad2deg(ty)
            tz = np.rad2deg(tz)

        rotEuler = np.array([tx, ty, tz])
        return rotEuler

    def Euler2Quaternion(self, e):
        roll = np.deg2rad(e[0])
        pitch = np.deg2rad(e[1])
        yaw = np.deg2rad(e[2])

        cosRoll = np.cos(roll/2.0)
        sinRoll = np.sin(roll / 2.0)
        cosPitch = np.cos(pitch / 2.0)
        sinPitch = np.sin(pitch / 2.0)
        cosYaw = np.cos(yaw / 2.0)
        sinYaw = np.sin(yaw / 2.0)

        q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw
        q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw
        q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw
        q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw

        rotQuat = [q1, q2, q3, q0]
        return rotQuat

    def SlerpFunction(self, Quaternion, initQuaternion, weight):
        e = 0.0000001
        theta = math.acos(np.dot(initQuaternion, Quaternion))
        return (math.sin((1 - weight) * theta)/ math.sin(theta) + e) * initQuaternion + (math.sin(weight * theta)/ math.sin(theta) + e) * quaternion




