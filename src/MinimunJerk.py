import time
from itertools import cycle as iter_cycle

import numpy as np
from matplotlib import pyplot as plt
from src.DataManager import DataPlotManager
import lib.self.CustomFunction as cf
from src.SensorManager import FootSwitchManager
import threading


class MinimumJerk:
    def __init__(self, Target: list, xArmConfig: dict, Threshold = 300) -> None:
        self.initPos = xArmConfig['InitPos']
        initRot = cf.Convert2InverseMatrix(cf.Euler2Quaternion(xArmConfig['InitRot']))
        self.predictedPosition = []
        self.predictedRotation = []
        self.predictedGripper = []
        self.Threshold = Threshold
        self.target = Target
        self.q_init = []
        for target in self.target:
            target['position'] -= np.array(self.initPos)
            target['rotation'] = np.dot(cf.Convert2Matrix(cf.Euler2Quaternion(target['rotation'])), np.dot(initRot, [0, 0, 0, 1]))
            if target['rotation'][3] < 0:
                target['rotation'] = -target['rotation']
        self.flag = False
        self.initThreshold = 100
        self.wayPoint = []
        self.freq = 240
        self.acc_flag = True
        self.before_acc = 0
        self.before_vel = 0
        self.time_list = []
        self.t0 = 0
        self.tf = 0
        self.x0 = [0, 0, 0]
        self.target_index = 0
        self.pos_list = []
        self.tn = 0
        self.a = 0

        self.switchManager = FootSwitchManager()
        switchThread = threading.Thread(target=self.switchManager.detect_sensor)
        switchThread.setDaemon(True)
        switchThread.start()

    # def GetPosition(self):
    #     try:
    #         position = self.posRetained = next(self.predictedPosition)
    #         isMoving = True
    #     except StopIteration:
    #         position, isMoving = self.posRetained, False

    #     return position, isMoving

    def GetPosition(self, elaspedTime):
        position, isMoving, weight, velocity = self.CaluculateMotion(elaspedTime, self.target[self.target_index]['position'])
        self.posRetained = position

        if isMoving == False:
            position, isMoving = self.posRetained, False

        return position, isMoving, weight, velocity

    def GetRotation(self):
        try:
            rotation = self.rotRetained = np.dot(cf.Convert2Matrix(self.q_init), next(self.predictedRotation))
            isMoving = True
        except StopIteration:
            rotation, isMoving = self.rotRetained, False

        return rotation, isMoving

    def GetGripperValue(self):
        try:
            gripper = self.gripRetained = next(self.predictedGripper)
            isMoving = True
        except StopIteration:
            gripper, isMoving = self.gripRetained, False

        return gripper, isMoving

    def MonitoringMotion(self, position, rotation, gripper, velocity, accelaration, elaspedTime):
        isMoving = False
        diff_init = np.linalg.norm(np.array(position))
        self.time_list.append(elaspedTime)

        if self.switchManager.flag == True:
            if diff_init >= self.initThreshold:
                target_index = self.DetermineTarget(self.target, position)
                self.tf = self.CalculateReachingTime(self.time_list[-1], velocity, self.target[self.target_index]['position'])
                print(self.tf)
                self.CreateMotionData(rotation, gripper, self.target[target_index]['position'], self.target[target_index]['rotation'], self.target[target_index]['gripper'], elaspedTime)
                isMoving = True
                self.switchManager.flag = False

        return isMoving

    def CreateMotionData(self, rot_n, grip_n, pos_f, rot_f, grip_f, tn):
        self.tn = tn
        DataPlotManager.thres = tn
        frameLength = int((self.tf-(tn - self.t0))* self.freq)
        print(frameLength)

        # self.CreateMotionMinimumJerk(t4, tf, x0, pos_f, frameLength, t0)
        self.CreateSlerpMotion(rot_n, rot_f, frameLength)
        self.CreateGripMotion(grip_n, grip_f, frameLength, gripFrame = 300)

    def DetermineTarget(self, target_list, position):
        diffList = []
        for target in target_list:
            diffList.append(np.linalg.norm(np.array(position) - target['position']))

        return diffList.index(min(diffList))

    # def CalculateReachingTime(self, t, v, xf):
    #     a = np.sqrt((xf[0] - self.x0[0])**2 + (xf[1] - self.x0[1])**2 + (xf[2] - self.x0[2])**2)
    #     A = (25 + 135*a*v + 3*np.sqrt(5)*np.sqrt(250*a*v + 165*(a**2)*(v**2) + 192*(a**3)*(v**3)))**(1/3)
    #     B = 5 - 12*a*v
    #     C = 185**(2/3)
    #     D = B/(C*A)
    #     E = A/C
    #     F = -1/12 + E + F

    #     return 1/12 + 0.5*np.sqrt(F) + 0.5*np.sqrt(-1/6 - D - E -5/108*F)

    def CalculateReachingTime(self, t, v, xf):
        a = self.a =  np.sqrt((xf[0] - self.x0[0])**2 + (xf[1] - self.x0[1])**2 + (xf[2] - self.x0[2])**2)
        b = v/(30*a)
        c = 0.5*(1 - np.sqrt(1 - 4*np.sqrt(b)))
        print(c)

        return (t - self.t0)/c


    def CreateGripMotion(self, grip_n, grip_f, frameLength, gripFrame):
        def CreateMotion_Liner(target, data, split):
            motionlist = np.linspace(data, target, split)
            return motionlist

        diffGrip = [850] * frameLength
        diffGrip = np.concatenate([diffGrip, CreateMotion_Liner(grip_f, grip_n, gripFrame)], 0)

        self.predictedGripper = iter(diffGrip)

    def CreateSlerpMotion(self, rot_n, rot_f, frameLength):
        weight_list = np.linspace(0, 1, int(frameLength*0.7))
        rot_list = []
        for weight in weight_list:
            rot_list.append(cf.Slerp_Quaternion(rot_f, rot_n[0], weight))

        self.predictedRotation = iter(np.array(rot_list))

    def CreateMotionMinimumJerk(self, t3, tf, x0, xf, frameLength, t0):

        def function(x0, xf, flame):
            return x0 + (xf- x0)* (6* (flame** 5)- 15* (flame** 4)+ 10* (flame** 3))

        flame = np.linspace((t3-t0)/tf, 1, frameLength)
        # flame = np.linspace(0, 1, frameLength)

        position = []
        for i in range(3):
            position.append(function(x0[i], xf[i], flame))
        print('t3: {}'.format(t3))

        # print(np.transpose(position)[1:])

        self.predictedPosition = iter(np.transpose(np.array(position))[1:])

    def CaluculateMotion(self, elaspedTime, xf):
        isMoving = True
        t = (elaspedTime - self.t0)/self.tf
        if t > 1:
            t = 1
            isMoving = False
        weight = (t - (self.tn - self.t0)/self.tf)/(1-(self.tn - self.t0)/self.tf)
        print(weight)

        return self.x0 + (xf- self.x0)* (6* (t** 5)- 15* (t** 4)+ 10* (t** 3)), isMoving, weight, 30 * self.a * (t**4 - 2*(t**3) + t**2)

if __name__ == '__main__':
    def CalculateReachingTime(t, v, xf, x0):
        a = np.sqrt((xf[0] - x0[0])**2 + (xf[1] - x0[1])**2 + (xf[2] - x0[2])**2)
        b = v/(30*a)

        return 0.5*(1 - np.sqrt(1 - 4*np.sqrt(b)))

    T = CalculateReachingTime(3, 20, [80, 60, 0], [0, 0, 0])
    print(T)
