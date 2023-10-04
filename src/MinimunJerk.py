import threading
import time
from itertools import cycle as iter_cycle

import numpy as np
from matplotlib import pyplot as plt

import lib.self.CustomFunction as cf
import lib.self.function as fc
from src.DataManager import DataPlotManager
from src.SensorManager import FootSwitchManager


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
        self.elaspedTime = 0
        self.init_time = time.perf_counter()

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
        self.elaspedTime = time.perf_counter() - self.init_time
        position, isMoving, weight, velocity = self.CaluculateMotion(self.elaspedTime, self.target[self.target_index]['position'])
        self.posRetained = position

        if isMoving == False:
            position, isMoving = self.posRetained, False

        return position, isMoving, weight, velocity

    # def GetRotation(self):
    #     try:
    #         rotation = self.rotRetained = np.dot(cf.Convert2Matrix(self.q_init), next(self.predictedRotation))
    #         isMoving = True
    #     except StopIteration:
    #         rotation, isMoving = self.rotRetained, False

    #     return rotation, isMoving

    def GetRotation(self, elaspedTime):
        self.elaspedTime = time.perf_counter() - self.init_time
        rotation, isMoving, weight = self.CaluculateSlerpMotion(self.elaspedTime, self.target[self.target_index]['rotation'])
        self.rotRetained = rotation

        if isMoving == False:
            rotation, isMoving = self.rotRetained, False

        return rotation, isMoving, weight

    def GetGripperValue(self):
        try:
            gripper = self.gripRetained = next(self.predictedGripper)
            isMoving = True
        except StopIteration:
            gripper, isMoving = self.gripRetained, False

        return gripper, isMoving

    def MonitoringMotion(self, position, rotation, gripper, velocity, accelaration):
        isMoving = False

        if self.switchManager.flag == True:
            self.init_time = time.perf_counter()
            self.x0 = position
            self.flag = True
            self.switchManager.flag = False

        if self.flag == True:
            self.elaspedTime = time.perf_counter() - self.init_time
            diff_init = np.linalg.norm(np.array(position) - np.array(self.x0))
            self.time_list.append(self.elaspedTime)
            self.pos_list.append(position)

            if diff_init >= self.initThreshold:
                self.rot_n = rotation[0]
                self.target_index = self.DetermineTarget(self.target, position, self.pos_list[-1]-self.pos_list[-2])
                self.tf = self.CalculateReachingTime(self.time_list[-1], velocity, self.target[self.target_index]['position'])
                self.CreateMotionData(rotation, gripper, self.target[self.target_index]['position'], self.target[self.target_index]['rotation'], self.target[self.target_index]['gripper'], self.elaspedTime)
                isMoving = True
                self.flag = False

        return isMoving

    def CreateMotionData(self, rot_n, grip_n, pos_f, rot_f, grip_f, tn):
        self.tn = tn
        DataPlotManager.thres = tn
        frameLength = int((self.tf-(tn - self.t0))* self.freq)

        # self.CreateMotionMinimumJerk(t4, tf, x0, pos_f, frameLength, t0)
        # self.CreateSlerpMotion(rot_n, rot_f, frameLength)
        self.CreateGripMotion(grip_n, grip_f, frameLength, gripFrame = 200)

    # def DetermineTarget(self, target_list, position):
    #     diffList = []
    #     for target in target_list:
    #         diffList.append(np.linalg.norm(np.array(position) - target['position']))

    #     return diffList.index(min(diffList))

    def DetermineTarget(self, target_list, position, vector):
        D_list = []
        x = position
        a = vector
        alfa = beta = gamma = 0
        for target in target_list:
            y = target['position']
            alfa = np.dot(a, x-y)
            beta = np.dot(a, a)
            k = -alfa/beta
            z = x + k*a
            d = np.linalg.norm(z - y)
            D_list.append(d)

            print('current: {}, target: {}, distance: {}'.format(x, y, d))
        print(D_list)

        return D_list.index(min(D_list))

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

    def CaluculateSlerpMotion(self, elaspedTime, xf):
        isMoving = True
        t = (self.elaspedTime - self.t0)/self.tf
        if t > 1:
            t = 1
            isMoving = False
        weight = (t - (self.tn - self.t0)/self.tf)/(1-(self.tn - self.t0)/self.tf)


        # print(weight)

        return cf.Slerp_Quaternion(xf, self.rot_n, weight), isMoving, weight


    # def CaluculateMotion(self, elaspedTime, xf): # デフォルト
    #     isMoving = True
    #     t = (self.elaspedTime - self.t0)/self.tf
    #     if t > 1:
    #         t = 1
    #         isMoving = False
    #     weight = (t - (self.tn - self.t0)/self.tf)/(1-(self.tn - self.t0)/self.tf)
    #     # print(weight)

    #     return self.x0 + (xf- self.x0)* (6* (t** 5)- 15* (t** 4)+ 10* (t** 3)), isMoving, weight, 30 * self.a * (t**4 - 2*(t**3) + t**2)

    def CaluculateMotion(self, elaspedTime, xf): # 割合変化をアレンジしたバージョン
        isMoving = True
        t = (self.elaspedTime - self.t0)/self.tf
        if t > 1:
            t = 1
            isMoving = False
        weight = (t - (self.tn - self.t0)/self.tf)/(1-(self.tn - self.t0)/self.tf)
        weight = fc.trapezium(weight)
        print(weight)

        return self.x0 + (xf- self.x0)* (6* (t** 5)- 15* (t** 4)+ 10* (t** 3)), isMoving, weight, 30 * self.a * (t**4 - 2*(t**3) + t**2)

if __name__ == '__main__':
    def CalculateReachingTime(t, v, xf, x0):
        a = np.sqrt((xf[0] - x0[0])**2 + (xf[1] - x0[1])**2 + (xf[2] - x0[2])**2)
        b = v/(30*a)

        return 0.5*(1 - np.sqrt(1 - 4*np.sqrt(b)))

    T = CalculateReachingTime(3, 20, [80, 60, 0], [0, 0, 0])
    print(T)
