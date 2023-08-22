import time
from itertools import cycle as iter_cycle

import numpy as np
from matplotlib import pyplot as plt

import lib.self.CustomFunction as cf


class MinimumJerk:
    def __init__(self, Target: list, xArmConfig: dict, Threshold = 200) -> None:
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
        self.x0 = 0
        self.target_index = 0
        self.pos_list = []

    # def GetPosition(self, elaspedTime):
    #     try:
    #         position = self.posRetained = self.CaluculateMotion(elaspedTime, self.target[self.target_index]['position'])
    #         isMoving = True
    #     except StopIteration:
    #         position, isMoving = self.posRetained, False

    #     return position, isMoving

    def GetPosition(self, elaspedTime):
        position, isMoving = self.CaluculateMotion(elaspedTime, self.target[self.target_index]['position'])
        self.posRetained = position

        if isMoving == False:
            position, isMoving = self.posRetained, False

        return position, isMoving

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
        self.pos_list.append(position)

        if diff_init < self.initThreshold:
            self.flag = True
            # print('hello')

        if self.flag == True:
            if diff_init >= self.initThreshold:
                if self.acc_flag == 1:
                    self.wayPoint.append({'time': self.time_list[-1], 'position': self.pos_list[-1], 'velocity': velocity})
                    self.acc_flag = 2
                    print('1st')

                elif self.acc_flag == 2:
                    if self.before_acc * accelaration < 0:
                        self.wayPoint.append({'time': self.time_list[-1], 'position': self.pos_list[-1], 'velocity': velocity})
                        self.acc_flag = 3
                        print('2nd')
                    self.before_acc = accelaration

                elif self.acc_flag == 3:
                    if elaspedTime >= 1.5 * self.wayPoint[1]['time'] - 0.5 * self.wayPoint[0]['time']:
                        self.wayPoint.append({'time': self.time_list[-1], 'position': self.pos_list[-1], 'velocity': velocity})
                        self.wayPoint.append({'time': self.time_list[-1], 'position': self.pos_list[-1], 'velocity': velocity})
                        self.acc_flag = 1
                        print('3rd')

                if len(self.wayPoint) == 4:
                    target_index = self.DetermineTarget(self.target, position)
                    self.CreateMotionData(self.wayPoint, rotation, gripper, self.target[target_index]['position'], self.target[target_index]['rotation'], self.target[target_index]['gripper'])
                    isMoving = True
                    self.flag = False
                    self.wayPoint = []

        # if self.flag == True:
        #     if diff_init >= self.initThreshold:
        #         if self.acc_flag == 1:
        #             self.wayPoint.append({'time': self.time_list[-1], 'position': position, 'velocity': velocity})
        #             self.acc_flag = 2
        #             print('1st')

        #         elif self.acc_flag == 2:
        #             if (accelaration - self.before_acc) < 0:
        #                 self.wayPoint.append({'time': self.time_list[-1], 'position': position, 'velocity': velocity})
        #                 self.acc_flag = 3
        #                 print('2nd')
        #             self.before_acc = accelaration

        #         elif self.acc_flag == 3:
        #             if self.before_acc * accelaration < 0:
        #                 self.wayPoint.append({'time': self.time_list[-1], 'position': position, 'velocity': velocity})
        #                 self.wayPoint.append({'time': self.time_list[-1], 'position': position, 'velocity': velocity})
        #                 self.acc_flag = 1
        #             self.before_acc = accelaration

        # if self.flag == True:
        #     if position[2]**2 + position[0]**2 >= self.initThreshold**2:
        #         self.wayPoint.append({'time': elaspedTime, 'position': position, 'velocity': velocity})

        #         if (velocity - self.before_vel) < 0:
        #             self.wayPoint.append({'time': elaspedTime, 'position': position, 'velocity': velocity})
        #             self.target_index = self.DetermineTarget(self.target, position)
        #             self.CreateMotionData([self.wayPoint[0], self.wayPoint[int(len(self.wayPoint)/1.8)], self.wayPoint[-1]], rotation, gripper, self.target[self.target_index]['position'], self.target[self.target_index]['rotation'], self.target[self.target_index]['gripper'])
        #             isMoving = True
        #             self.flag = False
        #             self.wayPoint = []
        #             velocity = 0

        #         self.before_vel = velocity

        return isMoving

    def DetermineTarget(self, target_list, position):
        diffList = []
        for target in target_list:
            diffList.append(np.linalg.norm(np.array(position) - target['position']))

        return diffList.index(min(diffList))

    def CreateMotionData(self, wayPoint, rot_n, grip_n, pos_f, rot_f, grip_f):
        t1, t2, t3, t4 = wayPoint[0]['time'], wayPoint[1]['time'], wayPoint[2]['time'], wayPoint[3]['time']
        v1, v2, v3 = wayPoint[0]['velocity'], wayPoint[1]['velocity'], wayPoint[2]['velocity']
        self.t0, self.tf, self.x0 = self.GetMinimumJerkParams(t1, t2, t3, v1, v2, v3, pos_f, wayPoint[3]['position'], t4)
        frameLength = int((self.tf-(t4 - self.t0))* self.freq)
        print(frameLength)

        # self.CreateMotionMinimumJerk(t4, tf, x0, pos_f, frameLength, t0)
        self.CreateSlerpMotion(rot_n, rot_f, frameLength)
        self.CreateGripMotion(grip_n, grip_f, frameLength, gripFrame = 300)

    def GetMinimumJerkParams(self, t1, t2, t3, v1, v2, v3, pf, x3, t4):
        def CalculateInitialTime(t1, t2, t3, v1, v2, v3):
            v1, v2 = np.sqrt(v1/ v2), np.sqrt(v2/ v3)
            a = t1 - t2 - t1*v2 + t3*v2 + t2*v1*v2 -t3*v1*v2
            b = -(t1**2 - t2**2 - (t1**2)*v2 + (t3**2)*v2 + (t2**2)*v1*v2 -(t3**2)*v1*v2)
            c = (t1**2)*t2 - t1*(t2**2) - (t1**2)*t3*v2 + t1*(t3**2)*v2 + (t2**2)*t3*v1*v2 - t2*(t3**2)*v1*v2
            return (-b - np.sqrt(b**2 - 4*a*c)) / (2*a)

        def CalculateReachingTime(t0, t1, t2, v1, v2):
            v1 = np.sqrt(v1/ v2)
            return ((t1 - t0)**2 - v1*(t2 - t0)**2) / ((t1 - t0) - v1*(t2 - t0))

        def CalculateInitialPosition(t0, t4, tf, xf, x3):
            t = (t4 - t0)/ tf
            print(t)
            s = 6*(t**5) - 15*(t**4) + 10*(t**3)
            return (x3 - xf*s)/ (1 - s)

        # def CalculateInitialPosition(t0, t3, tf, xf, v3):
        #     a = v3 * (tf**5)
        #     b = 30* ((t3 - t0)**2)*((t3 - t0 - tf)**2)
        #     return xf - a/b

        t0 = CalculateInitialTime(t1, t2, t3, v1, v2, v3)
        tf = CalculateReachingTime(t0, t1, t2, v1, v2)
        x0 = CalculateInitialPosition(t0, t4, tf, pf, x3)

        print(t0, tf, x0)

        return t0, tf, x0

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
        if (elaspedTime - self.t0)/self.tf > 1:
            isMoving = False
        print((elaspedTime - self.t0)/self.tf)
        return self.x0 + (xf- self.x0)* (6* (((elaspedTime - self.t0)/self.tf)** 5)- 15* (((elaspedTime - self.t0)/self.tf)** 4)+ 10* (((elaspedTime - self.t0)/self.tf)** 3)), isMoving
