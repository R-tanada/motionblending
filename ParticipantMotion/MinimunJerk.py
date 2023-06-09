import numpy as np
import time
import CustomFunction.CustomFunction as cf
from itertools import cycle as iter_cycle
from matplotlib import pyplot as plt

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
        self.initThreshold = 50
        self.startTime = time.perf_counter()
        self.wayPoint = []
        self.freq = 200
        self.acc_flag = True
        self.before_acc = 0
        self.posBox = []
        self.timeBox = []
        
    def GetPosition(self):
        try:
            position = self.posRetained = next(self.predictedPosition)
            isMoving = True
        except StopIteration:
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

    def MonitoringMotion(self, position, rotation, gripper, velocity, accelaration):
        isMoving = False
        timeMoving = time.perf_counter() - self.startTime
        diff_init = np.linalg.norm(np.array(position))
        # velocity = np.linalg.norm(np.array(velocity))
        # accelaration = np.linalg.norm(np.array(accelaration))

        if diff_init > self.initThreshold:
            self.flag = True

        if self.flag == True:
            if diff_init >= self.initThreshold:
                self.posBox.append(position[0])
                self.timeBox.append(time.perf_counter() - self.startTime)
                if self.acc_flag == 1:
                    self.wayPoint.append({'time': timeMoving, 'position': position, 'velocity': velocity})
                    self.acc_flag = 2
                    print('1st')

                elif self.acc_flag == 2:
                    if self.before_acc * accelaration < 0:
                        self.wayPoint.append({'time': timeMoving, 'position': position, 'velocity': velocity})
                        self.acc_flag = 3
                        print('2nd')
                    self.before_acc = accelaration

                elif self.acc_flag == 3:
                    if timeMoving >= 1.5 * self.wayPoint[1]['time'] - 0.5 * self.wayPoint[0]['time']:
                        self.wayPoint.append({'time': timeMoving, 'position': position, 'velocity': velocity})
                        self.acc_flag = 1
                        print('3rd')

                if len(self.wayPoint) == 3:
                    print('currentPosition: {}'.format(position))
                    target_index = self.DetermineTarget(self.target, position)
                    self.CreateMotionData(self.wayPoint, rotation, gripper, self.target[target_index]['position'], self.target[target_index]['rotation'], self.target[target_index]['gripper'])
                    isMoving = True
                    self.flag = False
                    self.wayPoint = []

        return isMoving
    
    def DetermineTarget(self, target_list, position):
        diffList = []
        for target in target_list:
            diffList.append(np.linalg.norm(np.array(position) - target['position']))

        return diffList.index(min(diffList))
    
    def CreateMotionData(self, wayPoint, rot_n, grip_n, pos_f, rot_f, grip_f):
        t3, tf, x0, t0 = self.GetMinimumJerkParams(wayPoint[0]['time'], wayPoint[1]['time'], wayPoint[2]['time'], wayPoint[0]['velocity'], wayPoint[1]['velocity'], wayPoint[2]['velocity'], pos_f)
        frameLength = int((tf- t3)* self.freq)

        self.CreateMotionMinimumJerk(t3, tf, x0, pos_f, frameLength, t0)
        self.CreateSlerpMotion(rot_n, rot_f, frameLength)
        self.CreateGripMotion(grip_n, grip_f, frameLength, gripFrame = 300)

    def GetMinimumJerkParams(self, t1, t2, t3, v1, v2, v3, pf):
        def CalculateInitialTime(t1, t2, t3, v1, v2, v3):
            v1, v2 = np.sqrt(v1/ v2), np.sqrt(v2/ v3)
            a = t1 - t2 - t1*v2 + t3*v2 + t2*v1*v2 -t3*v1*v2
            b = -(t1**2 - t2**2 - (t1**2)*v2 + (t3**2)*v2 + (t2**2)*v1*v2 -(t3**2)*v1*v2)
            c = (t1**2)*t2 - t1*(t2**2) - (t1**2)*t3*v2 + t1*(t3**2)*v2 + (t2**2)*t3*v1*v2 - t2*(t3**2)*v1*v2
            return (-b + np.sqrt(b**2 - 4*a*c)) / (2*a)
    
        def CalculateReachingTime(t0, t1, t2, v1, v2):
            v12 = np.sqrt(v1/ v2)
            return -((t0- t1)** 2- v12* (t0- t2)** 2)/ ((t0- t1)- v12* (t0- t2))
    
        def CalculateInitialPosition(t0, t3, tf, v3, xf):
            return xf- (v3* tf** 4)/ (30* ((t3- t0)* (t3- t0- tf))** 2)

        t0 = CalculateInitialTime(t1, t2, t3, v1, v2, v3)
        tf = CalculateReachingTime(t0, t1, t2, v1, v2)
        x0 = CalculateInitialPosition(t0, t3, tf, v3, pf)

        print(t0, tf, x0)

        return t3- t0, tf, x0, t0

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
            return x0 + (xf- x0)* (6* flame** 5- 15* flame** 4+ 10* flame** 3)

        flame = np.linspace(t3, tf, frameLength)

        position = []
        for i in range(3):
            position.append(function(x0[i], xf[i], flame))

        plt.plot(flame, position[0])
        plt.plot(self.timeBox - t0, self.posBox)
        plt.show()

        self.predictedPosition = iter(np.transpose(np.array(position)))