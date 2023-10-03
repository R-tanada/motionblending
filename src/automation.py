import threading
import time
from itertools import cycle as iter_cycle

import numpy as np
from matplotlib import pyplot as plt

import lib.self.CustomFunction as cf
from src.datamanage import DataPlotManager
from src.sensor import FootSwitchManager


class Automation:
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

        self.method = Minimumjerk()

        self.switchManager = FootSwitchManager()
        switchThread = threading.Thread(target=self.switchManager.detect_sensor)
        switchThread.setDaemon(True)
        switchThread.start()

    def monitor_motion(self, position, rotation, gripper, velocity):
        isMoving = False
        motion = {'position': position, 'rotation': rotation, 'gripper': gripper}

        if self.switchManager.flag == True:
            self.method.set_init_params(position)
            self.flag = True
            self.switchManager.flag = False

        if self.flag == True:
            self.elaspedTime = time.perf_counter() - self.init_time
            diff_init = np.linalg.norm(np.array(position) - np.array(self.x0))
            self.time_list.append(self.elaspedTime)
            self.pos_list.append(position)

            if diff_init >= self.initThreshold:#動作の切り替え地点threshold
                self.target_index = self.detect_target(self.target, position, self.pos_list[-1]-self.pos_list[-2])
                self.tf = self.get_reaching_time(self.time_list[-1], velocity, self.target[self.target_index]['position'])
                self.method.set_via_params()
                self.method.calculate_traject_params()
                isMoving = True

            if isMoving == True:
                motion, isMoving = self.method.get_motion()
                if isMoving == False:
                    self.flag = False
            else:
                pass

        return motion

    def detect_target(self, target_list, position, vector):
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

    def get_reaching_time(self, t, v, xf):
        a = self.a =  np.sqrt((xf[0] - self.x0[0])**2 + (xf[1] - self.x0[1])**2 + (xf[2] - self.x0[2])**2)
        b = v/(30*a)
        c = 0.5*(1 - np.sqrt(1 - 4*np.sqrt(b)))
        print(c)

        return (t - self.t0)/c
    
    def update_init_position(self, position):
        self.updated_init_position = self.initPosition - (np.array(position) - self.get_position())
        p_list = np.linspace(self.updated_init_position, self.init_position, 500)
        self.iter_init_position = iter(p_list)

    def update_init_rotation(self, rotation):
        q_zero = [0, 0, 0, 1]
        quaternion = self.get_rotation()
        q_inverse = np.dot(cf.Convert2InverseMatrix(quaternion), q_zero)
        self.updateInitQuaternion = np.dot(cf.Convert2Matrix(rotation[0]), q_inverse)
        weight_list = np.linspace(0, 1, 300)
        q_list = []
        for weight in weight_list:
            q_list.append(cf.Slerp_Quaternion(self.initQuaternion, self.updateInitQuaternion, weight))

        self.iter_initRot = iter(q_list)

    def lerp_init_position(self):
        try:
            self.initPosition, flag = next(self.iter_init_position), True
        except StopIteration:
            flag = False

        return flag

    def slerp_init_rotation(self):
        try:
            rot = next(self.iter_initRot)
            self.initQuaternion, self.initInverseMatrix, flag = rot, cf.Convert2InverseMatrix(rot), True
        except StopIteration:
            flag = False
            self.initQuaternion = self.automation.q_init
            self.initInverseMatrix = cf.Convert2InverseMatrix(self.automation.q_init)

        return flag
    
class Minimumjerk:
    def __init__(self) -> None:
        self.init_time = 0
        self.x0 = 0

    def set_init_params(self, position):
        self.init_time = time.perf_counter()
        self.x0 = position

    def set_via_params(self):
        pass

    def get_motion(self):
        position, pos_flag = self.get_position()
        rotation, rot_flag = self.get_rotation()
        gripper, grip_flag = self.get_gripper()


    def get_position(self, elaspedTime):
        self.elaspedTime = time.perf_counter() - self.init_time
        position, isMoving, weight, velocity = self.CaluculateMotion(self.elaspedTime, self.target[self.target_index]['position'])
        self.posRetained = position

        if isMoving == False:
            position, isMoving = self.posRetained, False

        return position, isMoving, weight, velocity

    def get_rotation(self, elaspedTime):
        self.elaspedTime = time.perf_counter() - self.init_time
        rotation, isMoving, weight = self.CaluculateSlerpMotion(self.elaspedTime, self.target[self.target_index]['rotation'])
        self.rotRetained = rotation

        if isMoving == False:
            rotation, isMoving = self.rotRetained, False

        return rotation, isMoving, weight

    def get_gripper(self):
        try:
            gripper = self.gripRetained = next(self.predictedGripper)
            isMoving = True
        except StopIteration:
            gripper, isMoving = self.gripRetained, False

        return gripper, isMoving
    
    def calculate_traject_params(self):
        pass




if __name__ == '__main__':
    def CalculateReachingTime(t, v, xf, x0):
        a = np.sqrt((xf[0] - x0[0])**2 + (xf[1] - x0[1])**2 + (xf[2] - x0[2])**2)
        b = v/(30*a)

        return 0.5*(1 - np.sqrt(1 - 4*np.sqrt(b)))

    T = CalculateReachingTime(3, 20, [80, 60, 0], [0, 0, 0])
    print(T)
