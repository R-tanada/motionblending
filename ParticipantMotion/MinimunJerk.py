import numpy as np
import time
import CustomFunction.CustomFunction as cf

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
        self.target_index = 0
        self.flag = False
        self.initThreshold = 50
        self.timer = True
        self.loopCount = 0
        self.startTime = 0

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
        diff = np.linalg.norm(self.target[self.target_index]['position'] - np.array(position))
        diff_init = np.linalg.norm(self.initPos - np.array(position))

        if diff_init > self.initThreshold:
            self.flag = True

        if self.flag == True:
            if diff_init >= self.initThreshold:
                if self.timer:
                    self.startTime = time.perf_counter()
                    self.loopCount = 0
                    self.timer = False

                self.loopCount += 1

                if diff <= self.Threshold:
                    tn = time.perf_counter() - self.startTime
                    self.CreateMotionData(tn, self.loopCount, position, velocity, accelaration, rotation, gripper, self.target[self.target_index]['position'], self.target[self.target_index]['rotation'], self.target[self.target_index]['gripper'])
                    self.target_index += 1
                    if self.target_index == 2:
                        self.target_index = 0
                    isMoving = True
                    self.flag = False
                    self.timer = True


        return isMoving
    
    def CreateMotionData(self, tn, loopCount, pos_n, vel_n, acc_n, rot_n, grip_n, pos_f, rot_f, grip_f):
        tf = self.Threshold/ np.linalg.norm(vel_n)
        frameLength = (loopCount/ tn) * (tf - tn)

        self.CreateMotionMinimumJerk(tn, pos_n, vel_n, acc_n, tf, pos_f, frameLength)
        self.CreateSlerpMotion(rot_n, rot_f, frameLength)
        self.CreateGripMotion(grip_n, grip_f, frameLength, gripFrame = 300)

    def CreateGripMotion(self, grip_n, grip_f, frameLength, gripFrame):
        def CreateMotion_Liner(target, data, split):
            motionlist = np.linspace(data, target, split)
            return motionlist
        
        diffGrip = [850] * frameLength
        diffGrip = np.concatenate([diffGrip, CreateMotion_Liner(grip_f, grip_n, gripFrame)], 0)

        self.predictedGripper = iter(diffGrip)
    
    def CreateSlerpMotion(self, rot_n, rot_f, frameLength):
        weight_list = np.linspace(0, 1, frameLength)
        rot_list = []
        for weight in weight_list:
            rot_list.append(cf.Slerp_Quaternion(rot_f, rot_n, weight))

        self.predictedRotation = iter(np.transpose(np.array(rot_list)))

    def CreateMotionMinimumJerk(self, tn, pos_n, vel_n, acc_n, tf, pos_f, frameLength, vel_f = [0, 0, 0], acc_f = [0, 0, 0]):
        a_matrix = [
            [1, tn, tn**2,  tn**3,      tn**4,      tn**5       ],
            [0, 1,  2*tn,   3*(tn**2),  4*(tn**3),  5*(tn**4)   ],
            [0, 0,  2,      6*tn,       12*(tn**2), 20*(tn**3)  ],
            [1, tf, tf**2,  tf**3,      tf**4,      tf**5       ],
            [0, 1,  2*tf,   3*(tf**2),  4*(tf**3),  5*(tf**4)   ],
            [0, 0,  2,      6*tf,       12*(tf**2), 20*(tf**3)  ]
        ]
        b_matrix = [pos_n, vel_n, acc_n, pos_f, vel_f, acc_f]
        coeff = np.linalg.solve(a_matrix, b_matrix)

        def function(coeff, x):
            return coeff[0] + coeff[1]*x + coeff[2]*(x**2) + coeff[3]*(x**3) + coeff[4]*(x**4) + coeff[5]*(x**5)

        flame = np.linspace(tn, tf, int(frameLength))

        position = []
        for i in range(3):
            position.append(function(coeff[:, i], flame))

        self.predictedPosition = iter(np.transpose(np.array(position)))